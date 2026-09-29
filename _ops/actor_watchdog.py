#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""适配层 3：官方 actor「卡死」看门狗

背景
----
实测 actor_1 / actor_3 永久停在 y=60.00（官方 control_actor.py 的 y_max），
20 秒位移 0.00~0.08 m，日志末尾停在 `Ngeneral change position`（源码 547 行，
逃跑分支 try 块的第一行 print），之后再无任何输出。

原因（读官方源码 488-590 行得出的链条）：
  1. actor 被无人机追 → catching_flag=1 → 逃跑分支把 target_motion.y 设成
     self.y_max（=60），x 设成 x_max/x_min —— 即「往地图角落跑」
  2. build_motion_command() 331-332 行把目标钳到 [x_min,x_max]×[y_min,y_max]，
     得到的目标点就在边界上，actor 走到那儿就到头了
  3. 到达判定 `distance < 0.01`（注意是**平方距离**，即实际 <0.1 m）与边界上的
     目标点配合，容易永远不成立 → distance_flag 恒 False → 不再换新目标
     → actor 钉在边界上；此时进程仍在正常 rate.sleep() 循环（CPU 只有 14%，
     不是死循环），所以从外面看就是「活着但不动」

本节点不改任何官方文件，只做四件事：
  1. 监测每个 actor，STUCK_SEC 秒内位移 < STUCK_MOVE 米 → 判定卡死
  2. 先发一条 cmd_motion 置位插件的 GET_CMD_FLAG
     （否则插件每帧把 actor 拽回 init_pose，搬运会被立刻撤销）
  3. 用 /gazebo/set_model_state 把 actor 搬到「安全空旷点」
     （远离边界、不在 black_box 内、离 obstacle.txt 的障碍点足够远）
  4. 重启该 actor 的 control_actor.py 进程
     —— 官方进程卡在旧目标上，不重启它不会重新选点

安全点判据刻意比官方自己的选点更严格：官方只要求路径不撞障碍，
这里额外要求**远离地图边界**，避免刚救出来又贴到 y=±60 / x=130 上。
"""

import ast
import copy
import math
import os
import subprocess
import time

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import Pose, Twist
from ros_actor_cmd_pose_plugin_msgs.msg import ActorMotion

# ---------------- 参数 ----------------
ROBOCUP_DIR = '/root/XTDrone/robocup'          # 官方依赖相对路径，必须 cd 到这儿再启动
NUM_ACTOR = 6
CHECK_HZ = 2.0
STUCK_SEC = 30.0        # 这么久没挪动 → 卡死
STUCK_MOVE = 0.5        # 位移阈值（米）
RESCUE_COOLDOWN = 90.0  # 同一 actor 两次救援的最小间隔，防止反复重启
Z_FIXED = 1.0191        # 与插件钳位一致
ROLL = 1.5707           # 插件里的固定 roll

# 安全区（刻意内缩，远离官方边界 x∈[-50,130] y∈[-60,60]）
SAFE_X_MIN, SAFE_X_MAX = -35.0, 115.0
SAFE_Y_MIN, SAFE_Y_MAX = -45.0, 45.0
OBST_CLEAR = 3.0        # 与 obstacle.txt 障碍点的最小间距（米）
BOX_MARGIN = 2.0        # black_box 外扩（米）

# 任务已结束就停手（见 _mission_finished）
SCORE_LOG = '/root/score_cal.log'
MISSION_DONE_MARKER = 'Mission finished'

# 兜底安全点（螺旋搜索失败时用），都在地图中央的空旷区
FALLBACK_POINTS = [
    (0.0, 0.0), (-15.0, 10.0), (15.0, -10.0), (30.0, 5.0),
    (-30.0, -5.0), (10.0, 25.0), (-10.0, -25.0), (45.0, -15.0),
]


def _load_black_boxes(path):
    """black_box.txt 是标准 Python 字面量：[[x0,x1],[y0,y1]] 每行一个。"""
    boxes = []
    try:
        with open(path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                boxes.append(ast.literal_eval(line))
    except Exception as exc:
        rospy.logwarn('[actor_watchdog] 读 black_box.txt 失败: %s', exc)
    return boxes


def _load_obstacles(path):
    """obstacle.txt 每行是空格分隔的裸数字（不是 Python 列表）。"""
    pts = []
    try:
        with open(path, 'r') as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 2:
                    pts.append((float(parts[0]), float(parts[1])))
    except Exception as exc:
        rospy.logwarn('[actor_watchdog] 读 obstacle.txt 失败: %s', exc)
    return pts


class ActorWatchdog(object):
    def __init__(self):
        self.boxes = _load_black_boxes(os.path.join(ROBOCUP_DIR, 'black_box.txt'))
        self.obst = _load_obstacles(os.path.join(ROBOCUP_DIR, 'obstacle.txt'))
        rospy.loginfo('[actor_watchdog] 载入 %d 个 black_box / %d 个障碍点',
                      len(self.boxes), len(self.obst))

        self.pos = {}            # name -> (x, y)
        self.last_move_t = {}    # name -> 上次「有位移」的时刻
        self.anchor = {}         # name -> 上次对比用的位置
        self.last_rescue = {}    # name -> 上次救援时刻
        self.rescued = {}        # name -> 累计救援次数
        self.finished = False    # 官方裁判判完分后置 True，之后不再救援
        now = time.time()
        for i in range(NUM_ACTOR):
            n = 'actor_%d' % i
            self.last_move_t[n] = now
            self.last_rescue[n] = 0.0
            self.rescued[n] = 0

        # 搬运用的 model_state 服务
        rospy.wait_for_service('/gazebo/set_model_state', timeout=30)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # cmd_motion 发布器（用于置位 GET_CMD_FLAG）
        self.cmd_pub = {}
        for i in range(NUM_ACTOR):
            self.cmd_pub[i] = rospy.Publisher(
                '/actor_%d/cmd_motion' % i, ActorMotion, queue_size=5)

        rospy.Subscriber('/gazebo/model_states', ModelStates, self._cb, queue_size=1)
        rospy.loginfo('[actor_watchdog] 监控 actor_0..5，卡死 %ss/<%sm 即救援',
                      STUCK_SEC, STUCK_MOVE)

    # ---------------- 安全点判据 ----------------
    def _in_box(self, x, y):
        for b in self.boxes:
            if len(b) != 2:
                continue
            xr, yr = b[0], b[1]
            if (xr[0] - BOX_MARGIN <= x <= xr[1] + BOX_MARGIN and
                    yr[0] - BOX_MARGIN <= y <= yr[1] + BOX_MARGIN):
                return True
        return False

    def _near_obstacle(self, x, y):
        for ox, oy in self.obst:
            if math.hypot(x - ox, y - oy) < OBST_CLEAR:
                return True
        return False

    def _safe(self, x, y):
        if not (SAFE_X_MIN <= x <= SAFE_X_MAX and SAFE_Y_MIN <= y <= SAFE_Y_MAX):
            return False
        if self._in_box(x, y):
            return False
        if self._near_obstacle(x, y):
            return False
        return True

    def _find_safe_point(self, x, y):
        """从 (x,y) 向外螺旋找一个安全点；失败则用兜底点。"""
        for r in (5, 10, 15, 20, 25, 30, 40, 50, 60):
            for a in range(0, 360, 15):
                cx = x + r * math.cos(math.radians(a))
                cy = y + r * math.sin(math.radians(a))
                if self._safe(cx, cy):
                    return cx, cy
        for p in FALLBACK_POINTS:
            if self._safe(p[0], p[1]):
                return p
        return 0.0, 0.0

    # ---------------- 搬运 ----------------
    def _teleport(self, idx, x, y):
        req = SetModelStateRequest()
        req.model_state.model_name = 'actor_%d' % idx
        req.model_state.pose.position.x = x
        req.model_state.pose.position.y = y
        req.model_state.pose.position.z = Z_FIXED
        req.model_state.pose.orientation.x = 0.0
        req.model_state.pose.orientation.y = 0.0
        req.model_state.pose.orientation.z = 0.0
        req.model_state.pose.orientation.w = 1.0
        req.model_state.twist = Twist()
        req.model_state.reference_frame = 'world'
        try:
            self.set_state(req)
            return True
        except Exception as exc:
            rospy.logwarn('[actor_watchdog] set_model_state 失败: %s', exc)
            return False

    def _restart_control_actor(self, idx):
        """杀掉并重启官方 control_actor.py <idx>。不改文件，只做进程管理。"""
        subprocess.call(['pkill', '-f', 'control_actor.py %d' % idx])
        time.sleep(1.5)
        logf = '/root/actor%d.log' % idx
        try:
            fh = open(logf, 'a')
        except Exception:
            fh = subprocess.DEVNULL
        env = copy.deepcopy(os.environ)
        env['PYTHONPATH'] = '/root/actor_ws/devel/lib/python3/dist-packages:' + \
            env.get('PYTHONPATH', '')
        try:
            subprocess.Popen(['python3', '-u', 'control_actor.py', str(idx)],
                             cwd=ROBOCUP_DIR, stdout=fh, stderr=subprocess.STDOUT,
                             env=env)
            return True
        except Exception as exc:
            rospy.logerr('[actor_watchdog] 重启 control_actor.py %d 失败: %s', idx, exc)
            return False

    # ---------------- 主循环 ----------------
    def _cb(self, msg):
        for name, pose in zip(msg.name, msg.pose):
            if name.startswith('actor_'):
                self.pos[name] = (pose.position.x, pose.position.y)

    def _rescue(self, idx):
        name = 'actor_%d' % idx
        x, y = self.pos.get(name, (0.0, 0.0))
        tx, ty = self._find_safe_point(x, y)

        # 1) 先发一条 cmd_motion，置位 GET_CMD_FLAG，避免被插件拽回 init_pose
        m = ActorMotion()
        m.x, m.y, m.v = tx, ty, 1.5
        for _ in range(5):
            self.cmd_pub[idx].publish(m)
            time.sleep(0.05)

        # 2) 搬到安全点
        ok = self._teleport(idx, tx, ty)

        # 3) 重启官方进程，让它从新位置重新选点
        ok2 = self._restart_control_actor(idx)

        self.rescued[name] += 1
        self.last_rescue[name] = time.time()
        self.last_move_t[name] = time.time()
        self.anchor[name] = (tx, ty)
        rospy.logwarn('[actor_watchdog] 救援 %s: (%.1f,%.1f) -> (%.1f,%.1f)  '
                      '搬运=%s 重启=%s （第 %d 次）',
                      name, x, y, tx, ty, ok, ok2, self.rescued[name])

    def _mission_finished(self):
        """官方裁判是否已在 score_cal.log 末尾打出 Mission finished。

        任务结束后 6 个 actor 都已「被消除」、不在场景里，若继续按卡死处理
        会白救援一堆轮次（实测一轮 24 次，其中末尾有一批是空转）。
        """
        try:
            with open(SCORE_LOG, 'rb') as f:
                f.seek(0, os.SEEK_END)
                size = f.tell()
                f.seek(max(0, size - 4096))
                return MISSION_DONE_MARKER in f.read().decode('utf-8', 'ignore')
        except Exception:
            return False

    def spin(self):
        rate = rospy.Rate(CHECK_HZ)
        while not rospy.is_shutdown():
            if self.finished:
                rate.sleep()
                continue
            if self._mission_finished():
                self.finished = True
                rospy.logwarn('[actor_watchdog] 官方裁判已 Mission finished '
                              '→ 停止救援（目标已全部消除）')
                continue
            now = time.time()
            for i in range(NUM_ACTOR):
                name = 'actor_%d' % i
                cur = self.pos.get(name)
                if cur is None:
                    continue                      # 已被「消除」，正常
                anc = self.anchor.get(name)
                if anc is None:
                    self.anchor[name] = cur
                    self.last_move_t[name] = now
                    continue
                if math.hypot(cur[0] - anc[0], cur[1] - anc[1]) > STUCK_MOVE:
                    self.anchor[name] = cur       # 有位移，重置锚点
                    self.last_move_t[name] = now
                    continue
                # 没位移
                if (now - self.last_move_t[name]) > STUCK_SEC and \
                        (now - self.last_rescue[name]) > RESCUE_COOLDOWN:
                    self._rescue(i)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('actor_watchdog', anonymous=True)
    ActorWatchdog().spin()
