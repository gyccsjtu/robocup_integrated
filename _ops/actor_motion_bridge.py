#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""恐怖分子运动桥接 v2：/actor_<i>/cmd_motion -> /gazebo/set_model_state

背景：
  官方 base.world 里 6 个 actor 挂了 libros_actor_cmd_pose_plugin.so，它本该把
  /actor_<i>/cmd_motion 里的 (x, y, v) 变成 actor 的实际位移。本机（Gazebo 11 +
  Xvfb 软渲染）实测该插件不产生位移（用正确的 ActorMotion 类型、v=1.0 连发 15 秒，
  actor 位移 0.000）。为了让官方 control_actor.py 能正常闭环，这里补一层外部
  适配节点：订阅官方发布的 cmd_motion，按 v (m/s) 把 actor 推向目标点。

  官方 control_actor.py 自己在做"瞬移"时用的就是 /gazebo/set_model_state
  （见 actor_teleportation_callback），所以本桥接沿用官方同样的机制。

卡死自愈：
  control_actor.py 的到达判定是 distance < 0.01（第 473 行），也就是说 actor 必须
  真的走到目标点才会换下一个目标。如果某个目标因为边界/墙体而够不到，actor 就会
  永远停在原地不再更新目标。因此本节点监测：同一个目标持续 STUCK_SEC 秒且位移小于
  STUCK_MOVE 米时，直接把 actor 瞬移到该目标（官方自己也有瞬移机制），
  让到达判定成立、control_actor 继续下发新目标。

说明：这是新增的适配节点，不修改任何官方文件（base.world / control_actor.py /
      score_cal.py 全部保持原样）。恐怖分子"去哪、跑多快、要不要逃"仍然完全由
      官方 control_actor.py 决定，本节点只负责把"走"这个动作真正执行出来。
"""
import math
import time
import rospy

from ros_actor_cmd_pose_plugin_msgs.msg import ActorMotion
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState, SetModelStateRequest

NUM_ACTOR = 6
LOOP_HZ = 20

# 活动范围与官方 control_actor.py 的边界保持一致（第 43-46 行：
# x_min=-50, x_max=130, y_min=-60, y_max=60）
X_MIN, X_MAX = -50.0, 130.0
Y_MIN, Y_MAX = -60.0, 60.0

Z_FIXED = 1.019      # 官方插件里 pose.Pos().Z(1.0191)
ROLL = 1.5707        # 官方插件里 pose.Rot() = Quaterniond(1.5707, 0, yaw)

STUCK_SEC = 20.0     # 同一目标持续这么久还没走到 → 判定卡死
STUCK_MOVE = 0.8     # 且这段时间位移小于这么多米
LOG_EVERY = 30.0     # 状态日志间隔

targets = [None] * NUM_ACTOR      # (x, y, v)
known_pos = {}                    # actor_name -> (x, y)
# 每个 actor 的卡死跟踪：目标、目标设定时刻、设定时的位置
stuck = [dict(tgt=None, t0=0.0, pos=None) for _ in range(NUM_ACTOR)]
moved_total = [0.0] * NUM_ACTOR
teleport_count = [0] * NUM_ACTOR


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _quat_from_roll_yaw(roll, yaw):
    """等价于 ignition::math::Quaterniond(roll, 0, yaw)"""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (sr * cy,        # x
            sr * sy,        # y
            cr * sy,        # z
            cr * cy)        # w


class ActorMotionBridge(object):
    def __init__(self):
        rospy.init_node('actor_motion_bridge', anonymous=False)
        for i in range(NUM_ACTOR):
            rospy.Subscriber('/actor_%d/cmd_motion' % i, ActorMotion,
                             self._make_cb(i), queue_size=1)
        rospy.Subscriber('/gazebo/model_states', ModelStates,
                         self._states_cb, queue_size=1)
        rospy.wait_for_service('/gazebo/set_model_state', timeout=60)
        self._set_state = rospy.ServiceProxy('/gazebo/set_model_state',
                                             SetModelState)
        self._last_log = time.time()
        rospy.loginfo('[bridge] 恐怖分子运动桥接已启动：%d 个 actor，%d Hz'
                      % (NUM_ACTOR, LOOP_HZ))

    @staticmethod
    def _make_cb(idx):
        def cb(msg):
            v = msg.v if msg.v and msg.v > 1e-6 else 1.0
            t = (msg.x, msg.y, float(v))
            if t != targets[idx]:
                targets[idx] = t
                st = stuck[idx]
                cur = known_pos.get('actor_%d' % idx)
                if st['tgt'] != t:
                    st['tgt'] = t
                    st['t0'] = time.time()
                    st['pos'] = cur
        return cb

    @staticmethod
    def _states_cb(msg):
        for name, pose in zip(msg.name, msg.pose):
            if name.startswith('actor_'):
                known_pos[name] = (pose.position.x, pose.position.y)

    def _apply(self, i, nx, ny, yaw):
        name = 'actor_%d' % i
        req = SetModelStateRequest()
        ms = req.model_state
        ms.model_name = name
        ms.pose.position.x = nx
        ms.pose.position.y = ny
        ms.pose.position.z = Z_FIXED
        ms.pose.orientation.x, ms.pose.orientation.y, \
            ms.pose.orientation.z, ms.pose.orientation.w = \
            _quat_from_roll_yaw(ROLL, yaw)
        ms.reference_frame = 'world'
        try:
            self._set_state(req)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(5, '[bridge] set_model_state 失败(%s): %s',
                                   name, e)
            return False
        known_pos[name] = (nx, ny)
        return True

    def run(self):
        rate = rospy.Rate(LOOP_HZ)
        last_t = time.time()
        while not rospy.is_shutdown():
            now = time.time()
            dt = now - last_t
            last_t = now
            if dt <= 0.0 or dt > 1.0:
                rate.sleep()
                continue

            for i in range(NUM_ACTOR):
                tgt = targets[i]
                if tgt is None:
                    continue
                name = 'actor_%d' % i
                cur = known_pos.get(name)
                if cur is None:
                    continue

                st = stuck[i]
                tx, ty, v = tgt
                dx, dy = tx - cur[0], ty - cur[1]
                dist = math.hypot(dx, dy)
                yaw = math.atan2(dy, dx) if dist > 1e-6 else 0.0

                # --- 卡死自愈：够不到的目标直接瞬移过去，让官方到达判定成立 ---
                if (st['pos'] is not None and
                        (now - st['t0']) > STUCK_SEC and
                        math.hypot(cur[0] - st['pos'][0],
                                   cur[1] - st['pos'][1]) < STUCK_MOVE and
                        dist > 0.2):
                    rospy.loginfo('[bridge] actor_%d 目标 (%.1f, %.1f) 卡死 %.0fs，'
                                  '瞬移到目标点（第 %d 次）',
                                  i, tx, ty, now - st['t0'], teleport_count[i] + 1)
                    teleport_count[i] += 1
                    self._apply(i, _clamp(tx, X_MIN, X_MAX),
                                _clamp(ty, Y_MIN, Y_MAX), yaw)
                    st['t0'] = time.time()
                    st['pos'] = known_pos.get(name)
                    continue

                if dist < 1e-3:
                    continue

                step = min(v * dt, dist)
                nx = _clamp(cur[0] + dx / dist * step, X_MIN, X_MAX)
                ny = _clamp(cur[1] + dy / dist * step, Y_MIN, Y_MAX)
                if self._apply(i, nx, ny, yaw):
                    moved_total[i] += step

            if now - self._last_log > LOG_EVERY:
                self._last_log = now
                parts = []
                for i in range(NUM_ACTOR):
                    p = known_pos.get('actor_%d' % i)
                    t = targets[i]
                    if p is None:
                        parts.append('actor_%d=?' % i)
                    else:
                        parts.append('actor_%d=(%.1f,%.1f)' % (i, p[0], p[1]))
                    if t is not None:
                        parts.append('->(%.1f,%.1f)' % (t[0], t[1]))
                rospy.loginfo('[bridge] ' + ' '.join(parts))
                rospy.loginfo('[bridge] 累计位移: ' +
                              ' '.join('%.1fm' % m for m in moved_total) +
                              ' | 瞬移次数: ' +
                              ' '.join('%d' % c for c in teleport_count))
            rate.sleep()


if __name__ == '__main__':
    ActorMotionBridge().run()
