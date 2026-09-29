#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""集群搜索单机节点（每架无人机跑一个实例，自建实现）。

职责（步骤 1：两机共享状态）：
  1. 从 /gazebo/model_states 读自身模型的世界坐标（地图系，与生成器 metadata 一致），
     从 /uav_N/mavros/local_position/pose 读高度与连接状态；
  2. 发布 /swarm/uav_status（UavStatus 自建消息）给集中式管理器；
  3. 订阅 /swarm/assignment（SearchAssignment），过滤出指派给自己的搜索格，
     用 ENU 速度控制飞向格中心。

坐标系说明（关键，避免多机坐标系踩坑）：
  世界/地图系与 gazebo 世界都是 ENU（x 东 y 北 z 上），各机本地 ENU 系只是
  原点不同、轴向平行。因此「世界坐标差」算出的速度向量可直接作为
  setpoint_velocity/cmd_vel 下发，无需逐机做 TF 变换。

只依赖标准库 + rospy + 标准消息，无 ROS 自定义依赖之外的第三方库。
"""

import os
import rospy
import math
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
from std_msgs.msg import String

from robocup_swarm.msg import UavStatus, SearchAssignment, TargetState, TargetDetection
from robocup_navigation.astar import load_metadata, GridMap, plan
from swarm_task import LineOfSight, DETECT_RADIUS

# ============================ 参数 ============================
# 工作空间根目录：可用环境变量 ROBOCUP_WS 覆盖（云端部署/换用户时无需改代码）
WS_ROOT = os.environ.get("ROBOCUP_WS", "/home/ros/team_ws/robocup")
SEARCH_ALTITUDE = 6.0     # 搜索高度 m（与单机避障比赛高度一致）

# 按 ID 分配基础高度层（6m 以内，垂直分离减少水平避障压力）
# 6 架飞机分 3 层：5.0m / 5.5m / 6.0m，层间距 0.5m，同层 2 机
# ORCA 同层判断 >2m，分层后六机不在同一层，垂直空间利用起来
ALTITUDE_BY_ID = {
    "uav_1": 5.0,
    "uav_2": 5.0,
    "uav_3": 5.5,
    "uav_4": 5.5,
    "uav_5": 6.0,
    "uav_6": 6.0,
}
ALTITUDE_ADJUST = 0.3   # 追踪/降落时允许的微调 ±0.3m（不超 6m）

MAX_SPEED       = 3.0     # 巡航速度上限 m/s
POS_KP          = 0.8     # 位置 P 控制增益
ARRIVE_TOL      = 0.8     # 到达格中心判定半径 m（< 此值视为已到，开始原地搜索）
PUB_RATE        = 10.0    # 状态发布频率 Hz
CTRL_RATE       = 20.0    # 控制频率 Hz
DETECT_RATE     = 5.0     # 目标检测发布频率 Hz（规则3 几何判定）

# ---- A* 避障飞行 ----
METADATA_PATH   = os.environ.get(
    "ROBOCUP_METADATA",
    os.path.join(WS_ROOT, "src/robocup_training_worlds/worlds/generated/training_city_full_s7.json"))
INFLATE_M       = 0.5     # A* 障碍膨胀半径 m（避免贴墙规划，覆盖旋翼半径+裕度）
LOOKAHEAD       = 3.0     # 路径跟踪前瞻距离 m（> 刹停距离 v²/2a=2.25m）
STABLE_NEEDED   = 40      # EKF 稳定判定：连续多少次 20Hz 采样高度达标（40 = 2s）


def inflate_grid(grid, inflation_m):
    """对障碍物向外膨胀 inflation_m（圆盘结构元素），返回新 GridMap。

    复用团队单机避障的同名工具逻辑：A* 若直接在原始栅格规划会贴墙，
    膨胀后路径与墙面保持安全距离。
    """
    r = int(math.ceil(inflation_m / grid.resolution))
    if r <= 0:
        return grid
    w, h = grid.width, grid.height
    occ = [(ix, iy) for iy in range(h) for ix in range(w)
           if not grid.is_free((ix, iy))]
    cells = bytearray(grid.cells)
    r2 = r * r
    for cx, cy in occ:
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r2:
                    continue
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < w and 0 <= ny < h:
                    cells[ny * w + nx] = 1
    return GridMap(w, h, grid.resolution, grid.origin, bytes(cells), grid.frame_id)


def smooth_path(points, samples_per_segment=6):
    """Catmull-Rom 样条平滑：折线拐点 -> 连续可跟踪轨迹。"""
    pts = list(points)
    if len(pts) < 3:
        return pts
    pts = [pts[0]] + pts + [pts[-1]]
    out = []
    for i in range(len(pts) - 3):
        p0, p1, p2, p3 = pts[i], pts[i + 1], pts[i + 2], pts[i + 3]
        for j in range(samples_per_segment):
            t = j / float(samples_per_segment)
            t2, t3 = t * t, t * t * t
            x = 0.5 * ((2 * p1[0]) + (-p0[0] + p2[0]) * t +
                       (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2 +
                       (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3)
            y = 0.5 * ((2 * p1[1]) + (-p0[1] + p2[1]) * t +
                       (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2 +
                       (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3)
            out.append((x, y))
    out.append(pts[-2])
    return out


class SwarmAgent(object):
    def __init__(self, uav_id, model_name):
        self.uav_id = uav_id
        self.model_name = model_name

        # ---- 状态 ----
        self.local_xy = None        # (x, y) MAVROS 局部坐标（轻量、高频）
        self.offset = None          # 局部->世界 的恒定平移 (dx, dy)，启动时标定一次
        self.local_z = None         # 高度（来自 local_position）
        self.state = State()
        self.assignment = None      # SearchAssignment 当前任务

        # ---- A* 避障 ----
        self.md, _ = load_metadata(METADATA_PATH)
        self.grid = inflate_grid(GridMap.from_metadata(self.md), INFLATE_M)
        self.path = []              # 当前全局路径（世界坐标航点列表）
        self.path_target = None     # 当前路径终点（格中心），用于判断是否需重规划
        self._last_plan_t = 0.0

        # ---- 目标检测（规则3 几何判定：距离 + 视线遮挡）----
        # 用**未膨胀**的原始栅格做 LOS 判定（膨胀是给飞行留裕度的，
        # 判定遮挡要用真实建筑轮廓，否则会把建筑边缘 0.5m 内误判为遮挡）。
        raw_grid = GridMap.from_metadata(self.md)
        self.los = LineOfSight(
            lambda ix, iy: (not raw_grid.in_bounds((ix, iy)))
                           or (not raw_grid.is_free((ix, iy))),
            cell_size=raw_grid.resolution, origin=raw_grid.origin)
        self.targets = {}           # target_id -> (x, y, vx, vy)，来自 /swarm/target_states
        self._last_detect_t = 0.0

        # ---- MAVROS 服务 ----
        self.arm_srv = rospy.ServiceProxy("/%s/mavros/cmd/arming" % uav_id, CommandBool)
        self.mode_srv = rospy.ServiceProxy("/%s/mavros/set_mode" % uav_id, SetMode)
        self.param_srv = rospy.ServiceProxy("/%s/mavros/param/set" % uav_id, ParamSet)

        # ---- 订阅 ----
        # 注意：**不订阅 /gazebo/model_states**。它是 250Hz × 687 模型 × 6.4MB/s 的
        # 巨型消息，每个 agent 都要用 Python 反序列化，实测每机吃掉 ~37% CPU
        # （比 PX4 还高），6 机时是灾难。改为只订阅轻量的 MAVROS local_position，
        # 启动时用一次 model_states 标定「局部->世界」的恒定平移即可。
        rospy.Subscriber("/%s/mavros/state" % uav_id, State, self._state_cb)
        rospy.Subscriber("/%s/mavros/local_position/pose" % uav_id, PoseStamped, self._local_cb)
        rospy.Subscriber("/swarm/assignment", SearchAssignment, self._assign_cb)
        rospy.Subscriber("/swarm/target_states", TargetState, self._target_cb)
        rospy.Subscriber("/swarm/finish", String, self._finish_cb)

        # ---- 发布 ----
        self.status_pub = rospy.Publisher("/swarm/uav_status", UavStatus, queue_size=5)
        self.detect_pub = rospy.Publisher("/swarm/detection", TargetDetection, queue_size=10)
        self.vel_pub = rospy.Publisher("/%s/mavros/setpoint_velocity/cmd_vel" % uav_id,
                                       TwistStamped, queue_size=5)

        self.ctrl_rate = rospy.Rate(CTRL_RATE)
        self.pub_rate = rospy.Rate(PUB_RATE)
        self._last_status_t = rospy.Time.now()
        self._mission_finished = False  # 任务完成标志

    @property
    def world_xy(self):
        """世界坐标 = 局部坐标 + 标定偏移（偏移未标定好前返回 None）。"""
        if self.local_xy is None or self.offset is None:
            return None
        return (self.local_xy[0] + self.offset[0], self.local_xy[1] + self.offset[1])

    def _calibrate_offset(self):
        """用一次 /gazebo/model_states 标定局部->世界的恒定平移，随后不再订阅。

        世界系（地图/metadata）与各机 MAVROS 局部系轴向平行、仅原点不同，
        故偏移是常量（实测两机分别恒定，误差 <0.01m），标定一次即可。
        """
        try:
            msg = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=20.0)
        except rospy.ROSException as exc:
            rospy.logerr("[%s] 标定失败：取不到 model_states（%s）", self.uav_id, exc)
            return False
        try:
            i = msg.name.index(self.model_name)
        except ValueError:
            rospy.logerr("[%s] 标定失败：model_states 里没有 %s", self.uav_id, self.model_name)
            return False
        # 等 local_xy 就绪
        for _ in range(200):
            if self.local_xy is not None:
                break
            self.ctrl_rate.sleep()
        if self.local_xy is None:
            rospy.logerr("[%s] 标定失败：local_position 无数据", self.uav_id)
            return False
        p = msg.pose[i].position
        self.offset = (p.x - self.local_xy[0], p.y - self.local_xy[1])
        rospy.loginfo("[%s] 坐标系标定完成：offset=(%.2f, %.2f)", self.uav_id,
                      self.offset[0], self.offset[1])
        return True

    # ---------------- 回调 ----------------
    def _state_cb(self, msg):
        self.state = msg

    def _local_cb(self, msg):
        self.local_z = msg.pose.position.z
        self.local_xy = (msg.pose.position.x, msg.pose.position.y)

    def _assign_cb(self, msg):
        if msg.uav_id == self.uav_id:
            self.assignment = msg

    def _target_cb(self, msg):
        """缓存恐怖分子真值位置（几何判定用；真机上是视觉/裁判给出的观测）。"""
        if msg.eliminated:
            self.targets.pop(msg.target_id, None)
            return
        self.targets[msg.target_id] = (msg.x, msg.y, msg.vx, msg.vy)

    def _finish_cb(self, msg):
        """收到任务完成广播后退出搜索循环。"""
        if msg.data == "MISSION_FINISHED":
            rospy.loginfo("[%s] 收到任务完成广播，退出搜索", self.uav_id)
            self._mission_finished = True

    # ---------------- 目标检测（规则3：几何判定） ----------------
    def _detect_targets(self):
        """对每个已知目标做「距离 + 视线遮挡」判定，命中则发布 TargetDetection。

        规则3 的几何判定：水平距离 < DETECT_RADIUS **且** 中间无建筑遮挡。
        注意只用水平距离 —— 无人机在 6m 高度、目标在地面，垂直差恒定，
        水平距才是决定「能否看到」的量（与比赛判定的平面几何一致）。
        """
        wx = self.world_xy
        if wx is None:
            return
        now = rospy.Time.now().to_sec()
        if now - self._last_detect_t < 1.0 / DETECT_RATE:
            return
        self._last_detect_t = now

        for tid, (tx, ty, _vx, _vy) in list(self.targets.items()):
            d = math.hypot(tx - wx[0], ty - wx[1])
            if d > DETECT_RADIUS:
                continue
            if not self.los.visible(wx[0], wx[1], tx, ty):
                continue    # 隔着建筑，不算看到
            m = TargetDetection()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = "map"
            m.uav_id = self.uav_id
            m.target_id = tid
            m.x, m.y = tx, ty
            m.confidence = 1.0
            m.source = 0     # 0=几何判定
            self.detect_pub.publish(m)

    # ---------------- 参数与模式 ----------------
    def _set_param(self, param_id, value):
        from mavros_msgs.msg import ParamValue
        try:
            return self.param_srv(param_id, ParamValue(integer=value, real=0.0)).success
        except Exception as exc:
            rospy.logwarn_throttle(10, "[%s] 设置 %s 异常: %s", self.uav_id, param_id, exc)
            return False

    def _configure_fcu(self):
        """SITL 无遥控器会触发 RC 失联 failsafe，拒绝解锁/进 OFFBOARD。"""
        self._set_param("NAV_RCL_ACT", 0)
        self._set_param("COM_RCL_EXCEPT", 4)

    def _arm_and_offboard(self):
        """预热 setpoint → 切 OFFBOARD → 解锁。与单机避障已验证的时序一致。

        多机 SITL 注意：第二架及以后的机 EKF 收敛更慢，local_position 可能
        出现 -12m 之类的瞬时坏值，直接切 OFFBOARD 会失败。故先等 EKF 收敛
        （local_z 就绪且 |z| 合理），OFFBOARD 切换失败则重试而非直接放弃。

        关键：预热阶段必须发「纯零速」(vz=0)，而非 _send_vel 的竖直爬升速度。
        单机脚本 dwa_avoidance 预热发 _send_vel(0,0,0)，若此处带 vz=0.5*(6-z)
        的爬升速度，EKF 未收敛时 PX4 会拒绝 OFFBOARD。
        """
        # 等 EKF 大致稳定：要求「连续 STABLE_NEEDED 次采样」高度在出生点附近。
        # 单次采样不够 —— 多机 SITL 下 EKF 位置先收敛、速度/姿态后稳定，瞬时达标
        # 就切 OFFBOARD 会被 PX4 拒。阈值放宽到 1.5m（地面上有噪声抖动），真正
        # 是否就绪交给 PX4 preflight 判定，靠后续多次重试兜底。
        stable = 0
        while not rospy.is_shutdown():
            if self.local_z is not None and abs(self.local_z) < 1.5:
                stable += 1
                if stable >= STABLE_NEEDED:
                    break
            else:
                if stable > 0:
                    rospy.logwarn_throttle(5, "[%s] EKF 抖动（z=%.2f），重新计数",
                                           self.uav_id,
                                           self.local_z if self.local_z is not None else -999)
                stable = 0
            self._send_vel(0.0, 0.0, vz=0.0)
            self.ctrl_rate.sleep()
        rospy.loginfo("[%s] EKF 已稳定（z=%.2f，连续 %d 次采样达标）", self.uav_id,
                      self.local_z if self.local_z is not None else -999, stable)

        # 预热：持续发纯零速，让 OFFBOARD setpoint 生效（vz=0 而非爬升）
        for _ in range(120):
            self._send_vel(0.0, 0.0, vz=0.0)
            self.ctrl_rate.sleep()

        # 切 OFFBOARD：**持续重试直到成功**（多机 SITL 下 EKF 速度/姿态估计就绪
        # 时间不定，固定次数会误判放弃，导致必须重启整个 SITL。永不放弃 + 每次
        # 重试间隔继续发零速预热，EKF 稳了自然就进得去。）
        attempt = 0
        while not rospy.is_shutdown() and self.state.mode != "OFFBOARD":
            attempt += 1
            if not self.mode_srv(0, "OFFBOARD").mode_sent:
                rospy.logwarn_throttle(10, "[%s] 切换 OFFBOARD 请求失败（第 %d 次）",
                                       self.uav_id, attempt)
            for _ in range(50):
                if self.state.mode == "OFFBOARD":
                    break
                self._send_vel(0.0, 0.0, vz=0.0)
                self.ctrl_rate.sleep()
            if self.state.mode == "OFFBOARD":
                break
            rospy.logwarn_throttle(10, "[%s] OFFBOARD 未生效（第 %d 次，当前 %s），等待 EKF 稳定后重试",
                                   self.uav_id, attempt, self.state.mode)
            for _ in range(40):
                if self.state.mode == "OFFBOARD":
                    break
                self._send_vel(0.0, 0.0, vz=0.0)
                self.ctrl_rate.sleep()
        if rospy.is_shutdown():
            return False
        rospy.loginfo("[%s] 已进入 OFFBOARD（重试 %d 次）", self.uav_id, attempt)

        # 解锁：同样**持续重试直到成功**。PX4 preflight 会因 EKF 速度估计未稳 /
        # Roll failure 拒解锁，等 EKF 稳了自然会成功。
        attempt = 0
        while not rospy.is_shutdown() and not self.state.armed:
            attempt += 1
            if not self.arm_srv(True).success:
                rospy.logwarn_throttle(10, "[%s] 解锁请求失败（第 %d 次），等待 EKF 稳定后重试",
                                       self.uav_id, attempt)
            for _ in range(50):
                if self.state.armed:
                    break
                self._send_vel(0.0, 0.0, vz=0.0)
                self.ctrl_rate.sleep()
        if rospy.is_shutdown():
            return False
        rospy.loginfo("[%s] OFFBOARD + 解锁完成（解锁重试 %d 次）", self.uav_id, attempt)
        return True

    # ---------------- 控制 ----------------
    def _get_target_altitude(self, adjust=0.0):
        """获取当前飞机的目标飞行高度。

        adjust: 临时调整量（±m），用于追踪/降落时临时调整。
        返回: 目标高度 = ID对应基础高度 + adjust（限制不超过 6m）
        """
        base = ALTITUDE_BY_ID.get(self.uav_id, SEARCH_ALTITUDE)
        # 限制调整范围，且不超过 6m 上限
        adjust = max(-ALTITUDE_ADJUST, min(ALTITUDE_ADJUST, adjust))
        return min(6.0, base + adjust)

    def _send_vel(self, vx, vy, vz=None):
        """下发 ENU 水平速度 + 保持搜索高度。

        vz=None 时按当前高度竖直 P 控制爬升到搜索高度；预热/悬停阶段显式传
        vz=0.0 发纯零速，避免 EKF 未收敛时带爬升速度导致 OFFBOARD 被拒。
        """
        cmd = TwistStamped()
        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        # 纯速度模式：vz 语义是「保持当前高度」，这里给竖直 P 控制爬升到搜索高度
        if vz is not None:
            cmd.twist.linear.z = vz
        elif self.local_z is not None:
            target_alt = self._get_target_altitude()
            cmd.twist.linear.z = 0.5 * (target_alt - self.local_z)
        self.vel_pub.publish(cmd)

    def _plan_path(self, goal_xy):
        """从当前世界坐标 A* 规划到 goal_xy，绕开膨胀后的建筑障碍。"""
        if self.world_xy is None:
            return False
        route = plan(self.grid, self.world_xy, goal_xy, connectivity=8)
        if not route.success and route.reason == "START_OCCUPIED":
            # 起点落在膨胀障碍内（刚起飞/贴墙）→ 找最近自由栅格重试
            nearest = self._nearest_free_cell(goal_xy)
            if nearest is not None:
                route = plan(self.grid, nearest, goal_xy, connectivity=8)
        if not route.success:
            rospy.logwarn("[%s] A* 规划到 (%.1f,%.1f) 失败: %s",
                          self.uav_id, goal_xy[0], goal_xy[1], route.reason)
            return False
        self.path = smooth_path(route.points, samples_per_segment=6)
        self.path_target = goal_xy
        rospy.loginfo("[%s] A* 规划：%d 航点 -> %d 平滑点",
                      self.uav_id, len(route.points), len(self.path))
        return True

    def _nearest_free_cell(self, goal_xy):
        """起点被占时，找当前坐标附近最近自由栅格（局部搜索，避免全地图 O(w*h) 扫描）。"""
        if self.world_xy is None:
            return None
        cx, cy = self.world_xy
        # 以当前坐标为圆心，半径 5m 内找自由栅格（膨胀后建筑间距 > 5m）
        radius_cells = int(math.ceil(5.0 / self.grid.resolution))
        center_cell = self.grid.world_to_cell((cx, cy))
        if center_cell is None:
            return None
        best, best_d = None, float("inf")
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                cell = (center_cell[0] + dx, center_cell[1] + dy)
                if not self.grid.is_free(cell):
                    continue
                wx, wy = self.grid.cell_to_world(cell)
                d = math.hypot(wx - cx, wy - cy)
                if d < best_d:
                    best_d = d
                    best = (wx, wy)
        return best

    def _pick_local_goal(self):
        """沿全局路径从最近点往前取 LOOKAHEAD 距离的引导目标。"""
        if not self.path or self.world_xy is None:
            return None
        cx, cy = self.world_xy
        best = 0
        best_dist = float("inf")
        for i, (px, py) in enumerate(self.path):
            d = math.hypot(px - cx, py - cy)
            if d < best_dist:
                best_dist = d
                best = i
        acc = 0.0
        for i in range(best, len(self.path) - 1):
            x0, y0 = self.path[i]
            x1, y1 = self.path[i + 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            if acc + seg >= LOOKAHEAD:
                frac = (LOOKAHEAD - acc) / max(seg, 1e-6)
                return (x0 + (x1 - x0) * frac, y0 + (y1 - y0) * frac)
            acc += seg
        return self.path[-1]

    def _control(self):
        """有任务 → A* 绕障飞向格中心；无任务 → 原地悬停。"""
        if self.world_xy is None:
            self._send_vel(0.0, 0.0)
            return

        if self.assignment is None:
            self.path = []
            self.path_target = None
            self._send_vel(0.0, 0.0)
            return

        goal = (self.assignment.target_x, self.assignment.target_y)
        dist = math.hypot(goal[0] - self.world_xy[0], goal[1] - self.world_xy[1])

        if dist < ARRIVE_TOL:
            # 已到达格中心，原地盘旋搜索（速度 0）
            self.path = []
            self._send_vel(0.0, 0.0)
            return

        # 目标变了或还没有路径 → 重新 A* 规划（失败时 2s 内不重复尝试，避免刷屏）
        if self.path_target is None or self.path_target != goal or not self.path:
            now = rospy.Time.now().to_sec()
            if now - self._last_plan_t >= 2.0:
                self._last_plan_t = now
                if not self._plan_path(goal):
                    # 规划失败（无路可达）→ 原地悬停，等下一次分配
                    self._send_vel(0.0, 0.0)
                    return
            else:
                # 刚失败过，还在冷却期，悬停等待
                self._send_vel(0.0, 0.0)
                return

        # 沿路径跟踪：取 lookahead 引导点，P 控制 + 限速
        local_goal = self._pick_local_goal()
        if local_goal is None:
            self._send_vel(0.0, 0.0)
            return
        err_x = local_goal[0] - self.world_xy[0]
        err_y = local_goal[1] - self.world_xy[1]
        vx = POS_KP * err_x
        vy = POS_KP * err_y
        spd = math.hypot(vx, vy)
        if spd > MAX_SPEED:
            vx *= MAX_SPEED / spd
            vy *= MAX_SPEED / spd
        self._send_vel(vx, vy)

    # ---------------- 状态上报 ----------------
    def _publish_status(self):
        if self.world_xy is None:
            return
        st = UavStatus()
        st.header.stamp = rospy.Time.now()
        st.uav_id = self.uav_id
        st.x = self.world_xy[0]
        st.y = self.world_xy[1]
        st.z = self.local_z if self.local_z is not None else 0.0
        st.connected = self.state.connected
        if self.assignment is not None:
            st.cell_ix = self.assignment.cell_ix
            st.cell_iy = self.assignment.cell_iy
        else:
            st.cell_ix = -1
            st.cell_iy = -1
        # 到达格中心后视为「已覆盖」，置信度给 1.0（管理器据此标记 STATE_COVERED）
        if self.assignment is not None and self.world_xy is not None:
            d = math.hypot(self.assignment.target_x - self.world_xy[0],
                           self.assignment.target_y - self.world_xy[1])
            st.confidence = 1.0 if d < ARRIVE_TOL else 0.0
        else:
            st.confidence = 0.0
        self.status_pub.publish(st)

    # ---------------- 主循环 ----------------
    def run(self):
        # 等待连接
        while not rospy.is_shutdown() and not self.state.connected:
            self.ctrl_rate.sleep()
        rospy.loginfo("[%s] MAVROS 已连接", self.uav_id)
        # 等局部位置就绪
        while not rospy.is_shutdown() and self.local_xy is None:
            self.ctrl_rate.sleep()
        # 标定「局部->世界」的恒定平移（只取一次 model_states，之后不再订阅）
        if not self._calibrate_offset():
            raise SystemExit(1)

        self._configure_fcu()
        if not self._arm_and_offboard():
            raise SystemExit(1)

        # 打印本机的高度层配置并启动搜索
        target_alt = self._get_target_altitude()
        rospy.loginfo("[%s] 开始协同搜索（目标高度 %.1f m），等待管理器分配任务",
                      self.uav_id, target_alt)
        while not rospy.is_shutdown():
            # 收到任务完成广播后退出搜索循环
            if self._mission_finished:
                rospy.loginfo("[%s] 任务已完成，退出搜索循环", self.uav_id)
                break
            self._control()
            self._detect_targets()      # 规则3：几何判定，命中即上报管理器
            if (rospy.Time.now() - self._last_status_t).to_sec() >= 1.0 / PUB_RATE:
                self._publish_status()
                self._last_status_t = rospy.Time.now()
            self.ctrl_rate.sleep()


if __name__ == "__main__":
    rospy.init_node("swarm_agent", anonymous=True)
    uav_id = rospy.get_param("~uav_id", "uav_1")
    model_name = rospy.get_param("~model_name", "iris_1")
    rospy.loginfo("swarm_agent 启动: uav_id=%s model=%s", uav_id, model_name)
    SwarmAgent(uav_id, model_name).run()
