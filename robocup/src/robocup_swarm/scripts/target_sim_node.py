#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""恐怖分子仿真节点（自建实现，对应比赛规则 1/2/3/4/5）。

职责：模拟 6 名恐怖分子的行为，发布真值状态供无人机检测/判分使用。

规则实现：
  规则1 初始   : 从 metadata 的 6 个 goal_candidates 随机分配出生点（均在空旷区）
  规则2 游走   : 未感知无人机 → 1 m/s 随机走动（带避障转弯，不穿建筑）
  规则3 逃跑   : 感知到无人机靠近 → 2 m/s 逃离，并广播无人机位置给全体同伙
  规则4 瞬移   : 被持续感知 30s 仍未消除 → 随机瞬移到地图另一处躲藏
  规则5 消除   : 收到管理器的消除指令（已连续 15s 确认）→ 从世界移除

设计说明：
  - 恐怖分子只在地面活动，不进入建筑（用栅格碰撞检查保证）
  - 集体广播（规则3）：任一恐怖分子感知到无人机 → 全体进入 FLEE 状态
    并朝「远离最近无人机」的方向逃跑（信息共享的直接体现）
  - 瞬移（规则4）：计时由本节点维护（targeted_until），不依赖管理器
"""

import math
import os
import random
import time

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

# 设置随机种子，保证每次运行结果可复现（但不同运行之间不同）
random.seed(int(time.time()) % 100000)

from robocup_swarm.msg import TargetState, TargetDetection, UavStatus
from robocup_navigation.astar import load_metadata, GridMap

# ============================ 参数 ============================
METADATA_PATH = os.environ.get(
    "ROBOCUP_METADATA",
    os.path.join(os.environ.get("ROBOCUP_WS", "/home/ros/team_ws/robocup"),
                 "src/robocup_training_worlds/worlds/generated/training_city_full_s7.json"))
NUM_TARGETS       = 6      # 规则：6 名恐怖分子
WALK_SPEED        = 1.0    # 规则2：未感知无人机时的随机走动速度 m/s
FLEE_SPEED        = 2.0    # 规则3：感知到无人机后的逃跑速度 m/s
SENSE_RADIUS      = 20.0   # 恐怖分子感知无人机的半径 m（与无人机感知半径对称）
EVADE_TIME        = 30.0   # 规则4：被感知 30s 未消除 → 瞬移
TARGET_RADIUS     = 0.6    # 目标体半径 m（与 metadata goal_candidates 一致）
WANDER_TURN_PROB  = 0.6    # 每秒改变游走方向的概率
PUB_RATE          = 10.0   # 状态发布频率 Hz
SIM_RATE          = 20.0   # 行为更新频率 Hz

STATE_WANDER, STATE_FLEE, STATE_RELOCATED, STATE_ELIMINATED = 0, 1, 2, 3


class Terrorist(object):
    """单个恐怖分子的状态与运动。"""

    def __init__(self, target_id, x, y):
        self.target_id = target_id
        self.x, self.y = float(x), float(y)
        self.vx = self.vy = 0.0
        self.state = STATE_WANDER
        self.heading = random.uniform(0.0, 2.0 * math.pi)  # 当前游走方向
        self.tracked_time = 0.0    # 被感知累计时长（规则4 计时）
        self.confirm_time = 0.0    # 连续确认时长（规则5 计时，由管理器回传）
        self.eliminated = False

    def pos(self):
        return (self.x, self.y)


class TargetSimNode(object):
    def __init__(self, uav_ids):
        self.uav_ids = uav_ids
        self.uav_pos = {}       # uav_id -> (x, y)，来自 /swarm/uav_status

        # ---- 地图（用于碰撞检查，保证不穿建筑）----
        self.md, _ = load_metadata(METADATA_PATH)
        self.grid = GridMap.from_metadata(self.md)
        self.res = self.grid.resolution

        # ---- 生成 6 个恐怖分子（全地图随机可通行点）----
        self.targets = {}
        for i in range(NUM_TARGETS):
            tid = "t%d" % i
            # 全地图随机可通行点，不局限于 goal_candidates
            x, y = self._random_free_point()
            self.targets[tid] = Terrorist(tid, x, y)
        rospy.loginfo("[target_sim] 生成 %d 个恐怖分子（随机）：%s", NUM_TARGETS,
                      [(t.target_id, round(t.x, 1), round(t.y, 1))
                       for t in self.targets.values()])

        # ---- 发布 / 订阅 ----
        self.state_pub = rospy.Publisher("/swarm/target_states", TargetState, queue_size=10)
        self.marker_pub = rospy.Publisher("/swarm/target_markers", MarkerArray, queue_size=2)
        rospy.Subscriber("/swarm/uav_status", UavStatus, self._status_cb)
        # 消除指令：管理器发出 "eliminate:<target_id>"
        rospy.Subscriber("/swarm/target_command", String, self._cmd_cb)

        self.sim_rate = rospy.Rate(SIM_RATE)
        self.pub_rate = rospy.Rate(PUB_RATE)
        self._last_t = rospy.Time.now().to_sec()

    # ---------------- 工具 ----------------
    def _is_free(self, x, y):
        """(x,y) 是否在可通行区域（含目标半径的膨胀检查）。"""
        r = int(math.ceil((TARGET_RADIUS + 0.3) / self.res))
        c = self.grid.world_to_cell((x, y))
        if c is None:
            return False
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                cell = (c[0] + dx, c[1] + dy)
                if not self.grid.in_bounds(cell) or not self.grid.is_free(cell):
                    return False
        return True

    def _random_free_point(self):
        """在地图内随机取一个可通行点。"""
        for _ in range(500):
            x = random.uniform(self.grid.origin[0] + 2.0,
                               self.grid.origin[0] + self.grid.width * self.res - 2.0)
            y = random.uniform(self.grid.origin[1] + 2.0,
                               self.grid.origin[1] + self.grid.height * self.res - 2.0)
            if self._is_free(x, y):
                return (x, y)
        return (0.0, 0.0)   # 极端兜底

    def _nearest_uav_dist(self, x, y):
        """到最近无人机的水平距离（无无人机时返回 inf）。"""
        if not self.uav_pos:
            return float("inf"), None
        best, best_id = float("inf"), None
        for uid, (ux, uy) in self.uav_pos.items():
            d = math.hypot(ux - x, uy - y)
            if d < best:
                best, best_id = d, uid
        return best, best_id

    def _try_move(self, t, dt):
        """按当前速度 vx/vy 移动 dt 秒，撞建筑则随机改向（保证不穿墙）。

        注意 vx/vy 是**真实速度**（含 m/s 量纲，如 1.0 或 2.0），既用于此处位移，
        也直接发布给下游做匀速外推预测。
        """
        nx = t.x + t.vx * dt
        ny = t.y + t.vy * dt
        if self._is_free(nx, ny):
            t.x, t.y = nx, ny
            return True
        # 撞墙：换个方向重试（保持当前速度量级）
        speed = math.hypot(t.vx, t.vy) or WALK_SPEED
        t.heading = random.uniform(0.0, 2.0 * math.pi)
        t.vx = speed * math.cos(t.heading)
        t.vy = speed * math.sin(t.heading)
        return False

    # ---------------- 回调 ----------------
    def _status_cb(self, msg):
        self.uav_pos[msg.uav_id] = (msg.x, msg.y)

    def _cmd_cb(self, msg):
        """消除指令：eliminate:<target_id>（管理器已连续确认 15s 后发出）。"""
        parts = msg.data.split(":")
        if len(parts) != 2 or parts[0] != "eliminate":
            return
        tid = parts[1]
        t = self.targets.get(tid)
        if t is None or t.eliminated:
            return
        t.eliminated = True
        t.state = STATE_ELIMINATED
        rospy.loginfo("[target_sim] 规则5：%s 被消除（连续确认 15s），从世界移除", tid)

    # ---------------- 行为更新（规则 2/3/4） ----------------
    def _step(self, dt):
        # 先看「是否有任何恐怖分子感知到无人机」→ 规则3 的集体广播
        any_sensed = False
        for t in self.targets.values():
            if t.eliminated:
                continue
            d, _ = self._nearest_uav_dist(t.x, t.y)
            if d < SENSE_RADIUS:
                any_sensed = True
                break

        for t in self.targets.values():
            if t.eliminated:
                continue
            d, nearest_id = self._nearest_uav_dist(t.x, t.y)
            sensed = d < SENSE_RADIUS

            # ---- 规则4：被感知累计 30s 未消除 → 瞬移 ----
            if sensed:
                t.tracked_time += dt
                if t.tracked_time >= EVADE_TIME:
                    newx, newy = self._random_free_point()
                    t.x, t.y = newx, newy
                    t.tracked_time = 0.0
                    t.confirm_time = 0.0
                    t.state = STATE_RELOCATED
                    rospy.loginfo("[target_sim] 规则4：%s 被感知 %.0fs 未消除 → 瞬移到 "
                                  "(%.1f, %.1f) 躲藏", t.target_id, EVADE_TIME, newx, newy)
                    continue
            else:
                t.tracked_time = 0.0    # 脱离感知则计时清零

            # ---- 规则3：感知到无人机 → 逃跑 + 集体广播 ----
            if sensed or any_sensed:
                t.state = STATE_FLEE
                if nearest_id is not None:
                    ux, uy = self.uav_pos[nearest_id]
                    # 朝「远离最近无人机」的方向逃（群体共享该方向）
                    ang = math.atan2(t.y - uy, t.x - ux)
                    t.heading = ang + random.uniform(-0.4, 0.4)   # 略带随机，避免完全共线
                    t.vx = FLEE_SPEED * math.cos(t.heading)
                    t.vy = FLEE_SPEED * math.sin(t.heading)
                self._try_move(t, dt)   # 规则3：速度已含在 vx/vy 里（2 m/s）
            else:
                # ---- 规则2：随机走动 1 m/s ----
                t.state = STATE_WANDER
                if random.random() < WANDER_TURN_PROB * dt:
                    t.heading = random.uniform(0.0, 2.0 * math.pi)
                t.vx = WALK_SPEED * math.cos(t.heading)
                t.vy = WALK_SPEED * math.sin(t.heading)
                self._try_move(t, dt)   # 规则2：速度已含在 vx/vy 里（1 m/s）

    # ---------------- 发布 ----------------
    def _publish_states(self):
        now = rospy.Time.now()
        # 确保只补发一次 eliminated 消息
        if not hasattr(self, '_announced_eliminated'):
            self._announced_eliminated = set()

        for t in self.targets.values():
            if t.eliminated:
                # 补发一条 eliminated=True 的消息，让 agent 收到后退出盘旋
                if t.target_id not in self._announced_eliminated:
                    m = TargetState()
                    m.header.stamp = now
                    m.header.frame_id = "map"
                    m.target_id = t.target_id
                    m.x, m.y = t.x, t.y
                    m.vx, m.vy = 0, 0
                    m.state = STATE_ELIMINATED
                    m.tracked_time = t.tracked_time
                    m.confirm_time = t.confirm_time
                    m.eliminated = True
                    rospy.loginfo("[target_sim] 补发 %s eliminated=True", t.target_id)
                    self.state_pub.publish(m)
                    self._announced_eliminated.add(t.target_id)
                continue
            m = TargetState()
            m.header.stamp = now
            m.header.frame_id = "map"
            m.target_id = t.target_id
            m.x, m.y = t.x, t.y
            m.vx, m.vy = t.vx, t.vy
            m.state = t.state
            m.tracked_time = t.tracked_time
            m.confirm_time = t.confirm_time
            m.eliminated = False
            self.state_pub.publish(m)

    def _publish_markers(self):
        """RViz/Gazebo 可视化：不同状态不同颜色。"""
        arr = MarkerArray()
        for i, t in enumerate(sorted(self.targets.values(), key=lambda x: x.target_id)):
            if t.eliminated:
                continue
            mk = Marker()
            mk.header.frame_id = "map"
            mk.header.stamp = rospy.Time.now()
            mk.ns, mk.id = "targets", i
            mk.type = Marker.SPHERE
            mk.action = Marker.ADD
            mk.pose.position = Point(t.x, t.y, 0.5)
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = mk.scale.z = 1.2
            # 绿=游走 红=逃跑 蓝=已瞬移
            if t.state == STATE_WANDER:
                mk.color.r, mk.color.g, mk.color.b = 0.1, 1.0, 0.1
            elif t.state == STATE_FLEE:
                mk.color.r, mk.color.g, mk.color.b = 1.0, 0.1, 0.1
            else:
                mk.color.r, mk.color.g, mk.color.b = 0.2, 0.4, 1.0
            mk.color.a = 0.9
            mk.lifetime = rospy.Duration(0.5)
            arr.markers.append(mk)
        self.marker_pub.publish(arr)

    # ---------------- 主循环 ----------------
    def run(self):
        rospy.loginfo("[target_sim] 恐怖分子仿真节点启动，等待无人机状态...")
        last = rospy.Time.now().to_sec()
        last_pub = last
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            dt = now - last
            last = now
            if dt > 0.5:      # 时钟跳变保护
                dt = 0.5
            if dt <= 0.0:
                self.sim_rate.sleep()
                continue

            self._step(dt)

            if now - last_pub >= 1.0 / PUB_RATE:
                self._publish_states()
                self._publish_markers()
                last_pub = now
            self.sim_rate.sleep()


if __name__ == "__main__":
    rospy.init_node("target_sim_node")
    ids = rospy.get_param("~uav_ids", "uav_1,uav_2")
    if isinstance(ids, str):
        ids = ids.split(",")
    TargetSimNode(ids).run()
