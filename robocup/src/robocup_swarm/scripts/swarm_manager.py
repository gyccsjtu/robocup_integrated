#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""集群协同搜索集中式管理器（自建实现，步骤 1：两机共享状态）。

职责：
  1. 订阅各机 /swarm/uav_status，维护机队位置与任务格覆盖状态；
  2. 用 swarm_task.py 的 CoverageGrid + TaskAllocator 做集中式拍卖，给每机分配
     不重复的未搜索格；用 LeaseManager 处理掉线/卡死超时重分配；
  3. 发布 /swarm/assignment（SearchAssignment），每机按 uav_id 过滤自己的任务。

设计约束（答辩/查重）：
  - 只复用自研 swarm_task.py（纯逻辑，原创拍卖效用 + 租约），不复制 Crazyswarm2
    的任何节点/消息/类/配置。
  - 管理器只做「任务分配」，不做避障/轨迹，避障由各机 ESDF-DWA 自理。
"""

import os
import rospy
import math

from robocup_swarm.msg import UavStatus, SearchAssignment, TargetState, TargetDetection
from std_msgs.msg import String

# 直接 import 同目录的纯逻辑模块（scripts 目录已加进 PYTHONPATH）
from swarm_task import (CoverageGrid, TaskAllocator, LeaseManager, TargetTracker,
                        STATE_FREE, STATE_ASSIGNED, STATE_COVERED,
                        W_GAIN, W_FLIGHT, W_OVERLAP, W_RISK, W_DISTANCE, LEASE_DURATION,
                        CONFIRM_TIME, EVADE_TIME)

# 地图（用于过滤「格中心落在建筑内」的格子，与 agent 的 A* 膨胀判定一致）
from robocup_navigation.astar import load_metadata, GridMap

# ============================ 参数 ============================
MAP_X_MIN, MAP_X_MAX = -100.0, 100.0
MAP_Y_MIN, MAP_Y_MAX = -50.0, 50.0
GRID_SIZE_M = 10.0              # 搜索格边长（与 swarm_task 一致）
CRUISE_SPEED = 3.0              # 拍卖飞行时间估算用巡航速度
ALLOC_PERIOD = 5.0              # 拍卖周期 s（任务完成后重分配）
LEASE_CHECK_PERIOD = 1.0        # 租约到期检查周期 s
TARGET_CHECK_PERIOD = 1.0       # 目标确认计时更新周期 s（规则5 的 15s 按此粒度累计）
DEFAULT_UAV_IDS = ["uav_1", "uav_2"]  # 默认两机

METADATA_PATH = os.environ.get(
    "ROBOCUP_METADATA",
    os.path.join(os.environ.get("ROBOCUP_WS", "/home/ros/team_ws/robocup"),
                 "src/robocup_training_worlds/worlds/generated/training_city_full_s7.json"))
INFLATE_M = 0.5                 # 与 agent 一致：A* 障碍膨胀半径 m


def inflate_grid(grid, inflation_m):
    """对障碍物向外膨胀 inflation_m（圆盘结构元素），返回新 GridMap。

    与 swarm_agent 的 inflate_grid 完全一致，保证 manager 过滤与 agent 规划
    用同一套「障碍」定义，避免 manager 分配了 agent 无法到达的格。
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


class SwarmManager(object):
    def __init__(self, uav_ids):
        self.uav_ids = uav_ids
        # 每机最新状态
        self.status = {}          # uav_id -> UavStatus
        # 每机最近一次上报时间（用于租约续租判定）
        self.last_report = {}     # uav_id -> rospy.Time

        # ---- 纯逻辑模块 ----
        self.grid = CoverageGrid(MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX, GRID_SIZE_M)
        self.allocator = TaskAllocator(self.grid, W_GAIN, W_FLIGHT, W_OVERLAP, W_RISK,
                                       w_distance=W_DISTANCE, cruise_speed=CRUISE_SPEED)
        self.lease = LeaseManager(self.grid, duration=LEASE_DURATION)

        # ---- 过滤「格中心落在建筑内」的格子：这些格 agent 的 A* 无法到达 ----
        self._blocked_cells = self._find_blocked_cells()

        # ---- 目标确认/消除（规则4/5）----
        self.tracker = TargetTracker()
        self._cur_targets = {}    # target_id -> 本周期是否有检测（用于 lose 判定）
        self._eliminated = set()  # 已判定消除的目标 id

        # ---- 任务完成标志 ----
        self._finished = False    # 任务全部完成后置 True，停止分配

        # ---- 订阅 / 发布 ----
        rospy.Subscriber("/swarm/uav_status", UavStatus, self._status_cb)
        rospy.Subscriber("/swarm/detection", TargetDetection, self._detection_cb)
        self.assign_pub = rospy.Publisher("/swarm/assignment", SearchAssignment, queue_size=10)
        # 消除指令发给 target_sim_node（"eliminate:<target_id>"）
        self.cmd_pub = rospy.Publisher("/swarm/target_command", String, queue_size=10)
        # 任务完成广播
        self.finish_pub = rospy.Publisher("/swarm/finish", String, queue_size=10)

        self._last_alloc_t = rospy.Time.now()
        self._last_lease_t = rospy.Time.now()
        self._last_target_t = rospy.Time.now()

    def _find_blocked_cells(self):
        """加载地图并膨胀，返回「格中心在膨胀障碍内」的格 key 集合。

        这些格中心 agent 的 A* 会报 GOAL_OCCUPIED，manager 应跳过不分配。
        """
        blocked = set()
        try:
            md, _ = load_metadata(METADATA_PATH)
            g = inflate_grid(GridMap.from_metadata(md), INFLATE_M)
            for key, c in self.grid.cells.items():
                cell = g.world_to_cell((c.cx, c.cy))
                if cell is None or not g.is_free(cell):
                    blocked.add(key)
        except Exception as exc:
            rospy.logwarn("[manager] 加载地图过滤建筑格失败: %s（将不过滤）", exc)
        rospy.loginfo("[manager] 过滤建筑内格中心：%d/%d 格不可达",
                      len(blocked), len(self.grid.cells))
        return blocked

    # ---------------- 回调 ----------------
    def _status_cb(self, msg):
        self.status[msg.uav_id] = msg
        self.last_report[msg.uav_id] = rospy.Time.now()

        # 覆盖标记：agent 上报置信度足够 → 该格标记已覆盖
        if msg.confidence >= 1.0 and msg.cell_ix >= 0 and msg.cell_iy >= 0:
            key = (msg.cell_ix, msg.cell_iy)
            c = self.grid.cell(key)
            if c is not None and c.state != STATE_COVERED:
                c.state = STATE_COVERED
                c.confidence = 1.0
                c.owner = None
                c.lease_until = 0.0
                rospy.loginfo("[manager] 格 (%d,%d) 已覆盖（%s）", key[0], key[1], msg.uav_id)

    # ---------------- 目标检测与消除（规则4/5） ----------------
    def _detection_cb(self, msg):
        """收到某机对某目标的检测 → 记录本周期该目标「被观测到」。

        真正的计时逻辑在 _update_targets 里统一处理（按周期判定连续/中断），
        避免「多机同时上报时重复计时」。
        """
        if msg.target_id in self._eliminated:
            return
        tid = msg.target_id
        if tid not in self.tracker.targets:
            self.tracker.add_target(tid, msg.x, msg.y,
                                    tracker=msg.uav_id, now=rospy.Time.now().to_sec())
            rospy.loginfo("[manager] 发现新目标 %s @ (%.1f, %.1f)，由 %s 首次检测",
                          tid, msg.x, msg.y, msg.uav_id)
        self._cur_targets[tid] = (msg.x, msg.y, msg.uav_id)

    def _update_targets(self):
        """按周期更新每个目标的确认计时，处理规则4/5。"""
        now = rospy.Time.now().to_sec()

        for tid in list(self.tracker.targets.keys()):
            if tid in self._eliminated:
                continue

            if tid in self._cur_targets:
                # 本周期被观测到 → 累计连续确认
                x, y, uid = self._cur_targets[tid]
                done = self.tracker.observe(tid, now, observer=uid)
                if done:
                    # 规则5：连续 15s 确认 → 判定消除
                    self._eliminated.add(tid)
                    self.cmd_pub.publish(String(data="eliminate:%s" % tid))
                    rospy.loginfo("[manager] 规则5：目标 %s 连续确认 %.0fs → 广播消除",
                                  tid, CONFIRM_TIME)
                    continue
                prog = self.tracker.confirm_progress(tid, now)
                rospy.loginfo_throttle(5, "[manager] 目标 %s 确认进度 %.0f%%",
                                       tid, prog * 100)
            else:
                # 本周期无人观测 → 规则5 要求「连续」，故计时清零
                self.tracker.lose(tid, now)

            # 规则4：被感知累计 30s 未消除 → 目标侧会瞬移，管理器同步重置
            if self.tracker.check_evade(tid, now):
                rospy.loginfo("[manager] 规则4：目标 %s 被感知 %.0fs 未消除 → 判定瞬移，"
                              "重置确认计时重新搜索", tid, EVADE_TIME)
                # 目标实际位置由 target_sim_node 搬移，管理器这里只需清空计时；
                # 等下一帧检测到新位置时会以新坐标重新登记。
                self._cur_targets.pop(tid, None)

        # 清空本周期检测缓存（下一周期重新收集）
        self._cur_targets = {}
        self._last_target_t = rospy.Time.now()

    # ---------------- 拍卖分配 ----------------
    def _idle_uavs(self):
        """返回「空闲机」列表：无活跃任务（没有 owner==自己的 STATE_ASSIGNED 格）。

        这样可避免一台机还在飞往旧格途中就被重复分配新格、旧格无人接管。
        机完成某格（confidence≥1.0 → 格变 STATE_COVERED，owner 清空）后自动变空闲。
        """
        idle = []
        for uid in self.uav_ids:
            if uid not in self.status or not self.status[uid].connected:
                continue
            busy = False
            for c in self.grid.cells.values():
                if c.state == STATE_ASSIGNED and c.owner == uid:
                    busy = True
                    break
            if not busy:
                idle.append(uid)
        return idle

    def _allocate(self):
        """只给空闲机拍卖分配未搜索格 → 授租约 → 发布 assignment。"""
        # 任务已完成，不再分配
        if self._finished:
            return

        now = rospy.Time.now()

        # 到期租约先回退（掉线/卡死的机占用的格释放回候选池）
        expired = self.lease.expire(now.to_sec())
        if expired:
            rospy.loginfo("[manager] 租约到期重分配 %d 格", len(expired))

        # 建筑内格中心标记为 STATE_COVERED（agent 无法到达，视为无需搜索）
        for key in self._blocked_cells:
            c = self.grid.cell(key)
            if c is not None and c.state == STATE_FREE:
                c.state = STATE_COVERED

        # 只对空闲机分配
        idle = self._idle_uavs()
        if not idle:
            return

        uavs = {uid: (self.status[uid].x, self.status[uid].y) for uid in idle}

        # 拍卖（每机取剩余格中自身效用最大者，不重复）
        assign = self.allocator.allocate(uavs)
        for uid, key in assign.items():
            if key is None:
                # 无可用格，不发布任务
                continue
            # 授租约（续期到 now+duration）
            self.lease.grant(uid, key, now.to_sec())
            c = self.grid.cell(key)
            self._publish_assignment(uid, key, c)

        self._last_alloc_t = now

    # ---------------- 续租 ----------------
    def _renew_leases(self):
        """对持续上报的机续租（防误判到期）。"""
        now = rospy.Time.now()
        for uid in self.uav_ids:
            if uid not in self.last_report:
                continue
            # 3 秒内有上报 → 认为仍在执行，续租其当前任务格
            if (now - self.last_report[uid]).to_sec() < 3.0:
                st = self.status.get(uid)
                if st is not None and st.cell_ix >= 0 and st.cell_iy >= 0:
                    self.lease.renew(uid, (st.cell_ix, st.cell_iy), now.to_sec())
        self._last_lease_t = now

    # ---------------- 发布 ----------------
    def _publish_assignment(self, uid, key, cell):
        msg = SearchAssignment()
        msg.header.stamp = rospy.Time.now()
        msg.uav_id = uid
        msg.cell_ix = key[0]
        msg.cell_iy = key[1]
        msg.target_x = cell.cx
        msg.target_y = cell.cy
        msg.task_type = 0  # 搜索
        self.assign_pub.publish(msg)
        rospy.loginfo("[manager] 分配 %s → 格 (%d,%d) 中心 (%.1f,%.1f)",
                      uid, key[0], key[1], cell.cx, cell.cy)

    # ---------------- 主循环 ----------------
    def _check_finished(self):
        """检查是否所有任务完成（搜索格全覆盖 + 目标全消除 + 飞机全降落）。"""
        # 1. 检查是否还有未覆盖的搜索格
        uncovered = self.grid.uncovered_cells()

        # 2. 检查是否还有未消除的目标
        pending_targets = self.tracker.pending_targets()

        # 3. 检查是否所有飞机都已降落
        all_landed = True
        for uid in self.uav_ids:
            if uid not in self.status:
                all_landed = False
                break
            st = self.status[uid]
            # 检查是否在地面（高度 < 1m）或模式为 LAND
            if st.connected and st.z > 1.0:
                all_landed = False
                break

        if not uncovered and not pending_targets and all_landed:
            if not self._finished:
                self._finished = True
                rospy.loginfo("[manager] ========== 全部任务完成，停止分配 ==========")
                # 广播任务完成消息
                self.finish_pub.publish(String(data="MISSION_FINISHED"))
                return True
        return False

    def run(self):
        rate = rospy.Rate(10)
        rospy.loginfo("[manager] 集群管理器启动，机队: %s", self.uav_ids)
        while not rospy.is_shutdown():
            now = rospy.Time.now()

            # 定期续租
            if (now - self._last_lease_t).to_sec() >= LEASE_CHECK_PERIOD:
                self._renew_leases()

            # 目标确认计时（规则4/5）
            if (now - self._last_target_t).to_sec() >= TARGET_CHECK_PERIOD:
                self._update_targets()

            # 检查是否任务完成
            if not self._finished:
                self._check_finished()

            # 定期拍卖（有未覆盖格且距上次分配超周期）
            if (now - self._last_alloc_t).to_sec() >= ALLOC_PERIOD:
                self._allocate()

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("swarm_manager")
    uav_ids = rospy.get_param("~uav_ids", DEFAULT_UAV_IDS)
    if isinstance(uav_ids, str):
        uav_ids = uav_ids.split(",")
    rospy.loginfo("swarm_manager 启动，机队: %s", uav_ids)
    SwarmManager(uav_ids).run()
