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
import json
import rospy
import math
import time
from swarm_task import LineOfSight
import re
import traceback

from robocup_swarm.msg import UavStatus, SearchAssignment, TargetState, TargetDetection
from std_msgs.msg import String, Float32

# 直接 import 同目录的纯逻辑模块（scripts 目录已加进 PYTHONPATH）
from swarm_task import (CoverageGrid, TaskAllocator, LeaseManager,
                        STATE_FREE, STATE_ASSIGNED, STATE_COVERED,
                        W_GAIN, W_FLIGHT, W_OVERLAP, W_RISK, W_BALANCE, W_DISTANCE, W_ZONE, LEASE_DURATION,
                        CONFIRM_TIME, EVADE_TIME, DETECT_RADIUS)
from cooperative_tracker import CooperativeTracker

# 地图（用于过滤「格中心落在建筑内」的格子，与 agent 的 A* 膨胀判定一致）
from robocup_navigation.astar import load_metadata, GridMap

# ============================ 参数 ============================
MAP_X_MIN, MAP_X_MAX = -100.0, 100.0
MAP_Y_MIN, MAP_Y_MAX = -50.0, 50.0
# 2026-09-28：默认必须与 swarm_task.py 一致（那边已是 7.0）。
# 10.0 vs 7.0 会导致 manager 的栅格与 task 的拍卖栅格错位（不传 env 时必踩）。
GRID_SIZE_M = float(os.environ.get("GRID_SIZE_M", "7.0"))   # 搜索格边长（与 swarm_task 必须一致，都读同一个环境变量）
CRUISE_SPEED = 5.0              # 拍卖飞行时间估算用巡航速度（与 agent MAX_SPEED 一致）
ALLOC_PERIOD = 2.0              # 拍卖周期 s（任务完成后重分配）
LEASE_CHECK_PERIOD = 1.0        # 租约到期检查周期 s
TARGET_CHECK_PERIOD = 1.0       # 目标确认计时更新周期 s（规则5 的 15s 按此粒度累计）

# ---- 确认期冗余派机（可全用环境变量关掉/调参，无需改代码）----
BACKUP_ENABLE = int(os.environ.get("BACKUP_ENABLE", "1"))   # 0=关闭
AUTO_LAND = int(os.environ.get("AUTO_LAND", "0"))   # 0=禁止自动降落（默认）
BACKUP_STALE  = float(os.environ.get("BACKUP_STALE",  "3.0"))  # 断流多少秒后加派
BACKUP_AFTER  = float(os.environ.get("BACKUP_AFTER",  "12.0")) # 进入确认期多少秒仍未消除则加派
BACKUP_MAX    = int(os.environ.get("BACKUP_MAX",    "2"))    # 全场同时存在的备份机上限
BACKUP_MAX_DIST = float(os.environ.get("BACKUP_MAX_DIST", "60.0"))  # 距离超过此值就不派
                                                                    # （飞过去的时间比等还久，净亏损）

# ---- 真值播种开关 ----
# 1=把官方 actor 真值直接播种给 tracker（现状，等于"上帝视角"直接派机）
# 0=只能靠 /swarm/detection（飞机自己的观测）发现目标 —— 这才是接 YOLO 后的真实链路
# 关掉它才能量出协同搜索算法的真实能力，否则覆盖栅格/拍卖/分区全是摆设。
SEED_TRUTH = int(os.environ.get("SEED_TRUTH", "1"))
MAP_BOUNDS_FROM_META = os.environ.get("MAP_BOUNDS_FROM_META", "1") not in ("0", "false", "False")
# 覆盖记忆：把各机当前位置探测半径内的格子标记为已覆盖。
# COVER_MARK=0 关闭（复现旧行为）；比例可调（1.0 = 用满 DETECT_RADIUS）
COVER_MARK = float(os.environ.get("COVER_MARK", "1.0"))
COVER_MARK_RATIO = float(os.environ.get("COVER_MARK_RATIO", "1.0"))
REOPEN_MIN_DIST = float(os.environ.get("REOPEN_MIN_DIST", "30.0"))  # 重开时跳过飞机脚下这个半径内的格
DEFAULT_UAV_IDS = ["uav_1", "uav_2"]  # 默认两机

METADATA_PATH = os.environ.get(
    "ROBOCUP_METADATA",
    os.path.join(os.environ.get("ROBOCUP_WS", "/home/ros/team_ws/robocup"),
                 "src/robocup_training_worlds/worlds/generated/training_city_full_s7.json"))
INFLATE_M = 0.5                 # 与 agent 一致：A* 障碍膨胀半径 m

# ---- 2026-09-27：楼边修复（建筑内的 actor 藏身区曾是永久搜索黑洞）----
EDGE_ENABLE   = os.environ.get("EDGE_ENABLE", "1") not in ("0", "false", "False", "")
W_EDGE        = float(os.environ.get("W_EDGE", "0.6"))     # 楼边格效用偏置权重
EDGE_RANGE    = float(os.environ.get("EDGE_RANGE", "12.0"))  # 距障碍多远还算「楼边」 m
EDGE_NEAR     = float(os.environ.get("EDGE_NEAR", "2.0"))    # 距障碍 <= 此值给满分
# 格中心落在建筑里时，去这些半径的环上找可达的替代航点（m）
WAYPOINT_RING = (2.0, 3.5, 5.0)
CLEARANCE_MAX = float(os.environ.get("CLEARANCE_MAX", "24.0"))  # 距离场最大计算范围 m

# ---- LOS 感知 next-best-view（2026-09-28）----
# 与 W_EDGE 的区别：EDGE 是静态的「贴楼加分」（实测成共同吸引子，6 架挤一起）；
# NBV 是动态的「从这个视点能看见多少久未见的区域」，且派单即承诺 → 次模贪心自动分散。
VIS_ENABLE = os.environ.get("VIS_ENABLE", "1") not in ("0", "false", "False", "")
VIS_RADIUS = float(os.environ.get("VIS_RADIUS", "20.0"))   # 与 DETECT_RADIUS 一致
VIS_COMMIT = os.environ.get("VIS_COMMIT", "1") not in ("0", "false", "False", "")


class _ClearanceField(object):
    """到最近障碍格的距离场（4 邻域多源 BFS，保守下界）。

    用途只有一个：把「距离建筑多远」变成一个 0~1 的分数，喂给任务拍卖的
    效用函数，让搜索队形优先贴着楼边铺开 —— actor 的出生点几乎全都
    贴着建筑（实测 6 个出生点到最近建筑的间距只有 0~11m）。
    """

    def __init__(self, g, max_m=CLEARANCE_MAX):
        from collections import deque
        self.w, self.h = g.width, g.height
        self.res = g.resolution
        self.ox, self.oy = g.origin[0], g.origin[1]
        limit = int(max_m / self.res)
        dist = [None] * (self.w * self.h)
        q = deque()
        row = 0
        for y in range(self.h):
            row = y * self.w
            for x in range(self.w):
                if g.cells[row + x] != 0:
                    dist[row + x] = 0
                    q.append((x, y))
        self.dist = dist
        while q:
            cx, cy = q.popleft()
            d = dist[cy * self.w + cx]
            if d >= limit:
                continue
            nd = d + 1
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < self.w and 0 <= ny < self.h:
                    idx = ny * self.w + nx
                    if dist[idx] is None:
                        dist[idx] = nd
                        q.append((nx, ny))

    def clearance(self, point):
        """世界坐标 -> 到最近障碍的距离(m)；不可达/越界返回 None。"""
        ix = int(math.floor((point[0] - self.ox) / self.res))
        iy = int(math.floor((point[1] - self.oy) / self.res))
        if not (0 <= ix < self.w and 0 <= iy < self.h):
            return None
        d = self.dist[iy * self.w + ix]
        return None if d is None else d * self.res



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



def _search_bounds():
    """搜索栅格的边界。

    硬编码的 ±100/±50 与官方 actor 可达范围（control_actor.py 写死的
    x[-50,130] / y[-60,60]）不一致：
      - x>100、y<-50、y>50 这三块 actor 真的会去的地方**永远不会被搜索**；
      - x[-100,-50] 那 5000 m² actor 根本到不了，却每轮都白扫。
    栅格元数据 robocup_base.json 的 bounds 就是按官方 actor 活动范围生成的，
    直接用它。读不到就退回硬编码。
    """
    if not MAP_BOUNDS_FROM_META:
        return MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX
    try:
        with open(METADATA_PATH) as _f:
            b = json.load(_f).get("bounds")
        if b:
            return (float(b["x_min"]), float(b["x_max"]),
                    float(b["y_min"]), float(b["y_max"]))
    except Exception as _exc:
        print("[manager] 读元数据 bounds 失败，用硬编码边界: %s" % _exc)
    return MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX


class SwarmManager(object):
    def __init__(self, uav_ids):
        self.uav_ids = uav_ids
        # 每机最新状态
        self.status = {}          # uav_id -> UavStatus
        # 每机最近一次上报时间（用于租约续租判定）
        self.last_report = {}     # uav_id -> rospy.Time

        # ---- 纯逻辑模块 ----
        _bx0, _bx1, _by0, _by1 = _search_bounds()
        rospy.loginfo("[manager] 搜索栅格边界 x[%.0f,%.0f] y[%.0f,%.0f]（%s）",
                      _bx0, _bx1, _by0, _by1,
                      "元数据" if MAP_BOUNDS_FROM_META else "硬编码")
        self.grid = CoverageGrid(_bx0, _bx1, _by0, _by1, GRID_SIZE_M, len(uav_ids))
        self.allocator = TaskAllocator(self.grid, W_GAIN, W_FLIGHT, W_OVERLAP, W_RISK, W_BALANCE,
                                       w_distance=W_DISTANCE, w_zone=W_ZONE, cruise_speed=CRUISE_SPEED)
        self.lease = LeaseManager(self.grid, duration=LEASE_DURATION)

        # ---- 过滤「格中心落在建筑内」的格子：这些格 agent 的 A* 无法到达 ----
        self._cell_waypoint = {}   # cell_key -> (wx,wy) 楼边格的可达代表点
        self._cell_edge = {}       # cell_key -> 0~1 临楼度
        self._blocked_cells = self._find_blocked_cells()
        # 把「临楼度」交给拍卖器做效用偏置
        try:
            self.allocator.cell_edge = self._cell_edge
        except Exception:
            pass

        # ---- LOS 感知 NBV：预计算每个候选视点的可见集 ----
        self._vis_set = {}
        if VIS_ENABLE:
            try:
                self._vis_set = self._build_visibility()
                self.allocator.set_visibility(self._vis_set)
                _n = [len(v) for v in self._vis_set.values()]
                rospy.loginfo("[manager] NBV 可见集：%d 个视点，平均可见 %.1f 格（最少 %d / 最多 %d）",
                              len(self._vis_set),
                              (sum(_n) / float(len(_n))) if _n else 0.0,
                              min(_n) if _n else 0, max(_n) if _n else 0)
            except Exception as exc:
                rospy.logwarn("[manager] NBV 可见集构建失败: %s（退化为无 NBV）", exc)
                self._vis_set = {}

        # ---- 目标确认/消除（规则4/5）----
        self.tracker = CooperativeTracker()
        self._truth_cache = {}     # target_id -> (x, y) 缓存（CooperativeTracker 不存位置，派追踪机用）
        self._cur_targets = {}    # target_id -> 本周期是否有检测（用于 lose 判定）
        self._eliminated = set()
        # 确认期冗余备份机：tid -> uav_id（见 _dispatch_backup）
        self._backup = {}
        # tid -> 最近一次收到 /swarm/detection 的 wall 时刻（断流检测用）
        self._last_detect = {}
        # tid -> 官方 /find_actor_N 首次发布时刻（= 进入 15s 确认期）
        self._confirm_since = {}
        self._backup_noted = set()  # 已提示过「无近机可派」的 tid（避免刷屏）
        self._tracking = {}       # target_id -> assigned_uav_id 当前追踪任务
        self._mission_finished = False  # 任务已完成（搜索格 + 目标全部结束）

        # ---- 订阅 / 发布 ----
        rospy.Subscriber("/swarm/uav_status", UavStatus, self._status_cb)
        rospy.Subscriber("/swarm/detection", TargetDetection, self._detection_cb)
        self.assign_pub = rospy.Publisher("/swarm/assignment", SearchAssignment, queue_size=10)
        # 消除指令发给 target_sim_node（"eliminate:<target_id>"）
        self.cmd_pub = rospy.Publisher("/swarm/target_command", String, queue_size=10)
        # 任务完成广播
        self.finish_pub = rospy.Publisher("/swarm/finish", String, queue_size=10)

        # 官方裁判的剩余 actor 清单（权威）。
        # 用途：兜住「提前收工」死结 —— manager 的 tracker 只在收到 /swarm/detection 后
        # 才登记目标，于是「一个都还没发现」时 remaining_targets 恒为空，格子一刷完就
        # 广播 MISSION_FINISHED、全体降落，从此再也不可能发现任何目标。
        self._left_actors = []
        self._left_seen = False   # 是否已收到过第一条 /left_actors（防空清单误释放）
        rospy.Subscriber("/left_actors", String, self._left_cb, queue_size=5)
        # 官方 /find_actor_N（Float32）：首次发布 = 该 actor 进入 15s 连续确认期
        for _i in range(6):
            rospy.Subscriber("/find_actor_%d" % _i, Float32,
                             self._make_find_cb(_i), queue_size=5)
        # 官方 actor 真值（official_target_bridge 从 /gazebo/model_states 转发）
        rospy.Subscriber("/swarm/target_states", TargetState, self._truth_cb, queue_size=30)

        self._last_alloc_t = rospy.Time.now()
        self._last_lease_t = rospy.Time.now()
        self._last_target_t = rospy.Time.now()
        self._mission_finished = False  # 任务完成标志

    def _find_blocked_cells(self):
        """加载地图并膨胀，给每格找一个「可达代表航点」，并算临楼度。

        旧实现只用「10m 格中心」判可达：建筑只要压住格中心，整格就被永久
        标记 STATE_COVERED，_reopen_covered_cells() 也跳过它 —— 场上留下
        一批永不被搜索的黑洞。实测 actor_4(-32,-27) / actor_5(-35,-28)
        的出生点正落在 x[-35.4,-31.4] y[-29.4,-13.4] 那栋楼里。

        新做法：中心不可达时按 WAYPOINT_RING 在格内环形找替代可达点，
        只有整格都找不到可达点才真的判 dead。同时算「临楼度」喂给拍卖。
        """
        blocked = set()
        waypoint = {}
        edge = {}
        try:
            if EDGE_ENABLE:
                md, _ = load_metadata(METADATA_PATH)
                g = inflate_grid(GridMap.from_metadata(md), INFLATE_M)
                self._g_infl = g
                clearance = _ClearanceField(g)
                for key, c in self.grid.cells.items():
                    ctr = g.world_to_cell((c.cx, c.cy))
                    wp = (c.cx, c.cy) if (ctr is not None and g.is_free(ctr)) else None
                    if wp is None:
                        for r in WAYPOINT_RING:
                            for a in range(12):
                                th = a * math.pi / 6.0
                                px = c.cx + r * math.cos(th)
                                py = c.cy + r * math.sin(th)
                                pc = g.world_to_cell((px, py))
                                if pc is not None and g.is_free(pc):
                                    # 取离建筑尽量远的候选点（留足旋翼余量）
                                    if clearance.clearance((px, py)) is not None and \
                                            clearance.clearance((px, py)) < 1.0:
                                        continue
                                    wp = (px, py)
                                    break
                            if wp is not None:
                                break
                    if wp is None:
                        blocked.add(key)
                        continue
                    waypoint[key] = wp
                    d = clearance.clearance(wp)
                    if d is None or d >= EDGE_RANGE:
                        edge[key] = 0.0
                    elif d <= EDGE_NEAR:
                        edge[key] = 1.0
                    else:
                        edge[key] = (EDGE_RANGE - d) / (EDGE_RANGE - EDGE_NEAR)
            else:
                md, _ = load_metadata(METADATA_PATH)
                g = inflate_grid(GridMap.from_metadata(md), INFLATE_M)
                self._g_infl = g
                for key, c in self.grid.cells.items():
                    cell = g.world_to_cell((c.cx, c.cy))
                    if cell is None or not g.is_free(cell):
                        blocked.add(key)
        except Exception as exc:
            rospy.logwarn("[manager] 加载地图过滤建筑格失败: %s（将不过滤）", exc)
        self._cell_waypoint = waypoint
        self._cell_edge = edge
        n_alt = 0
        for _k, _wp in waypoint.items():
            _c = self.grid.cell(_k)
            if _c is not None and (abs(_wp[0] - _c.cx) > 1e-6 or abs(_wp[1] - _c.cy) > 1e-6):
                n_alt += 1
        rospy.loginfo("[manager] 楼边修复：%d/%d 格完全不可达；%d 格改用替代航点；"
                      "%d 格临楼(edge>0)",
                      len(blocked), len(self.grid.cells), n_alt,
                      sum(1 for v in edge.values() if v > 0.0))
        return blocked

    # ---------------- 回调 ----------------
    def _status_cb(self, msg):
        self.status[msg.uav_id] = msg

    @staticmethod
    def _tid_to_actor(tid):
        """'t3' -> 3；非 tN 形式返回 None。"""
        m = re.match(r'^t(\d+)$', str(tid))
        return int(m.group(1)) if m else None

    def _make_find_cb(self, actor_id):
        """官方 /find_actor_N 回调 → 对齐 CooperativeTracker 的规则4 计时起点。

        manager 自己的 _confirm_since 字典已被 CooperativeTracker.targets[tid]._first_confirm_t
        取代，后者才是规则4 真正用的计时起点。
        """
        def cb(msg):
            now = rospy.Time.now().to_sec()
            tid = "t%d" % actor_id
            self.tracker.mark_confirmed(tid, now)
            rospy.loginfo("[manager] 官方首次发现 %s → 规则4 计时起点对齐 %.1f", tid, now)
        return cb

    def _release_finished(self, left_ids):
        """官方已确认并从场上删除的 actor → 立刻释放它的追踪机。

        背景（第六轮实测故障）：agent 连续确认满 15s 后只在自己日志里打印
        「目标 t5 已连续确认，可消除」，manager 的 tracker 却没有把它推进
        _eliminated。后果是 _tracking 里 t3/t4/t5（早已被官方删掉的 actor）
        一直占着飞机 —— 而 _idle_uavs() 把「正在追踪」的机视为不空闲，
        这些机永远拿不到新任务，只能对着旧坐标空转；唯一剩下的 actor_1
        （/left_actors="[1]"）反倒没有飞机去追，官方收不到检测，任务卡到超时。

        权威来源就是官方 /left_actors：不在清单里 = 已被官方确认并删除。
        """
        for tid in list(self._tracking.keys()):
            aid = self._tid_to_actor(tid)
            if aid is None or aid in left_ids:
                continue
            uav = self._tracking.pop(tid)
            bu = self._backup.pop(tid, None)
            self._eliminated.add(tid)
            t = self.tracker.targets.get(tid)
            if t is not None:
                t.eliminated = True
            rospy.loginfo("[manager] 官方已消除 %s（不在 left_actors），"
                          "释放 %s 回归搜索%s", tid, uav,
                          ("（含备份机 %s）" % bu) if bu else "")
        # 顺带把 tracker 里同样已消除、但当时没派机的目标也标掉
        for tid in list(self.tracker.targets.keys()):
            aid = self._tid_to_actor(tid)
            if aid is not None and aid not in left_ids:
                self._eliminated.add(tid)

    def _left_cb(self, msg):
        """官方裁判发布的剩余 actor：'[]' 或 '[0, 2, 5]'。"""
        ids = [int(x) for x in re.findall(r'-?\d+', str(msg.data))]
        if ids != self._left_actors:
            rospy.loginfo("[manager] 官方剩余 actor: %s", ids)
        self._left_actors = ids
        # 第一条之前不动作：空清单不等于「全部已消除」
        if not self._left_seen:
            if ids:
                self._left_seen = True
            return
        self._release_finished(ids)

    def _build_visibility(self):
        """预计算「从每个候选视点能看见哪些格」（20m 半径 + LOS 不穿楼）。

        视点取该格的**可达航点**（楼边格的中心可能在建筑里，见 _find_blocked_cells），
        被观察对象取格中心。这一步只在启动时算一次。
        """
        g = getattr(self, "_g_infl", None)
        if g is None:
            return {}
        los = LineOfSight(lambda ix, iy: not g.is_free((ix, iy)),
                          g.resolution, (g.origin[0], g.origin[1]))
        try:
            self.allocator.set_los(los)
        except Exception:
            pass
        cells = self.grid.cells
        keys = [k for k in cells if k not in self._blocked_cells]
        vis = {}
        r2 = VIS_RADIUS * VIS_RADIUS
        for k in keys:
            wp = self._cell_waypoint.get(k) or (cells[k].cx, cells[k].cy)
            out = []
            for k2 in keys:
                c2 = cells[k2]
                dx = c2.cx - wp[0]
                dy = c2.cy - wp[1]
                if dx * dx + dy * dy > r2:
                    continue
                if los.visible(wp[0], wp[1], c2.cx, c2.cy):
                    out.append(k2)
            vis[k] = out
        return vis

    def _reopen_covered_cells(self):
        """所有格都已标记覆盖、但目标还没找全时，把覆盖状态重置重新扫一遍。

        不加这一步的话：日志会说"继续搜索"，但没有未覆盖格可分配，
        飞机实际上拿不到新任务，只能原地待命到超时 —— 等于还是收工。
        """
        # 跳过就在飞机脚下的格：否则拍卖的 W_FLIGHT(距离) 又会把飞机派回原地，
        # 变成"重开=原地打转"（第二轮实测 136 次派位全是同一批格子）。
        near = set()
        for _st in self.status.values():
            if not getattr(_st, "connected", False):
                continue
            for _key, _c in self.grid.cells.items():
                if math.hypot(_c.cx - _st.x, _c.cy - _st.y) < REOPEN_MIN_DIST:
                    near.add(_key)
        n = 0
        for key, c in self.grid.cells.items():
            if key in self._blocked_cells or key in near:
                continue
            if c.state == STATE_COVERED:
                c.state = STATE_FREE
                n += 1
        if n:
            rospy.loginfo("[manager] 已重新开放 %d 个已覆盖格（跳过 %d 个近距格），开始下一轮巡逻",
                          n, len(near))
        return n

    def _truth_cb(self, msg):
        """官方 actor 真值 → 缓存位置 + 给 CooperativeTracker 播种目标 ID。

        CooperativeTracker 不存目标位置（它只管融合和计时），目标真值位置
        缓存在 self._truth_cache 里，派追踪机时用。确认计时仍然只有 UAV 真的
        观测到才累计（规则 5 的连续 15s）。
        """
        tid = str(msg.target_id)
        self._truth_cache[tid] = (msg.x, msg.y)
        if msg.eliminated:
            self._truth_cache.pop(tid, None)
            return
        if tid in self._eliminated:
            return
        if tid not in self.tracker.targets:
            now = rospy.Time.now().to_sec()
            self.tracker.add_target(tid, now)
            rospy.loginfo("[manager] 真值播种目标 %s @ (%.1f, %.1f)", tid, msg.x, msg.y)

    def _get_target_pos(self, tid, now=None):
        """取目标最新位置：优先真值缓存 → 退回 CooperativeTracker 融合估计。"""
        if tid in self._truth_cache:
            return self._truth_cache[tid]
        ct = self.tracker.targets.get(tid)
        if ct is not None:
            fx = ct.fused(now) if now is not None else None
            if fx is not None:
                return fx
        return None, None

    def _dispatch_pending_targets(self):
        """给「已知但未消除」的目标派追踪机（只派一次，已在追的只更新位置）。"""
        for tid in list(self.tracker.targets.keys()):
            if tid in self._eliminated:
                continue
            ct = self.tracker.targets.get(tid)
            if ct is None or ct.eliminated:
                continue
            tx, ty = self._get_target_pos(tid)
            if tx is None:
                continue
            if tid in self._tracking:
                self._update_tracker_position(tid, tx, ty)
                continue
            busy = set(self._tracking.values()) | set(self._backup.values())
            best, best_d = None, None
            for uid, st in self.status.items():
                if not getattr(st, "connected", False) or uid in busy:
                    continue
                d = math.hypot(st.x - tx, st.y - ty)
                if best_d is None or d < best_d:
                    best, best_d = uid, d
            if best is None:
                continue
            self._tracking[tid] = best
            # CooperativeTracker 的协同关键：派出去的追踪机必须登记为 observer，
            # 否则它看到的观测会被忽略（规则5 需要多机接力 + 误差/间隔判定）。
            existing = list(ct.observers)
            if best not in existing:
                self.tracker.assign_observers(tid, existing + [best])
            # actor 真值可能落在栅格外（官方 actor 可达 y[-60,60]，栅格可能更小）
            # 直接派出去会让 A* 报 GOAL_OUT_OF_BOUNDS 原地打转，先 clamp 到栅格内
            tx_c = min(max(tx, self.grid.x_min + 0.5), self.grid.x_max - 0.5)
            ty_c = min(max(ty, self.grid.y_min + 0.5), self.grid.y_max - 0.5)
            msg = SearchAssignment()
            msg.header.stamp = rospy.Time.now()
            msg.uav_id = best
            msg.cell_ix = -1
            msg.cell_iy = -1
            msg.target_x = tx_c
            msg.target_y = ty_c
            if hasattr(msg, "target_id"):
                msg.target_id = tid
            msg.task_type = 1  # 目标确认/追踪
            self.assign_pub.publish(msg)
            rospy.loginfo("[manager] 派追踪：%s → 目标 %s @ (%.1f, %.1f), 距离 %.1fm",
                          best, tid, tx_c, ty_c, best_d)

    # ---------------- 目标检测与消除（规则4/5） ----------------
    def _detection_cb(self, msg):
        """收到某机对某目标的检测 → 喂给 CooperativeTracker。

        CooperativeTracker 内部维护多机融合 + 三门槛（误差 1m / 间隔 1s / 连续 15s）
        计时，_update_targets 周期性调 update() 统一处理规则 4/5 事件。
        """
        if msg.target_id in self._eliminated:
            return
        tid = msg.target_id
        now = rospy.Time.now().to_sec()
        self._last_detect[tid] = now
        self._truth_cache[tid] = (msg.x, msg.y)

        # CooperativeTracker 还不知道这个目标？登记
        if tid not in self.tracker.targets:
            self.tracker.add_target(tid, now)
            rospy.loginfo("[manager] 发现新目标 %s @ (%.1f, %.1f)，由 %s 首次检测",
                          tid, msg.x, msg.y, msg.uav_id)

        ct = self.tracker.targets[tid]
        # 把检测的 UAV 登记为 observer，否则 CooperativeTracker 会忽略它的观测
        if msg.uav_id not in ct.observers:
            self.tracker.assign_observers(tid, list(ct.observers) + [msg.uav_id])

        # 喂观测 → CooperativeTracker 内部做 alpha-beta 融合 + 三门槛判定
        tx_truth, ty_truth = self._truth_cache.get(tid, (None, None))
        truth = (tx_truth, ty_truth) if tx_truth is not None else None
        self.tracker.report(msg.uav_id, tid, now, msg.x, msg.y, truth=truth)
        self._cur_targets[tid] = (msg.x, msg.y, msg.uav_id)

        # 已在追踪？更新盘旋位置
        if tid in self._tracking:
            self._update_tracker_position(tid, msg.x, msg.y)
        else:
            self._dispatch_tracker(msg.target_id, msg.x, msg.y)

    def _dispatch_tracker(self, target_id, tx, ty):
        """派遣最近空闲机去追踪目标（盘旋确认）

        如果没有空闲机，则中断一台正在搜索的飞机（避免目标因无人确认而瞬移）。
        """
        # P1 修复：检查目标是否已被追踪
        if target_id in self._tracking:
            existing_uav = self._tracking[target_id]
            rospy.loginfo("[manager] 目标 %s 已被 %s 追踪，跳过派遣", target_id, existing_uav)
            return

        idle = self._idle_uavs()

        # 优先用空闲机
        best_uav = None
        best_dist = float('inf')
        for uid in idle:
            if uid not in self.status:
                continue
            ux, uy = self.status[uid].x, self.status[uid].y
            dist = math.hypot(tx - ux, ty - uy)
            if dist < best_dist:
                best_dist = dist
                best_uav = uid

        # 没有空闲机？中断正在搜索的飞机
        if best_uav is None:
            rospy.loginfo("[manager] 无空闲机，尝试中断搜索机去追踪 %s", target_id)
            # 找正在执行搜索任务的飞机，优先选预计到达时间 ≤ 15s 的
            candidate_uavs = []
            for uid in self.uav_ids:
                if uid not in self.status or not self.status[uid].connected:
                    continue
                # 跳过已在追踪的
                if uid in self._tracking.values():
                    continue
                # 找有搜索任务的
                has_search_task = False
                for c in self.grid.cells.values():
                    if c.state == STATE_ASSIGNED and c.owner == uid:
                        has_search_task = True
                        break
                if not has_search_task:
                    continue
                # 计算预计到达时间
                ux, uy = self.status[uid].x, self.status[uid].y
                dist = math.hypot(tx - ux, ty - uy)
                flight_time = dist / CRUISE_SPEED  # 用 CRUISE_SPEED 估算
                candidate_uavs.append((uid, dist, flight_time))

            if candidate_uavs:
                # 按飞行时间排序，选最快的（15s 内能到的优先）
                candidate_uavs.sort(key=lambda x: x[2])
                best_uav, best_dist, flight_time = candidate_uavs[0]
                rospy.loginfo("[manager] 中断 %s 的搜索任务去追踪 %s（预计 %.1fs）",
                              best_uav, target_id, flight_time)
                # 释放该机的搜索格租约
                for key, c in self.grid.cells.items():
                    if c.state == STATE_ASSIGNED and c.owner == best_uav:
                        c.state = STATE_FREE
                        c.owner = None
                        c.lease_until = 0.0
            else:
                rospy.loginfo("[manager] 无可中断的搜索机，无法派遣追踪 %s", target_id)
                return

        if best_uav is None:
            return

        # 发布追踪任务（task_type=1）
        msg = SearchAssignment()
        msg.header.stamp = rospy.Time.now()
        msg.uav_id = best_uav
        msg.cell_ix = -1
        msg.cell_iy = -1
        msg.target_x = tx
        msg.target_y = ty
        msg.task_type = 1  # 目标确认/追踪
        self.assign_pub.publish(msg)
        # 记录追踪任务
        self._tracking[target_id] = best_uav
        rospy.loginfo("[manager] 派遣 %s 追踪目标 %s @ (%.1f, %.1f), 距离 %.1fm",
                      best_uav, target_id, tx, ty, best_dist)

    def _update_tracker_position(self, target_id, tx, ty):
        """更新追踪机的目标位置"""
        if target_id not in self._tracking:
            return
        uid = self._tracking[target_id]
        # 发布更新后的追踪任务
        msg = SearchAssignment()
        msg.header.stamp = rospy.Time.now()
        msg.uav_id = uid
        msg.cell_ix = -1
        msg.cell_iy = -1
        msg.target_x = tx
        msg.target_y = ty
        msg.task_type = 1  # 目标确认/追踪
        self.assign_pub.publish(msg)
        # 备份机同步更新目标位置，否则它会飞向旧坐标
        bid = self._backup.get(target_id)
        if bid is not None:
            msg2 = SearchAssignment()
            msg2.header.stamp = rospy.Time.now()
            msg2.uav_id = bid
            msg2.cell_ix = -1
            msg2.cell_iy = -1
            msg2.target_x = tx
            msg2.target_y = ty
            msg2.task_type = 1
            self.assign_pub.publish(msg2)

    def _dispatch_backup(self):
        """确认期冗余派机：用 CooperativeTracker.needs_backup() 定向增派。

        CooperativeTracker 自己维护了两个精确判据（替换掉旧版 manager 自己算的 stale/stuck）：
          (a) resets >= reset_thresh 次：被裁判多次重置，单机扛不住；
          (b) confirm_since 停滞 >= stall_thresh 秒：进度条卡住。

        两机只要有一架保持可见，上报链就不中断 —— 这正是「协同追踪」真正要落地的地方。
        """
        if not BACKUP_ENABLE:
            return
        now = rospy.Time.now().to_sec()

        needs = self.tracker.needs_backup(now,
                                          reset_thresh=2,
                                          stall_thresh=BACKUP_AFTER)
        if not needs:
            return

        busy = set(self._tracking.values()) | set(self._backup.values())

        for tid, reason in needs:
            if tid in self._backup:
                continue
            aid = self._tid_to_actor(tid)
            if self._left_seen and aid is not None and aid not in self._left_actors:
                continue
            ct = self.tracker.targets.get(tid)
            if ct is None or ct.eliminated:
                continue
            if len(self._backup) >= BACKUP_MAX:
                continue
            tx, ty = self._get_target_pos(tid, now)
            if tx is None:
                continue

            best, best_d = None, None
            for uid, st in self.status.items():
                if not getattr(st, "connected", False) or uid in busy:
                    continue
                d = math.hypot(st.x - tx, st.y - ty)
                if best_d is None or d < best_d:
                    best, best_d = uid, d
            if best is None:
                continue
            if best_d > BACKUP_MAX_DIST:
                if tid not in self._backup_noted:
                    self._backup_noted.add(tid)
                    rospy.loginfo("[manager] %s 确认受阻（%s），但最近空闲机 %.1fm "
                                  "> BACKUP_MAX_DIST=%.0fm，暂不冗余派机",
                                  tid, reason, best_d, BACKUP_MAX_DIST)
                continue

            # 关键：backup 机也必须登记为 observer，否则 CooperativeTracker 会忽略它的观测
            ct.observers.add(best)
            self._backup[tid] = best
            busy.add(best)
            tx_c = min(max(tx, self.grid.x_min + 0.5), self.grid.x_max - 0.5)
            ty_c = min(max(ty, self.grid.y_min + 0.5), self.grid.y_max - 0.5)
            msg = SearchAssignment()
            msg.header.stamp = rospy.Time.now()
            msg.uav_id = best
            msg.cell_ix = -1
            msg.cell_iy = -1
            msg.target_x = tx_c
            msg.target_y = ty_c
            msg.task_type = 1
            if hasattr(msg, "target_id"):
                msg.target_id = tid
            self.assign_pub.publish(msg)
            rospy.loginfo("[manager] 冗余派机：%s 协同确认 %s @ (%.1f,%.1f), "
                          "距离 %.1fm, 原因=%s, observers=%s",
                          best, tid, tx_c, ty_c, best_d, reason,
                          ",".join(sorted(ct.observers)))

    def _update_targets(self):
        """按周期更新每个目标的确认计时，处理规则4/5。"""
        now = rospy.Time.now().to_sec()
        # 先给「已知但未消除」的目标派追踪机（真值播种后才有这一步）
        self._dispatch_pending_targets()
        # 确认期断流/卡住 → 加派第二架协同确认
        self._dispatch_backup()

        # CooperativeTracker.update(now) 内部处理规则 4/5 全部逻辑：
        # 三门槛判定（误差 1m / 间隔 1s / 连续 15s）、confirm_since 重置、
        # 墙钟 25s 瞬移、B1/B2/B3/B4 全修齐。返回 tid → event 字典。
        events = self.tracker.update(now)

        for tid in list(self.tracker.targets.keys()):
            if tid in self._eliminated:
                continue
            ct = self.tracker.targets.get(tid)
            if ct is None:
                continue
            ev = events.get(tid)

            if ev == "confirmed":
                self._eliminated.add(tid)
                aid = self._tid_to_actor(tid)
                if aid is None or (self._left_seen and aid not in self._left_actors):
                    self.cmd_pub.publish(String(data="eliminate:%s" % tid))
                else:
                    rospy.loginfo("[manager] 规则5：%s 团队侧已确认，"
                                  "但官方仍未消除 → 保留在场继续上报", tid)
                rospy.loginfo("[manager] 规则5：目标 %s 连续确认 %.0fs → 广播消除",
                              tid, CONFIRM_TIME)
                self._tracking.pop(tid, None)
                self._backup.pop(tid, None)
                self._truth_cache.pop(tid, None)
                continue
            if ev == "evade":
                rospy.loginfo("[manager] 规则4：目标 %s 首次确认后墙钟 %.0fs 未消除 → 瞬移，"
                              "relocate 计时重置", tid, EVADE_TIME)
                self._cur_targets.pop(tid, None)
                continue
            if ev == "reset":
                rospy.loginfo_throttle(3,
                    "[manager] 目标 %s 规则5 被重置（误差或间隔超限）→ confirm_since 清零", tid)

            prog = ct.progress(now)
            if prog > 0:
                cs = ct.confirm_since
                cs_elapsed = (now - cs) if cs is not None else 0.0
                rospy.loginfo_throttle(5,
                    "[manager] 目标 %s 确认进度 %.0f%% "
                    "(confirm_since=%.1fs, resets=%d, rejects=%d, observers=%s)",
                    tid, prog * 100, cs_elapsed, ct.resets, ct.rejects,
                    ",".join(sorted(ct.observers)))

        # 清空本周期检测缓存（下一周期重新收集）
        self._cur_targets = {}
        self._last_target_t = rospy.Time.now()

    # ---------------- 拍卖分配 ----------------
    def _idle_uavs(self):
        """返回「空闲机」列表：无活跃任务（没有 owner==自己的 STATE_ASSIGNED 格，且不在追踪目标）。

        这样可避免一台机还在飞往旧格途中就被重复分配新格、旧格无人接管。
        机完成某格（confidence≥1.0 → 格变 STATE_COVERED，owner 清空）后自动变空闲。
        正在追踪目标的机也不应被分配搜索格。
        """
        idle = []
        for uid in self.uav_ids:
            if uid not in self.status or not self.status[uid].connected:
                continue
            # 如果正在追踪目标，视为不空闲
            if uid in self._tracking.values() or uid in self._backup.values():
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

        # 用**所有**在飞飞机的实时位置刷新「真正看见过」的区域（不只空闲机）
        if VIS_ENABLE:
            try:
                _tn = time.time()
                for _uid, _st in self.status.items():
                    if not getattr(_st, "connected", False):
                        continue
                    self.allocator.mark_seen(_st.x, _st.y, _tn)
            except Exception:
                pass

        # 拍卖（每机取剩余格中自身效用最大者，不重复）
        assign = self.allocator.allocate(uavs)
        all_assigned = True
        for uid, key in assign.items():
            if key is None:
                # 无可用格，不发布任务
                all_assigned = False
                continue
            # 授租约（自适应时长：飞行时间 × 1.5 + 10s 缓冲）
            uav_pos = uavs[uid]
            cell = self.grid.cell(key)
            _wp0 = self._cell_waypoint.get(key) or (cell.cx, cell.cy)
            dist = math.hypot(_wp0[0] - uav_pos[0], _wp0[1] - uav_pos[1])
            duration = max(10.0, dist / 3.0 * 1.5 + 10.0)  # 3.0 m/s 巡航速度（30→10：30s 下限是吞吐瓶颈，
                                                   #   600s 每机最多 20 格，实测只扫到 44/247）
            self.lease.grant(uid, key, now.to_sec(), duration)
            rospy.loginfo("[manager] 分配 %s → 格 (%d,%d) 飞行距离 %.1fm 租约 %.1fs",
                          uid, key[0], key[1], dist, duration)
            self._publish_assignment(uid, key, cell)

        # === 终局行为 ===
        if not all_assigned:
            # 没有更多搜索格可用，检查是否有未消除的目标
            remaining_targets = [t for t in self.tracker.targets if t not in self._eliminated]
            n_left = len(self._left_actors)
            if n_left > 0 and not remaining_targets:
                # tracker 空 + 裁判说还有 actor = 一个都没搜到，绝不能收工
                rospy.loginfo_throttle(
                    15, "[manager] tracker 无目标，但官方裁判还剩 %d 个 actor %s "
                        "—— 判定为未发现，继续巡逻", n_left, self._left_actors)
                remaining_targets = ['official_left:%d' % n_left]
            if remaining_targets:
                # 有未消除目标：继续搜索，不要 RTL
                # 原因：20m 半径覆盖可能有盲区，目标可能未被"偶遇"
                rospy.loginfo("[manager] 搜索格已覆盖完毕，剩余目标 %d 个，继续搜索",
                              len(remaining_targets))
                # 不返回，继续分配搜索任务（让飞机继续巡逻）
                # 下次分配会重新扫描所有未覆盖格子 —— 但此时已经「没有未覆盖格」了，
                # 所以必须显式把覆盖状态重置，否则飞机拿不到任务只能待命。
                # 只有在没有追踪任务时才重开：否则「重开→拍卖又选最近的脚下那格」
                # 会让 6 架飞机原地打转（实测 136 次派位全是同一批格子）
                # 以前只在"没有追踪任务"时才重开，怕重开后飞机又回原地打转。
                # 现在重开会跳过脚下的格（见 _reopen_covered_cells），可以无条件重开 ——
                # 否则只要有一架在追踪（无播种时一发现目标就是这样），格子又都覆盖了，
                # 剩下的机就彻底没任务可做（实测 32 次派位后停摆到 600s 超时）。
                self._reopen_covered_cells()
            else:
                # 全部完成，发布降落
                # 2026-09-28：实测这里误判过 —— tracker 瞬间为空 + _left_actors 还没到
                # 就被当成「搜完了」，把空闲机降下来，一架落地就再也救不回来（见
                # patch_guard2）。默认不再自动降落，宁可继续巡逻。
                if not AUTO_LAND:
                    rospy.loginfo_throttle(
                        20, "[manager] 疑似全部完成（tracker 空且剩余 0），但 AUTO_LAND=0"
                            " → 继续巡逻，不降落")
                else:
                    rospy.loginfo("[manager] 全部任务完成，各机降落")
                    self._publish_land(idle)
                # 广播任务完成消息
                self.finish_pub.publish(String(data="MISSION_FINISHED"))

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
        _wp = self._cell_waypoint.get(key)
        if _wp is None:
            _wp = (cell.cx, cell.cy)
        msg.target_x = _wp[0]
        msg.target_y = _wp[1]
        msg.task_type = 0  # 搜索
        self.assign_pub.publish(msg)
        rospy.loginfo("[manager] 分配 %s → 格 (%d,%d) 中心 (%.1f,%.1f)",
                      uid, key[0], key[1], cell.cx, cell.cy)

    def _publish_rtl(self, uavs):
        """发布 RTL 返航任务（发给所有连接的飞机，不只是 idle 的）

        注意：有剩余目标时不设置 _mission_finished，因为还要继续跟踪目标确认计时
        """
        # 不设置 _mission_finished = True，因为还有剩余目标需要追踪
        # 发给所有连接的飞机，不只是 idle 的
        rtl_uavs = [uid for uid in self.uav_ids if uid in self.status and self.status[uid].connected]
        for uid in rtl_uavs:
            msg = SearchAssignment()
            msg.header.stamp = rospy.Time.now()
            msg.uav_id = uid
            msg.cell_ix = -1
            msg.cell_iy = -1
            msg.target_x = 0.0
            msg.target_y = 0.0
            msg.task_type = 2  # RTL
            self.assign_pub.publish(msg)
        rospy.loginfo("[manager] RTL 返航: %s", rtl_uavs)

    def _publish_land(self, uavs):
        """发布降落任务（发给所有连接的飞机，不只是 idle 的）

        全部任务完成，设置 _mission_finished = True
        """
        self._mission_finished = True
        # 发给所有连接的飞机，不只是 idle 的
        land_uavs = [uid for uid in self.uav_ids if uid in self.status and self.status[uid].connected]
        for uid in land_uavs:
            msg = SearchAssignment()
            msg.header.stamp = rospy.Time.now()
            msg.uav_id = uid
            msg.cell_ix = -1
            msg.cell_iy = -1
            msg.target_x = 0.0
            msg.target_y = 0.0
            msg.task_type = 3  # 降落
            self.assign_pub.publish(msg)
        rospy.loginfo("[manager] 降落: %s", uavs)

    # ---------------- 主循环 ----------------
    def run(self):
        rate = rospy.Rate(10)
        rospy.loginfo("[manager] 集群管理器启动，机队: %s", self.uav_ids)
        while not rospy.is_shutdown():
            now = rospy.Time.now()

            try:
                # 任务已完成，跳过分配
                if self._mission_finished:
                    rate.sleep()
                    continue

                # 定期续租
                if (now - self._last_lease_t).to_sec() >= LEASE_CHECK_PERIOD:
                    self._renew_leases()

                # 目标确认计时（规则4/5）
                if (now - self._last_target_t).to_sec() >= TARGET_CHECK_PERIOD:
                    self._update_targets()

                # 定期拍卖（有未覆盖格且距上次分配超周期）
                if (now - self._last_alloc_t).to_sec() >= ALLOC_PERIOD:
                    self._allocate()
            except Exception as e:
                rospy.logerr("[manager] 主循环异常: %s\n%s", e, traceback.format_exc())

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("swarm_manager")
    uav_ids = rospy.get_param("~uav_ids", DEFAULT_UAV_IDS)
    if isinstance(uav_ids, str):
        uav_ids = uav_ids.split(",")
    rospy.loginfo("swarm_manager 启动，机队: %s", uav_ids)
    SwarmManager(uav_ids).run()
