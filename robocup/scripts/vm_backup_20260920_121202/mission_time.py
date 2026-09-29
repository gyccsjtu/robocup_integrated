#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""整体任务时间仿真：6 架 UAV 清完 6 个恐怖分子需要多久。

对比三种编组策略
----------------
  fixed3  —— 每目标恒定 3 架 tracker（保守）
  fixed2  —— 每目标恒定 2 架 tracker（激进）
  adaptive—— 从 2 架起步，**确认进度停滞就加第 3 架**（动态加机）

（旧版这里有 fixed3 = 2T+1B 的 Blocker 编组，已删除：离线 n=200 实测
  3 tracker 67.5% 优于 2T+1B 60.5%，Blocker 路线放弃。）

规则约束（均已编码）
--------------------
  规则2  未感知时目标 1 m/s 随机游走
  规则3  被感知后 2 m/s 逃跑，方向 = 远离最近无人机
  规则4  被感知累计 30 s 未消除 → 瞬移躲藏（需**重新搜索**）
  规则5  连续 15 s 确认 → 消除

测量口径（用户确认）
--------------------
  * 时间从**无人机开始起飞**计起，含搜索阶段
  * 目标瞬移后**需要重新搜索**，搜索时间计入总时长

简化说明（会明确标注）
----------------------
  * 无人机速度按 SITL 实测的有效值（6 m/s 下发 → 约 4.7 m/s 实际）
  * 忽略机间碰撞规避耗时（那是协同核心的职责，不在本仿真范围）
"""

import argparse
import copy
import math
import os
import random
import sys

WS = os.environ.get("ROBOCUP_WORKSPACE",
                    os.environ.get("ROBOCUP_WS", os.path.expanduser("~/team_ws/robocup")))
sys.path.insert(0, os.path.join(WS, "scripts", "vm"))
sys.path.insert(0, os.path.join(WS, "src", "robocup_swarm", "scripts"))

from strategy_compare import (MapGrid, METADATA, pick_flee_dir,  # noqa: E402
                              HIDE_IDEAL, TARGET_WALK, TARGET_FLEE, SENSE_R, DT)
from cooperative_tracker import CooperativeTracker  # noqa: E402
from observer_assign import ObserverAssigner  # noqa: E402

# SITL 实测：下发 6 m/s 时实际均速约 4.7（转弯掉速），搜索巡航用 5 m/s
UAV_SEARCH_SPEED = 5.0
UAV_TRACK_SPEED = 4.7
ALT = 6.0
GRID_M = 10.0
TRACK_STANDOFF = 8.0     # Tracker 与目标保持的观察距离
TRACK_SPLIT_DEG = 40.0   # 多架 tracker 绕目标的角向分离（防叠在一起）

# ---- 编组内空间管理（消融开关；默认全关，保证既有结论不被悄悄改动）----
#
# 三条都是**轻量软约束**，不引入固定槽位。理由见 tracker_blocker.py 的消融表：
# 固定 slot（0°/120°/240°）已被实测否掉两次 —— 几架从同一走廊接近时，绝对
# 槽位会让它们塌缩到同一侧，达成率 60.5% → 35.0%。所以这里只在"已经挤了"
# 的时候才出手，正常情况下完全不动，保住那条被验证过的行为。
#
# friend_safe: 友机排斥触发距离（m）。0 = 关闭。
#   速度层叠加 F_rep = k·(d_safe − d)，方向沿两机连线远离友机，各推自己一半。
# angle_dedup: 角度去重阈值（度）。0 = 关闭。
#   两架相对目标的方位差小于此值就把它们**反向**推开各一半。与固定 slot 的
#   区别：它作用在**实际方位**上，方位本来就散开时完全不动。
# slot_mode: "chase" = 现行追尾 + 固定 fan；"dedup" = 追尾 + 角度去重（推荐）；
#            "abs" = 固定槽位（仅用于复现那次否证，别当优化用）。
FRIEND_SAFE = 0.0
FRIEND_K = 3.0
FRIEND_MAX = 2.0         # 排斥合力的限幅（m/s），防止近距时推力爆炸
ANGLE_DEDUP_DEG = 0.0
SLOT_MODE = "chase"

# 编组站位跟踪日志（抓"同目标两机重合"现场用）。None = 关闭。
# 只对 FORM_TRACE_UAVS 里的机、且 t 在 [FORM_TRACE_T0, FORM_TRACE_T1] 内打印，
# 否则一局几千帧会把日志淹掉。
FORM_TRACE_UAVS = None   # 例：{"uav_4", "uav_5"}
FORM_TRACE_T0 = 0.0
FORM_TRACE_T1 = 1e9


def _form_log(msg):
    print(msg, flush=True)

# 动态加机：进度停滞多久就加派
STALL_ADD_S = 5.0
MAX_OBS_PER_TARGET = 3


class Sim(object):
    def __init__(self, g, n_uavs=6, n_targets=6, hide=HIDE_IDEAL, seed=0,
                 policy="adaptive"):
        self.g = g
        self.rng = random.Random(seed)
        self.hide = hide
        self.policy = policy
        self.assigner = ObserverAssigner(g)

        self.uavs = {}
        self.tgt = {}          # target_id -> dict(x, y, vx, vy, known, ...)
        self.tr = CooperativeTracker()
        self.t = 0.0
        self.events = []
        self.done = []
        self._search_paths = {}   # uav_id -> 蛇形覆盖航点
        self._search_wp = {}      # uav_id -> 当前航点索引

        self._spawn(n_uavs, n_targets)
        # 预生成各机的搜索路径（纵带蛇形覆盖，天然分工）
        for i in range(n_uavs):
            u = "uav_%d" % (i + 1)
            self._search_paths[u] = self._build_search_path(i, n_uavs)
            self._search_wp[u] = 0

    # ---------------- 初始化 ----------------
    def _spawn(self, n_uavs, n_targets):
        md_ok = False
        # 无人机：沿场地北侧排开（与 fleet.yaml 类似）
        for i in range(n_uavs):
            for _ in range(200):
                x = -18.0 + (i - (n_uavs - 1) / 2.0) * 8.0
                y = 45.0
                if self.g.free(x, y):
                    self.uavs["uav_%d" % (i + 1)] = [x, y]
                    md_ok = True
                    break
        # 目标：随机散落
        made = 0
        while made < n_targets:
            for _ in range(500):
                x = self.rng.uniform(-88, 88)
                y = self.rng.uniform(-40, 40)
                if not self.g.free(x, y):
                    continue
                if self.g.open_space_score(x, y) < 0.5:
                    continue
                tid = "t%d" % made
                self.tgt[tid] = dict(x=x, y=y, vx=0.0, vy=0.0,
                                     heading=self.rng.uniform(0, 2 * math.pi),
                                     known=False, observers=[], stall=0.0,
                                     last_prog=0.0)
                made += 1
                break
            else:
                break
        if not md_ok:
            raise RuntimeError("UAV 初始化失败")

    # ---------------- 工具 ----------------
    def _nearest_uav(self, x, y, only=None):
        pool = {u: p for u, p in self.uavs.items()
                if only is None or u in only}
        if not pool:
            pool = self.uavs
        best, bd = None, float("inf")
        for u, (ux, uy) in pool.items():
            d = math.hypot(ux - x, uy - y)
            if d < bd:
                bd, best = d, u
        return best, bd

    def _visible(self, ux, uy, tx, ty):
        return (math.hypot(tx - ux, ty - uy) < SENSE_R
                and self.g.visible(ux, uy, tx, ty))

    # ---------------- 搜索阶段 ----------------
    def _build_search_path(self, idx, n):
        """为第 idx 架生成所在纵带的蛇形覆盖路径（预生成航点列表）。

        比"每帧找最近的格"可靠：后者会让多架挤向同一片区域（实测 60 s 只
        发现 1 个目标）。蛇形路径保证整条带被扫过，且各机天然分工。
        """
        span = 190.0 / n
        x_lo = -95.0 + idx * span
        x_hi = min(x_lo + span, 95.0)
        x_mid = (x_lo + x_hi) / 2.0
        pts = []
        # 从上到下蛇形：每层两个端点交替
        rows = 11
        for r in range(rows):
            y = 44.0 - r * (88.0 / (rows - 1))
            xs = (x_lo + 2.0, x_hi - 2.0) if r % 2 == 0 else (x_hi - 2.0, x_lo + 2.0)
            for x in xs:
                # 若端点落在障碍里，改用带中心
                px, py = (x, y) if self.g.free(x, y) else (x_mid, y)
                if self.g.free(px, py):
                    pts.append((px, py))
        return pts

    def _search_step(self, free_uavs):
        """空闲机沿各自纵带的蛇形路径覆盖搜索。"""
        n = max(1, len(self.uavs))
        for u in free_uavs:
            idx = int(u.split("_")[-1]) - 1
            path = self._search_paths.get(u)
            if not path:
                continue
            wp_i = self._search_wp.get(u, 0)
            # 跳过已到达的航点
            ux, uy = self.uavs[u]
            while wp_i < len(path) - 1 and math.hypot(
                    path[wp_i][0] - ux, path[wp_i][1] - uy) < 3.0:
                wp_i += 1
            self._search_wp[u] = wp_i
            # 到达路径末端后从头再来（覆盖可能因目标瞬移而变化）
            tx, ty = path[wp_i]
            ax, ay, _ = self.g.speed_toward(ux, uy, tx, ty, UAV_SEARCH_SPEED)
            self.uavs[u] = [ax, ay]
            ux, uy = ax, ay
            # 搜索途中若看到未发现的目标 → 标记发现
            for tid, tg in self.tgt.items():
                if tid in self.done or tg["known"]:
                    continue
                if self._visible(ux, uy, tg["x"], tg["y"]):
                    tg["known"] = True
                    self.tr.add_target(tid, now=self.t)
                    self.events.append((self.t, "discover", tid))

    # ---------------- 编组决策 ----------------
    def _comp_for(self, tid):
        """该目标需要的 **tracker 架数**。

        Blocker 路线已放弃 —— 离线 n=200 实测（tracker_blocker.py）：
            3 trackers          67.5% (135/200)  同时观测 2.09
            2 tracker + blocker  60.5% (121/200)  同时观测 1.41
            1 tracker + blocker  41.5%           同时观测 0.70
        且消融显示「让 Blocker 抢最近机」与「Tracker 站观测侧」两个"改进"
        都是负收益（60.5% → 62.0% / 35.0%）。所以 3 架全部做 tracker：
        多一个观测点比多一个拦截者更值。

        （旧版这里写的是「2T+1B 93.3% 优于 3T 66.7%」，那个数字出自有缺陷的
        旧脚本：它的堵路分支从未真正生效，实际测的是两个观测等价的配置。）
        """
        if self.policy == "fixed3":
            return 3               # 3 架：全 tracker
        if self.policy == "fixed2":
            return 2               # 2 架
        # adaptive：起步 2 架；确认进度停滞则加第 3 架（上限 3）
        tg = self.tgt[tid]
        n = tg.get("team_size", 2)
        if tg["stall"] >= STALL_ADD_S and n < 3:
            tg["stall"] = 0.0
            n += 1
        tg["team_size"] = n
        return n

    def _assign(self, free_uavs):
        """给所有已发现且未消除的目标分配编组，返回 {tid: [(uav, role), ...]}。

        **稳定性约束**（重要）：每帧无脑重算会导致无人机被反复改派 ——
        实测 t0 的观察员在 (uav_1,uav_3) → (uav_1,uav_2) → (uav_3,uav_1) 之间
        来回跳，无人机永远在飞过去的路上，没有任何一架稳定观测。这会同时
        拖垮确认进度和搜索覆盖（有目标 90 s 都没被发现）。

        做法：只要现有编组里仍有**在位的**（已到达且当前有 LOS）成员，就保持
        不动；只有成员丢失/失效时才重新编组。
        """
        pending = [tid for tid in self.tr.pending() if self.tgt[tid]["known"]]
        teams = {}
        locked = set()          # 已被稳定编组占用的 UAV

        # ---- 第一轮：保留仍然有效的既有编组 ----
        for tid in pending:
            tg = self.tgt[tid]
            roles = tg.get("roles", {})
            if not roles:
                continue
            keep = {}
            for u, r in roles.items():
                if u in locked:
                    continue
                ux, uy = self.uavs[u]
                d = math.hypot(tg["x"] - ux, tg["y"] - uy)
                if d > SENSE_R * 1.5:
                    continue                     # 掉队太远 → 需要重新编组
                if r == "tracker":
                    # tracker 必须能看到目标才算"在位"
                    if not self._visible(ux, uy, tg["x"], tg["y"]):
                        continue
                keep[u] = r
            if keep:
                teams[tid] = list(keep.items())
                locked.update(keep.keys())

        # ---- 第二轮：给缺口重新编组（不含已锁定的机）----
        #
        # 全 tracker 之后不再用 assign_teams（它按 (n_tracker, n_blocker) 编组，
        # blocker 已放弃）。改为逐目标调用 choose()，按紧迫度依次取机，天然
        # 保证一架机只归属一个目标。
        need = sorted((tid for tid in pending if tid not in teams),
                      key=lambda t: -self.tr.targets[t].progress(self.t))
        pool = {u: p for u, p in self.uavs.items() if u not in locked}
        for tid in need:
            want = self._comp_for(tid)
            picks = self.assigner.choose(
                (self.tgt[tid]["x"], self.tgt[tid]["y"]), pool, n=want)
            if not picks:
                continue
            teams[tid] = [(u, "tracker") for u in picks]
            for u in picks:
                pool.pop(u, None)

        return teams

    # ---------------- 主循环 ----------------
    def run(self, max_t=300.0, on_step=None):
        """主循环。on_step(self) 每帧末尾调用一次（供外部采集指标）。

        加这个钩子是为了让 task_allocator.py 能在**同一套仿真**里采集 LOS/
        丢失/冲突这类逐帧指标，而不必复制整个循环 —— 复制会立刻产生两份
        逐渐分叉的规则实现，对照实验就失去意义了。
        """
        while self.t < max_t and len(self.done) < len(self.tgt):
            self.t += DT

            # --- 每机观测它能看到的所有已发现目标 ---
            for u, (ux, uy) in self.uavs.items():
                for tid, tg in self.tgt.items():
                    if tid in self.done or not tg["known"]:
                        continue
                    vis = self._visible(ux, uy, tg["x"], tg["y"])
                    self.tr.report(u, tid, self.t, tg["x"], tg["y"], los=vis)

            # --- 分配 / 派机（全 tracker）---
            teams = self._assign([])
            for tid, picks in teams.items():
                self.tgt[tid]["observers"] = [u for u, r in picks if r == "tracker"]
                self.tgt[tid]["roles"] = dict(picks)
                self.tr.assign_observers(tid, self.tgt[tid]["observers"])

            # --- 推进确认计时 ---
            events = self.tr.update(self.t)
            for tid, ev in events:
                if ev == "confirmed":
                    self.done.append(tid)
                    self.events.append((self.t, "eliminated", tid))
                    # 释放编组：清空观察员与角色，让这些机回到搜索池
                    self.tgt[tid]["observers"] = []
                    self.tgt[tid]["roles"] = {}
                elif ev == "evade":
                    self.events.append((self.t, "evade", tid))
                    # 规则4：瞬移 + 需要重新搜索（用户确认：重搜时间计入）
                    nx, ny = self.tgt[tid]["x"], self.tgt[tid]["y"]
                    for _ in range(500):
                        nx = self.rng.uniform(-88, 88)
                        ny = self.rng.uniform(-40, 40)
                        if (self.g.free(nx, ny)
                                and self.g.open_space_score(nx, ny) >= 0.5
                                and math.hypot(nx - self.tgt[tid]["x"],
                                               ny - self.tgt[tid]["y"]) > 40.0):
                            break
                    tg = self.tgt[tid]
                    tg["x"], tg["y"] = nx, ny
                    tg["known"] = False      # 需要重新搜索
                    tg["observers"] = []
                    tg["stall"] = 0.0
                    tg["last_prog"] = 0.0
                    self.tr.targets[tid].relocate(self.t)

            # --- 进度停滞检测（自适应加机用）---
            for tid in self.tr.pending():
                if tid in self.done or not self.tgt[tid]["known"]:
                    continue
                prog = self.tr.targets[tid].progress(self.t)
                tg = self.tgt[tid]
                if prog <= tg["last_prog"] + 1e-6:
                    tg["stall"] += DT
                else:
                    tg["stall"] = 0.0
                tg["last_prog"] = prog

            # --- 目标运动 ---
            for tid, tg in self.tgt.items():
                if tid in self.done:
                    continue
                nu, _ = self._nearest_uav(tg["x"], tg["y"])
                d = math.hypot(self.uavs[nu][0] - tg["x"], self.uavs[nu][1] - tg["y"]) \
                    if nu else 1e9
                if tg["known"] and d < SENSE_R:
                    fd = pick_flee_dir(self.g, (tg["x"], tg["y"]),
                                       (self.uavs[nu][0], self.uavs[nu][1]), self.hide)
                    nx, ny, _ = self.g.speed_toward(
                        tg["x"], tg["y"], tg["x"] + fd[0] * 10, tg["y"] + fd[1] * 10,
                        TARGET_FLEE)
                else:
                    # 规则2：未感知时 1 m/s 随机游走
                    if self.rng.random() < 0.6 * DT:
                        tg["heading"] = self.rng.uniform(0, 2 * math.pi)
                    nx, ny, _ = self.g.speed_toward(
                        tg["x"], tg["y"],
                        tg["x"] + math.cos(tg["heading"]) * 10,
                        tg["y"] + math.sin(tg["heading"]) * 10, TARGET_WALK)
                tg["x"], tg["y"] = nx, ny

            # --- 无人机运动（全 tracker）---
            engaged = self._engaged()
            for u, tid in engaged.items():
                if tid not in self.tgt:
                    continue
                tg = self.tgt[tid]
                tx, ty = tg["x"], tg["y"]
                ux, uy = self.uavs[u]
                # Tracker 站位：**追尾 standoff + 少量角向分离**。
                #
                # 注意离线实测否掉了"站到逃跑反方向/均布互补方位"的做法：
                # 那会让几架从"沿轨迹一字排开"塌缩成"挤在同一侧"，达成率
                # 60.5% → 35.0%。保持径向后，命中靠"合并观测"拼连续。
                trk = [t for t in tg["observers"] if t in self.uavs]
                k = len(trk)
                d = math.hypot(tx - ux, ty - uy)
                if SLOT_MODE == "abs":
                    # 用户提议的绝对槽位：θ_i = 2π·i/k，固定 0°/120°/240°。
                    # （保留代码路径以便随时复跑那次对照；实测更差，见顶部注释）
                    ang = 2.0 * math.pi * trk.index(u) / max(1, k)
                    gx = tx + TRACK_STANDOFF * math.cos(ang)
                    gy = ty + TRACK_STANDOFF * math.sin(ang)
                elif SLOT_MODE == "bearing":
                    # 分离作用在**接近射线**上（修"远机分离失效"的根因）。
                    #
                    # 现场证据（probe_form.py --trace，seed=1 t=11.7）：
                    #   uav_3 current=(-42.76,33.13) pre_fan=(-70.34,24.34)
                    #         post_fan=(-70.34,24.34) fan_vec_norm=0.000 off_deg=80.0
                    # 远机 d>standoff 时落点 = 目标本体，旋转的向量是 (0,0)，
                    # 于是"绕目标转 80°"是**恒等变换** —— 分离在接近段完全
                    # 不存在，两架从同一走廊进场就必然连线重合。
                    # 另外 d 落进 [7,9] 死区时 gx,gy=ux,uy 原地冻结，而目标是
                    # 动的，d 在 8.63↔9.11 之间反复穿边界 → 走走停停。
                    #
                    # 改法：落点取「目标 + standoff·单位向量(自身方位 + fan)」。
                    #   (a) 该点恒在 standoff 圆上 → 旋转向量长度恒为 standoff，
                    #       分离永远有效（含 fan 偏移为 0 的中间那架，它靠"自身
                    #       方位"和同伴分开，不再靠 index 取 0°）；
                    #   (b) 无死区 → 不再有"进去就冻结"；
                    #   (c) 该点就在"UAV→目标"这条射线上 → 接近路径与原来
                    #       **完全同一条直线**，实测最优的"径向一字排开"被保留，
                    #       只是提前 8 m 停住而不是冲进去再退。
                    # 方位取自自身位置而非固定槽号 —— 这就是它与 "abs" 的
                    # 本质区别：不把机硬拽到指定角度，只纠正"撞到一起"。
                    idx = trk.index(u)
                    off = (math.radians(TRACK_SPLIT_DEG) * (2.0 * idx - (k - 1))
                           if k >= 2 else 0.0)
                    a_u = math.atan2(uy - ty, ux - tx) if d > 1e-6 else 0.0
                    a_g = a_u + off
                    gx = tx + TRACK_STANDOFF * math.cos(a_g)
                    gy = ty + TRACK_STANDOFF * math.sin(a_g)
                else:
                    if d > TRACK_STANDOFF + 1.0:
                        gx, gy = tx, ty
                    elif d < TRACK_STANDOFF - 1.0 and d > 1e-6:
                        gx = ux + (ux - tx) / d * (TRACK_STANDOFF - d)
                        gy = uy + (uy - ty) / d * (TRACK_STANDOFF - d)
                    else:
                        gx, gy = ux, uy

                # 友机排斥见下方（在算出速度之后叠加），这里只定落点。
                # 记下 fan **之前**的落点：远机（d > standoff+1）这里是 (tx,ty)，
                # 于是 fan 的旋转向量 (gx−tx, gy−ty) = (0,0)，转多少度都不动。
                _pf_x, _pf_y, _fan_off = gx, gy, 0.0
                if SLOT_MODE == "dedup":
                    # 角度去重：只推开**方位撞车**的机，且按实际方位推。
                    #
                    # 与固定 slot 的关键差别：固定 slot 是"你必须去 120°"，不管
                    # 你现在在哪 —— 几架从同一走廊来时会被硬拽到不同侧，脱离
                    # 原来的观测几何（实测 60.5% → 35.0%）。角度去重是"你已经
                    # 和别人撞方位了，才各让一半"，方位本来就散开时**完全不动**。
                    for other in trk:
                        if other == u:
                            continue
                        V = self.uavs.get(other)
                        if V is None:
                            continue
                        a1 = math.atan2(uy - ty, ux - tx)
                        a2 = math.atan2(V[1] - ty, V[0] - tx)
                        dif = abs(math.degrees(a1 - a2)) % 360.0
                        if dif > 180.0:
                            dif = 360.0 - dif
                        if dif < ANGLE_DEDUP_DEG:
                            # 反向推开：各自远离对方的方位角一半
                            half = math.radians(ANGLE_DEDUP_DEG) / 2.0
                            sgn = 1.0 if math.sin(a1 - a2) >= 0 else -1.0
                            na = a1 + sgn * half
                            rr = math.hypot(ux - tx, uy - ty)
                            gx = tx + rr * math.cos(na)
                            gy = ty + rr * math.sin(na)
                elif SLOT_MODE != "abs" and k >= 2:
                    off = math.radians(TRACK_SPLIT_DEG) * (2.0 * trk.index(u) - (k - 1))
                    dx, dy = gx - tx, gy - ty
                    ca, sa = math.cos(off), math.sin(off)
                    gx = tx + dx * ca - dy * sa
                    gy = ty + dx * sa + dy * ca
                    _fan_off = off
                _trace = (FORM_TRACE_UAVS is not None and u in FORM_TRACE_UAVS
                          and FORM_TRACE_T0 <= self.t <= FORM_TRACE_T1)
                if _trace:
                    # 同时打 fan **前**的落点，用来验证"远机奔目标中心 → 旋转
                    # 零向量 → 分离失效"这个机制（0 向量转过任何角度还是 0）。
                    _form_log(
                        "[FORM] t=%.1f uav=%s current=(%.2f,%.2f) "
                        "pre_fan=(%.2f,%.2f) post_fan=(%.2f,%.2f) "
                        "fan_vec_norm=%.3f off_deg=%.1f tgt=%s tgt_xy=(%.2f,%.2f) "
                        "d=%.2f k=%d idx=%d trk=%s"
                        % (self.t, u, ux, uy, _pf_x, _pf_y, gx, gy,
                           math.hypot(_pf_x - tx, _pf_y - ty),
                           math.degrees(_fan_off), tid, tx, ty, d, k,
                           trk.index(u), trk))
                if not self.g.free(gx, gy):
                    _branch = "->tgt" if d > TRACK_STANDOFF else "->stay"
                    if _trace:
                        _form_log(
                            "[FALLBACK] t=%.1f uav=%s desired=(%.2f,%.2f) "
                            "free=False branch=%s d=%.2f (d>%.1f? %s)"
                            % (self.t, u, gx, gy, _branch, d, TRACK_STANDOFF,
                               d > TRACK_STANDOFF))
                    gx, gy = (tx, ty) if d > TRACK_STANDOFF else (ux, uy)
                elif _trace:
                    _form_log("[FALLBACK] t=%.1f uav=%s desired=(%.2f,%.2f) "
                              "free=True 无回退" % (self.t, u, gx, gy))
                ax, ay, _ = self.g.speed_toward(ux, uy, gx, gy, UAV_TRACK_SPEED)

                # 友机排斥（**速度层**叠加，不在目标点上推）。
                #
                # 为什么必须放在速度层：一开始我把推力加到**目标点**上，结果
                # d_safe=8.0 时 F_rep 可达 3.0×8.0=24 m，目标点被甩到很远处，
                # 很可能落到障碍里 → 下面的 `free` 检查直接回退成 (tx,ty)，
                # 整个推力被丢弃。实测表现为冲突数暴涨 +795.7、最小间距 0.00 m
                # （比不加还差）。速度层不会碰到"落点是否可通行"的问题。
                #
                # F_rep = k·(d_safe − d)，方向沿两机连线远离友机，与用户给的
                # 公式一致；各机都推自己那一半，所以是对称软化，不会有一架
                # 永远退让。近距离时限幅，否则推力会大到把速度撑爆。
                if FRIEND_SAFE > 0.0:
                    rx, ry = 0.0, 0.0
                    for other, V in self.uavs.items():
                        if other == u or V is None:
                            continue
                        fdx, fdy = ux - V[0], uy - V[1]
                        fd = math.hypot(fdx, fdy)
                        if 1e-6 < fd < FRIEND_SAFE:
                            mag = FRIEND_K * (FRIEND_SAFE - fd)
                            rx += fdx / fd * mag
                            ry += fdy / fd * mag
                    if rx or ry:
                        rn = math.hypot(rx, ry)
                        if rn > FRIEND_MAX:
                            rx, ry = rx / rn * FRIEND_MAX, ry / rn * FRIEND_MAX
                        # 限幅必须在**位移**上做，不能在合成后的位置上做。
                        #
                        # 这里踩过一个严重的坑：原本写的是
                        #     an = hypot(ax, ay); ax, ay = ax/an*speed, ...
                        # 意图是"限制速度"，但 ax,ay 此刻是**位置**不是速度，
                        # hypot(ax,ay) 量的是"离世界原点多远"。于是任何离原点
                        # 超过 4.7 m 的机，只要被友机推一下，就被瞬移到
                        # "离原点 4.7 m、方位不变"的点上 —— 一帧跳几十米。
                        # 诊断里表现为大量单帧位移 30~70 m 的"瞬移"，且**只**
                        # 在 FRIEND_SAFE>0 时出现（关掉时整块不执行）。
                        # 实测 6 例，落点到原点的距离全是 4.700 ± 0.003
                        # （即 UAV_TRACK_SPEED），方位恰等于原位置的方位。
                        #
                        # 更坏的是它会**自己制造冲突**：几架被压到同一个 4.7 m
                        # 圆上就叠在一起，于是"开空间管理反而更近"是假象。
                        # 修掉之后 space-on 的近距段从 13（100% 含瞬移）掉到
                        # 0~2，消融表里冲突 -85% 的数字才是真的。
                        #
                        # 正确做法：把追踪位移与排斥位移相加后再限幅。
                        dxm, dym = (ax - ux) + rx * DT, (ay - uy) + ry * DT
                        dm = math.hypot(dxm, dym)
                        cap = UAV_TRACK_SPEED * DT
                        if dm > cap:                  # 仍受速度上限约束
                            dxm, dym = dxm / dm * cap, dym / dm * cap
                        ax, ay = ux + dxm, uy + dym

                self.uavs[u] = [ax, ay]

            # --- 空闲机继续搜索 ---
            free = [u for u in self.uavs if u not in engaged]
            self._search_step(free)

            if on_step is not None:
                on_step(self)

        return self.t, list(self.done), list(self.events)

    def _engaged(self):
        """{uav_id: target_id} 当前正在追踪的无人机。"""
        out = {}
        for tid in self.tr.pending():
            if tid in self.done or not self.tgt[tid]["known"]:
                continue
            for u in self.tgt[tid]["observers"]:
                out[u] = tid
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=8)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--max-t", type=float, default=300.0)
    args = ap.parse_args()

    g = MapGrid(METADATA)
    print("=" * 76)
    print("整体任务时间：6 架 UAV 清完 6 个恐怖分子")
    print("=" * 76)
    print("地图 %s | 目标 6 | UAV 6" % os.path.basename(METADATA))
    print("搜索 %.1f m/s | 追踪 %.1f m/s（SITL 实测有效值）| 躲藏=理想"
          % (UAV_SEARCH_SPEED, UAV_TRACK_SPEED))
    print("规则：游走 1 / 逃跑 2 / 确认 15 s / 瞬移 30 s | 计时含搜索与重搜")
    print("")

    for policy in ("fixed3", "fixed2", "adaptive"):
        tot, done_n, evades = [], [], []
        for k in range(args.runs):
            sim = Sim(g, seed=args.seed + k, policy=policy)
            t_end, done, evs = sim.run(args.max_t)
            tot.append(t_end)
            done_n.append(len(done))
            evades.append(sum(1 for e in evs if e[1] == "evade"))
        n = len(tot)
        avg = sum(tot) / n
        full = sum(1 for d in done_n if d == 6)
        tag = {"fixed3": "每目标 3T（3 架）",
               "fixed2": "每目标 2T（2 架）",
               "adaptive": "动态加机（2→3 架）"}[policy]
        print("%-22s | 平均用时 %6.1f s | 平均消除 %.1f/6 | 全清 %d/%d | 瞬移 %.1f 次"
              % (tag, avg, sum(done_n) / n, full, n, sum(evades) / n))
    print("")
    print("=" * 76)
    print("说明：结果基于几何仿真（视线遮挡 + 规则模型），未含机间避碰开销。")
    print("      T=tracker；全 tracker，Blocker 方案已放弃（见 _comp_for 注释）。")
    print("=" * 76)
    return 0


if __name__ == "__main__":
    sys.exit(main())
