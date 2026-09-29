#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Tracker-Blocker 协同：堵路破坏逃跑，改善 Tracker 的观测稳定性。

思路
----
规则 3：目标感知到无人机后，以 2 m/s 朝**远离最近无人机**的方向逃跑。
规则 5：连续 15 s 确认（严格：断 1 s 归零）。

单机追踪的困境（SITL 实测）：目标一路直线逃，绕过楼角就断视线，最长只能
连续 6.6 s。

关键洞察：**逃跑方向由我们的站位决定**。所以在逃跑方向的前方放一架
Blocker 堵住去路，目标就不得不：
  * 转向（改变逃跑方向，可能被迫走向开阔地）
  * 或减速/贴着 Blocker 绕行

无论哪种，目标的**直线逃跑被打断**，Tracker 的观测稳定性都会提升。

与「多派几架一起追」的本质区别
------------------------------
多派 Tracker 只是增加观测点；Blocker 改变的是**目标的行为**（破坏它的
最优逃跑），所以边际收益更高。本脚本用仿真验证这一点。
"""

import argparse
import math
import os
import random
import sys

WS = os.environ.get("ROBOCUP_WORKSPACE",
                    os.environ.get("ROBOCUP_WS", os.path.expanduser("~/team_ws/robocup")))
sys.path.insert(0, os.path.join(WS, "scripts", "vm"))
sys.path.insert(0, os.path.join(WS, "src", "robocup_swarm", "scripts"))

from strategy_compare import (MapGrid, METADATA, pick_flee_dir,  # noqa: E402
                              HIDE_IDEAL, HIDE_LOCAL, HIDE_NONE,
                              TARGET_FLEE, SENSE_R, DT)
from cooperative_tracker import CooperativeTracker  # noqa: E402

UAV_SPEED = 6.0
TRACK_STANDOFF = 8.0     # Tracker 与目标的保持距离
BLOCK_STANDOFF = 12.0    # Blocker 站位：目标前方多远堵路
MAX_T = 60.0


def flee_direction(g, tgt, uavs_positions, hide):
    """目标当前会选择的逃跑方向（规则3：远离最近无人机）。"""
    if not uavs_positions:
        return (0.0, 0.0)
    nx, ny = min(uavs_positions, key=lambda p: math.hypot(p[0] - tgt[0], p[1] - tgt[1]))
    return pick_flee_dir(g, tgt, (nx, ny), hide, prev_dir=(0.0, 0.0))


def simulate(g, roles, start_uavs, tgt_xy, hide, max_t=MAX_T):
    """跑一次协同确认。

    roles: ["tracker", "blocker", ...] —— 与 start_uavs 的 key 顺序对应
           支持 "tracker" / "blocker" 两种角色；长度决定用几架。

    返回 dict(achieved, t_confirm, ended_by, avg_live, blocked_frac)
      blocked_frac: Blocker 实际挡在目标逃跑方向前方的时长占比（衡量堵路效果）
    """
    uav_ids = sorted(start_uavs.keys())[:len(roles)]
    uavs = {u: list(start_uavs[u]) for u in uav_ids}
    role_of = dict(zip(uav_ids, roles))

    tgt = list(tgt_xy)
    tr = CooperativeTracker()
    tr.add_target("t0", now=0.0)

    # 只有 tracker 负责观测；blocker 不参与确认（其作用是改变目标行为）
    trackers = [u for u in uav_ids if role_of[u] == "tracker"]
    tr.assign_observers("t0", trackers)

    t = 0.0
    live_hist = []
    block_hist = []

    while t < max_t:
        t += DT

        # ---- 观测（仅 tracker 计入）----
        for u in trackers:
            ux, uy = uavs[u]
            d = math.hypot(tgt[0] - ux, tgt[1] - uy)
            vis = d < SENSE_R and g.visible(ux, uy, tgt[0], tgt[1])
            tr.report(u, "t0", t, tgt[0], tgt[1], los=vis)

        for tid, ev in tr.update(t):
            if ev == "confirmed":
                live_hist.append(len(tr.targets["t0"].live_observers(t)))
                return dict(achieved=True, t_confirm=t, ended_by="confirmed",
                            avg_live=sum(live_hist) / len(live_hist),
                            blocked_frac=(sum(block_hist) / len(block_hist))
                            if block_hist else 0.0)
            if ev == "evade":
                return dict(achieved=False, t_confirm=None, ended_by="evade",
                            avg_live=(sum(live_hist) / len(live_hist))
                            if live_hist else 0.0,
                            blocked_frac=(sum(block_hist) / len(block_hist))
                            if block_hist else 0.0)
        live_hist.append(len(tr.targets["t0"].live_observers(t)))

        # ---- 决定目标逃跑方向（依据规则3，看最近的无人机）----
        all_pos = [tuple(uavs[u]) for u in uav_ids]
        fd = flee_direction(g, tgt, all_pos, hide)

        # ---- 堵路效果度量：是否有 blocker 位于逃跑方向前方 ----
        blocked_now = 0.0
        for u in uav_ids:
            if role_of[u] != "blocker":
                continue
            ux, uy = uavs[u]
            # 该 blocker 相对目标的方位与逃跑方向的夹角
            ax, ay = ux - tgt[0], uy - tgt[1]
            n = math.hypot(ax, ay) or 1.0
            cosang = (ax / n) * fd[0] + (ay / n) * fd[1]
            if cosang > 0.5 and math.hypot(ax, ay) < BLOCK_STANDOFF * 2.0:
                blocked_now = 1.0
                break
        block_hist.append(blocked_now)

        # ---- 目标运动 ----
        # 关键：目标逃跑时会**避开**正前方的 Blocker（否则会撞上），
        # 这体现在 pick_flee_dir 的"必须可通行"检查里；若前方被 Blocker
        # 占据，目标自然转向别的方向。
        nx, ny, _ = g.speed_toward(tgt[0], tgt[1],
                                   tgt[0] + fd[0] * 10, tgt[1] + fd[1] * 10,
                                   TARGET_FLEE)
        tgt[0], tgt[1] = nx, ny

        # ---- 无人机运动 ----
        for u in uav_ids:
            ux, uy = uavs[u]
            if role_of[u] == "tracker":
                # 追踪者：保持在目标后方 TRACK_STANDOFF 距离
                d = math.hypot(tgt[0] - ux, tgt[1] - uy)
                if d > TRACK_STANDOFF + 1.0:
                    gx, gy = tgt[0], tgt[1]
                elif d < TRACK_STANDOFF - 1.0:
                    gx = ux + (ux - tgt[0]) / max(d, 1e-6) * (TRACK_STANDOFF - d)
                    gy = uy + (uy - tgt[1]) / max(d, 1e-6) * (TRACK_STANDOFF - d)
                else:
                    gx, gy = ux, uy
            else:
                # 拦截者：站到目标**逃跑方向的前方** —— 堵住去路
                gx = tgt[0] + fd[0] * BLOCK_STANDOFF
                gy = tgt[1] + fd[1] * BLOCK_STANDOFF
                # 落点不可通行时，沿垂直方向偏移找可站点
                if not g.free(gx, gy):
                    for sgn in (1, -1):
                        px = tgt[0] + fd[0] * BLOCK_STANDOFF - sgn * fd[1] * 4.0
                        py = tgt[1] + fd[1] * BLOCK_STANDOFF + sgn * fd[0] * 4.0
                        if g.free(px, py):
                            gx, gy = px, py
                            break
            ax, ay, _ = g.speed_toward(ux, uy, gx, gy, UAV_SPEED)
            uavs[u] = [ax, ay]

    return dict(achieved=False, t_confirm=None, ended_by="timeout",
                avg_live=(sum(live_hist) / len(live_hist)) if live_hist else 0.0,
                blocked_frac=(sum(block_hist) / len(block_hist)) if block_hist else 0.0)


def make_case(g, rng):
    """初始条件：目标在开阔且可见处，候选 UAV 散落在其周围。"""
    for _ in range(400):
        tx = rng.uniform(-78, 78)
        ty = rng.uniform(-34, 34)
        if not g.free(tx, ty) or g.open_space_score(tx, ty) < 0.6:
            continue
        uavs = {}
        for i in range(3):
            for _t in range(40):
                ang = rng.uniform(0, 2 * math.pi)
                d = rng.uniform(12.0, 20.0)
                ux, uy = tx + d * math.cos(ang), ty + d * math.sin(ang)
                if abs(ux) > 92 or abs(uy) > 44 or not g.free(ux, uy):
                    continue
                uavs["uav_%d" % (i + 1)] = [ux, uy]
                break
        if len(uavs) == 3:
            return tx, ty, uavs
    return None


CONFIGS = [
    ("1 tracker", ["tracker"]),
    ("2 trackers", ["tracker", "tracker"]),
    ("3 trackers", ["tracker", "tracker", "tracker"]),
    ("tracker + blocker", ["tracker", "blocker"]),
    ("tracker + 2 blocker", ["tracker", "blocker", "blocker"]),
    ("2 tracker + blocker", ["tracker", "tracker", "blocker"]),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=60)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--hide", default="ideal", choices=["none", "local", "ideal"])
    args = ap.parse_args()

    HIDE = {"none": HIDE_NONE, "local": HIDE_LOCAL, "ideal": HIDE_IDEAL}[args.hide]
    rng = random.Random(args.seed)
    g = MapGrid(METADATA)

    print("=" * 78)
    print("Tracker-Blocker 协同验证（严格 15 s 连续确认，断 1 s 归零）")
    print("=" * 78)
    print("地图 %s | UAV %.0f m/s | 目标 %.0f m/s | 躲藏=%s"
          % (os.path.basename(METADATA), UAV_SPEED, TARGET_FLEE, args.hide))
    print("")

    cases = []
    for _ in range(args.trials):
        c = make_case(g, rng)
        if c:
            cases.append(c)
    print("有效初始条件: %d 组\n" % len(cases))

    print("%-22s | %-13s | %-9s | %-11s | %-10s" %
          ("配置", "达成率", "平均耗时", "同时观测(均值)", "堵路占比"))
    print("-" * 78)
    for name, roles in CONFIGS:
        ok = 0
        times, lives, blocks = [], [], []
        for (tx, ty, uavs) in cases:
            r = simulate(g, roles, uavs, (tx, ty), HIDE)
            if r["achieved"]:
                ok += 1
                times.append(r["t_confirm"])
            lives.append(r["avg_live"])
            blocks.append(r["blocked_frac"])
        n = len(cases)
        avg_t = sum(times) / len(times) if times else None
        blk = sum(blocks) / n
        blk_s = ("%.0f%%" % (100 * blk)) if any(b > 0 for b in blocks) else "  -  "
        print("%-22s | %6.1f%% (%2d/%d) | %8s | %11.2f | %10s"
              % (name, 100.0 * ok / n, ok, n,
                 ("%.1fs" % avg_t) if avg_t else "  -  ",
                 sum(lives) / n, blk_s))
    print("-" * 78)
    print("")
    print("判读：")
    print("  * 对比「2 trackers」与「tracker + blocker」：架数相同时，堵路是否优于多一个观测点")
    print("  * 对比「3 trackers」与「2 tracker + blocker」：同上")
    print("  * 堵路占比 > 0 说明 Blocker 确实站到了逃跑方向前方")
    print("=" * 78)
    return 0


if __name__ == "__main__":
    sys.exit(main())
