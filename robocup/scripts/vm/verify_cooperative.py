#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""关键验证：多机合并观测能否达成 15 秒连续覆盖（严格 1 秒标准）。

要回答的问题
------------
单机 SITL 实测在理想躲藏下最长连续只有 6.6 秒（需 15 秒）—— 单机必然失败。
整个多机架构的基石假设是：**多架从互补方位观察，能把彼此的视线缺口补上**。

本脚本在真实地图上离线验证这个假设，并回答：
  * 2 架够不够？3 架呢？
  * 达成 15 秒需要多久（会不会超过规则4 的 30 秒预算）？
  * 观察员分配器选的「互补组合」比「最近的 N 架」好多少？

为什么先做离线
--------------
这是纯几何问题（视线遮挡能否互补）。离线能跑大量样本、扫参数；
若离线显示 2 架够而 SITL 不够，那是飞行精度问题，可单独优化。

用法：
    python3 verify_cooperative.py --trials 100 --team 2
    python3 verify_cooperative.py --trials 100 --sweep      # 扫 1/2/3/4 架
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
from observer_assign import ObserverAssigner  # noqa: E402

UAV_SPEED = 6.0
ALT = 6.0
MAX_T = 60.0


def simulate(g, assigner, n_uavs, start_uavs, tgt_xy, hide, max_t=MAX_T,
             strategy="assigned", rebalance_period=0.5, standoff=8.0):
    """跑一次多机协同确认。

    strategy:
      "assigned"  —— ObserverAssigner 选观察员**并给出互补站位**
      "nearest"   —— 取最近的 n 架，全部挤向目标周围同一半径（基线）

    关键差异（上一版仿真的缺陷）：上一版所有观察员都用同一句「朝目标靠近到
    8 m」，等于实现了 "nearest" 的行为，**互补分配器选的方位根本没用上**，
    所以两种策略结果几乎一样。本版让 assigned 策略真正飞到各自被分配的
    方位站位上，互补性才有可能体现。
    """
    tgt = list(tgt_xy)
    uavs = {uid: list(p) for uid, p in start_uavs.items()}
    tr = CooperativeTracker()
    tr.add_target("t0", now=0.0)

    state = {"slots": {}}      # uid -> (sx, sy) 当前站位

    def reassign():
        if strategy == "nearest":
            d = sorted(uavs.items(), key=lambda kv: math.hypot(
                kv[1][0] - tgt[0], kv[1][1] - tgt[1]))
            picked = [uid for uid, _ in d[:n_uavs]]
            # 基线：不分工位，都挤向目标周围 —— 这正是"最近 N 架"的真实行为
            state["slots"] = {uid: None for uid in picked}
            observers = picked
        else:
            pairs = assigner.choose_with_slots(tgt, uavs, n=n_uavs,
                                               standoff=standoff)
            observers = [uid for uid, _ in pairs]
            state["slots"] = {uid: pos for uid, pos in pairs}
        tr.assign_observers("t0", observers)
        return observers

    observers = reassign()
    t = 0.0
    last_rebalance = 0.0
    confirm_t = None
    obs_hist = []

    while t < max_t:
        t += DT

        # ---- 每架观测（LOS 判定）----
        for uid, (ux, uy) in uavs.items():
            d = math.hypot(tgt[0] - ux, tgt[1] - uy)
            vis = d < SENSE_R and g.visible(ux, uy, tgt[0], tgt[1])
            tr.report(uid, "t0", t, tgt[0], tgt[1], los=vis)

        # ---- 推进确认计时 ----
        for tid, ev in tr.update(t):
            if ev == "confirmed":
                obs_hist.append(len(tr.targets["t0"].live_observers(t)))
                return dict(achieved=True, t_confirm=t,
                            n_obs=len(observers), ended_by="confirmed",
                            avg_live=(sum(obs_hist) / len(obs_hist)) if obs_hist else 0.0)
            if ev == "evade":
                return dict(achieved=False, t_confirm=None,
                            n_obs=len(observers), ended_by="evade",
                            avg_live=(sum(obs_hist) / len(obs_hist)) if obs_hist else 0.0)

        obs_hist.append(len(tr.targets["t0"].live_observers(t)))

        # ---- 周期重分配（目标会跑，站位要跟着调）----
        if t - last_rebalance >= rebalance_period:
            last_rebalance = t
            observers = reassign()

        # ---- 目标逃跑（按当前观察员质心判断"最近无人机"）----
        if observers:
            cx = sum(uavs[u][0] for u in observers) / len(observers)
            cy = sum(uavs[u][1] for u in observers) / len(observers)
        else:
            cx, cy = uavs["uav_1"][0], uavs["uav_1"][1]
        fd = pick_flee_dir(g, tgt, (cx, cy), hide)
        nx, ny, _ = g.speed_toward(tgt[0], tgt[1],
                                   tgt[0] + fd[0] * 10, tgt[1] + fd[1] * 10, TARGET_FLEE)
        tgt[0], tgt[1] = nx, ny

        # ---- 每架飞向自己的站位 ----
        for uid in observers:
            if uid not in uavs:
                continue
            ux, uy = uavs[uid]
            slot = state["slots"].get(uid)
            if slot is None:
                # 基线策略：只维持 8 m 观察距离，不分工位
                d = math.hypot(tgt[0] - ux, tgt[1] - uy)
                if d > standoff + 1.0:
                    tx, ty = tgt[0], tgt[1]
                elif d < standoff - 1.0:
                    tx = ux + (ux - tgt[0]) / max(d, 1e-6) * (standoff - d)
                    ty = uy + (uy - tgt[1]) / max(d, 1e-6) * (standoff - d)
                else:
                    tx, ty = ux, uy
            else:
                # 互补策略：飞向被分配的方位站位（站位随目标移动平移）
                tx, ty = slot
            ax, ay, _ = g.speed_toward(ux, uy, tx, ty, UAV_SPEED)
            uavs[uid] = [ax, ay]

    return dict(achieved=confirm_t is not None, t_confirm=confirm_t,
                n_obs=len(observers), ended_by="timeout",
                avg_live=(sum(obs_hist) / len(obs_hist)) if obs_hist else 0.0)


def make_case(g, rng):
    """初始条件：目标在开阔且可见处，UAV 在其周围散开。"""
    for _ in range(400):
        tx = rng.uniform(-80, 80)
        ty = rng.uniform(-36, 36)
        if not g.free(tx, ty) or g.open_space_score(tx, ty) < 0.6:
            continue
        uavs = {}
        ok = True
        for i in range(6):
            for _try in range(30):
                ang = rng.uniform(0, 2 * math.pi)
                d = rng.uniform(12.0, 22.0)
                ux = tx + d * math.cos(ang)
                uy = ty + d * math.sin(ang)
                if abs(ux) > 92 or abs(uy) > 44:
                    continue
                if not g.free(ux, uy):
                    continue
                uavs["uav_%d" % (i + 1)] = [ux, uy]
                break
            else:
                ok = False
                break
        if ok and len(uavs) == 6:
            return tx, ty, uavs
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=80)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--team", type=int, default=2, help="观察员数量")
    ap.add_argument("--sweep", action="store_true", help="扫 1/2/3/4 架")
    ap.add_argument("--hide", default="ideal", choices=["none", "local", "ideal"])
    args = ap.parse_args()

    HIDE = {"none": HIDE_NONE, "local": HIDE_LOCAL, "ideal": HIDE_IDEAL}[args.hide]
    rng = random.Random(args.seed)
    g = MapGrid(METADATA)
    assigner = ObserverAssigner(g)

    print("=" * 76)
    print("关键验证：多机合并观测能否达成 15 s 连续覆盖（严格 1 s 标准）")
    print("=" * 76)
    print("地图 %s | 无人机 %.0f m/s | 目标 %.0f m/s | 躲藏=%s"
          % (os.path.basename(METADATA), UAV_SPEED, TARGET_FLEE, args.hide))
    print("确认 15 s | 瞬移 30 s | 单次上限 %.0f s | 步长 %.1f s"
          % (MAX_T, DT))
    print("")

    cases = []
    for _ in range(args.trials):
        c = make_case(g, rng)
        if c:
            cases.append(c)
    print("有效初始条件: %d 组\n" % len(cases))

    teams = [1, 2, 3, 4] if args.sweep else [args.team]

    for n in teams:
        for strat in ("assigned", "nearest"):
            ok = 0
            times = []
            evades = 0
            lives = []
            for (tx, ty, uavs) in cases:
                r = simulate(g, assigner, n, uavs, (tx, ty), HIDE, strategy=strat)
                if r["achieved"]:
                    ok += 1
                    times.append(r["t_confirm"])
                elif r["ended_by"] == "evade":
                    evades += 1
                lives.append(r.get("avg_live", 0.0))
            total = len(cases)
            avg_t = sum(times) / len(times) if times else None
            avg_live = sum(lives) / total if lives else 0.0
            tag = "观察员分配器（互补）" if strat == "assigned" else "最近 N 架（基线）"
            print("%-22s | 达成 %5.1f%% (%2d/%d) | 耗时 %s | 瞬移 %2d | 同时可见 %.2f 架"
                  % (tag, 100.0 * ok / total, ok, total,
                     ("%.1fs" % avg_t) if avg_t else "  - ", evades, avg_live))
        print("")

    print("=" * 76)
    print("判读：")
    print("  * 若 1 架远低于 2 架 → 合并观测确实有效，多机架构成立")
    print("  * 若 2 架已达高达成率 → 每目标派 2 架即可")
    print("  * 若 3 架仍不足 → 需重新考虑策略（或确认裁判的 obs_ttl 定义）")
    print("=" * 76)
    return 0


if __name__ == "__main__":
    sys.exit(main())
