#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""正式消融：SEP 阶跃 + 事件归因（含**搜索阶段**）。

与 run_sep_big.py 的差别：
  1. 事件归因分 5 类：group(同目标) / diff(异目标) / 一追一搜 / 双搜 / 未知
  2. 报"最紧事件"现场（位置/时刻/类别），用于定位残余 0.10 m 是谁造成的
  3. 报每类的最小间距 —— 用户要的 3.0 m 约束**不能只算同目标**，
     若最小值来自搜索阶段，编组几何再修也够不到

命令行：python run_ablation.py <sep> <outfile> <n_seeds>
环境变量：PUSH 覆盖 SEP_PUSH_MAX_DEG（默认 30）
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

sep = float(sys.argv[1])
out = sys.argv[2]
nmax = int(sys.argv[3]) if len(sys.argv) > 3 else 40

import mission_time as MT
import task_allocator as TA
from strategy_compare import MapGrid

MT.SEP_TARGET_M = sep
MT.SEP_PUSH_MAX_DEG = float(os.environ.get("PUSH", "30"))
MT.SEARCH_PHASE = int(os.environ.get("SEARCH_PHASE", "0"))
MT.FRIEND_SAFE = float(os.environ.get("FRIEND_SAFE", "0"))
MT.FRIEND_K = float(os.environ.get("FRIEND_K", "3"))
MT.FRIEND_MAX = float(os.environ.get("FRIEND_MAX", "2"))
MT.SLOT_MODE = "chase"

D_HIT = 1.5
D_TIGHT = 0.5
UAV_RADIUS = 0.45
COLLISION_DIST = 2.0 * UAV_RADIUS        # 0.90 m 实际几何碰撞
NEAR_MISS_DIST = 1.20                    # 危险接近（仅统计）
CATS = ["group", "diff", "t-s", "s-s"]


def run(seeds, targets=6, max_t=300.0):
    g = MapGrid(TA.METADATA)
    full = 0
    tgt_done = 0
    tgt_total = 0
    n_ev = 0
    ev_by = {c: 0 for c in CATS}
    min_by = {c: None for c in CATS}
    tight_by = {c: 0 for c in CATS}
    stuck_by = {c: 0.0 for c in CATS}
    tight_all = []
    weird = []            # 未知归属的紧事件
    closest = []          # 每局最紧事件 (d, sd, t, u, v, cat)
    # 碰撞 / 危险接近 / LOS 中断（**事件计数，不是帧数**）
    n_collide = 0         # UAV-UAV 间距 < 0.90 m 的碰撞事件
    n_near = 0            # UAV-UAV 0.90~1.20 m 的危险接近事件
    n_bldg = 0            # UAV 撞建筑（free True->False 转变）事件
    n_los = 0             # LOS 中断（对已追踪目标，LOS True->False）事件
    los_breaks_by = {c: 0 for c in CATS}
    avg_los_dur = []      # 每次 LOS 连续保持时长（帧数*DT）
    _prev_free = None     # 上一帧 free 状态（bool）
    _prev_los = {}        # uav -> 上一帧 LOS 状态（bool）
    prev_collide_pairs = set()   # previous frame colliding pairs
    prev_near_pairs = set()      # previous frame near-miss pairs

    # === 搜索阶段统计 ===
    all_first_detect = []   # 每局首次发现时间列表
    all_detect_to_assign = []  # 每局发现到分配的延迟列表
    all_region_coverage = {} # 区域覆盖时间累加 {(rx,ry): [t1, t2, ...]}
    all_remaining_ratio = [] # 每局剩余搜索面积比例

    for sd in seeds:
        sim = TA.AllocSim(g, strategy="dynamic", targets=targets, seed=sd)
        frames = {}

        def hook(s, frames=frames, sd=sd):
            nonlocal n_collide, n_near, n_bldg, n_los, _prev_free, _prev_los, prev_collide_pairs, prev_near_pairs
            eng = s._engaged()
            pos = {u: tuple(p) for u, p in s.uavs.items()}
            # --- UAV-building collision: free True->False transition ---
            cur_free = all(s.g.free(px, py) for px, py in pos.values())
            if _prev_free is True and not cur_free: n_bldg += 1
            _prev_free = cur_free
            # --- LOS break for tracked targets: visible True->False ---
            for uu, ttid in eng.items():
                if ttid is None: continue
                tgt = s.tgt.get(ttid)
                if tgt is None: continue
                px, py = pos[uu]
                los_now = s.g.visible(px, py, tgt["x"], tgt["y"])
                if _prev_los.get(uu, True) and not los_now:
                    n_los += 1
                _prev_los[uu] = los_now
            # --- UAV-UAV 碰撞 / 危险接近（事件计数，非帧数）---
            pair_collide = set()
            pair_near = set()
            for ui in sorted(pos):
                for vi in sorted(pos):
                    if vi <= ui:
                        continue
                    dd = math.hypot(pos[ui][0] - pos[vi][0],
                                    pos[ui][1] - pos[vi][1])
                    if dd < COLLISION_DIST: pair_collide.add((ui,vi))
                    if dd < NEAR_MISS_DIST: pair_near.add((ui,vi))
                    if dd >= D_HIT:
                        continue
                    eu, ev = eng.get(ui), eng.get(vi)
                    if eu is None and ev is None:
                        c = "s-s"
                    elif eu is None or ev is None:
                        c = "t-s"
                    elif eu == ev:
                        c = "group"
                    else:
                        c = "diff"
                    frames.setdefault((ui, vi), []).append((s.t, c, dd))
            # count collisions/near-misses as events (once per pair)
            n_collide += len(pair_collide - prev_collide_pairs)
            n_near += len(pair_near - prev_near_pairs)
            prev_collide_pairs = set(pair_collide)
            prev_near_pairs = set(pair_near)
        sim.run(max_t, on_step=lambda s: (s._observe_metrics(), hook(s)))
        done = len(sim.done)
        tgt_done += done
        tgt_total += targets
        # === 收集搜索统计 ===
        stats = sim.get_search_stats()
        # 首次发现时间
        first_times = list(stats["first_detect_time"].values())
        if first_times:
            all_first_detect.append(min(first_times))
        # 发现到分配的延迟
        delay_times = list(stats["detect_to_assign_delay"].values())
        if delay_times:
            all_detect_to_assign.append(sum(delay_times) / len(delay_times))
        # 区域覆盖
        for (rx, ry), cov_time in stats["region_coverage"].items():
            all_region_coverage.setdefault((rx, ry), []).append(cov_time)
        # 剩余搜索面积比例
        all_remaining_ratio.append(stats["remaining_search_ratio"])

        if done == targets:
            full += 1
        else:
            # === 失败诊断 ===
            print("")
            print("  === 失败诊断: seed=%d 完成 %d/%d ===" % (sd, done, targets))
            # 获取未消除目标列表
            pending = [tid for tid in sim.tgt if tid not in sim.done]
            # 按进度排序，最惨的先输出
            pending.sort(key=lambda tid: sim.tr.targets[tid].progress(sim.t), reverse=True)
            for tid in pending:
                tg = sim.tgt[tid]
                ct = sim.tr.targets[tid]
                prog = ct.progress(sim.t)
                # 观察员状态
                obs = list(ct.observers)
                live_obs = ct.live_observers(sim.t)
                # 各观察员的 LOS 状态
                obs_los = {}
                for u in obs:
                    ux, uy = sim.uavs[u]
                    los = sim.g.visible(ux, uy, tg["x"], tg["y"])
                    obs_los[u] = los
                # 最近 UAV
                nearest = None
                min_d = 1e9
                for u, (ux, uy) in sim.uavs.items():
                    d = math.hypot(tg["x"] - ux, tg["y"] - uy)
                    if d < min_d:
                        min_d = d
                        nearest = (u, ux, uy, d)
                # 目标已知状态
                known = tg.get("known", False)
                # 输出
                print("  Target %s: known=%s progress=%.1f%%" % (tid, known, 100*prog))
                print("    位置: (%.1f, %.1f)" % (tg["x"], tg["y"]))
                print("    观察员: %s" % (obs if obs else "无"))
                print("    观察员LOS: %s" % ("无" if not obs_los else ", ".join("%s=%s"%(u,"通"if l else "断") for u,l in obs_los.items())))
                print("    有效观察员: %s" % (live_obs if live_obs else "无"))
                print("    最近UAV: %s 距离%.1fm" % (nearest[0], nearest[3]) if nearest else "无")
                print("    消除时间: %.1fs / %.1fs" % (ct.confirm_since if ct.confirm_since else 0, sim.t))
            print("  === 诊断结束 ===")
            print("")

        worst = None
        for pair, seq in frames.items():
            seq.sort()
            for t, c, dd in seq:
                n_ev += 1
                ev_by[c] += 1
                if min_by[c] is None or dd < min_by[c]:
                    min_by[c] = dd
                if dd < D_TIGHT:
                    tight_by[c] += 1
                if worst is None or dd < worst[0]:
                    worst = (dd, t, pair[0], pair[1], c)
                if dd < 0.05 and len(weird) < 15:
                    weird.append((sd, t, pair[0], pair[1], c, dd))
            # 连续贴身时长
            for c in CATS:
                run0 = 0.0
                prev_t = None
                for t, cc, dd in seq:
                    if cc == c and dd < D_TIGHT:
                        run0 = run0 + MT.DT if (prev_t is not None
                                                and abs(t - prev_t - MT.DT) < 1e-6) else MT.DT
                        prev_t = t
                    elif cc == c:
                        run0 = 0.0
                        prev_t = None
                        continue
                    else:
                        continue
                    stuck_by[c] = max(stuck_by[c], run0)
        if worst is not None:
            closest.append((worst[0], sd) + worst[1:])

        print("  seed=%-3d 完成 %d/%d  事件 %4d  最紧 %.2f m (%s t=%.1f %s-%s)"
              % (sd, done, targets, len(frames) and sum(len(v) for v in frames.values()),
                 worst[0] if worst else float("nan"),
                 worst[4] if worst else "-", worst[1] if worst else 0.0,
                 worst[2] if worst else "-", worst[3] if worst else "-"),
              flush=True)

    n = len(seeds)
    print("-" * 96)
    print("汇总（n=%d, 每局 %d 目标）SEP=%.1f  PUSH_MAX=%.0f°"
          % (n, targets, sep, MT.SEP_PUSH_MAX_DEG))
    print("  UAV-UAV碰撞(<0.90m) : %6d 次" % n_collide)
    print("  UAV-UAV危险接近     : %6d 次 (0.90~1.20m)" % n_near)
    print("  UAV-建筑碰撞        : %6d 次 (free True->False)" % n_bldg)
    print("  LOS中断次数         : %6d 次 (visible True->False)" % n_los)

    print("  15s成功率(全有全无) : %6.1f%% (%d/%d)" % (100.0 * full / n, full, n))
    print("  目标级完成率        : %6.1f%% (%d/%d)"
          % (100.0 * tgt_done / tgt_total, tgt_done, tgt_total))
    print("  同目标近距事件      : %6d 次 (平均每局 %.0f)" % (n_ev, n_ev / n))
    print("")
    print("  %-8s %8s %10s %10s" % ("类别", "事件数", "最小间距", "贴身时长"))
    for c in CATS:
        mv = min_by[c]
        print("  %-8s %8d %10s %8.1f s"
              % (c, ev_by[c], "%.3f m" % mv if mv is not None else "无事件",
                 stuck_by[c]))
    print("")
    print("  全场最小间距（不分阶段）: %s"
          % ("%.3f m" % min(c[0] for c in closest) if closest else "无事件"))

    # === 搜索阶段统计输出 ===
    print("")
    print("  === 搜索阶段统计 ===")
    if all_first_detect:
        print("    首次发现时间: 平均 %.1fs, 最小 %.1fs, 最大 %.1fs"
              % (sum(all_first_detect)/len(all_first_detect),
                 min(all_first_detect), max(all_first_detect)))
    else:
        print("    首次发现时间: 无数据")
    if all_detect_to_assign:
        print("    发现到分配延迟: 平均 %.1fs, 最小 %.1fs, 最大 %.1fs"
              % (sum(all_detect_to_assign)/len(all_detect_to_assign),
                 min(all_detect_to_assign), max(all_detect_to_assign)))
    else:
        print("    发现到分配延迟: 无数据")
    # 区域覆盖统计
    if all_region_coverage:
        all_cov = [v for vs in all_region_coverage.values() for v in vs]
        print("    区域覆盖时间: 平均 %.1fs, 最小 %.1fs, 最大 %.1fs"
              % (sum(all_cov)/len(all_cov), min(all_cov), max(all_cov)))
    if all_remaining_ratio:
        print("    剩余搜索面积比例: 平均 %.1f%%"
              % (100.0 * sum(all_remaining_ratio)/len(all_remaining_ratio)))
    print("")

    print("  最紧的 5 个事件现场（用于定位残余最小值来自谁）:")
    for d, sd, t, u, v, c in sorted(closest)[:5]:
        print("     d=%.3f m  seed=%-3d t=%6.1f  %s-%s  [%s]" % (d, sd, t, u, v, c))
    if weird:
        print("")
        print("  dd<0.05 样本:")
        for sd, t, u, v, c, dd in weird:
            print("     seed=%-3d t=%6.1f  %s-%s  d=%.3f  [%s]" % (sd, t, u, v, dd, c))
    return dict(full=full, n=n, tgt_done=tgt_done, tgt_total=tgt_total,
                n_ev=n_ev, mind=min((c[0] for c in closest), default=float("nan")),
                stuck=max(stuck_by.values(), default=0.0), ev_by=ev_by,
                min_by=min_by)


if __name__ == "__main__":
    with open(out, "w", encoding="utf-8") as fh:
        real = sys.stdout
        sys.stdout = fh
        try:
            print("########## SEP=%.1f PUSH=%.0f PHASE=%d FR=%.1f seeds 1..%d ##########"
                  % (sep, MT.SEP_PUSH_MAX_DEG, MT.SEARCH_PHASE, MT.FRIEND_SAFE, nmax),
                  flush=True)
            run(list(range(1, nmax + 1)))
        finally:
            sys.stdout = real
    print("SEP=%.1f 完成 -> %s" % (sep, out), flush=True)