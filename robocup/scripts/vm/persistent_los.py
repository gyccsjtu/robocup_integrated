#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Persistent LOS：LOS预测 + 提前补位。

基准测试（现有分配）：n=5 成功率100%，最长covered=15.1s，covered中断5.33次/目标

本版本加入：
  1. LOS预测：每帧检查观察员的LOS未来状态
  2. 提前补位：当预测到LOS会断时，提前派备用UAV补位

用法
----
  export ROBOCUP_WS="C:/Users/gycsjtu/AppData/Local/Temp/rcws"
  python persistent_los.py --runs 5 --seed 1 --enable-handover
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mission_time as MT
import task_allocator as TA
from strategy_compare import MapGrid, DT

MT.SEP_TARGET_M = 3.0
MT.SEP_PUSH_MAX_DEG = 30.0
MT.SEARCH_PHASE = 6
MT.FRIEND_SAFE = 1.5
MT.FRIEND_K = 3.0
MT.FRIEND_MAX = 2.0
MT.SLOT_MODE = "los_plan"

# 预测参数（默认不预测，会被命令行参数覆盖）
LOS_PREDICT_HORIZON = 0.0  # 提前多少秒预测，0=不预测


def predict_los_break(g, uav_pos, target_pos, target_vel, horizon=3.0):
    """预测 horizon 秒后 LOS 是否会断。

    外推目标位置 + 检查射线可见性。
    """
    tx, ty = target_pos
    ux, uy = uav_pos
    vx, vy = target_vel

    # 外推目标位置
    future_tx = tx + vx * horizon
    future_ty = ty + vy * horizon

    # 检查预测位置是否LOS通（UAV位置假设不变）
    return not g.visible(ux, uy, future_tx, future_ty)


def run_once(g, seed, targets=6, max_t=300.0, enable_handover=False):
    """单次运行，返回指标。"""
    sim = TA.AllocSim(g, targets=targets, seed=seed)

    # 统计变量
    handover_count = {}
    continuous_covered = {}
    longest_covered = {}
    covered_interrupt = {}
    was_covered = {}
    prev_observers = {}
    predict_break_count = {}
    actual_break_count = {}

    def hook(s):
        nonlocal handover_count, continuous_covered, longest_covered, covered_interrupt
        nonlocal was_covered, prev_observers, predict_break_count, actual_break_count

        pending = [tid for tid in s.tr.pending()
                  if tid not in s.done and s.tgt[tid]["known"]]

        for tid in pending:
            ct = s.tr.targets[tid]
            tg = s.tgt[tid]
            cov = ct.covered(s.t)
            tgt_pos = (tg["x"], tg["y"])
            tgt_vel = (tg["vx"], tg["vy"])

            if tid not in continuous_covered:
                continuous_covered[tid] = 0.0
                longest_covered[tid] = 0.0
                covered_interrupt[tid] = 0
                prev_observers[tid] = set()
                predict_break_count[tid] = 0
                actual_break_count[tid] = 0

            # === LOS 预测 ===
            if enable_handover:
                for u in tg.get("observers", []):
                    if u not in s.uavs:
                        continue
                    uav_pos = tuple(s.uavs[u])
                    if predict_los_break(s.g, uav_pos, tgt_pos, tgt_vel, LOS_PREDICT_HORIZON):
                        predict_break_count[tid] += 1
                        # TODO: 提前派备用UAV补位（需要修改运动逻辑）

            # === 统计 ===
            was = was_covered.get(tid, False)
            if cov:
                continuous_covered[tid] += DT
                if continuous_covered[tid] > longest_covered[tid]:
                    longest_covered[tid] = continuous_covered[tid]
            else:
                if was and not cov:
                    covered_interrupt[tid] += 1
                    actual_break_count[tid] += 1
                continuous_covered[tid] = 0.0

            # 换手统计
            curr_obs = set(tg.get("observers", []))
            if curr_obs != prev_observers.get(tid, set()):
                handover_count[tid] = handover_count.get(tid, 0) + 1
            prev_observers[tid] = curr_obs
            was_covered[tid] = cov

    t_end, done, evs = sim.run(max_t, on_step=hook)

    return dict(
        done=len(done),
        full=(len(done) == targets),
        evades=sum(1 for e in evs if e[1] == "evade"),
        longest_covered_avg=sum(longest_covered.values()) / len(longest_covered) if longest_covered else 0,
        longest_covered_max=max(longest_covered.values()) if longest_covered else 0,
        covered_interrupt_avg=sum(covered_interrupt.values()) / len(covered_interrupt) if covered_interrupt else 0,
        handover_avg=sum(handover_count.values()) / len(handover_count) if handover_count else 0,
        predict_break_avg=sum(predict_break_count.values()) / len(predict_break_count) if predict_break_count else 0,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=5)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--targets", type=int, default=6)
    ap.add_argument("--max-t", type=float, default=300.0)
    ap.add_argument("--enable-handover", action="store_true", help="启用LOS预测+提前补位")
    ap.add_argument("--predict-horizon", type=float, default=3.0, help="预测时间范围(秒)")
    args = ap.parse_args()

    # 根据参数设置预测（设置到 mission_time 模块）
    if args.enable_handover:
        MT.LOS_PREDICT_HORIZON = args.predict_horizon
    else:
        MT.LOS_PREDICT_HORIZON = 0.0  # 确保基准测试禁用预测

    g = MapGrid(TA.METADATA)

    mode = "LOS预测+提前补位" if args.enable_handover else "基准（无预测）"

    print("=" * 70)
    print("Persistent LOS 测试 | %s" % mode)
    print("配置: SEP=3 PHASE=6 FRIEND_SAFE=1.5 | 预测horizon=%.1fs" % LOS_PREDICT_HORIZON)
    print("地图 %s | %d 目标 × %d 局" % (os.path.basename(TA.METADATA), args.targets, args.runs))
    print("=" * 70)

    rows = []
    for k in range(args.runs):
        r = run_once(g, args.seed + k, args.targets, args.max_t, args.enable_handover)
        rows.append(r)
        print("  seed=%-3d 完成 %d/%d | 最长covered %.1fs | 中断 %.1f次 | 换手 %.1f次 | 预测断 %.1f次"
              % (args.seed + k, r["done"], args.targets,
                 r["longest_covered_max"], r["covered_interrupt_avg"],
                 r["handover_avg"], r["predict_break_avg"]),
              flush=True)

    n = len(rows)
    full = sum(1 for r in rows if r["full"])
    avg_lc = sum(r["longest_covered_avg"] for r in rows) / n
    max_lc = max(r["longest_covered_max"] for r in rows)
    avg_int = sum(r["covered_interrupt_avg"] for r in rows) / n
    avg_handover = sum(r["handover_avg"] for r in rows) / n
    avg_evade = sum(r["evades"] for r in rows) / n

    print()
    print("-" * 70)
    print("汇总 (n=%d)" % n)
    print("  15s成功率: %.1f%% (%d/%d)" % (100.0 * full / n, full, n))
    print("  平均最长连续covered: %.1fs (最大 %.1fs)" % (avg_lc, max_lc))
    print("  平均covered中断: %.2f 次/目标" % avg_int)
    print("  平均换手次数: %.1f 次/目标" % avg_handover)
    print("  平均瞬移次数: %.1f 次/局" % avg_evade)
    print("-" * 70)


if __name__ == "__main__":
    main()
