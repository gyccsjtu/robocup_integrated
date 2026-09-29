#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第一步：抓"同目标两机重合"的现场。

按用户给的三步走：
  1. 在编组站位计算附近打 FORM / FALLBACK 日志
  2. 只跑**一局**，找到 distance ≈ 0.01 m 的那对机
  3. 判读：free=False 走回退（情况A）还是 free=True 但最终重合（情况B）

两个阶段（避免日志淹掉）：
  recon  —— 便宜：扫种子，找第一处「同目标 + 距离<1.5 m」，打印进入前的航迹
  trace  —— 只对命中那一局、命中时刻附近开日志窗口

关键：日志必须能区分**实际代码的两条回退分支**。本版 mission_time 的回退是

    if not self.g.free(gx, gy):
        gx, gy = (tx, ty) if d > TRACK_STANDOFF else (ux, uy)

即：**远的机退到"目标本体"**（运动学陷阱：远处直线奔向目标，一旦距离落到
8 m 以内就永久停在 8 m 那一圈），**近的机退到"原地不动"**。两架都退成不动
→ 恒等运动 → 永久锁死。日志把 branch 打出来。
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mission_time as MT
import task_allocator as TA
from strategy_compare import MapGrid

D_HIT = 1.5
# chase / dedup / bearing —— 用环境变量 SLOT_MODE 覆盖，便于 A/B 同脚本对比
SLOT_MODE = os.environ.get("SLOT_MODE", "chase")


def _find_hit(sim):
    """返回该局里第一处「同目标 + 距离<D_HIT」的事件。"""
    eng_hist = []

    def hook(s):
        eng_hist.append((s.t, dict(s._engaged()), {u: tuple(p) for u, p in s.uavs.items()}))

    sim.run(300.0, on_step=lambda s: (s._observe_metrics(), hook(s)))

    for t, eng, pos in eng_hist:
        for ui in sorted(pos):
            for vi in sorted(pos):
                if vi <= ui:
                    continue
                if eng.get(ui) is None or eng.get(ui) != eng.get(vi):
                    continue
                d = math.hypot(pos[ui][0] - pos[vi][0], pos[ui][1] - pos[vi][1])
                if d < D_HIT:
                    return dict(t=t, u=ui, v=vi, d=d, tid=eng[ui])
    return None


def metrics(seeds, targets=6, max_t=300.0):
    """第四步：只跑小实验，看用户指定的 4 个指标。

      1. 15s 成功率
      2. 同目标近距事件数（<1.5 m 且两机同目标）
      3. 最小机间距
      4. 最长同点/卡死时间（连续 <0.5 m 的最长时长）
    """
    g = MapGrid(TA.METADATA)
    MT.SLOT_MODE = SLOT_MODE
    print("=" * 100)
    print("小实验：SLOT_MODE=%s  FRIEND_SAFE=%.1f  ANGLE_DEDUP=%.0f  "
          "TRACK_SPLIT=%.0f°  n=%d 目标=%d"
          % (MT.SLOT_MODE, MT.FRIEND_SAFE, MT.ANGLE_DEDUP_DEG,
             MT.TRACK_SPLIT_DEG, len(seeds), targets))
    print("=" * 100)

    ok = 0
    same_n, mind_all, stuck_all, close_all = [], [], [], []
    for sd in seeds:
        sim = TA.AllocSim(g, strategy="dynamic", targets=targets, seed=sd)
        # 逐帧扫：同目标近距段数 / 最小间距 / 卡死时长
        eng_prev = {}

        def hook(s):
            eng = s._engaged()
            pos = {u: tuple(p) for u, p in s.uavs.items()}
            for ui in sorted(pos):
                for vi in sorted(pos):
                    if vi <= ui:
                        continue
                    dd = math.hypot(pos[ui][0] - pos[vi][0],
                                    pos[ui][1] - pos[vi][1])
                    close_all.append((s.t, ui, vi, dd))
                    if dd < D_HIT and eng.get(ui) is not None and eng.get(ui) == eng.get(vi):
                        same_n.append((sd, s.t, ui, vi, dd))
        sim.run(max_t, on_step=lambda s: (s._observe_metrics(), hook(s)))

        done = len(sim.done)
        if done == targets:
            ok += 1
        # 最小间距：该局全场最小（只看近距样本，省内存）
        mn = min((d for _t, _u, _v, d in close_all), default=float("nan"))
        mind_all.append(mn)

        # 卡死：把 <0.5 m 的帧按机对串起来，找最长连续时长
        bypair = {}
        for t, u, v, d in close_all:
            if d < 0.5:
                bypair.setdefault((u, v), []).append(t)
        longest = 0.0
        for pair, ts in bypair.items():
            ts.sort()
            run0 = 0.0
            for i in range(1, len(ts)):
                if abs(ts[i] - ts[i - 1] - MT.DT) < 1e-6:
                    run0 += MT.DT
                else:
                    run0 = 0.0
                longest = max(longest, run0 + MT.DT)
        stuck_all.append(longest)
        close_all.clear()
        print("  seed=%-3d 完成 %d/%d  同目标近距 %3d 次  最小间距 %.2f m  "
              "最长<0.5m 伴飞 %.1f s"
              % (sd, done, targets,
                 sum(1 for e in same_n if e[0] == sd), mn, longest))

    n = len(seeds)
    per_min = []
    print("-" * 100)
    print("汇总（n=%d）：" % n)
    print("  15s 成功率        : %6.1f%% (%d/%d)" % (100.0 * ok / n, ok, n))
    print("  同目标近距事件    : %6d 次" % len(same_n))
    print("  最小机间距        : %6.2f m" % min(mind_all, default=float("nan")))
    print("  最长<0.5m 伴飞    : %6.1f s" % max(stuck_all, default=0.0))
    print("=" * 100)
    return 0


def recon(seeds):
    g = MapGrid(TA.METADATA)
    MT.SLOT_MODE = SLOT_MODE
    print("=" * 100)
    print("阶段 recon：扫种子找「同目标近距」现场（目标 6，单局 300 s）"
          " SLOT_MODE=%s FRIEND_SAFE=%.1f ANGLE_DEDUP=%.0f"
          % (MT.SLOT_MODE, MT.FRIEND_SAFE, MT.ANGLE_DEDUP_DEG))
    print("=" * 100)
    for sd in seeds:
        sim = TA.AllocSim(g, strategy="dynamic", targets=6, seed=sd)
        hit = _find_hit(sim)
        if hit:
            print("  seed=%-3d  命中 t=%.1f  %s-%s  d=%.3f m  同目标=%s"
                  % (sd, hit["t"], hit["u"], hit["v"], hit["d"], hit["tid"]))
            # 进入前的航迹（sim._trail 是最近 DIAG_TRAIL 帧）
            for u in (hit["u"], hit["v"]):
                tr = sim._trail.get(u, [])
                print("      %s 最近 %d 帧: %s" % (
                    u, len(tr),
                    " ".join("(%.2f,%.2f)" % p for _t, p in tr)))
            return sd, hit
        print("  seed=%-3d  无命中" % sd)
    return None, None


def trace(seed, hit):
    g = MapGrid(TA.METADATA)
    u, v = hit["u"], hit["v"]
    # 只在命中前 6 s 到命中后 2 s 之间开日志，并只跟这两架
    MT.FORM_TRACE_UAVS = {u, v}
    MT.FORM_TRACE_T0 = max(0.0, hit["t"] - 6.0)
    MT.FORM_TRACE_T1 = hit["t"] + 2.0
    print("")
    print("=" * 100)
    print("阶段 trace：seed=%d 窗口 t∈[%.1f, %.1f]  只跟 %s / %s"
          % (seed, MT.FORM_TRACE_T0, MT.FORM_TRACE_T1, u, v))
    print("=" * 100)
    sim = TA.AllocSim(g, strategy="dynamic", targets=6, seed=seed)

    def hook(s):
        s._observe_metrics()
        if MT.FORM_TRACE_T0 <= s.t <= MT.FORM_TRACE_T1:
            if s.t % 1.0 < 1e-9:      # 每秒打一行两机距离，便于对齐
                p, q = s.uavs[u], s.uavs[v]
                print("[DIST] t=%.1f %s-%s = %.3f m  tgt %s/%s"
                      % (s.t, u, v, math.hypot(p[0] - q[0], p[1] - q[1]),
                         s._engaged().get(u), s._engaged().get(v)), flush=True)

    sim.run(300.0, on_step=hook)
    return 0


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--metrics":
        n = int(sys.argv[2]) if len(sys.argv) > 2 else 10
        sys.exit(metrics(list(range(1, n + 1))))
    if len(sys.argv) > 1 and sys.argv[1] == "--trace":
        sys.exit(trace(int(sys.argv[2]), dict(t=float(sys.argv[3]), u=sys.argv[4],
                                              v=sys.argv[5], d=0.0, tid=sys.argv[6])))
    sd, hit = recon(list(range(1, 11)))
    if sd is None:
        print("所有种子都没出现同目标近距事件。")
        sys.exit(0)
    print("")
    print(">>> 复现命令：python3 -u probe_form.py --trace %d %.1f %s %s %s"
          % (sd, hit["t"], hit["u"], hit["v"], hit["tid"]))
