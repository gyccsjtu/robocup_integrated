#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证 bearing 模式为什么更差：怀疑"目标点随自身方位旋转 → 环绕/振铃"。

如果假设成立，被卡住的那对机会**绕着目标转圈**：它们相对目标的方位角
持续变化，而不是收敛到某个固定方位。chase 模式则相反（方位基本恒定）。

只读仿真状态，不改被测代码。
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mission_time as MT
import task_allocator as TA
from strategy_compare import MapGrid


def run(seed, mode, max_t=300.0):
    g = MapGrid(TA.METADATA)
    MT.SLOT_MODE = mode
    sim = TA.AllocSim(g, strategy="dynamic", targets=6, seed=seed)

    rows = []

    def hook(s):
        eng = s._engaged()
        rows.append((s.t, {u: tuple(p) for u, p in s.uavs.items()},
                     dict(eng), {t: (g2["x"], g2["y"]) for t, g2 in s.tgt.items()}))
        s._observe_metrics()

    sim.run(max_t, on_step=hook)
    return rows, sim


def analyse(seed, mode):
    rows, sim = run(seed, mode)
    # 找全场持续最久的 <0.5 m 机对
    best = (0.0, None, None)
    cur = {}
    for t, pos, eng, tg in rows:
        for ui in sorted(pos):
            for vi in sorted(pos):
                if vi <= ui:
                    continue
                d = math.hypot(pos[ui][0] - pos[vi][0], pos[ui][1] - pos[vi][1])
                if d < 0.5:
                    key = (ui, vi)
                    t0 = cur.get(key)
                    if t0 is None:
                        cur[key] = t
                    dur = t - cur[key]
                    if dur > best[0]:
                        best = (dur, key, cur[key])
                else:
                    cur.pop((ui, vi), None)
    dur, pair, t0 = best
    print("=" * 96)
    print("mode=%-8s seed=%d  最长 <0.5m 机对 %s  时长 %.1f s  起始 t=%.1f"
          % (mode, seed, pair, dur, t0 if t0 else -1))
    if pair is None:
        return
    ui, vi = pair
    print("  两机的编组状态与位置（每 1 s 采一次）：")
    print("  %-7s %-9s %-9s %-24s %-24s %s"
          % ("t", "A->目标", "B->目标", "A位置", "B位置", "间距m"))
    for t, pos, eng, tg in rows:
        if t < t0 - 0.5 or t > t0 + min(dur, 16.0):
            continue
        if abs((t - round(t)) % 1.0) > 1e-6 and abs((t - round(t)) % 1.0 - 1.0) > 1e-6:
            continue
        d = math.hypot(pos[ui][0] - pos[vi][0], pos[ui][1] - pos[vi][1])
        print("  %-7.1f %-9s %-9s (%7.2f,%7.2f)%s (%7.2f,%7.2f)%s %-.3f"
              % (t, eng.get(ui) or "-", eng.get(vi) or "-",
                 pos[ui][0], pos[ui][1], "  ", pos[vi][0], pos[vi][1], "  ", d))


if __name__ == "__main__":
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else 9
    for mode in ("chase", "bearing"):
        analyse(seed, mode)
