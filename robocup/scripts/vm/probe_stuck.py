#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""确认 67 s 卡死的根因：_search_step 的航点推进在 len(path)-1 处卡住。

假设：while 的条件是 `wp_i < len(path) - 1`，于是 wp_i 最多只能到
len(path)-2，**永远到不了末航点**。若某机在 path[wp_i] 的 3 m 内而
wp_i == len(path)-2，就永远瞄着同一个点 → speed_toward 返回原地 → 冻结。
不修改被测代码，只 dump 内部状态。
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mission_time as MT
import task_allocator as TA
from strategy_compare import MapGrid


def main(seed=9, mode="bearing", max_t=300.0):
    g = MapGrid(TA.METADATA)
    MT.SLOT_MODE = mode
    sim = TA.AllocSim(g, strategy="dynamic", targets=6, seed=seed)

    rows = []
    sim.run(max_t, on_step=lambda s: (rows.append(
        (s.t, {u: tuple(p) for u, p in s.uavs.items()},
         dict(s._engaged()),
         {u: s._search_wp.get(u) for u in s.uavs},
         {u: len(s._search_paths.get(u) or []) for u in s.uavs})),
        s._observe_metrics()))

    # 找冻结点：位置连续 5 s 完全不变
    frozen = {}
    for t, pos, eng, wp, plen in rows:
        for u, p in pos.items():
            key = u
            prev = frozen.get(key)
            if prev and abs(prev[1][0] - p[0]) < 1e-9 and abs(prev[1][1] - p[1]) < 1e-9:
                frozen[key] = (prev[0], p, prev[2] + MT.DT, eng.get(u), wp.get(u), plen.get(u))
            else:
                frozen[key] = (t, p, 0.0, eng.get(u), wp.get(u), plen.get(u))

    print("=" * 96)
    print("mode=%s seed=%d  冻结 >=3 s 的机：" % (mode, seed))
    for u in sorted(frozen):
        t0, p, dur, tid, wp, plen = frozen[u]
        if dur >= 3.0:
            print("  %-7s 冻结 %5.1f s  位置=(%.2f,%.2f)  目标=%s  "
                  "wp_idx=%s / len(path)=%s" %
                  (u, dur, p[0], p[1], tid or "-", wp, plen))
            path = sim._search_paths.get(u) or []
            if path and wp is not None:
                aim = path[wp]
                print("          wp_idx=%d 的航点 = (%.2f,%.2f)  距冻结位置 %.3f m"
                      " （<3.0 会再次触发跳过）"
                      % (wp, aim[0], aim[1], math.hypot(aim[0] - p[0], aim[1] - p[1])))
                print("          末航点 idx=%d = (%.2f,%.2f)  —— wp_idx 最大只能到 %d"
                      % (len(path) - 1, path[-1][0], path[-1][1], len(path) - 2))
                # 关键判据：冻结点的 free() 情况。speed_toward 只在**所有**
                # 候选方向都不可通行时才原地不动 —— 所以若此处 free=False 或
                # 被围死，就是"卡在几何里"，与航点推进无关。
                print("          free(冻结位置) = %s" % g.free(p[0], p[1]))
                blocked = []
                for ang in range(0, 360, 30):
                    r = math.radians(ang)
                    qx, qy = p[0] + 0.5 * math.cos(r), p[1] + 0.5 * math.sin(r)
                    if not g.free(qx, qy):
                        blocked.append(ang)
                print("          周围 0.5 m、每 30° 采一个方向：不可通行的角度 %s"
                      % (blocked if blocked else "无（四周开阔！）"))
                # 朝向航点的方向能不能走
                aim = path[wp]
                ux_, uy_ = aim[0] - p[0], aim[1] - p[1]
                nn = math.hypot(ux_, uy_) or 1.0
                qx, qy = p[0] + ux_ / nn * 0.5, p[1] + uy_ / nn * 0.5
                print("          朝航点走 0.5 m 的落点 (%.2f,%.2f) free=%s"
                      % (qx, qy, g.free(qx, qy)))
    print("=" * 96)


if __name__ == "__main__":
    for mode in ("chase", "bearing", "dedup"):
        main(seed=int(sys.argv[1]) if len(sys.argv) > 1 else 9, mode=mode)
