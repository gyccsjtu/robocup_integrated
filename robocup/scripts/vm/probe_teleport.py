#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一次性探针：判定 37 m 单帧位移是真异常还是仪器问题。

不修改 task_allocator.py / mission_time.py，只读它们的公开状态。

三个自洽性不变式（都在**同一帧内**成立，与采样顺序无关）：
  I1. _diag_pair 收到的 pu 必须等于同帧 append 的 trail_u[-1][1]
      （两者都取 tuple(self.uavs[u])，同帧必然相等；不等 = 仪器问题）
  I2. 任意相邻两帧，同一架机的位移 <= UAV_TRACK_SPEED * DT
      （mission_time 每条写 self.uavs 的路径都过 speed_toward 限幅；
       UAV_SEARCH_SPEED 可能更大，所以上限取两者的 max）
  I3. t 严格按 DT 递增，不跳帧
"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mission_time as MT
import task_allocator as TA
from strategy_compare import MapGrid, METADATA

LIM = max(MT.UAV_TRACK_SPEED, MT.UAV_SEARCH_SPEED) * MT.DT


def probe(space_on, n_sims=8, seed=2024):
    MT.FRIEND_SAFE = 3.0 if space_on else 0.0
    MT.ANGLE_DEDUP_DEG = 60.0 if space_on else 0.0
    MT.SLOT_MODE = "dedup" if space_on else "chase"

    g = MapGrid(METADATA)
    bad_self, bad_step, bad_time = [], [], []
    n_diag = 0

    print("=" * 78)
    print("探针 space_on=%s  单帧位移上限 = max(track %.1f, search %.1f) * DT %.2f"
          " = %.3f m"
          % (space_on, MT.UAV_TRACK_SPEED, MT.UAV_SEARCH_SPEED, MT.DT, LIM))
    print("=" * 78)

    for k in range(n_sims):
        rng = random.Random(seed + k)
        sim = TA.AllocSim(g, strategy="dynamic", targets=6, seed=seed + k)
        # 逐帧追踪：包一层 on_step，自己独立记 uav 位置，作为第三方参照
        mine = {}
        hist = {}          # uav -> [(t, pos)]，我独立记录的真值
        prev_t = [0.0]

        def hook(s):
            nonlocal n_diag
            # before = 我**独立**记录的上一帧末位置，不读 sim 的任何内部状态
            before = {u: mine.get(u) for u in s.uavs}

            # I3: 时间递增
            if abs((s.t - prev_t[0]) - MT.DT) > 1e-9:
                bad_time.append((k, s.t, prev_t[0]))

            # I2: 逐机位移（第三方真值，不依赖 sim 的 _prev）
            for u, p in s.uavs.items():
                q = before.get(u)
                if q is not None:
                    d = math.hypot(p[0] - q[0], p[1] - q[1])
                    if d > LIM + 1e-6:
                        bad_step.append((k, round(s.t, 2), u, round(q[0], 2),
                                         round(q[1], 2), round(p[0], 2),
                                         round(p[1], 2), round(d, 2)))
                mine[u] = tuple(p)
                hist.setdefault(u, []).append((round(s.t, 4), tuple(p)))
            prev_t[0] = s.t

            # **必须转发给 _observe_metrics** —— AllocSim 走的是
            # `on_step=lambda s: s._observe_metrics()`，只传自己的 hook 会把它
            # 顶掉，diag 永远是空的（第一版探针就踩了这个，I1 检查了 0 帧）。
            n0 = len(s.diag)
            s._observe_metrics()

            # I1: diag 记的 pu/pv 就是**本帧**位置（_diag_pair 读的是当前
            # self.uavs），所以该等于我第三方记录的当前值 mine[ukey]。
            # 我在这里连错两次参照系：先跟 _trail 末帧比（差一帧，0.469 m），
            # 又跟 before（上一帧）比 —— 两次都是**检查写错**，不是数据错。
            # 教训：自比式的自洽检查很容易写成恒真或恒假，必须有外部真值。
            for e in s.diag[n0:]:
                n_diag += 1
                for key, ukey in (("pu", "u"), ("pv", "v")):
                    q = mine.get(e[ukey])
                    if q is None:
                        continue
                    got = e[key]
                    gap = math.hypot(got[0] - q[0], got[1] - q[1])
                    if gap > LIM + 1e-6:
                        bad_self.append((k, round(e["t"], 2), e[ukey],
                                         "(%.2f,%.2f)" % (got[0], got[1]),
                                         "(%.2f,%.2f)" % (q[0], q[1]),
                                         round(gap, 3)))

        sim.run(max_t=90.0, on_step=hook)

        print("  sim %2d: 段 %2d, diag 帧 %4d, 累计单帧超限 %d"
              % (k, len(sim.segments), len(sim.diag), len(bad_step)))

    print("-" * 78)
    print("I1 diag 的 pu/pv == 第三方真值(本帧): %s  (检查 %d 帧)"
          % ("通过" if not bad_self else "**失败 %d 处**" % len(bad_self), n_diag))
    for b in bad_self[:5]:
        print("     sim%d t=%.2f %s: diag=%s 真值=%s 差=%s m" % b)
    print("I2 单帧位移 <= %.3f m        : %s"
          % (LIM, "通过（无物理不可能位移）" if not bad_step
             else "**失败 %d 处**" % len(bad_step)))
    for b in bad_step[:10]:
        print("     sim%d t=%.2f %s: (%.2f,%.2f) -> (%.2f,%.2f) = %.2f m" % b)
    print("I3 时间步恒为 DT              : %s"
          % ("通过" if not bad_time else "**失败 %d 处**" % len(bad_time)))
    for b in bad_time[:5]:
        print("     sim%d t=%.2f 上一帧 t=%.2f" % b)
    print("=" * 78)
    return bad_self, bad_step, bad_time


if __name__ == "__main__":
    print("\n########## baseline (space off) ##########")
    probe(False)
    print("\n########## space on ##########")
    probe(True)
