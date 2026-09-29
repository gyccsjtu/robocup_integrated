#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""判定：(-45.67,-3.55) 是个**封闭口袋**还是贪婪法进不去的有出口区域？

用 BFS 在栅格上做可通行连通域分析：
  - 从这个点出发，能走到多远？能不能走到它的航点？
  - 这个连通域有多大？是不是被围死的小袋？
这决定了"改 speed_toward"能不能解决问题 —— 如果连通域本身就把
航点排除在外，那不是局部步进算法能修的。
"""
import math
import os
import sys
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from strategy_compare import MapGrid, METADATA


def cell_center(g, c):
    return (g.origin[0] + (c[0] + 0.5) * g.res,
            g.origin[1] + (c[1] + 0.5) * g.res)


def bfs(g, start, limit=200000):
    """从 start 格出发的 4 连通可通行连通域，返回 (格子集合, 到每格步数)。"""
    seen = {start: 0}
    q = deque([start])
    while q and len(seen) < limit:
        c = q.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (c[0] + dx, c[1] + dy)
            if n in seen or not g.free_cell(n):
                continue
            seen[n] = seen[c] + 1
            q.append(n)
    return seen


def main():
    g = MapGrid(METADATA)
    for (px, py, wx, wy) in ((-45.67, -3.55, -33.67, 8.80),
                             (-45.73, -3.52, 2.00, 44.00)):
        c = g.to_cell(px, py)
        wc = g.to_cell(wx, wy)
        print("=" * 92)
        print("起点 (%.2f,%.2f) 格%s   free=%s" % (px, py, c, g.free(px, py)))
        print("航点 (%.2f,%.2f) 格%s  free=%s" % (wx, wy, wc, g.free(wx, wy)))

        seen = bfs(g, c)
        print("  起点所在连通域大小: %d 格 (= %.1f m²)" % (len(seen), len(seen) * g.res ** 2))

        if wc in seen:
            print("  **航点在同一连通域内**，步数 %d（欧氏约 %.1f m）→ "
                  "有通路，是局部算法没找到，可以修。"
                  % (seen[wc], math.hypot(wx - px, wy - py)))
        else:
            print("  **航点不在同一连通域** → 图论上就走不过去！"
                  "这不是 speed_toward 能修的，是上层路径/纵带划分的问题。")

        # 连通域的形状：包围盒 + 从起点最远能到哪
        xs = [k[0] for k in seen]
        ys = [k[1] for k in seen]
        print("  连通域包围盒: x[%d,%d] y[%d,%d]  即 x[%.1f,%.1f] y[%.1f,%.1f] m"
              % (min(xs), max(xs), min(ys), max(ys),
                 g.origin[0] + min(xs) * g.res, g.origin[0] + (max(xs) + 1) * g.res,
                 g.origin[1] + min(ys) * g.res, g.origin[1] + (max(ys) + 1) * g.res))
        # 域内离起点最远的格
        far = max(seen, key=lambda k: seen[k])
        fx, fy = cell_center(g, far)
        print("  离起点最远可达: (%.2f,%.2f)  步数 %d  ≈ %.1f m"
              % (fx, fy, seen[far], seen[far] * g.res))
        # 若航点在同域，给它一条 BFS 最短路，看和贪婪方向差多远
        if wc in seen:
            print("  → 存在最短路，长度 %d 步；说明贪婪法被口袋口的方向选择卡住。"
                  % seen[wc])
    print("=" * 92)


if __name__ == "__main__":
    main()
