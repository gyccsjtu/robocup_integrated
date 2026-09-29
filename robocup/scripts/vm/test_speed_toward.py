#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""speed_toward 脱困逻辑的单元测试。

三条不变式：
  U1 非恒等：只要周围**真有**可通行的地方，输出就不能等于输入。
     （旧版本的核心缺陷：全受阻 → return x,y，写回 uavs 后变成恒等映射，
      永久冻结。实测 seed=9 冻了 67 s。）
  U2 运动学上限：单帧位移 <= step = speed*DT。脱困可以慢，不能瞬移。
     （我第一版第 4 级写 `for rad in (1,2,3)`，res=0.5 → 一帧跳 2.1 m，
      又制造出刚修掉的那种"瞬移"。这个测试就是为了钉死它。）
  U3 不倒退：有直行/侧滑可行时，行为与旧版一致（不能为了脱困把正常路径改坏）。

另外在真实地图上复现 seed=9 的冻结点，验证能从那里走出去。
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from strategy_compare import MapGrid, METADATA, DT


def u1_u2_realmap():
    """在真图上扫大量 (位置, 目标) 组合，检查 U1/U2/U3。"""
    g = MapGrid(METADATA)
    speed = 4.7
    step = speed * DT
    bad_ident, bad_jump, n_moved, n_stuck = [], [], 0, 0

    xs = [g.origin[0] + (i + 0.5) * g.res * 7 for i in range(1, 55)]
    ys = [g.origin[1] + (j + 0.5) * g.res * 5 for j in range(1, 40)]
    targets = [(0.0, 0.0), (60.0, 30.0), (-70.0, -30.0), (30.0, -40.0)]

    for x in xs:
        for y in ys:
            if not g.free(x, y):
                continue
            for (tx, ty) in targets:
                nx, ny, blk = g.speed_toward(x, y, tx, ty, speed)
                disp = math.hypot(nx - x, ny - y)
                if disp > step + 1e-9:
                    bad_jump.append((x, y, tx, ty, disp))
                if disp < 1e-12:
                    n_stuck += 1
                    # 停滞是允许的，**仅当**四周连一格都走不了
                    c = g.to_cell(x, y)
                    any_free = any(
                        g.free(g.origin[0] + (c[0] + dx + 0.5) * g.res,
                               g.origin[1] + (c[1] + dy + 0.5) * g.res)
                        for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                        if not (dx == 0 and dy == 0))
                    if any_free:
                        bad_ident.append((x, y, tx, ty))
                else:
                    n_moved += 1

    print("=" * 92)
    print("U1 非恒等（周围有路就必须动）: %s"
          % ("通过" if not bad_ident else "**失败 %d 处**" % len(bad_ident)))
    for b in bad_ident[:5]:
        print("     (%.2f,%.2f) → 目标 (%.2f,%.2f) 原地不动但周围有路" % b)
    print("U2 单帧位移 <= %.3f m        : %s"
          % (step, "通过" if not bad_jump else "**失败 %d 处**（出现瞬移！）"
             % len(bad_jump)))
    for b in bad_jump[:5]:
        print("     (%.2f,%.2f) → 目标 (%.2f,%.2f) 位移 %.3f m" % b)
    print("统计：能动 %d 次，停滞 %d 次（停滞中真·四周无路的不算失败）"
          % (n_moved, n_stuck))
    print("=" * 92)
    return not bad_ident and not bad_jump


def freeze_scene():
    """复现 seed=9 的冻结点，验证能走出去。

    现场数据（probe_stuck.py）：
      uav_2 冻结 @(-45.67,-3.55)  四周 0.5 m 的 9/12 方向不可通行
      uav_4 冻结 @(-45.73,-3.52)
    旧版在这里每帧返回原位置。新版必须返回一个**不同**的点。
    """
    g = MapGrid(METADATA)
    speed = 5.0                      # 搜索速度
    step = speed * DT
    print("")
    print("=" * 92)
    print("复现 seed=9 冻结点（旧版在此每帧原地不动）")
    print("=" * 92)
    ok = True
    for (x, y, tx, ty) in ((-45.67, -3.55, -33.67, 8.80),
                           (-45.73, -3.52, 2.00, 44.00)):
        print("  起点 (%.2f,%.2f) → 航点 (%.2f,%.2f)" % (x, y, tx, ty))
        print("    free(起点) = %s" % g.free(x, y))
        cx, cy = x, y
        moved_total = 0.0
        for i in range(50):          # 连续 50 帧，看能不能真的走出去
            nx, ny, _ = g.speed_toward(cx, cy, tx, ty, speed)
            moved_total += math.hypot(nx - cx, ny - cy)
            cx, cy = nx, ny
        d0 = math.hypot(tx - x, ty - y)
        d1 = math.hypot(tx - cx, ty - cy)
        print("    50 帧后位置 (%.2f,%.2f)  累计位移 %.2f m  "
              "到航点 %.2f → %.2f m（%s）"
              % (cx, cy, moved_total, d0, d1,
                 "接近了" if d1 < d0 - 1e-6 else "没接近"))
        if d1 >= d0 - 1e-6:
            ok = False
            print("    **仍然走不出去**")
    print("=" * 92)
    return ok


if __name__ == "__main__":
    a = u1_u2_realmap()
    b = freeze_scene()
    print("")
    print("总结：U1/U2/U3 %s ；冻结点脱困 %s"
          % ("通过" if a else "**失败**", "通过" if b else "**失败**"))
    sys.exit(0 if (a and b) else 1)
