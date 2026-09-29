#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""诊断：冻结点上 speed_toward 到底走了哪一级？为什么净位移≈0。

不猜，逐帧打印：当前点、目标方向是否可行、选中方向的落点、位移。
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from strategy_compare import MapGrid, METADATA, DT


def probe(x, y, tx, ty, speed=5.0, nframes=24):
    g = MapGrid(METADATA)
    step = speed * DT
    print("=" * 96)
    print("起点 (%.2f,%.2f) → 航点 (%.2f,%.2f)  step=%.2f res=%.2f"
          % (x, y, tx, ty, step, g.res))
    print("=" * 96)
    print("  %-4s %-18s %-18s %-8s %-9s %-18s %s"
          % ("帧", "当前位置", "目标方向落点", "直行free", "位移", "新位置", "方向角°"))

    cx, cy = x, y
    for i in range(nframes):
        d = math.hypot(tx - cx, ty - cy)
        ux, uy = (tx - cx) / d, (ty - cy) / d
        straight = (cx + ux * step, cy + uy * step)
        sf = g.free(*straight)
        nx, ny, blk = g.speed_toward(cx, cy, tx, ty, speed)
        disp = math.hypot(nx - cx, ny - cy)
        ang = math.degrees(math.atan2(ny - cy, nx - cx)) if disp > 1e-12 else float("nan")
        print("  %-4d (%8.3f,%8.3f) (%8.3f,%8.3f) %-8s %-9.4f (%8.3f,%8.3f) %7.1f"
              % (i, cx, cy, straight[0], straight[1], sf, disp, nx, ny, ang))
        cx, cy = nx, ny

    # 逐方向：从起点出发，各方向的落点是否可行、clear_run 多少
    print("")
    print("从起点出发，36 个方向的落点可行性（step=%.2f）：" % step)
    ux, uy = (tx - x) / math.hypot(tx - x, ty - y), (ty - y) / math.hypot(tx - x, ty - y)
    base = math.atan2(uy, ux)
    for a in range(0, 360, 30):
        ang = base + math.radians(a)
        pt = (x + math.cos(ang) * step, y + math.sin(ang) * step)
        run = g._clear_run(x, y, ang, maxd=step * 4.0)
        print("    相对目标 %+4d°  落点(%7.3f,%7.3f) free=%-5s clear_run=%.2f"
              % (a, pt[0], pt[1], g.free(*pt), run))

    # **关键**：直接实测每一级的返回值，确认到底谁在处理这个点。
    print("")
    print("逐级验证（从起点出发）：")
    d = math.hypot(tx - x, ty - y)
    ux, uy = (tx - x) / d, (ty - y) / d
    st = (x + ux * step, y + uy * step)
    print("  级1 直行落点 (%7.3f,%7.3f) free=%s" % (st[0], st[1], g.free(*st)))
    hit1 = None
    for ang in (20, -20, 40, -40, 60, -60, 80, -80, 100, -100, 120, -120):
        r = math.radians(ang)
        vx = ux * math.cos(r) - uy * math.sin(r)
        vy = ux * math.sin(r) + uy * math.cos(r)
        cx, cy = x + vx * step, y + vy * step
        if g.free(cx, cy):
            hit1 = (ang, cx, cy)
            break
    print("  级1 侧滑：%s" % ("命中 ang=%+d 落点(%.3f,%.3f)" % hit1 if hit1
                          else "全部失败（共 12 个角度）"))
    esc = g._escape_dir(x, y, tx, ty)
    print("  级3 BFS 脱困方向：%s" % ("(%.3f,%.3f)" % esc if esc else "None"))
    if esc:
        cx, cy = x + esc[0] * step, y + esc[1] * step
        print("       沿它走一步 → (%.3f,%.3f) free=%s" % (cx, cy, g.free(cx, cy)))
        # 长跑：每帧重算 BFS 第一步（等价于沿距离场下降），看**能不能到**。
        # 只看 10 帧会误判 —— 绕楼时前半程距离变大是正常的。
        print("       长跑 400 帧（每帧重算 BFS 第一步，看最终能否抵达航点）：")
        ax, ay = x, y
        best = math.hypot(tx - x, ty - y)
        best_at = 0
        for i in range(400):
            e = g._escape_dir(ax, ay, tx, ty)
            if e is None:
                print("         帧%d: BFS 无解，停在 (%.3f,%.3f)" % (i, ax, ay))
                break
            ax, ay = ax + e[0] * step, ay + e[1] * step
            dd = math.hypot(tx - ax, ty - ay)
            if dd < best - 1e-9:
                best, best_at = dd, i
            if dd < 1.0:
                print("         帧%d **抵达** (%.3f,%.3f)  距航点 %.3f m"
                      % (i, ax, ay, dd))
                break
        else:
            print("         400 帧未抵达，最终 (%.3f,%.3f) 距航点 %.3f m"
                  % (ax, ay, math.hypot(tx - ax, ty - ay)))
        print("         历史最近 %.3f m（第 %d 帧），起点距离 %.3f m"
              % (best, best_at, math.hypot(tx - x, ty - y)))

    # **逐帧判定谁在返回** —— 怀疑 级1 和 BFS 在互相拆台：
    # BFS 把机挪出袋口，下一帧 级1 的直行又成功了，直接把它拉回袋里。
    print("")
    print("逐帧：级1 是否成功？（若与 BFS 交替，就是振荡的根因）")
    ax, ay = x, y
    for i in range(24):
        d = math.hypot(tx - ax, ty - ay)
        ux2, uy2 = (tx - ax) / d, (ty - ay) / d
        st = (ax + ux2 * step, ay + uy2 * step)
        l1_straight = g.free(*st)
        l1_slide = None
        if not l1_straight:
            for ang in (20, -20, 40, -40, 60, -60, 80, -80, 100, -100, 120, -120):
                r = math.radians(ang)
                vx = ux2 * math.cos(r) - uy2 * math.sin(r)
                vy = ux2 * math.sin(r) + uy2 * math.cos(r)
                cx, cy = ax + vx * step, ay + vy * step
                if g.free(cx, cy):
                    l1_slide = ang
                    break
        nx, ny, _ = g.speed_toward(ax, ay, tx, ty, 5.0)
        who = "直行" if l1_straight else ("侧滑%+d" % l1_slide if l1_slide is not None
                                        else "BFS/兜底")
        moved = math.hypot(nx - ax, ny - ay)
        print("  帧%-3d (%.3f,%.3f) 级1=%s 实际走 %s 位移%.3f → (%.3f,%.3f) 距航点%.2f"
              % (i, ax, ay, ("直行free" if l1_straight else
                             ("侧滑%+d" % l1_slide if l1_slide is not None else "全失败")),
                 who, moved, nx, ny, math.hypot(tx - nx, ty - ny)))
        ax, ay = nx, ny
    print("=" * 96)


if __name__ == "__main__":
    probe(-45.67, -3.55, -33.67, 8.80)
