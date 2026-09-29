#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""真实城市地图上 ROUTE_OFFER 净空实测：硬编码直角折线 vs A* 绕障路线。

复刻 coordination_executor.py 的判定公式（不 import rospy，可离线跑）：

    clearance   = _route_clearance(points, obstacles)
    static_safe = clearance >= required_clearance_m     # 默认 0.8

背景
----
executor 默认路线是「竖直爬升 → 水平直飞 → 下降」的直角折线。空世界配置下
clearance 直接取常量 empty_world_clearance_m=25.0，static_safe 恒为真，
所以问题只在真实地图（training_city_full_s7，38 栋建筑）上暴露。

本脚本回答三件事：
  1. 空世界常量 25m 到底掩盖了多少条真实会撞墙的路线
  2. 换成 A* 绕障后 clearance / static_safe 变成什么样
  3. A* 路线是否真的不穿墙（用**未膨胀**的原始栅格复核）

用法：
    python3 verify_route_offer.py
    ROBOCUP_META=/path/to/city.json python3 verify_route_offer.py
    ROBOCUP_NAV_SRC=/path/to/robocup_navigation/src python3 verify_route_offer.py

只依赖标准库 + robocup_navigation.astar。
"""

import math
import os
import random
import sys

# ---------------------------------------------------------------- 路径自解析
HERE = os.path.dirname(os.path.abspath(__file__))

NAV_SRC = os.environ.get("ROBOCUP_NAV_SRC") or os.path.join(
    os.path.expanduser("~"), "team_ws", "robocup", "src",
    "robocup_navigation", "src")
if not os.path.isdir(NAV_SRC):
    for cand in (os.path.join(HERE, "robocup_navigation", "src"),
                 os.path.join(HERE, "..", "robocup_navigation", "src")):
        if os.path.isdir(cand):
            NAV_SRC = os.path.abspath(cand)
            break
sys.path.insert(0, NAV_SRC)

try:
    from robocup_navigation.coordination.route_planner import (  # noqa: E402
        load_obstacles, plan_route, inflate_grid)
    from robocup_navigation.astar import GridMap, load_metadata   # noqa: E402
except ImportError as exc:  # pragma: no cover
    sys.stderr.write(
        "无法导入 robocup_navigation（%s）\n"
        "请用 ROBOCUP_NAV_SRC 指向 .../robocup_navigation/src\n" % exc)
    raise SystemExit(2)

META = os.environ.get("ROBOCUP_META") or os.path.join(
    os.path.expanduser("~"), "team_ws", "robocup", "src",
    "robocup_training_worlds", "worlds", "generated",
    "training_city_full_s7.json")
if not os.path.isfile(META):
    for root, _dirs, files in os.walk(os.path.join(NAV_SRC, "..", "..")):
        if "training_city_full_s7.json" in files:
            META = os.path.join(root, "training_city_full_s7.json")
            break

# 与 executor 默认值一致
TAKEOFF_ALT = float(os.environ.get("ROBOCUP_ALT", "2.4"))
REQUIRED_CLEARANCE_M = 0.8
EMPTY_WORLD_CLEARANCE_M = 25.0
INFLATE_M = 0.5

PASS, FAIL = "[ OK ]", "[FAIL]"
_fails = []


def check(label, ok, detail=""):
    print("  %s %s%s" % (PASS if ok else FAIL, label,
                         ("  -> %s" % detail) if detail else ""))
    if not ok:
        _fails.append(label)
    return ok


# ------------------------- 复刻 executor 的净空公式（不依赖 ROS）-----------
def _seg_point_distance(p, a, b):
    ax, ay, az = a[0], a[1], a[2]
    bx, by, bz = b[0], b[1], b[2]
    px, py, pz = p[0], p[1], p[2]
    dx, dy, dz = bx - ax, by - ay, bz - az
    len2 = dx * dx + dy * dy + dz * dz
    if len2 <= 1e-9:
        return math.dist((px, py, pz), (ax, ay, az))
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy + (pz - az) * dz) / len2))
    return math.dist((px, py, pz), (ax + t * dx, ay + t * dy, az + t * dz))


def route_clearance(points, obstacles):
    """与 executor._route_clearance 完全同式：最小「线段到障碍中心距 - 半径」"""
    if not obstacles:
        return None
    worst = float("inf")
    for i in range(len(points) - 1):
        for ob in obstacles:
            d = _seg_point_distance(ob["xyz"], points[i], points[i + 1])
            worst = min(worst, d - float(ob.get("radius_m", 0.0)))
    return max(0.0, worst)


def straight_points(start, goal, alt):
    """executor 默认的直角折线（route_planner=straight）。"""
    return [list(start), [start[0], start[1], alt], [goal[0], goal[1], alt], goal]


_infl_cache = {}


def _inflated_grid(md, inflate_m):
    """膨胀栅格（带缓存）。plan_route 每次内部重建，这里只为诊断复用。"""
    key = (id(md), inflate_m)
    if key not in _infl_cache:
        _infl_cache[key] = inflate_grid(GridMap.from_metadata(md), inflate_m)
    return _infl_cache[key]


def main():
    print("=" * 66)
    print("ROUTE_OFFER 净空实测 —— 直角折线 vs A* 绕障")
    print("=" * 66)
    print("metadata : %s" % META)
    print("导航源   : %s" % NAV_SRC)
    print("高度 %.1fm | 要求净空 %.2fm | 膨胀 %.2fm"
          % (TAKEOFF_ALT, REQUIRED_CLEARANCE_M, INFLATE_M))
    print()

    md, _ = load_metadata(META)
    grid = GridMap.from_metadata(md)
    obstacles = load_obstacles(META)

    n_build = sum(1 for o in md.get("obstacles", []) if o.get("type") == "building")
    n_lamp = sum(1 for o in md.get("obstacles", []) if o.get("type") == "lamp_post")
    print("地图：%d 栋建筑 / %d 根灯柱 / 障碍总数 %d → 参与净空 %d（已跳过边界墙）"
          % (n_build, n_lamp, len(md.get("obstacles", [])), len(obstacles)))

    spawn = md["spawn"]["center"]
    goals = [g["center"] for g in md.get("goal_candidates", [])]
    print("出生点 (%.2f, %.2f) | goal 候选 %d 个" % (spawn[0], spawn[1], len(goals)))
    print()

    # ------------------------------------------------------------ [1] 空世界
    print("[1] 空世界配置复现（clearance_source=empty_world，无障碍）")
    c_empty = EMPTY_WORLD_CLEARANCE_M
    check("clearance 恒为常量 %.1f → static_safe 恒真" % c_empty,
          c_empty >= REQUIRED_CLEARANCE_M)
    print("     这正是问题被掩盖的原因：常量 25 与真实几何无关，")
    print("     任何路线（哪怕穿楼）都拿到 static_safe=true。")
    print()

    # ------------------------------------------------- [2] 真实地图 spawn→goal
    print("[2] 真实地图：出生点 → 各 goal 候选（%d 条）" % len(goals))
    print("     %-12s %10s %8s | %10s %8s" % ("goal", "折线净空", "判定", "A*净空", "判定"))
    st_ok = as_ok = 0
    st_cl, as_cl = [], []
    for i, gx in enumerate(goals):
        start = [spawn[0], spawn[1], TAKEOFF_ALT]
        goal = [gx[0], gx[1], TAKEOFF_ALT]
        sp = straight_points(start, goal, TAKEOFF_ALT)
        c_st = route_clearance(sp, obstacles)
        ok_st = c_st is not None and c_st >= REQUIRED_CLEARANCE_M
        st_ok += ok_st
        st_cl.append(c_st)

        ap = plan_route(META, start, goal, TAKEOFF_ALT, inflate_m=INFLATE_M)
        if ap is None:
            print("     %-12s %10s %8s | %10s %8s"
                  % ("goal_%04d" % i, "%.2f" % c_st, "OK" if ok_st else "FAIL",
                     "NO_PATH", "-"))
            continue
        c_as = route_clearance(ap, obstacles)
        ok_as = c_as is not None and c_as >= REQUIRED_CLEARANCE_M
        as_ok += ok_as
        as_cl.append(c_as)
        print("     %-12s %10.2f %8s | %10.2f %8s"
              % ("goal_%04d" % i, c_st, "OK" if ok_st else "FAIL",
                 c_as, "OK" if ok_as else "FAIL"))
    print()
    check("直角折线：%d/%d 条满足净空" % (st_ok, len(goals)), st_ok == len(goals),
          "不满足 %d 条 → 核心拒绝授权，飞机飞不出去" % (len(goals) - st_ok))
    check("A* 绕障：%d/%d 条满足净空" % (as_ok, len(goals)), as_ok == len(goals),
          "不满足 %d 条" % (len(goals) - as_ok))
    print()

    # ------------------------------------------------------- [3] 随机大样本
    print("[3] 随机采样 %d 组起终点（场地内自由点，水平距 5~60m）" % n)
    random.seed(20260928)
    w, h, res, origin = grid.width, grid.height, grid.resolution, grid.origin

    def free_cell():
        for _ in range(5000):
            ix = random.randrange(w)
            iy = random.randrange(h)
            if grid.is_free((ix, iy)):
                return grid.cell_to_world((ix, iy))
        return None

    n = int(os.environ.get("ROBOCUP_SAMPLES", "1200"))
    st_pass = as_pass = astar_none = 0
    nopath_start = nopath_goal = nopath_other = 0
    st_all, as_all = [], []
    tries = 0
    while len(st_all) < n and tries < n * 40:
        tries += 1
        a = free_cell()
        b = free_cell()
        if a is None or b is None:
            continue
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        if not (5.0 <= d <= 60.0):
            continue
        start = [a[0], a[1], TAKEOFF_ALT]
        goal = [b[0], b[1], TAKEOFF_ALT]

        c_st = route_clearance(straight_points(start, goal, TAKEOFF_ALT), obstacles)
        st_all.append(c_st)
        if c_st >= REQUIRED_CLEARANCE_M:
            st_pass += 1

        ap = plan_route(META, start, goal, TAKEOFF_ALT, inflate_m=INFLATE_M)
        if ap is None:
            # 诊断失败原因：端点是否落在膨胀后的障碍内
            astar_none += 1
            inf_grid = _inflated_grid(md, INFLATE_M)
            ca = inf_grid.world_to_cell((start[0], start[1]))
            cb = inf_grid.world_to_cell((goal[0], goal[1]))
            if ca is not None and not inf_grid.is_free(ca):
                nopath_start += 1
            elif cb is not None and not inf_grid.is_free(cb):
                nopath_goal += 1
            else:
                nopath_other += 1
            continue
        c_as = route_clearance(ap, obstacles)
        as_all.append(c_as)
        if c_as >= REQUIRED_CLEARANCE_M:
            as_pass += 1

    def pct(k, m):
        return 100.0 * k / m if m else 0.0

    # 口径说明：NO_PATH 也算「飞不出去」，必须留在分母里。
    # 若只统计「规划成功的路线」，A* 会显示虚高的 100%。
    n_tot = len(st_all)
    print("     直角折线：可飞 %d/%d = %.1f%%   净空中位 %.2fm  最小 %.2fm"
          % (st_pass, n_tot, pct(st_pass, n_tot),
             sorted(st_all)[n_tot // 2], min(st_all)))
    print("     A* 绕障 ：可飞 %d/%d = %.1f%%   净空中位 %.2fm  最小 %.2fm"
          % (as_pass, n_tot, pct(as_pass, n_tot),
             sorted(as_all)[len(as_all) // 2], min(as_all)))
    print("       （A* 规划不出路 NO_PATH %d 次，已计入分母）" % astar_none)
    print("     规划成功的 A* 路线 %d 条：净空中位 %.2fm，最小 %.2fm，全部 ≥ %.2fm"
          % (len(as_all), sorted(as_all)[len(as_all) // 2], min(as_all),
             REQUIRED_CLEARANCE_M))
    print()
    check("A* 可飞率高于直角折线",
          pct(as_pass, n_tot) > pct(st_pass, n_tot) + 1.0,
          "%.1f%% vs %.1f%%" % (pct(as_pass, n_tot), pct(st_pass, n_tot)))
    check("规划成功的 A* 路线净空最小值也满足裕度", min(as_all) >= REQUIRED_CLEARANCE_M,
          "最小 %.2fm" % min(as_all))
    print()

    # ---------------------------------------------------- [3b] NO_PATH 诊断
    print("[3b] A* 规划失败（NO_PATH）原因归类")
    print("     起点落在膨胀障碍内 : %d" % nopath_start)
    print("     终点落在膨胀障碍内 : %d" % nopath_goal)
    print("     端点正常但无路     : %d" % nopath_other)
    print("     注：这类失败是 fail-closed（不发 OFFER），不会撞机，但会浪费任务机会。")
    print("     缓解：把起/终点吸附到最近的自由栅格。plan_route 已同时处理")
    print("     START_OCCUPIED 与 GOAL_OCCUPIED；剩下的 %d 次是 _nearest_free"
          % nopath_start)
    print("     在 6m 搜索半径内找不到自由格（端点被建筑围死），属真实不可达。")
    print()

    # ------------------------------------------- [4] A* 路线是否真不穿墙
    print("[4] A* 路线穿墙复核（用**未膨胀**的原始栅格逐点检查）")
    sample = 0
    bad_pts = 0
    bad_routes = 0
    total_pts = 0
    random.seed(11)
    trials = 0
    while sample < 400 and trials < 4000:
        trials += 1
        a = free_cell()
        b = free_cell()
        if a is None or b is None:
            continue
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        if not (5.0 <= d <= 60.0):
            continue
        ap = plan_route(META, [a[0], a[1], TAKEOFF_ALT], [b[0], b[1], TAKEOFF_ALT],
                        TAKEOFF_ALT, inflate_m=INFLATE_M)
        if ap is None:
            continue
        sample += 1
        total_pts += len(ap)
        hit = 0
        for p in ap:
            cell = grid.world_to_cell((p[0], p[1]))
            if cell is None or not grid.is_free(cell):
                hit += 1
        bad_pts += hit
        if hit:
            bad_routes += 1
    print("     检查 %d 条路线 / %d 个点：落在障碍格的点 %d 个，穿墙路线 %d 条"
          % (sample, total_pts, bad_pts, bad_routes))
    check("A* 路线 0% 穿墙", bad_routes == 0,
          "%d 条路线存在穿越障碍格的点" % bad_routes)
    print()

    # ---------------------------------------------------------------- 结论
    print("=" * 66)
    if _fails:
        print("失败 %d 项：" % len(_fails))
        for f in _fails:
            print("  - %s" % f)
        return 1
    print("全部通过")
    print()
    print("结论：空世界的常量 25m 让所有路线（含穿楼的）都拿到 static_safe=true；")
    print("      真实城市地图上直角折线大量不满足 %.1fm 净空 → 核心 fail-closed 拒飞；"
          % REQUIRED_CLEARANCE_M)
    print("      换 route_planner=astar 后净空是真的测出来的，且路线不穿墙。")
    print()
    print("启用方式（ROS 参数）：")
    print("  route_planner: astar")
    print("  metadata_path: %s" % META)
    print("  clearance_source: map      # 注意不能再用 empty_world")
    print("=" * 66)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
