#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线验证：A* 路线 vs 直角折线，在真实城市地图上的 clearance 对比。

不需要 ROS/Gazebo —— 直接调 route_planner 与 executor 的净空函数，
比较两种路线生成方式在同一批起终点上的净空。

预期：直角折线穿建筑 -> clearance 很小 -> static_safe=False -> 核心拒绝；
      A* 路线绕开建筑 -> clearance 达标 -> static_safe=True。
"""

import math
import os
import sys

WS = os.path.expanduser("~/team_ws/robocup")
sys.path.insert(0, os.path.join(WS, "src/robocup_navigation/src"))
sys.path.insert(0, os.path.join(WS, "src/robocup_navigation/scripts"))

from robocup_navigation.astar import GridMap, load_metadata  # noqa: E402
from robocup_navigation.coordination.route_planner import (  # noqa: E402
    load_obstacles, plan_route)

METADATA = os.path.join(
    WS, "src/robocup_training_worlds/worlds/generated/training_city_full_s7.json")
TAKEOFF_ALT = 2.4
REQUIRED_CLEARANCE = 0.8


def seg_point_distance(p, a, b):
    """点到线段最短距离（与 executor 的 _seg_point_distance 同语义）。"""
    ax, ay, az = a
    bx, by, bz = b
    px, py, pz = p
    vx, vy, vz = bx - ax, by - ay, bz - az
    wx, wy, wz = px - ax, py - ay, pz - az
    vv = vx * vx + vy * vy + vz * vz
    if vv <= 1e-20:
        return math.dist(p, a)
    t = max(0.0, min(1.0, (wx * vx + wy * vy + wz * vz) / vv))
    cx, cy, cz = ax + t * vx, ay + t * vy, az + t * vz
    return math.dist(p, (cx, cy, cz))


def route_clearance(points, obstacles):
    """与 executor._route_clearance 同语义：最小净空。"""
    if not obstacles:
        return None
    worst = float("inf")
    for i in range(len(points) - 1):
        for ob in obstacles:
            c = seg_point_distance(ob["xyz"], points[i], points[i + 1])
            worst = min(worst, c - float(ob.get("radius_m", 0.0)))
    return max(0.0, worst)


def straight_route(start, goal, alt):
    """executor 原来的直角折线。"""
    return [list(start),
            [start[0], start[1], alt],
            [goal[0], goal[1], alt],
            list(goal)]


def main():
    print("=" * 72)
    print("A* 绕障路线 vs 直角折线 —— clearance 对比（真实城市地图）")
    print("=" * 72)

    obstacles = load_obstacles(METADATA)
    md, _ = load_metadata(METADATA)
    raw = GridMap.from_metadata(md)
    print("地图: %s" % os.path.basename(METADATA))
    print("障碍数: %d（要求净空 >= %.1f m）\n" % (len(obstacles), REQUIRED_CLEARANCE))

    # 场景：从出生点飞到场地各处。目标点先经**可达性筛选** ——
    # 落在建筑内的点必须先剔除，否则 A* 报 GOAL_OCCUPIED 是正确行为，
    # 但那测不出路线质量。six_uav_gazebo.json 里的默认目标就没有筛选过：
    # (20, 25) 实测落在 building_2040 里。
    spawn = [-18.0, 45.0, 0.0]
    candidates = [
        [-20.0, 25.0], [-12.0, 25.0], [-4.0, 25.0], [4.0, 25.0],
        [12.0, 25.0], [20.0, 25.0], [6.0, -10.0], [-40.0, -20.0],
        [0.0, 0.0], [30.0, 10.0], [-30.0, -30.0], [50.0, -20.0],
    ]
    goals = []
    rejected = []
    for gx, gy in candidates:
        c = raw.world_to_cell((gx, gy))
        if c is not None and raw.is_free(c):
            goals.append([gx, gy, TAKEOFF_ALT])
        else:
            rejected.append((gx, gy))
    if rejected:
        print("剔除落在建筑内的目标点（A* 报 GOAL_OCCUPIED 属正确行为）：")
        for gx, gy in rejected:
            print("  (%7.1f, %6.1f)" % (gx, gy))
        print("")

    straight_ok = astar_ok = 0
    print("%-22s | %-26s | %-26s" % ("目标 (x, y)", "直角折线", "A* 绕障"))
    print("-" * 82)

    for goal in goals:
        s_pts = straight_route(spawn, goal, TAKEOFF_ALT)
        s_clr = route_clearance(s_pts, obstacles)
        s_safe = s_clr is not None and s_clr >= REQUIRED_CLEARANCE

        a_pts = plan_route(METADATA, spawn, goal, TAKEOFF_ALT)
        if a_pts is None:
            a_clr, a_safe, a_n = None, False, 0
        else:
            a_clr = route_clearance(a_pts, obstacles)
            a_safe = a_clr is not None and a_clr >= REQUIRED_CLEARANCE
            a_n = len(a_pts)

        if s_safe:
            straight_ok += 1
        if a_safe:
            astar_ok += 1

        s_txt = "净空 %6.2f %s" % (s_clr, "OK" if s_safe else "拒绝") \
            if s_clr is not None else "无净空"
        a_txt = "净空 %6.2f %s (%d点)" % (a_clr, "OK" if a_safe else "拒绝", a_n) \
            if a_clr is not None else "规划失败"

        print("(%7.1f, %6.1f)      | %-26s | %-26s" % (goal[0], goal[1], s_txt, a_txt))

    print("-" * 82)
    n = len(goals)
    print("\n可通过（static_safe=True）的比例:")
    print("  直角折线: %d/%d" % (straight_ok, n))
    print("  A* 绕障 : %d/%d" % (astar_ok, n))

    print("\n结论:")
    if astar_ok >= straight_ok and astar_ok > 0:
        print("  A* 不劣于直角折线，且在需要绕行的目标上仍能生成安全路线。")
        if astar_ok > straight_ok:
            print("  （本批 A* 更优：折线被核心拒绝的目标，A* 能通过。）")
    else:
        print("  A* 未达预期，需检查障碍近似或膨胀半径。")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
