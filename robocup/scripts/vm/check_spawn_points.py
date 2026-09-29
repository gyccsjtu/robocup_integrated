#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""出生点校验工具：检查 fleet.yaml 的出生点是否落在建筑内。

为什么需要：
  无人机出生在建筑碰撞体内时，Gazebo 物理引擎会持续推挤模型 →
  陀螺仪报出虚假角速度（实测 0.34 rad/s，正常静止应 <0.005）→
  PX4 报 `WARN [ekf2] primary EKF changed N (gyro fault)` →
  EKF 不融合位置（`pos_horiz_abs_status_flag=False`）→
  `system_status` 卡在 3(STANDBY) 不升 4(ACTIVE) →
  **PX4 静默拒绝 OFFBOARD**（`set_mode` 返回 mode_sent=True 但模式不变，
  且不给任何 statustext 报错，极难排查）。

用法：
   ROBOCUP_WORKSPACE=$PWD python3 scripts/vm/check_spawn_points.py
   ROBOCUP_WORKSPACE=$PWD python3 scripts/vm/check_spawn_points.py --margin 3.0
   ROBOCUP_WORKSPACE=$PWD python3 scripts/vm/check_spawn_points.py --find   # 搜索安全出生排

退出码：0=全部安全；1=有出生点不安全（可用于 CI/启动前自检）。
"""
import argparse
import json
import math
import os
import sys

WS = os.environ.get("ROBOCUP_WORKSPACE",
                    os.environ.get("ROBOCUP_WS",
                                   os.path.expanduser("~/team_ws/robocup")))
sys.path.insert(0, os.path.join(WS, "scripts", "vm"))

DEFAULT_METADATA = os.path.join(
    WS, "src/robocup_training_worlds/worlds/generated/training_city_full_s7.json")


def load_obstacles(metadata_path):
    with open(metadata_path) as fh:
        md = json.load(fh)
    return [o for o in md.get("obstacles", []) if o.get("type") == "building"]


def clearance(x, y, obstacles):
    """(x,y) 到最近建筑**边缘**的距离（负值=在建筑内部）。"""
    best = float("inf")
    for o in obstacles:
        c = o.get("center")
        sz = o.get("size") or [0, 0, 0]
        if not c or len(sz) < 2:
            continue
        r = max(sz[0], sz[1]) / 2.0
        best = min(best, math.hypot(c[0] - x, c[1] - y) - r)
    return best


def nearest_building(x, y, obstacles):
    best, best_d = None, float("inf")
    for o in obstacles:
        c = o.get("center")
        sz = o.get("size") or [0, 0, 0]
        if not c or len(sz) < 2:
            continue
        r = max(sz[0], sz[1]) / 2.0
        d = math.hypot(c[0] - x, c[1] - y) - r
        if d < best_d:
            best_d, best = d, o.get("id")
    return best, best_d


def fleet_spawn_points(fleet_yaml):
    """从 fleet.yaml 派生每机出生点（复用 mu_fleet，保持单一事实来源）。"""
    import mu_fleet
    os.environ.setdefault("MU_FLEET_YAML", fleet_yaml)
    doc = mu_fleet.load()
    return [(r["ros_ns"].strip("/"), r["model_name"], r["spawn_x"], r["spawn_y"])
            for r in mu_fleet.uavs(doc)]


def do_check(rows, obstacles, margin):
    print("=== 出生点安全检查（安全裕度 %.1f m）===" % margin)
    bad = 0
    for uid, model, x, y in rows:
        clr = clearance(x, y, obstacles)
        if clr < margin:
            b, d = nearest_building(x, y, obstacles)
            print("  ✗ %-8s (%-7s) (%7.1f, %6.1f)  净空 %6.2f m  撞 %s（边缘距 %.2f m）"
                  % (uid, model, x, y, clr, b, d))
            bad += 1
        else:
            print("  ✓ %-8s (%-7s) (%7.1f, %6.1f)  净空 %6.2f m"
                  % (uid, model, x, y, clr))
    if bad:
        print("\n!!! %d/%d 架出生点不安全。" % (bad, len(rows)))
        print("    后果：Gazebo 物理推挤 → 陀螺仪假角速度 → PX4 gyro fault")
        print("          → EKF 不融合位置 → system_status 卡 STANDBY")
        print("          → PX4 静默拒绝 OFFBOARD（无报错，极难排查）")
        print("    修法：调整 config/multi_uav/fleet.yaml 的 spawn.start_xy")
        print("          用 --find 搜索安全位置。")
        return 1
    print("\n全部出生点安全。")
    return 0


def do_find(obstacles, margin, spacing, count):
    """搜索一排 count 个点、间距 spacing、最小净空 >= margin 的安全位置。"""
    print("搜索 %d 机一排（间距 %.1f m，最小净空 %.1f m）..." % (count, spacing, margin))
    best = None
    for x0 in [x * 1.0 for x in range(-120, 121)]:
        for y0 in [y * 1.0 for y in range(-60, 61)]:
            pts = [(x0 + i * spacing, y0) for i in range(count)]
            if not all(-95 <= p[0] <= 95 and -45 <= p[1] <= 45 for p in pts):
                continue
            clr = min(clearance(px, py, obstacles) for px, py in pts)
            if clr < margin:
                continue
            # 偏好居中（离原点近）且净空大
            center_pen = abs(x0 + (count - 1) * spacing / 2.0) + abs(y0)
            score = (round(clr, 1), -center_pen)
            if best is None or score > best[0]:
                best = (score, x0, y0, clr)
    if not best:
        print("未找到满足条件的位置（放宽 --margin 或 --spacing）")
        return 1
    _, x0, y0, clr = best
    print("\n推荐写入 fleet.yaml：")
    print("spawn:")
    print("  x_spacing_m: %.1f" % spacing)
    print("  y_spacing_m: 0.0")
    print("  z_m: 0.0")
    print("  start_xy: [%.1f, %.1f]" % (x0, y0))
    print("\n各机位置（最小净空 %.1f m）：" % clr)
    for i in range(count):
        x, y = x0 + i * spacing, y0
        print("  uav_%d -> (%.1f, %.1f)  净空 %.1f m" % (i + 1, x, y, clearance(x, y, obstacles)))
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--margin", type=float, default=3.0,
                    help="要求的最小净空 m（默认 3.0；旋翼半径 0.45 + 裕度）")
    ap.add_argument("--metadata", default=DEFAULT_METADATA)
    ap.add_argument("--fleet", default=os.environ.get(
        "MU_FLEET_YAML", os.path.join(WS, "config/multi_uav/fleet.yaml")))
    ap.add_argument("--find", action="store_true",
                    help="搜索安全出生排并输出可粘贴的 fleet.yaml 片段")
    ap.add_argument("--spacing", type=float, default=8.0, help="--find 用的间距")
    ap.add_argument("--count", type=int, default=6, help="--find 用的机数")
    args = ap.parse_args()

    obstacles = load_obstacles(args.metadata)
    if not obstacles:
        print("!!! 未从 %s 读到建筑障碍" % args.metadata)
        return 1
    print("地图: %s（%d 栋建筑）" % (os.path.basename(args.metadata), len(obstacles)))

    if args.find:
        return do_find(obstacles, args.margin, args.spacing, args.count)

    rows = fleet_spawn_points(args.fleet)
    return do_check(rows, obstacles, args.margin)


if __name__ == "__main__":
    sys.exit(main())
