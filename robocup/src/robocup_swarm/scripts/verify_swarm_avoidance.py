#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""集群搜索避障验证：轨迹穿墙检测 + 高度稳定性 + 停滞检测 + 覆盖率。

背景：原始 swarm_agent 用纯直线 P 控制飞向格中心，会穿过建筑卡死在墙里。
现接入 A* 绕障 + lookahead 跟踪，本脚本用「实际飞行轨迹」证明绕障生效。

检查项：
  1. 轨迹穿墙：采样点投影到**原始**障碍栅格，落在真障碍内的点数（应≈0）
  2. 贴墙程度：采样点到最近障碍的距离分布（应 > 安全裕度）
  3. 高度稳定：z 是否稳定在搜索高度（EKF 发散会出负值/大幅摆动）
  4. 停滞检测：末段位移是否仍在推进（卡墙会停在原地）
  5. 覆盖率：manager 日志「已覆盖」格数

用法（VM 上，SITL + swarm 已在跑）：
    source /opt/ros/noetic/setup.bash
    source ~/team_ws/robocup/devel/setup.bash
    python3 verify_swarm_avoidance.py --duration 30
"""

import argparse
import glob
import math
import os
import re
import sys

import rospy
from gazebo_msgs.msg import ModelStates

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robocup_navigation.astar import load_metadata, GridMap

METADATA_PATH = os.environ.get(
    "ROBOCUP_METADATA",
    os.path.join(os.environ.get("ROBOCUP_WS", "/home/ros/team_ws/robocup"),
                 "src/robocup_training_worlds/worlds/generated/training_city_full_s7.json"))
INFLATE_M = 0.5      # 与 agent/manager 一致
SAMPLE_DT = 0.2      # 轨迹采样间隔 s
LOG_ROOT = os.path.expanduser("~/team_ws/robocup/logs/swarm_search")


def inflate_grid(grid, inflation_m):
    """障碍圆盘膨胀（与 swarm_agent/swarm_manager 的同名函数一致）。"""
    r = int(math.ceil(inflation_m / grid.resolution))
    if r <= 0:
        return grid
    w, h = grid.width, grid.height
    occ = [(ix, iy) for iy in range(h) for ix in range(w)
           if not grid.is_free((ix, iy))]
    cells = bytearray(grid.cells)
    r2 = r * r
    for cx, cy in occ:
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r2:
                    continue
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < w and 0 <= ny < h:
                    cells[ny * w + nx] = 1
    return GridMap(w, h, grid.resolution, grid.origin, bytes(cells), grid.frame_id)


class TrajSampler(object):
    """按固定间隔采样各模型的世界坐标轨迹。"""

    def __init__(self, models):
        self.models = models
        self.traj = {m: [] for m in models}   # model -> [(x,y,z)]
        self._last_t = 0.0
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._cb)

    def _cb(self, msg):
        now = rospy.Time.now().to_sec()
        if now - self._last_t < SAMPLE_DT:
            return
        self._last_t = now
        for m in self.models:
            try:
                i = msg.name.index(m)
            except ValueError:
                continue
            p = msg.pose[i].position
            self.traj[m].append((p.x, p.y, p.z))


def nearest_obstacle_dist(grid_raw, x, y, search_m=8.0):
    """到最近**真障碍**栅格中心的大致距离（局部搜索，够用）。"""
    res = grid_raw.resolution
    r = int(math.ceil(search_m / res))
    c = grid_raw.world_to_cell((x, y))
    if c is None:
        return None
    best = None
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            cell = (c[0] + dx, c[1] + dy)
            if not grid_raw.in_bounds(cell):      # 越界跳过（cell_to_world 会抛异常）
                continue
            if grid_raw.is_free(cell):
                continue
            wx, wy = grid_raw.cell_to_world(cell)
            d = math.hypot(wx - x, wy - y)
            if best is None or d < best:
                best = d
    return best


def check_coverage(log_dir):
    """从 manager 日志统计「分配」与「已覆盖」次数。"""
    if not log_dir:
        return None
    path = os.path.join(log_dir, "manager.log")
    if not os.path.exists(path):
        return None
    with open(path, "rb") as f:
        text = f.read().decode("utf-8", "replace")
    return {
        "assign": len(re.findall(r"分配", text)),
        "covered": len(re.findall(r"已覆盖", text)),
        "blocked": (re.search(r"过滤建筑内格中心：(\d+)/(\d+)", text) or [None, "?", "?"])[1],
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--models", default="iris_1,iris_2")
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--log-dir", default=None,
                    help="默认自动取 logs/swarm_search 下最新目录")
    args = ap.parse_args()

    models = [s.strip() for s in args.models.split(",") if s.strip()]
    log_dir = args.log_dir
    if log_dir is None and os.path.isdir(LOG_ROOT):
        cands = sorted(glob.glob(os.path.join(LOG_ROOT, "*")))
        log_dir = cands[-1] if cands else None

    rospy.init_node("verify_swarm_avoidance", anonymous=True)

    md, _ = load_metadata(METADATA_PATH)
    grid_raw = GridMap.from_metadata(md)              # 真障碍（未膨胀）
    grid_inf = inflate_grid(grid_raw, INFLATE_M)      # 安全边界

    sampler = TrajSampler(models)
    rospy.loginfo("[verify] 采样 %d 个模型 %.0fs ...", len(models), args.duration)
    rospy.sleep(args.duration)

    print("\n" + "=" * 68)
    print("集群搜索避障验证报告")
    print("=" * 68)

    ok = True
    for m in models:
        pts = sampler.traj.get(m, [])
        print("\n--- %s（%d 个采样点）---" % (m, len(pts)))
        if len(pts) < 5:
            print("  ✗ 采样点太少，模型可能未 spawn 或未收到 model_states")
            ok = False
            continue

        # 1) 穿墙检测（落在真障碍栅格内）
        in_wall = []
        for x, y, z in pts:
            c = grid_raw.world_to_cell((x, y))
            if c is None:
                continue
            if not grid_raw.is_free(c):
                in_wall.append((x, y))
        pct_wall = 100.0 * len(in_wall) / max(len(pts), 1)
        flag = "✓" if pct_wall < 2.0 else "✗"
        if pct_wall >= 2.0:
            ok = False
        print("  %s 穿墙采样点: %d/%d (%.1f%%)" % (flag, len(in_wall), len(pts), pct_wall))
        if in_wall:
            print("     示例: %s" % [(round(x, 1), round(y, 1)) for x, y in in_wall[:3]])

        # 2) 贴墙距离分布（抽查后 1/3 采样点，避免太慢）
        sample = pts[len(pts) * 2 // 3:]
        dists = []
        for x, y, z in sample:
            d = nearest_obstacle_dist(grid_raw, x, y)
            if d is not None:
                dists.append(d)
        if dists:
            dmin, dmean = min(dists), sum(dists) / len(dists)
            print("  贴墙检查: 最小距障=%.2fm 平均=%.2fm（旋翼半径 0.45m）" % (dmin, dmean))

        # 3) 高度稳定（EKF 发散检测）
        zs = [z for _, _, z in pts]
        zmin, zmax = min(zs), max(zs)
        flag = "✓" if (zmin > 1.0 and zmax < 12.0) else "✗"
        if not (zmin > 1.0 and zmax < 12.0):
            ok = False
        print("  %s 高度范围: %.2f ~ %.2f m（期望≈6，发散会出负值）" % (flag, zmin, zmax))

        # 4) 停滞检测（末段位移）
        tail = pts[-min(25, len(pts)):]
        if len(tail) >= 2:
            dx = tail[-1][0] - tail[0][0]
            dy = tail[-1][1] - tail[0][1]
            moved = math.hypot(dx, dy)
            flag = "✓" if moved > 0.5 else "✗（疑似卡住）"
            if moved <= 0.5:
                ok = False
            print("  %s 末段位移: %.2f m（%d 采样点，卡墙会≈0）" % (flag, moved, len(tail)))

    # 5) 覆盖率
    cov = check_coverage(log_dir)
    print("\n--- 覆盖率（manager 日志）---")
    if cov is None:
        print("  (未找到 manager.log，跳过)")
    else:
        print("  分配次数=%s  已覆盖格数=%s  建筑内格过滤=%s"
              % (cov["assign"], cov["covered"], cov["blocked"]))
        if cov["covered"] and int(cov["covered"]) > 0:
            print("  ✓ 覆盖在推进（有格被标记已覆盖）")

    print("\n" + "=" * 68)
    print("总体: %s" % ("✓ 通过（两机均在绕障飞行）" if ok else "✗ 有问题，见上方 ✗ 项"))
    print("=" * 68 + "\n")


if __name__ == "__main__":
    main()
