#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""给 coordination_executor.py 接入 A* 绕障路线（可选，默认关闭）。"""

import io
import os

P = os.path.expanduser(
    "~/team_ws/robocup/src/robocup_navigation/scripts/coordination_executor.py")

OLD = """        points = [list(start),
                  [start[0], start[1], self.takeoff_alt],
                  [goal[0], goal[1], self.takeoff_alt], goal]
        if self.clearance_source == "empty_world" and not self.obstacles:
            clearance = self.empty_world_clearance_m
        else:
            clearance = _route_clearance(points, self.obstacles)"""

NEW = """        points = self._route_points(start, goal)
        if points is None:
            return None
        obstacles = self._obstacles()
        if self.clearance_source == "empty_world" and not obstacles:
            clearance = self.empty_world_clearance_m
        else:
            clearance = _route_clearance(points, obstacles)"""

HELPERS = '''    # --------------------------------------------------------------- offers
    def _obstacles(self):
        """Obstacle list: explicit config wins, else load from metadata.

        Clearance must be measured against real obstacles even when the route
        came from A*. Route generation and clearance verification stay
        independent, so a planner bug cannot turn into a false safety claim.
        """
        if self.obstacles:
            return self.obstacles
        if self.route_planner == "astar" and self.metadata_path and load_obstacles:
            try:
                return load_obstacles(self.metadata_path)
            except Exception as exc:  # noqa: BLE001
                self.event("OBSTACLES_LOAD_FAILED", reason=str(exc))
                return []
        return []

    def _route_points(self, start, goal):
        """Build the ROUTE_OFFER points.

        route_planner=straight (default): climb, fly level, descend.
        route_planner=astar: the level leg follows an A* detour around
        buildings. Everything else is unchanged.
        """
        if self.route_planner != "astar" or not self.metadata_path or plan_route is None:
            return [list(start),
                    [start[0], start[1], self.takeoff_alt],
                    [goal[0], goal[1], self.takeoff_alt], goal]
        try:
            pts = plan_route(self.metadata_path, start, goal, self.takeoff_alt,
                             inflate_m=self.route_inflate_m)
        except Exception as exc:  # noqa: BLE001
            self.event("ROUTE_PLAN_FAILED", reason=str(exc))
            return None
        if pts is None:
            # No safe route -> emit no offer -> the core stays fail-closed.
            self.event("ROUTE_PLAN_FAILED", reason="NO_PATH")
            return None
        return pts

    def _build_offer(self):'''

IMPORT_OLD = """try:
    from robocup_navigation.coordination_adapter import SustainedStop
except ImportError:  # pragma: no cover - host-side import fallback
    SustainedStop = None"""

IMPORT_NEW = """try:
    from robocup_navigation.coordination_adapter import SustainedStop
except ImportError:  # pragma: no cover - host-side import fallback
    SustainedStop = None

try:
    from robocup_navigation.coordination.route_planner import (
        load_obstacles, plan_route)
except ImportError:  # pragma: no cover - host-side import fallback
    load_obstacles = plan_route = None"""

CFG_OLD = ('        self.empty_world_clearance_m = '
           'float(self.p("empty_world_clearance_m", 25.0))')

CFG_NEW = '''        self.empty_world_clearance_m = float(self.p("empty_world_clearance_m", 25.0))
        # --- A* detour route (off by default, preserves prior behaviour) ---
        # When enabled the ROUTE_OFFER points follow an A* detour around the
        # buildings in the metadata, and obstacles are loaded from the same
        # metadata so clearance becomes a real measurement rather than a
        # constant. Not needed for empty-world runs.
        self.route_planner = self.p("route_planner", "straight")
        self.metadata_path = self.p("metadata_path", "")
        self.route_inflate_m = float(self.p("route_inflate_m", 0.5))'''

ANCHOR = ("    # --------------------------------------------------------------- "
          "offers\n    def _build_offer(self):")

# 幂等保护：每步的「已应用标记」。
# 本脚本早期版本没有这个保护，被重复执行过一次 —— 结果 executor 里出现了
# 两份 import 块和两份 route_planner 配置（英文版 + 中文版）。值虽然相同、
# 不影响运行，但重复执行会无限累积。现在每步先查标记，命中就跳过。
MARKS = {
    "import": "from robocup_navigation.coordination.route_planner import",
    "config": 'self.metadata_path = self.p("metadata_path", "")',
    "route": "points = self._route_points(start, goal)",
    "helpers": "def _route_points(self, start, goal)",
}


def main():
    with io.open(P, encoding="utf-8") as fh:
        s = fh.read()

    applied = 0
    for name, old, new in (("import", IMPORT_OLD, IMPORT_NEW),
                           ("config", CFG_OLD, CFG_NEW),
                           ("route", OLD, NEW),
                           ("helpers", ANCHOR, HELPERS)):
        if MARKS[name] in s:
            print("跳过(已就位): %s" % name)
            continue
        if old not in s:
            print("!! 未匹配: %s" % name)
            return 1
        s = s.replace(old, new, 1)
        applied += 1
        print("已改: %s" % name)

    if applied == 0:
        print("补丁已全部就位，无需重复执行：%s" % P)
        return 0

    with io.open(P, "w", encoding="utf-8") as fh:
        fh.write(s)
    print("写入完成（改动 %d 处）: %s" % (applied, P))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
