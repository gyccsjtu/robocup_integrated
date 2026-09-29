"""Unit tests for the depth-camera local occupancy grid (M3-2 core)."""
import math
import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/robocup_navigation/src"))
from robocup_navigation import local_grid as lg  # noqa: E402,F401
from robocup_navigation.local_grid import (  # noqa: E402
    FREE, OCCUPIED, UNKNOWN, LocalGridError, LocalOccupancyGrid)


def make_grid(**kw):
    kw.setdefault("side_m", 12.0)  # half=6 -> forward wall hits stay inside
    return LocalOccupancyGrid(**kw)


class ConstructionTests(unittest.TestCase):
    def test_defaults_consistent(self):
        g = make_grid()
        self.assertEqual(g.width * g.resolution, g.side_m)
        self.assertEqual(len(g.states), g.width * g.height)
        self.assertEqual(set(g.states), {UNKNOWN})
        # Window is centred on the world origin at construction.
        self.assertEqual(g.origin, [-6.0, -6.0])

    def test_bad_params_rejected(self):
        for kw in (dict(resolution_m=0), dict(side_m=-1), dict(near_m=2.0, far_m=1.0),
                   dict(z_min_m=3.0, z_max_m=0.8)):
            with self.assertRaises(LocalGridError):
                make_grid(**kw)


class UpdateTests(unittest.TestCase):
    def setUp(self):
        self.g = make_grid(resolution_m=0.25, side_m=12.0)
        # Straight-ahead pinhole: ray through the centre is +X.
        self.fx = self.fy = 60.0
        self.cx = 31.5
        self.cy = 23.5
        self.pose = dict(drone_x=0.0, drone_y=0.0, drone_z=2.4, yaw=0.0)

    def test_wall_ahead_marks_occupied(self):
        depth = [float("nan")] * (64 * 48)
        # Centre pixel: straight ahead, d=5 -> hit (5.12, 0, 2.44) inside band.
        depth[23 * 64 + 31] = 5.0
        self.g.update(depth, 64, 48, self.fx, self.fy, self.cx, self.cy, **self.pose)
        cell = self.g.world_to_cell((5.12, 0.0))
        self.assertIsNotNone(cell)
        self.assertEqual(self.g.states[cell[1] * self.g.width + cell[0]], OCCUPIED)

    def test_low_return_marks_free(self):
        depth = [float("nan")] * (64 * 48)
        # Drone low: bottom pixel pitched down 21deg, d=1 -> hit z ~0.68 < band.
        depth[47 * 64 + 31] = 1.0
        self.g.update(depth, 64, 48, self.fx, self.fy, self.cx, self.cy,
                      drone_x=0.0, drone_y=0.0, drone_z=1.0, yaw=0.0)
        rz = self.cy - 47
        norm = math.sqrt(self.fx ** 2 + rz ** 2)
        hz = 1.0 + 0.04 + 1.0 * (rz / norm)
        self.assertLess(hz, self.g.z_min_m)
        hit_x = 0.12 + 1.0 * (self.fx / norm)
        cell = self.g.world_to_cell((hit_x, 0.0))
        self.assertEqual(self.g.states[cell[1] * self.g.width + cell[0]], FREE)

    def test_out_of_range_leaves_unknown(self):
        depth = [float("nan")] * (64 * 48)
        depth[23 * 64 + 31] = 500.0  # beyond far clip
        self.g.update(depth, 64, 48, self.fx, self.fy, self.cx, self.cy, **self.pose)
        self.assertEqual(set(self.g.states), {UNKNOWN})

    def test_yaw_rotation(self):
        depth = [float("nan")] * (64 * 48)
        depth[23 * 64 + 31] = 5.0
        self.g.recenter(10.0, 10.0)
        self.g.update(depth, 64, 48, self.fx, self.fy, self.cx, self.cy,
                      drone_x=10.0, drone_y=10.0, drone_z=2.4, yaw=math.pi / 2)
        # Expected hit computed independently: camera offset rotates with the
        # body (cam at (10, 10.12)), ray (60, 0.5, 0.5)/norm, d=5, yaw=90deg.
        norm = math.sqrt(60.0 ** 2 + 0.5 ** 2 + 0.5 ** 2)
        bx, by = 60.0 / norm, 0.5 / norm
        hx = 10.0 - 5.0 * by
        hy = 10.12 + 5.0 * bx
        cell = self.g.world_to_cell((hx, hy))
        self.assertIsNotNone(cell)
        self.assertEqual(self.g.states[cell[1] * self.g.width + cell[0]], OCCUPIED)

    def test_shape_mismatch(self):
        with self.assertRaises(LocalGridError):
            self.g.update([1.0] * 10, 64, 48, 60, 60, 31.5, 23.5, **self.pose)

    def test_max_points_subsampling(self):
        depth = [float("nan")] * (64 * 48)
        for v in range(10, 38):  # a full wall band across all columns
            for u in range(64):
                depth[v * 64 + u] = 5.0
        self.g.update(depth, 64, 48, self.fx, self.fy, self.cx, self.cy,
                      max_points=8, **self.pose)
        s = self.g.summary()
        self.assertGreaterEqual(s["occupied"], 1)
        self.assertLessEqual(s["occupied"], 8)


class WindowTests(unittest.TestCase):
    def test_recenter_snaps_and_resets(self):
        g = make_grid(resolution_m=0.25, side_m=12.0)
        g.mark(0.0, 0.0, OCCUPIED)
        self.assertEqual(g.summary()["occupied"], 1)
        # mark() reports traversability changes but never bumps the version
        # (depth-noise flicker would inflate it every frame).
        self.assertEqual(g.map_version, 0)
        self.assertTrue(g.mark(0.0, 0.0, FREE))
        self.assertFalse(g.mark(0.0, 0.0, FREE))
        g.recenter(0.05, 0.0)  # snaps to same origin -> no reset
        self.assertEqual(g.summary()["occupied"], 0)
        self.assertEqual(g.map_version, 0)
        g.recenter(2.0, 1.0)  # moves -> reset + version bump
        self.assertEqual(set(g.states), {UNKNOWN})
        self.assertEqual(g.map_version, 1)
        self.assertEqual(g.origin, [-4.0, -5.0])

    def test_mark_outside_window_ignored(self):
        g = make_grid(resolution_m=0.25, side_m=12.0)
        g.mark(100.0, 0.0, OCCUPIED)
        self.assertEqual(g.summary()["occupied"], 0)


class ExportTests(unittest.TestCase):
    def test_unknown_is_obstacle_default(self):
        g = make_grid()
        g.mark(0.0, 0.0, OCCUPIED)
        m = g.to_grid_map()
        cell = m.world_to_cell((0.0, 0.0))
        self.assertEqual(m.cells[cell[1] * m.width + cell[0]], 1)
        # An untouched cell is UNKNOWN -> blocked.
        far_cell = m.world_to_cell((-4.0, 4.0))
        self.assertEqual(m.cells[far_cell[1] * m.width + far_cell[0]], 1)

    def test_unknown_is_free_variant(self):
        g = make_grid(unknown_is_obstacle=False)
        g.mark(0.0, 0.0, OCCUPIED)
        m = g.to_grid_map()
        far_cell = m.world_to_cell((-4.0, 4.0))
        self.assertEqual(m.cells[far_cell[1] * m.width + far_cell[0]], 0)

    def test_free_fold(self):
        g = make_grid()
        g.mark(0.0, 0.0, FREE)
        m = g.to_grid_map()
        cell = m.world_to_cell((0.0, 0.0))
        self.assertEqual(m.cells[cell[1] * m.width + cell[0]], 0)


if __name__ == "__main__":
    unittest.main()
