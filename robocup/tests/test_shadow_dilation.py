#!/usr/bin/env python3
"""FOV-shadow dilation contract (local_grid.to_grid_map + GridSource pass-through).

A depth camera marks what it sees; an obstacle's continuation past the scan
edge (FOV boundary) is UNKNOWN to the grid.  Planning must not graze the
real obstacle corner through that shadow, so UNKNOWN cells within
``obstacle_shadow_cells`` steps of an OCCUPIED cell fold as OCCUPIED:
  - depth 1..N inside the budget -> occupied
  - beyond the budget            -> back to the base-map policy (free)
  - FREE cells are never promoted
  - unknown_is_obstacle=True makes the shadow a no-op (already blocked)
"""
import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/robocup_navigation/src"))

from robocup_navigation.local_grid import (LocalOccupancyGrid, LocalGridError,
                                           FREE, OCCUPIED, UNKNOWN)  # noqa: E402


def make_grid(states, width, height, resolution=0.25, origin=(-6.0, -6.0)):
    grid = LocalOccupancyGrid(resolution_m=resolution, side_m=width * resolution,
                              z_min_m=0.8, z_max_m=3.5)
    # overwrite the whole window with the wanted layout
    grid.states = list(states)
    grid.width = width
    grid.height = height
    grid.origin = tuple(origin)
    return grid


class ShadowDilationTests(unittest.TestCase):
    W, H = 5, 5

    def states(self, occ, unk, free=None):
        s = [FREE] * (self.W * self.H)
        for x, y in occ:
            s[y * self.W + x] = OCCUPIED
        for x, y in unk:
            s[y * self.W + x] = UNKNOWN
        return s

    def test_shadow_within_budget_blocks(self):
        # obstacle at (1,2); unknown shadow to its right
        g = make_grid(self.states([(1, 2)], [(2, 2), (3, 2), (4, 2)]), self.W, self.H)
        folded = g.to_grid_map(unknown_is_obstacle=False, obstacle_shadow_cells=2)
        # depth 1 and 2 promoted, depth 3 beyond budget stays free
        self.assertEqual(folded.cells[2 * self.W + 2], 1)
        self.assertEqual(folded.cells[2 * self.W + 3], 1)
        self.assertEqual(folded.cells[2 * self.W + 4], 0)

    def test_free_cells_never_promoted(self):
        g = make_grid(self.states([(1, 2)], [(2, 2)]), self.W, self.H)
        g.states[2 * self.W + 3] = FREE  # scanned-free next to the shadow
        folded = g.to_grid_map(unknown_is_obstacle=False, obstacle_shadow_cells=3)
        self.assertEqual(folded.cells[2 * self.W + 2], 1)  # shadow blocked
        self.assertEqual(folded.cells[2 * self.W + 3], 0)  # free stays free

    def test_zero_shadow_is_noop(self):
        g = make_grid(self.states([(1, 2)], [(2, 2)]), self.W, self.H)
        folded = g.to_grid_map(unknown_is_obstacle=False, obstacle_shadow_cells=0)
        self.assertEqual(folded.cells[2 * self.W + 2], 0)

    def test_unknown_as_obstacle_makes_shadow_noop(self):
        g = make_grid(self.states([(1, 2)], [(2, 2)]), self.W, self.H)
        folded = g.to_grid_map(unknown_is_obstacle=True, obstacle_shadow_cells=2)
        self.assertEqual(folded.cells[2 * self.W + 2], 1)  # blocked, but by policy

    def test_invalid_shadow_cells_rejected(self):
        g = make_grid(self.states([(1, 2)], [(2, 2)]), self.W, self.H)
        for bad in (-1, 1.5, True):
            with self.assertRaises(LocalGridError):
                g.to_grid_map(unknown_is_obstacle=False, obstacle_shadow_cells=bad)

    def test_grid_source_passes_shadow_through(self):
        from robocup_navigation.route_runtime import GridSource
        from robocup_navigation.astar import GridMap
        g = make_grid(self.states([(1, 2)], [(2, 2), (3, 2)]), self.W, self.H)
        base = GridMap(self.W, self.H, 0.25, (-6.0, -6.0),
                       (0,) * (self.W * self.H), "map")
        src = GridSource.from_occupancy(g, base_grid=base, inflation_m=0.0,
                                        shadow_cells=2)
        folded, version = src.snapshot()
        self.assertEqual(folded.cells[2 * self.W + 2], 1)
        self.assertEqual(folded.cells[2 * self.W + 3], 1)
        self.assertEqual(folded.cells[2 * self.W + 4], 0)
        self.assertEqual(version, g.map_version)

    def test_grid_source_rejects_bad_shadow(self):
        from robocup_navigation.route_runtime import GridSource, RouteError
        g = make_grid(self.states([(1, 2)], [(2, 2)]), self.W, self.H)
        with self.assertRaises(RouteError):
            GridSource.from_occupancy(g, shadow_cells=-1)
        with self.assertRaises(RouteError):
            GridSource.from_occupancy(g, shadow_cells=True)


if __name__ == "__main__":
    unittest.main()
