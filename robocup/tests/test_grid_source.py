"""Offline contract tests for the GridSource abstraction and versioned replan.

M3-3: the same replan chain must run on the verified scene-metadata grid
(static source) and on a live LocalOccupancyGrid (perception source), with a
map_version freshness check so a route is never committed against a world
that moved underneath the planner.
"""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/robocup_navigation/src"))
from robocup_navigation import route_runtime  # noqa: E402
from robocup_navigation.astar import GridMap  # noqa: E402
from robocup_navigation.local_grid import (  # noqa: E402
    LocalOccupancyGrid, OCCUPIED)


def base_config():
    return {
        "route_start_world_xy": (-3.0, 0.0), "route_goal_world_xy": (3.0, 0.0),
        "route_altitude_m": 2.4, "route_max_expansions": 250000,
        "route_planning_timeout_s": 5.0, "vehicle_radius_m": 0.40,
        "horizontal_margin_m": 0.20, "vehicle_half_height_m": 0.15,
        "vertical_margin_m": 0.30, "minimum_clearance_m": 0.20,
        "planning_tracking_margin_m": 0.20,
        "max_radius_m": 7.0, "max_altitude_m": 3.0,
    }


class RacingOccupancy(object):
    """Fake occupancy whose map_version moves between snapshot and commit."""

    def __init__(self, grid):
        self.grid = grid
        self.map_version = 0
        self.calls = 0

    def to_grid_map(self, *args, **kwargs):
        # one snapshot() = two folds (plain + shadow); the version moves
        # after the FIRST snapshot batch, i.e. between plan and commit
        self.calls += 1
        if self.calls == 3:
            self.map_version = 1
        return self.grid


class BrokenOccupancy(object):
    map_version = 0

    def to_grid_map(self):
        raise ValueError("boom")


class MergedInflationTests(unittest.TestCase):
    """Perception source merged over the static base map, then inflated."""

    @classmethod
    def setUpClass(cls):
        cls.world = ROOT / "src/robocup_training_worlds/worlds/generated/training_city_unit_single_wall_s42.world"
        cls.metadata = json.loads(cls.world.with_suffix(".json").read_text())

    def test_merged_source_blocks_dynamic_and_defers_unknown_to_base(self):
        base = GridMap.from_metadata(self.metadata)
        resolution = base.resolution
        local = LocalOccupancyGrid(resolution_m=resolution,
                                   side_m=base.width * resolution,
                                   unknown_is_obstacle=True)
        local.recenter(base.origin[0] + base.width * resolution / 2.0,
                       base.origin[1] + base.height * resolution / 2.0)
        # Spawn a dynamic wall on ground the base map proves free: find a
        # free base cell inside the window and mark it occupied.
        free_world = None
        for cy in range(base.height):
            for cx in range(base.width):
                if base.cells[cy * base.width + cx] == 0:
                    free_world = base.cell_to_world((cx, cy))
                    break
            if free_world:
                break
        self.assertIsNotNone(free_world)
        local.mark(free_world[0], free_world[1], OCCUPIED)
        source = route_runtime.GridSource.from_occupancy(local, base_grid=base)
        grid, version = source.snapshot()
        self.assertEqual(version, local.map_version)
        self.assertEqual(grid.width, base.width)
        self.assertFalse(grid.is_free(grid.world_to_cell(free_world)))
        # A base-map occupied (static wall) cell stays blocked through merge.
        static_blocked = next((base.cell_to_world((cx, cy))
                               for cy in range(base.height) for cx in range(base.width)
                               if base.cells[cy * base.width + cx] == 1), None)
        self.assertIsNotNone(static_blocked)
        self.assertFalse(grid.is_free(grid.world_to_cell(static_blocked)))

    def test_unknown_defers_to_base_map(self):
        base = GridMap.from_metadata(self.metadata)
        resolution = base.resolution
        local = LocalOccupancyGrid(resolution_m=resolution,
                                   side_m=base.width * resolution,
                                   unknown_is_obstacle=True)
        local.recenter(base.origin[0] + base.width * resolution / 2.0,
                       base.origin[1] + base.height * resolution / 2.0)
        # No depth updates: every window cell is UNKNOWN, and the merged
        # grid must equal the base map (a goal behind an occluder stays
        # plannable; the drone does not refuse to fly over unobserved space).
        source = route_runtime.GridSource.from_occupancy(local, base_grid=base)
        grid, _ = source.snapshot()
        self.assertEqual(list(grid.cells), list(base.cells))

    def test_inflation_dilates_occupied_cells(self):
        base = GridMap.from_metadata(self.metadata)
        resolution = base.resolution
        radius = 3 * resolution
        inflated = route_runtime._inflate_grid(base, radius)
        static_blocked = next(((cx, cy) for cy in range(base.height)
                               for cx in range(base.width)
                               if base.cells[cy * base.width + cx] == 1), None)
        self.assertIsNotNone(static_blocked)
        ox, oy = static_blocked
        # A cell exactly `radius` away from the blocked cell must be blocked.
        target = (ox + 3, oy)
        self.assertTrue(inflated.cells[target[1] * base.width + target[0]])
        # Far outside the inflation radius the map is unchanged.
        far = (ox + 10, oy)
        self.assertEqual(inflated.cells[far[1] * base.width + far[0]],
                         base.cells[far[1] * base.width + far[0]])
        # Zero inflation returns the identical grid object.
        self.assertIs(route_runtime._inflate_grid(base, 0.0), base)

    def test_merged_replan_routes_around_dynamic_wall(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        receipt = Path(temp.name) / "active_launch.json"
        metadata_path = self.world.with_suffix(".json")
        payload = {"schema": route_runtime.RECEIPT_SCHEMA, "launch_id": "unit-test",
                   "created_utc": time.time(), "world_file": str(self.world),
                   "metadata_file": str(metadata_path),
                   "world_sha256": hashlib.sha256(self.world.read_bytes()).hexdigest(),
                   "metadata_sha256": hashlib.sha256(metadata_path.read_bytes()).hexdigest(),
                   "container_id": "test", "container_started_at": time.time() - 1}
        receipt.write_text(json.dumps(payload))
        config = dict(base_config(), scene_receipt_file=str(receipt),
                      world_file=str(self.world), metadata_file=str(metadata_path))
        checked = route_runtime.prepare_route(config)
        base = GridMap.from_metadata(self.metadata)
        resolution = base.resolution
        side = base.width * resolution
        local = LocalOccupancyGrid(resolution_m=resolution, side_m=side,
                                   unknown_is_obstacle=True)
        local.recenter(base.origin[0] + side / 2.0, base.origin[1] + side / 2.0)
        # Block the corridor near (0.9, 0.0) — on the original route past the
        # static wall — plus a 3-cell vertical stripe, then inflate.
        for offset in (-0.25, 0.0, 0.25):
            local.mark(0.9 + offset, 0.0, OCCUPIED)
            local.mark(0.9 + offset, 0.25, OCCUPIED)
            local.mark(0.9 + offset, -0.25, OCCUPIED)
        source = route_runtime.GridSource.from_occupancy(
            local, base_grid=base, inflation_m=0.5)
        points, version = route_runtime.replan_with_grid_source(
            checked, (-3.0, 0.0), (3.0, 0.0), config, source)
        self.assertEqual(version, local.map_version)
        # The route must deviate around the blocked stripe instead of
        # hugging y=0 through x=0.9 (the static-wall route climbs north; the
        # dynamic block forces a wider berth than the static route alone).
        report = checked.safety.validate_route(points, checked.start_world,
                                               config["max_radius_m"], config["max_altitude_m"])
        self.assertTrue(report["valid"])
        deviation = max(abs(point[1] - 0.0) for point in points if abs(point[0] - 0.9) < 1.0)
        self.assertGreater(deviation, 0.8)


class GridSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world = ROOT / "src/robocup_training_worlds/worlds/generated/training_city_unit_single_wall_s42.world"
        cls.metadata = json.loads(cls.world.with_suffix(".json").read_text())

    def config_with_receipt(self):
        temp = tempfile.TemporaryDirectory()
        receipt = Path(temp.name) / "active_launch.json"
        metadata_path = self.world.with_suffix(".json")
        payload = {"schema": route_runtime.RECEIPT_SCHEMA, "launch_id": "unit-test",
                   "created_utc": time.time(), "world_file": str(self.world),
                   "metadata_file": str(metadata_path),
                   "world_sha256": hashlib.sha256(self.world.read_bytes()).hexdigest(),
                   "metadata_sha256": hashlib.sha256(metadata_path.read_bytes()).hexdigest(),
                   "container_id": "test", "container_started_at": time.time() - 1}
        receipt.write_text(json.dumps(payload))
        config = dict(base_config(), scene_receipt_file=str(receipt),
                      world_file=str(self.world), metadata_file=str(metadata_path))
        return temp, config

    def checked_plan(self, config):
        return route_runtime.prepare_route(config)

    def mirror_metadata_into_local_grid(self):
        """Fold the metadata grid into a LocalOccupancyGrid of the same span."""
        meta_grid = GridMap.from_metadata(self.metadata)
        resolution = meta_grid.resolution
        side = meta_grid.width * resolution
        local = LocalOccupancyGrid(resolution_m=resolution, side_m=side,
                                   unknown_is_obstacle=False)
        local.recenter(meta_grid.origin[0] + side / 2.0,
                       meta_grid.origin[1] + side / 2.0)
        bounds = (local.origin[0], local.origin[1],
                  local.origin[0] + local.width * resolution,
                  local.origin[1] + local.height * resolution)
        occupied = []
        for cy in range(meta_grid.height):
            for cx in range(meta_grid.width):
                if meta_grid.cells[cy * meta_grid.width + cx] == 1:
                    wx, wy = meta_grid.cell_to_world((cx, cy))
                    occupied.append((wx, wy))
        self.assertTrue(occupied)
        self.assertTrue(all(bounds[0] <= x < bounds[2] and bounds[1] <= y < bounds[3]
                            for x, y in occupied),
                        "local window must cover every metadata occupied cell")
        for x, y in occupied:
            local.mark(x, y, OCCUPIED)
        return meta_grid, local

    # -- static (metadata) source -------------------------------------------

    def test_static_source_snapshot_is_stable(self):
        source = route_runtime.GridSource.from_metadata(self.metadata)
        grid, version = source.snapshot()
        self.assertIsInstance(grid, GridMap)
        self.assertEqual(version, 0)
        again_grid, again_version = source.snapshot()
        self.assertIs(again_grid, grid)
        self.assertEqual(again_version, version)

    def test_static_source_rejects_invalid_metadata(self):
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^GRID_SOURCE_METADATA_INVALID:"):
            route_runtime.GridSource.from_metadata({"schema": "unsupported"})

    def test_grid_source_validates_grid_and_version(self):
        grid = GridMap.from_metadata(self.metadata)
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^GRID_SOURCE_GRID_INVALID$"):
            route_runtime.GridSource(None)
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^GRID_SOURCE_VERSION_INVALID$"):
            route_runtime.GridSource(grid, -1)
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^GRID_SOURCE_VERSION_INVALID$"):
            route_runtime.GridSource(grid, True)
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^GRID_SOURCE_VERSION_INVALID$"):
            route_runtime.GridSource(grid, 1.0)

    # -- occupancy (perception) source ---------------------------------------

    def test_occupancy_source_folds_live_cells_and_versions(self):
        local = LocalOccupancyGrid(resolution_m=0.25, side_m=10.0,
                                   unknown_is_obstacle=False)
        local.recenter(0.0, 0.0)
        local.mark(0.125, 1.125, OCCUPIED)
        source = route_runtime.GridSource.from_occupancy(local)
        grid, version = source.snapshot()
        self.assertIsInstance(grid, GridMap)
        self.assertEqual(version, local.map_version)
        self.assertFalse(grid.is_free(grid.world_to_cell((0.125, 1.125))))
        # A window recenter resets cells and bumps the version; the next
        # snapshot must reflect the new observation, not a frozen copy.
        local.recenter(3.0, 3.0)
        grid2, version2 = source.snapshot()
        self.assertEqual(version2, local.map_version)
        self.assertGreater(version2, version)
        self.assertTrue(grid2.is_free(grid2.world_to_cell((0.125, 1.125))))

    def test_occupancy_source_rejects_non_grid_and_fold_failure(self):
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^GRID_SOURCE_OCCUPANCY_INVALID$"):
            route_runtime.GridSource.from_occupancy(object())
        source = route_runtime.GridSource.from_occupancy(BrokenOccupancy())
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^GRID_SOURCE_OCCUPANCY_FOLD_FAILED:"):
            source.snapshot()

    # -- versioned replan -----------------------------------------------------

    def test_versioned_replan_matches_explicit_grid_and_metadata_default(self):
        temp, config = self.config_with_receipt()
        self.addCleanup(temp.cleanup)
        checked = self.checked_plan(config)
        meta_grid, local = self.mirror_metadata_into_local_grid()
        direct = route_runtime.replan_route(checked, (-3.0, 0.0), (3.0, 0.0), config)
        # An explicitly supplied metadata grid must not change the result.
        self.assertEqual(
            route_runtime.replan_route(checked, (-3.0, 0.0), (3.0, 0.0), config,
                                       grid=meta_grid),
            direct)
        # A perception grid mirroring the same world replans identically and
        # is stamped with the occupancy version it was proven against.
        points, version = route_runtime.replan_with_grid_source(
            checked, (-3.0, 0.0), (3.0, 0.0), config,
            route_runtime.GridSource.from_occupancy(local))
        self.assertEqual(points, direct)
        self.assertEqual(version, local.map_version)
        self.assertGreater(max(point[1] for point in points), 0.0)

    def test_racing_source_is_replanned_not_committed(self):
        temp, config = self.config_with_receipt()
        self.addCleanup(temp.cleanup)
        checked = self.checked_plan(config)
        _, local = self.mirror_metadata_into_local_grid()
        racer = RacingOccupancy(local.to_grid_map())
        points, version = route_runtime.replan_with_grid_source(
            checked, (-3.0, 0.0), (3.0, 0.0), config,
            route_runtime.GridSource.from_occupancy(racer), max_attempts=2)
        # Attempt 1 raced the world move; attempt 2 replanned on version 1.
        self.assertEqual(version, 1)
        # two folds (plain + shadow) per snapshot, two snapshots per attempt
        self.assertEqual(racer.calls, 8)
        self.assertEqual(route_runtime.replan_with_grid_source(
            checked, (-3.0, 0.0), (3.0, 0.0), config,
            route_runtime.GridSource.from_occupancy(racer), max_attempts=1)[1], 1)
        racer2 = RacingOccupancy(local.to_grid_map())
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^REPLAN_GRID_SOURCE_RACING$"):
            route_runtime.replan_with_grid_source(
                checked, (-3.0, 0.0), (3.0, 0.0), config,
                route_runtime.GridSource.from_occupancy(racer2), max_attempts=1)

    def test_rejects_non_source_and_bad_attempts_and_bad_grid(self):
        temp, config = self.config_with_receipt()
        self.addCleanup(temp.cleanup)
        checked = self.checked_plan(config)
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^REPLAN_GRID_SOURCE_INVALID$"):
            route_runtime.replan_with_grid_source(
                checked, (-3.0, 0.0), (3.0, 0.0), config, self.metadata)
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^REPLAN_MAX_ATTEMPTS_INVALID$"):
            route_runtime.replan_with_grid_source(
                checked, (-3.0, 0.0), (3.0, 0.0), config,
                route_runtime.GridSource.from_metadata(self.metadata), max_attempts=0)
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^REPLAN_GRID_SOURCE_INVALID$"):
            route_runtime.replan_route(checked, (-3.0, 0.0), (3.0, 0.0), config,
                                       grid="not-a-grid")


if __name__ == "__main__":
    unittest.main()
