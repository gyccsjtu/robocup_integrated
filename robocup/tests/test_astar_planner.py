"""Offline A* contract and topology tests over generated training metadata."""

import copy
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/robocup_navigation/src"))

from robocup_navigation.astar import (GridMap, MapError, metadata_endpoints,  # noqa: E402
                                      plan, render_ascii)


GENERATOR = ROOT / "src/robocup_training_worlds/scripts/generate_training_city.py"
WORLD_CONFIG = ROOT / "src/robocup_training_worlds/config/training_city.yaml"


class GeneratedMaps(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.metadata = {}
        for category in ("single_wall", "wall_with_gap", "u_shape",
                         "narrow_corridor", "blocked", "no_path",
                         "goal_in_obstacle"):
            world = Path(cls.temporary.name) / (category + ".world")
            subprocess.run([
                sys.executable, str(GENERATOR), "--config", str(WORLD_CONFIG),
                "--preset", "unit", "--seed", "42", "--category", category,
                "--output", str(world),
            ], check=True, stdout=subprocess.DEVNULL)
            cls.metadata[category] = json.loads(world.with_suffix(".json").read_text())

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def route(self, category):
        metadata = self.metadata[category]
        grid = GridMap.from_metadata(metadata)
        start, goal, _goal_id = metadata_endpoints(metadata)
        return grid, plan(grid, start, goal)

    def assert_route_is_collision_free(self, grid, result):
        self.assertTrue(result.success, result.reason)
        self.assertGreater(len(result.cells), 1)
        for cell in result.cells:
            self.assertTrue(grid.is_free(cell), cell)
        for current, following in zip(result.cells, result.cells[1:]):
            dx, dy = following[0] - current[0], following[1] - current[1]
            self.assertIn((dx, dy), [(1, 0), (-1, 0), (0, 1), (0, -1),
                                      (1, 1), (1, -1), (-1, 1), (-1, -1)])
            if dx and dy:
                self.assertTrue(grid.is_free((current[0] + dx, current[1])))
                self.assertTrue(grid.is_free((current[0], current[1] + dy)))

    def test_single_wall_requires_a_detour(self):
        grid, result = self.route("single_wall")
        self.assert_route_is_collision_free(grid, result)
        start, goal = result.start_cell, result.goal_cell
        self.assertEqual(start[1], goal[1])
        self.assertTrue(any(not grid.is_free((x, start[1]))
                            for x in range(start[0], goal[0] + 1)))
        self.assertTrue(any(cell[1] != start[1] for cell in result.cells))
        self.assertGreater(result.path_length_m,
                           math.dist(result.points[0], result.points[-1]))

    def test_positive_unit_topologies(self):
        for category in ("wall_with_gap", "u_shape", "narrow_corridor"):
            with self.subTest(category=category):
                grid, result = self.route(category)
                self.assert_route_is_collision_free(grid, result)

    def test_blocked_topologies_return_no_path(self):
        for category in ("blocked", "no_path"):
            with self.subTest(category=category):
                _grid, result = self.route(category)
                self.assertFalse(result.success)
                self.assertEqual(result.reason, "NO_PATH")

    def test_invalid_goal_is_rejected_before_planning(self):
        metadata = self.metadata["goal_in_obstacle"]
        with self.assertRaisesRegex(MapError, "GOAL_NOT_OUTDOORS"):
            metadata_endpoints(metadata)

    def test_same_input_has_deterministic_path(self):
        _grid, first = self.route("single_wall")
        _grid, second = self.route("single_wall")
        self.assertEqual(first, second)

    def test_corrupt_rle_is_rejected(self):
        metadata = copy.deepcopy(self.metadata["single_wall"])
        metadata["grid"]["data"][-1][1] -= 1
        with self.assertRaisesRegex(MapError, "RLE_SIZE_MISMATCH"):
            GridMap.from_metadata(metadata)

    def test_world_endpoints_outside_grid_are_reported(self):
        grid, _result = self.route("single_wall")
        result = plan(grid, (-999.0, 0.0), (3.0, 0.0))
        self.assertEqual(result.reason, "START_OUT_OF_BOUNDS")
        result = plan(grid, (-3.0, 0.0), (999.0, 0.0))
        self.assertEqual(result.reason, "GOAL_OUT_OF_BOUNDS")

    def test_occupied_endpoint_is_reported(self):
        grid, _result = self.route("single_wall")
        blocked = next((index % grid.width, index // grid.width)
                       for index, value in enumerate(grid.cells) if value)
        blocked_world = grid.cell_to_world(blocked)
        self.assertEqual(plan(grid, blocked_world, (3.0, 0.0)).reason, "START_OCCUPIED")
        self.assertEqual(plan(grid, (-3.0, 0.0), blocked_world).reason, "GOAL_OCCUPIED")

    def test_diagonal_corner_squeeze_is_forbidden(self):
        # Row-major: S# / #G. Only an illegal diagonal can connect the cells.
        grid = GridMap(2, 2, 1.0, (0.0, 0.0), bytes([0, 1, 1, 0]))
        result = plan(grid, (0.5, 0.5), (1.5, 1.5), connectivity=8,
                      prevent_corner_cutting=True)
        self.assertEqual(result.reason, "NO_PATH")

    def test_ascii_contains_start_goal_route_and_obstacles(self):
        grid, result = self.route("single_wall")
        drawing = render_ascii(grid, result)
        for marker in "SG*#":
            self.assertIn(marker, drawing)

    def test_expansion_budget_is_bounded(self):
        grid = GridMap(10, 10, 1.0, (0.0, 0.0), bytes(100))
        result = plan(grid, (0.5, 0.5), (9.5, 9.5), max_expansions=1)
        self.assertEqual(result.reason, "EXPANSION_LIMIT")
        self.assertEqual(result.expanded_nodes, 1)
        self.assertEqual(result.cells, ())

    def test_float_rle_and_wrong_frame_are_rejected(self):
        metadata = copy.deepcopy(self.metadata["single_wall"])
        metadata["grid"]["data"][0][0] = 0.0
        with self.assertRaisesRegex(MapError, "RLE_VALUE_INVALID"):
            GridMap.from_metadata(metadata)
        metadata = copy.deepcopy(self.metadata["single_wall"])
        metadata["frame"]["convention"] = "NED"
        with self.assertRaisesRegex(MapError, "FRAME_CONVENTION_INVALID"):
            GridMap.from_metadata(metadata)

    def test_known_open_grid_shortest_distance(self):
        grid = GridMap(5, 4, 0.5, (-1.0, -1.0), bytes(20))
        start, goal = grid.cell_to_world((0, 0)), grid.cell_to_world((4, 3))
        self.assertAlmostEqual(plan(grid, start, goal, connectivity=4).path_length_m, 3.5)
        self.assertAlmostEqual(plan(grid, start, goal).path_length_m,
                               (3 * math.sqrt(2) + 1) * 0.5)

    def test_cli_success_no_path_and_stale_output_replacement(self):
        with tempfile.TemporaryDirectory() as directory:
            metadata_path = Path(directory) / "input with spaces.json"
            output = Path(directory) / "route.json"
            drawing = Path(directory) / "route.txt"
            environment = dict(os.environ)
            environment["PYTHONPATH"] = str(ROOT / "src/robocup_navigation/src")
            command = [sys.executable,
                       str(ROOT / "src/robocup_navigation/scripts/plan_training_route.py"),
                       "--metadata", str(metadata_path), "--config",
                       str(ROOT / "src/robocup_navigation/config/astar_planner.yaml"),
                       "--output", str(output), "--ascii-output", str(drawing)]
            for category, exit_code, reason in (("single_wall", 0, "SUCCEEDED"),
                                               ("blocked", 4, "NO_PATH")):
                metadata_path.write_text(json.dumps(self.metadata[category]))
                completed = subprocess.run(command, env=environment, capture_output=True, text=True)
                self.assertEqual(completed.returncode, exit_code, completed.stdout + completed.stderr)
                payload = json.loads(output.read_text())
                self.assertEqual(payload["reason"], reason)
                self.assertFalse(payload["executable"])
            metadata_path.write_text("{broken")
            completed = subprocess.run(command, env=environment, capture_output=True, text=True)
            self.assertEqual(completed.returncode, 2, completed.stdout + completed.stderr)
            payload = json.loads(output.read_text())
            self.assertFalse(payload["success"])
            self.assertEqual(payload["path_world"], [])
            self.assertIn("PLAN_INPUT_ERROR", drawing.read_text())


if __name__ == "__main__":
    unittest.main(verbosity=2)
