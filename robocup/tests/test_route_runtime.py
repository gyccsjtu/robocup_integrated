"""Offline contract tests for route-mode receipt, geometry, and fresh A*."""
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


class RouteRuntimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world = ROOT / "src/robocup_training_worlds/worlds/generated/training_city_unit_single_wall_s42.world"
        cls.metadata = cls.world.with_suffix(".json")
        cls.base = {
            "scene_receipt_max_age_s": 600.0,
            "route_start_world_xy": (-3.0, 0.0), "route_goal_world_xy": (3.0, 0.0),
            "route_altitude_m": 2.4, "route_max_expansions": 250000,
            "route_planning_timeout_s": 5.0, "vehicle_radius_m": 0.40,
            "horizontal_margin_m": 0.20, "vehicle_half_height_m": 0.15,
            "vertical_margin_m": 0.30, "minimum_clearance_m": 0.20,
            "planning_tracking_margin_m": 0.20,
            "max_radius_m": 7.0, "max_altitude_m": 3.0,
        }

    def config_with_receipt(self):
        temp = tempfile.TemporaryDirectory()
        receipt = Path(temp.name) / "active_launch.json"
        payload = {"schema": route_runtime.RECEIPT_SCHEMA, "launch_id": "unit-test",
                   "created_utc": time.time(), "world_file": str(self.world),
                   "metadata_file": str(self.metadata), "world_sha256": hashlib.sha256(self.world.read_bytes()).hexdigest(),
                   "metadata_sha256": hashlib.sha256(self.metadata.read_bytes()).hexdigest(),
                   "container_id": "test", "container_started_at": time.time() - 1}
        receipt.write_text(json.dumps(payload))
        config = dict(self.base, scene_receipt_file=str(receipt), world_file=str(self.world), metadata_file=str(self.metadata))
        return temp, config

    def test_receipt_and_world_match_metadata(self):
        temp, config = self.config_with_receipt()
        self.addCleanup(temp.cleanup)
        receipt = route_runtime.load_scene_receipt(config)
        self.assertEqual(receipt["world_file"], str(self.world))
        metadata = json.loads(self.metadata.read_text())
        expected = route_runtime.verify_world_geometry(metadata, str(self.world))
        self.assertEqual({item.name for item in expected}, {item["id"] for item in metadata["obstacles"]})

    def test_rejects_stale_or_changed_receipt_input(self):
        temp, config = self.config_with_receipt()
        self.addCleanup(temp.cleanup)
        receipt_path = Path(config["scene_receipt_file"])
        payload = json.loads(receipt_path.read_text())
        payload["created_utc"] = time.time() - 601
        receipt_path.write_text(json.dumps(payload))
        with self.assertRaisesRegex(route_runtime.RouteError, "SCENE_RECEIPT_STALE"):
            route_runtime.load_scene_receipt(config)
        payload["created_utc"] = time.time()
        payload["world_sha256"] = "0" * 64
        receipt_path.write_text(json.dumps(payload))
        with self.assertRaisesRegex(route_runtime.RouteError, "WORLD_SHA256_MISMATCH"):
            route_runtime.load_scene_receipt(config)

    def test_rejects_live_model_pose_mismatch(self):
        temp, config = self.config_with_receipt()
        self.addCleanup(temp.cleanup)
        metadata = json.loads(self.metadata.read_text())
        models = route_runtime.verify_world_geometry(metadata, config["world_file"])
        class Plan:
            expected_models = models
        observed = {item.name: (item.model_position, item.yaw_rad) for item in models}
        self.assertTrue(route_runtime.verify_runtime_models(Plan(), observed))
        first = models[0]
        observed[first.name] = ((first.model_position[0] + 1, first.model_position[1], first.model_position[2]), first.yaw_rad)
        with self.assertRaisesRegex(route_runtime.RouteError, "GAZEBO_MODEL_POSE_MISMATCH"):
            route_runtime.verify_runtime_models(Plan(), observed)

    def test_fresh_plan_has_exact_endpoints_and_safe_takeoff_line(self):
        temp, config = self.config_with_receipt()
        self.addCleanup(temp.cleanup)
        plan = route_runtime.prepare_route(config)
        self.assertEqual(plan.route_world[0], (-3.0, 0.0, 2.4))
        self.assertEqual(plan.route_world[-1], (3.0, 0.0, 2.4))
        self.assertGreaterEqual(plan.safety.clearance(plan.route_world[0]), 0.20)

    def test_grid_inflation_must_include_execution_margin(self):
        temp, config = self.config_with_receipt()
        self.addCleanup(temp.cleanup)
        config["planning_tracking_margin_m"] = 0.30
        with self.assertRaisesRegex(route_runtime.RouteError, "GRID_INFLATION_TOO_SMALL_FOR_EXECUTION"):
            route_runtime.prepare_route(config)

    def test_online_replan_returns_exact_immutable_route_around_wall(self):
        temp, config = self.config_with_receipt()
        self.addCleanup(temp.cleanup)
        checked = route_runtime.prepare_route(config)
        route = route_runtime.replan_route(checked, (-3.0, 0.0), (3.0, 0.0), config)
        self.assertIsInstance(route, tuple)
        self.assertEqual(route[0], (-3.0, 0.0, 2.4))
        self.assertEqual(route[-1], (3.0, 0.0, 2.4))
        self.assertTrue(all(isinstance(point, tuple) and point[2] == 2.4 for point in route))
        # The wall reaches y=-0.416; a valid route from left to right must
        # climb around its north end rather than cut through its footprint.
        self.assertGreater(max(point[1] for point in route), 0.0)
        report = checked.safety.validate_route(route, checked.start_world,
                                               config["max_radius_m"], config["max_altitude_m"])
        self.assertTrue(report["valid"])
        live_route = route_runtime.replan_route(
            checked, (-2.15, 0.12), (2.5, -2.0), config)
        self.assertEqual(live_route[0], (-2.15, 0.12, 2.4))
        self.assertEqual(live_route[-1], (2.5, -2.0, 2.4))

    def test_online_replan_rejects_goal_outside_reserved_radius(self):
        temp, config = self.config_with_receipt()
        self.addCleanup(temp.cleanup)
        checked = route_runtime.prepare_route(config)
        # The centre of (3, -3) is within 7 m of (-3, 0), but the 0.40 m
        # vehicle radius and 0.20 m static margin must also remain inside the
        # flight fence.  It is therefore not a safe external target.
        with self.assertRaisesRegex(route_runtime.RouteError, "^REPLAN_GOAL_RADIUS_LIMIT$"):
            route_runtime.replan_route(checked, (-2.15, 0.12), (3.0, -3.0), config)

    def test_online_replan_rejects_occupied_goal_and_out_of_bounds_goal(self):
        temp, config = self.config_with_receipt()
        self.addCleanup(temp.cleanup)
        checked = route_runtime.prepare_route(config)
        with self.assertRaisesRegex(route_runtime.RouteError, "^REPLAN_GOAL_OCCUPIED$"):
            route_runtime.replan_route(checked, (-3.0, 0.0), (0.2, -1.0), config)
        with self.assertRaisesRegex(route_runtime.RouteError, "^REPLAN_GOAL_OUT_OF_BOUNDS$"):
            route_runtime.replan_route(checked, (-3.0, 0.0), (6.0, 0.0), config)

    def test_online_replan_rejects_unsafe_start_connector(self):
        temp, config = self.config_with_receipt()
        self.addCleanup(temp.cleanup)
        checked = route_runtime.prepare_route(config)
        # A valid current point is not enough: the first exact connector must
        # also fit inside the radial flight fence plus body clearance.  Keep
        # the exact goal inside that same reserved fence; otherwise the
        # earlier goal-fence guard correctly wins and this would not exercise
        # the start-connector state.
        config["max_radius_m"] = 0.70
        with self.assertRaisesRegex(route_runtime.RouteError, "^REPLAN_START_CONNECTION_UNSAFE$"):
            route_runtime.replan_route(checked, (-3.0, 0.0), (-3.05, 0.0), config)

    def test_online_replan_rejects_route_without_required_clearance_reserve(self):
        temp, config = self.config_with_receipt()
        self.addCleanup(temp.cleanup)
        checked = route_runtime.prepare_route(config)
        config["minimum_clearance_m"] = 1.0
        with self.assertRaisesRegex(route_runtime.RouteError, "^REPLAN_CLEARANCE_INSUFFICIENT$"):
            route_runtime.replan_route(checked, (-3.0, 0.0), (3.0, 0.0), config)

    def test_acceleration_limited_slew_bounds_difference_and_stops(self):
        position, velocity = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        previous_velocity, previous_position = velocity, position
        for _index in range(2000):
            position, velocity = route_runtime.acceleration_limited_slew(
                position, (1.0, 0.0, 0.0), velocity, 0.05, 0.30, 0.25, 0.25, 0.20)
            actual_velocity = tuple((position[index] - previous_position[index]) / .05 for index in range(3))
            self.assertLessEqual(((actual_velocity[0] - previous_velocity[0]) ** 2 + (actual_velocity[1] - previous_velocity[1]) ** 2) ** .5, .25 * .05 + 1e-9)
            self.assertLessEqual(abs(actual_velocity[2] - previous_velocity[2]), .20 * .05 + 1e-9)
            previous_velocity, previous_position = actual_velocity, position
            if position == (1.0, 0.0, 0.0) and velocity == (0.0, 0.0, 0.0):
                break
        self.assertEqual(position, (1.0, 0.0, 0.0))
        self.assertEqual(velocity, (0.0, 0.0, 0.0))

    def test_slew_snaps_target_crossing_inside_terminal_band(self):
        # Regression for the SLEW_BRAKE flake: the physical controller crosses
        # a waypoint with a small residual velocity during takeoff/cruise
        # settling (z-overshoot ~0.1 m).  A crossing within
        # SLEW_SNAP_TOLERANCE_M at a speed the vehicle can actually fly must
        # snap the reference instead of failing the whole mission.
        pos, vel = route_runtime.acceleration_limited_slew(
            (0.99, 0.0, 1.97), (1.0, 0.0, 1.96), (0.25, 0.0, -0.25),
            0.05, 0.30, 0.25, 0.25, 0.20)
        self.assertEqual(pos, (1.0, 0.0, 1.96))
        self.assertEqual(vel, (0.0, 0.0, 0.0))

    def test_slew_still_raises_crossing_faster_than_vehicle_inside_band(self):
        # Review finding: a crossing inside the snap band but FASTER than the
        # vehicle's own speed limit is a fabricated state, not an overshoot —
        # it must fail closed even though the distance is small.
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^SLEW_BRAKE_INVARIANT_VIOLATED$"):
            route_runtime.acceleration_limited_slew(
                (0.98, 0.0, 1.97), (1.0, 0.0, 1.96), (3.0, 0.0, -0.30),
                0.05, 0.30, 0.25, 0.25, 0.20)

    def test_slew_still_raises_crossing_outside_terminal_band(self):
        # A real runaway (large remaining distance crossed at speed) must keep
        # failing closed.
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^SLEW_BRAKE_INVARIANT_VIOLATED$"):
            route_runtime.acceleration_limited_slew(
                (0.9, 0.0, 0.0), (1.0, 0.0, 0.0), (3.0, 0.0, 0.0),
                0.05, 0.30, 0.25, 0.25, 0.20)
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^SLEW_BRAKE_INVARIANT_VIOLATED$"):
            route_runtime.acceleration_limited_slew(
                (0.0, 0.0, 0.0), (0.0, 0.0, 0.2), (0.0, 0.0, -5.0),
                0.05, 0.30, 0.25, 0.25, 0.20)


if __name__ == "__main__":
    unittest.main(verbosity=2)
