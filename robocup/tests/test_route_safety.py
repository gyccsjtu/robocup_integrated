"""Offline regression tests for continuous route safety checks."""

import copy
import json
import math
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/robocup_navigation/src"))

from robocup_navigation.route_safety import (  # noqa: E402
    SafetyError,
    SafetyMap,
    WorldLocalTransform,
)


METADATA_PATH = (ROOT / "src/robocup_training_worlds/worlds/generated/"
                 "training_city_unit_single_wall_s42.json")


def box_obstacle(oid="wall", center=(0.0, 0.0), size=(0.4, 2.0),
                 height=2.0, z_min=0.0, blocking=True, yaw=0.0):
    x, y = center
    sx, sy = size
    quarter = int(round(yaw / (math.pi / 2.0))) % 4
    world_sx, world_sy = (sy, sx) if quarter % 2 else (sx, sy)
    return {
        "id": oid,
        "type": "wall",
        "shape": "box",
        "center": [x, y],
        "size": [sx, sy],
        "yaw": yaw,
        "height": height,
        "z_min": z_min,
        "z_max": z_min + height,
        "blocking": blocking,
        "bbox": [x - world_sx / 2.0, y - world_sy / 2.0,
                 x + world_sx / 2.0, y + world_sy / 2.0],
    }


def circle_obstacle(oid="post", center=(0.0, 0.0), radius=0.2,
                    height=2.0, z_min=0.0):
    x, y = center
    return {
        "id": oid,
        "type": "lamp_post",
        "shape": "cylinder",
        "center": [x, y],
        "radius": radius,
        "height": height,
        "z_min": z_min,
        "z_max": z_min + height,
        "blocking": False,
        "bbox": [x - radius, y - radius, x + radius, y + radius],
        "footprint": {
            "kind": "circle",
            "center": [x, y],
            "radius": radius,
        },
    }


def metadata_for(obstacles=None, bounds=(-5.0, -5.0, 5.0, 5.0),
                 ground_z=0.0):
    x_min, y_min, x_max, y_max = bounds
    return {
        "schema": "robocup_training_worlds/metadata/v1",
        "frame": {
            "frame_id": "map",
            "convention": "ENU",
            "units": "m",
            "ground_z": ground_z,
        },
        "bounds": {
            "x_min": x_min,
            "y_min": y_min,
            "x_max": x_max,
            "y_max": y_max,
        },
        "obstacles": list(obstacles or []),
        # This is intentionally not needed by SafetyMap.  Its inflation is
        # A* data and must not be applied a second time here.
        "grid": {"frame_id": "map", "inflation_m": 0.6},
    }


class SafetyMapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.metadata = json.loads(METADATA_PATH.read_text(encoding="utf-8"))

    def test_verified_defaults_and_original_geometry_are_used(self):
        safety_map = SafetyMap(self.metadata)
        self.assertAlmostEqual(safety_map.vehicle_radius_m, 0.40)
        self.assertAlmostEqual(safety_map.horizontal_margin_m, 0.20)
        self.assertAlmostEqual(safety_map.vehicle_half_height_m, 0.15)
        self.assertAlmostEqual(safety_map.vertical_margin_m, 0.10)

        # The point is inside the original wall footprint.  It remains unsafe
        # even if every planning-cell flag says that the map is free and every
        # obstacle is marked non-blocking.
        altered = copy.deepcopy(self.metadata)
        altered["grid"]["inflation_m"] = 99.0
        altered["grid"]["data"] = [[0, 2500]]
        for obstacle in altered["obstacles"]:
            obstacle["blocking"] = False
        self.assertFalse(SafetyMap(altered).point_safe((0.2, -2.0, 1.5)))

    def test_clearance_subtracts_radius_but_not_horizontal_margin(self):
        safety_map = SafetyMap(metadata_for([box_obstacle(
            center=(0.0, 0.0), size=(0.4, 2.0))]))
        # At x=1.0 the geometric clearance to the box is 0.8 m; after the
        # 0.4 m body radius, 0.4 m remains.  The 0.2 m margin is not included.
        self.assertAlmostEqual(safety_map.clearance((1.0, 0.0, 1.0)), 0.4)
        self.assertTrue(safety_map.point_safe((1.0, 0.0, 1.0)))
        self.assertFalse(safety_map.point_safe((0.79, 0.0, 1.0)))

    def test_diagonal_segment_cutting_wall_is_rejected(self):
        safety_map = SafetyMap(metadata_for([box_obstacle(
            center=(0.0, 0.0), size=(0.4, 2.0))]))
        with self.assertRaises(SafetyError):
            safety_map.validate_route(
                [(-1.0, -1.0, 1.0), (1.0, 1.0, 1.0)],
                origin_xyz=(-1.0, -1.0, 0.0),
                max_radius_m=5.0,
                max_altitude_m=2.0,
            )

    def test_segment_join_to_waypoint_is_checked_continuously(self):
        safety_map = SafetyMap(metadata_for([box_obstacle(
            center=(0.0, 0.0), size=(0.4, 2.0))]))
        # Both endpoints are clear, but the joining line cuts the upper-right
        # corner once the vehicle radius and margin are swept along it.
        with self.assertRaises(SafetyError):
            safety_map.validate_route(
                [(-1.0, 1.7, 1.0), (0.9, 1.7, 1.0),
                 (-0.9, -1.7, 1.0)],
                origin_xyz=(-1.0, 1.6, 0.0),
                max_radius_m=5.0,
                max_altitude_m=2.0,
            )

        safe = safety_map.validate_route(
            [(-1.0, 1.7, 1.0), (1.0, 1.7, 1.0),
             (1.0, -1.7, 1.0), (-1.0, -1.7, 1.0)],
            origin_xyz=(-1.0, 1.7, 0.0),
            max_radius_m=5.0,
            max_altitude_m=2.0,
        )
        self.assertTrue(safe["valid"])
        self.assertGreaterEqual(safe["minimum_clearance_m"], 0.2 - 1.0e-8)

    def test_boundary_and_ground_contact_are_handled(self):
        safety_map = SafetyMap(metadata_for(bounds=(-2.0, -2.0, 2.0, 2.0)))
        self.assertAlmostEqual(safety_map.clearance((1.5, 0.0, 0.0)), 0.1)
        self.assertTrue(safety_map.point_safe((0.0, 0.0, 0.0)))
        self.assertTrue(safety_map.landing_safe((0.0, 0.0, 0.0)))
        self.assertFalse(safety_map.point_safe((1.5, 0.0, 0.0)))
        with self.assertRaises(SafetyError):
            safety_map.validate_route(
                [(0.0, 0.0, 1.0), (1.5, 0.0, 1.0)],
                origin_xyz=(0.0, 0.0, 0.0),
                max_radius_m=5.0,
                max_altitude_m=2.0,
            )

    def test_grounded_vertical_landing_corridor_is_required(self):
        obstacle = box_obstacle(center=(0.0, 0.0), size=(0.4, 0.4),
                                blocking=False)
        safety_map = SafetyMap(metadata_for([obstacle]))
        self.assertFalse(safety_map.landing_safe((0.0, 0.0, 10.0)))
        self.assertFalse(safety_map.point_safe((0.0, 0.0, 10.0)))
        self.assertTrue(safety_map.landing_safe((1.0, 0.0, 10.0)))

        # The ground plane is not inserted into the obstacle set: an empty,
        # bounded map permits a ground-contact endpoint.
        empty_map = SafetyMap(metadata_for())
        self.assertTrue(empty_map.landing_safe((0.0, 0.0, 0.0)))

    def test_circle_footprint_is_supported_and_continuous(self):
        safety_map = SafetyMap(metadata_for([circle_obstacle()]))
        self.assertAlmostEqual(safety_map.clearance((1.0, 0.0, 1.0)), 0.4)
        with self.assertRaises(SafetyError):
            safety_map.validate_route(
                [(-1.0, -1.0, 1.0), (1.0, 1.0, 1.0)],
                origin_xyz=(-1.0, -1.0, 0.0),
                max_radius_m=5.0,
                max_altitude_m=2.0,
            )

    def test_nonfinite_points_and_metadata_are_rejected(self):
        with self.assertRaises(SafetyError):
            SafetyMap(metadata_for(ground_z=float("nan")))

        malformed = metadata_for([box_obstacle()])
        malformed["bounds"]["x_min"] = float("inf")
        with self.assertRaises(SafetyError):
            SafetyMap(malformed)

        malformed = metadata_for([box_obstacle()])
        malformed["obstacles"][0]["center"][0] = float("nan")
        with self.assertRaises(SafetyError):
            SafetyMap(malformed)

        safety_map = SafetyMap(metadata_for())
        with self.assertRaises(SafetyError):
            safety_map.clearance((float("nan"), 0.0, 1.0))
        self.assertFalse(safety_map.point_safe((0.0, 0.0, float("inf"))))
        with self.assertRaises(SafetyError):
            safety_map.validate_route(
                [(0.0, 0.0, float("nan"))], (0.0, 0.0, 0.0), 5.0, 2.0)

    def test_unknown_and_non_grounded_geometry_are_rejected(self):
        unknown = metadata_for([box_obstacle()])
        unknown["obstacles"][0]["shape"] = "mesh"
        with self.assertRaisesRegex(SafetyError, "UNSUPPORTED_OBSTACLE_GEOMETRY"):
            SafetyMap(unknown)

        rotated = metadata_for([box_obstacle(yaw=math.pi / 4.0)])
        with self.assertRaisesRegex(SafetyError, "NOT_AXIS_ALIGNED"):
            SafetyMap(rotated)

        floating = metadata_for([box_obstacle(z_min=0.5)])
        with self.assertRaisesRegex(SafetyError, "NOT_GROUNDED"):
            SafetyMap(floating)

    def test_duplicate_obstacle_ids_are_rejected(self):
        duplicate = metadata_for([
            box_obstacle(oid="same", center=(-1.0, 0.0)),
            box_obstacle(oid="same", center=(1.0, 0.0)),
        ])
        with self.assertRaisesRegex(SafetyError, "DUPLICATE_OBSTACLE_ID"):
            SafetyMap(duplicate)

    def test_axis_aligned_quarter_turn_is_accepted(self):
        obstacle = box_obstacle(center=(0.0, 0.0), size=(2.0, 0.4),
                                yaw=math.pi / 2.0)
        safety_map = SafetyMap(metadata_for([obstacle]))
        self.assertLess(safety_map.clearance((0.0, 1.0, 1.0)), 0.0)

    def test_frame_schema_grid_frame_and_bounds_are_strict(self):
        malformed = metadata_for()
        malformed["schema"] = "other/v1"
        with self.assertRaises(SafetyError):
            SafetyMap(malformed)

        malformed = metadata_for()
        malformed["frame"]["convention"] = "NED"
        with self.assertRaises(SafetyError):
            SafetyMap(malformed)

        malformed = metadata_for()
        malformed["grid"]["frame_id"] = "odom"
        with self.assertRaises(SafetyError):
            SafetyMap(malformed)

        malformed = metadata_for(bounds=(1.0, -1.0, -1.0, 1.0))
        with self.assertRaises(SafetyError):
            SafetyMap(malformed)

    def test_route_height_radius_and_minimum_clearance(self):
        safety_map = SafetyMap(metadata_for())
        result = safety_map.validate_route(
            [(0.0, 0.0, 1.0), (1.0, 0.0, 1.0)],
            origin_xyz=(0.0, 0.0, 0.0),
            max_radius_m=2.0,
            max_altitude_m=1.3,
        )
        self.assertTrue(result["valid"])
        self.assertAlmostEqual(result["minimum_clearance_m"], 3.6)

        with self.assertRaises(SafetyError):
            safety_map.validate_route(
                [(0.0, 0.0, 1.1)], (0.0, 0.0, 0.0), 2.0, 1.3)
        one_point = safety_map.validate_route(
            [(0.0, 0.0, 1.0)], (0.0, 0.0, 0.0), 2.0, 1.3)
        self.assertAlmostEqual(one_point["minimum_clearance_m"], 4.6)
        with self.assertRaises(SafetyError):
            safety_map.validate_route(
                [(0.0, 0.0, 1.0), (1.9, 0.0, 1.0)],
                (0.0, 0.0, 0.0), 2.0, 2.0)

    def test_world_local_transform_forward_reverse_yaw_and_roundtrip(self):
        transform = WorldLocalTransform(
            world_origin_xyz=(10.0, 20.0, 30.0),
            local_origin_xyz=(-1.0, 2.0, 3.0),
            yaw_rad=math.pi / 2.0,
        )
        local = transform.world_to_local((10.0, 21.0, 31.0))
        self.assertEqual(local, (0.0, 2.0, 4.0))
        self.assertEqual(transform.local_to_world(local), (10.0, 21.0, 31.0))
        self.assertTrue(transform.validate_pair((10.0, 21.0, 31.0), local,
                                                tolerance_m=1.0e-9))
        self.assertFalse(transform.validate_pair((10.0, 21.0, 31.0),
                                                 (0.1, 2.0, 4.0),
                                                 tolerance_m=1.0e-3))

        reverse = WorldLocalTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                      yaw_rad=-math.pi / 2.0)
        self.assertAlmostEqual(reverse.world_to_local((0.0, 1.0, 0.0))[0],
                               -1.0)
        self.assertAlmostEqual(reverse.world_to_local((0.0, 1.0, 0.0))[1],
                               0.0)

    def test_planar_transform_allows_estimator_z_rebase_only(self):
        transform = WorldLocalTransform((10.0, 20.0, 1.0),
                                        (0.0, 0.0, -0.2))
        self.assertTrue(transform.validate_planar_pair(
            (11.0, 22.0, 5.0), (1.0, 2.0, 99.0), tolerance_m=1.0e-9))
        self.assertFalse(transform.validate_planar_pair(
            (11.0, 22.0, 5.0), (1.3, 2.0, 99.0), tolerance_m=0.2))

    def test_world_local_transform_rejects_nonfinite_inputs(self):
        with self.assertRaises(SafetyError):
            WorldLocalTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                yaw_rad=float("nan"))
        transform = WorldLocalTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        with self.assertRaises(SafetyError):
            transform.world_to_local((0.0, float("inf"), 0.0))
        with self.assertRaises(SafetyError):
            transform.validate_pair((0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                    tolerance_m=-1.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
