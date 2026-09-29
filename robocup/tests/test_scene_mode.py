"""Offline contract tests for the scene_mode model-list rule (M3-4).

static keeps the legacy exact-match fail-closed behaviour; perception
tolerates extra dynamic obstacle models (the depth-camera stack's job to
observe) while still requiring every verified scene model at its pose.
"""
import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/robocup_navigation/src"))
from robocup_navigation import route_runtime  # noqa: E402


EXPECTED = ("wall_north", "wall_south", "iris", "training_ground")


class SceneModeTests(unittest.TestCase):
    def test_static_exact_match_is_unchanged(self):
        self.assertEqual(
            route_runtime.check_scene_model_list(EXPECTED, sorted(EXPECTED)), ())
        self.assertEqual(
            route_runtime.check_scene_model_list(EXPECTED, sorted(EXPECTED),
                                                 scene_mode="static"), ())

    def test_static_rejects_any_difference(self):
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^GAZEBO_MODEL_LIST_MISMATCH$"):
            route_runtime.check_scene_model_list(EXPECTED, sorted(EXPECTED) + ["dynamic_wall"])
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^GAZEBO_MODEL_LIST_MISMATCH$"):
            route_runtime.check_scene_model_list(EXPECTED, ["iris", "training_ground"])

    def test_perception_tolerates_dynamic_extras_and_reports_them(self):
        observed = sorted(EXPECTED) + ["dynamic_wall_a", "dynamic_wall_b"]
        extras = route_runtime.check_scene_model_list(EXPECTED, observed,
                                                      scene_mode="perception")
        self.assertEqual(extras, ("dynamic_wall_a", "dynamic_wall_b"))

    def test_perception_still_requires_every_expected_model(self):
        observed = [name for name in EXPECTED if name != "wall_south"] + ["dynamic_wall"]
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^GAZEBO_MODEL_LIST_MISSING:wall_south$"):
            route_runtime.check_scene_model_list(EXPECTED, observed,
                                                 scene_mode="perception")

    def test_perception_caps_dynamic_model_count(self):
        observed = sorted(EXPECTED) + ["wall_%d" % index for index in range(9)]
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^GAZEBO_MODEL_LIST_OVERFLOW:"):
            route_runtime.check_scene_model_list(EXPECTED, observed,
                                                 scene_mode="perception",
                                                 max_dynamic_models=8)
        # At or under the cap the same list is accepted and reported.
        extras = route_runtime.check_scene_model_list(EXPECTED, observed,
                                                      scene_mode="perception",
                                                      max_dynamic_models=9)
        self.assertEqual(len(extras), 9)
        # A cap of zero rejects any dynamic model.
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^GAZEBO_MODEL_LIST_OVERFLOW:dynamic_wall$"):
            route_runtime.check_scene_model_list(EXPECTED, sorted(EXPECTED) + ["dynamic_wall"],
                                                 scene_mode="perception",
                                                 max_dynamic_models=0)

    def test_rejects_invalid_mode_and_limit(self):
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^SCENE_MODE_INVALID:hybrid$"):
            route_runtime.check_scene_model_list(EXPECTED, sorted(EXPECTED),
                                                 scene_mode="hybrid")
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^SCENE_MODE_DYNAMIC_LIMIT_INVALID$"):
            route_runtime.check_scene_model_list(EXPECTED, sorted(EXPECTED),
                                                 scene_mode="perception",
                                                 max_dynamic_models=True)
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^SCENE_MODE_DYNAMIC_LIMIT_INVALID$"):
            route_runtime.check_scene_model_list(EXPECTED, sorted(EXPECTED),
                                                 scene_mode="perception",
                                                 max_dynamic_models=-1)

    def test_verify_runtime_models_ignores_dynamic_extras(self):
        # The pose contract only covers expected models; extra observations
        # are never a pose failure.  Mirrors the Plan fixture of
        # test_route_runtime.py.
        class Plan(object):
            expected_models = (route_runtime.ExpectedModel(
                "wall_north", (0.0, -1.0, 0.5), 0.0, (4.0, 0.2, 1.0), (0.0, -1.0, 0.5)),)
            ground_model = route_runtime.ExpectedModel(
                "training_ground", (0.0, 0.0, 0.0), 0.0, (40.0, 40.0, 0.1), (0.0, 0.0, 0.0))
        observed = {
            "wall_north": ((0.0, -1.0, 0.5), 0.0),
            "training_ground": ((0.0, 0.0, 0.0), 0.0),
            "dynamic_wall": ((2.0, 2.0, 0.5), 0.7),
        }
        self.assertTrue(route_runtime.verify_runtime_models(Plan(), observed))
        # ...while a moved expected model still fails closed.
        observed["wall_north"] = ((0.0, -1.5, 0.5), 0.0)
        with self.assertRaisesRegex(route_runtime.RouteError,
                                    "^GAZEBO_MODEL_POSE_MISMATCH:wall_north$"):
            route_runtime.verify_runtime_models(Plan(), observed)

    def test_scene_modes_constant(self):
        self.assertEqual(route_runtime.SCENE_MODES, ("static", "perception"))


if __name__ == "__main__":
    unittest.main()
