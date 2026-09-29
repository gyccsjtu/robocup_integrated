"""Controller safety regression tests; run with container's ROS Python 3."""
import copy
import importlib.util
import math
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import rospy
import yaml
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("motion", str(ROOT / "src/robocup_navigation/scripts/uav_navigation_node.py"))
motion = importlib.util.module_from_spec(spec)
spec.loader.exec_module(motion)
CONFIG = ROOT / "src/robocup_navigation/config/single_uav_waypoints.yaml"
ROUTE_CONFIG = ROOT / "src/robocup_navigation/config/single_wall_route.yaml"


class MotionTests(unittest.TestCase):
    def setUp(self):
        self.c = motion.load_config(str(CONFIG))

    def test_diagonal_and_vertical_slew(self):
        p = motion.slew([0, 0, 0], [10, 10, 10], 0.1, 0.4, 0.3)
        self.assertAlmostEqual(math.hypot(*p[:2]), 0.04)
        self.assertAlmostEqual(p[2], 0.03)
        self.assertEqual(motion.slew(p, [0, 0, 0], -1, 1, 1), p)

    def test_no_overshoot(self):
        self.assertEqual(motion.slew([0, 0, 0], [.001, .001, .001], .1, 1, 1), [.001, .001, .001])

    def test_takeoff_requires_physical_gazebo_progress(self):
        self.assertEqual(motion.takeoff_progress_problem(10, 0.05, 19.9, 0.10, 10, 0.15), "")
        self.assertEqual(motion.takeoff_progress_problem(10, 0.05, 20.0, 0.10, 10, 0.15),
                         "TAKEOFF_NO_PHYSICAL_CLIMB")
        self.assertEqual(motion.takeoff_progress_problem(10, 0.05, 20.0, 0.21, 10, 0.15), "")

    def test_prearm_requires_continuous_ground_stability(self):
        n = self.node()
        n.c = motion.load_config(str(ROUTE_CONFIG))
        n.transform = SimpleNamespace(world_origin_xyz=(0.0, 0.0, 0.05))
        n.prearm_stable_since = n.prearm_last_problem = None
        n.samples["models"] = ({"iris": ((0.0, 0.0, 0.05), 0.0)}, 100.0)
        n.samples["extended"][0].landed_state = ExtendedState.LANDED_STATE_ON_GROUND
        self.assertFalse(n.prearm_ground_stable(n.samples, 100.0, 10.0))
        self.assertFalse(n.prearm_ground_stable(n.samples, 101.0, 11.0))
        self.assertTrue(n.prearm_ground_stable(n.samples, 101.5, 11.5))
        n.samples["velocity"][0].twist.linear.z = n.c["prearm_max_vertical_speed_mps"] + 0.01
        self.assertFalse(n.prearm_ground_stable(n.samples, 101.6, 11.6))
        self.assertIsNone(n.prearm_stable_since)
        n.samples["velocity"][0].twist.linear.z = 0.0
        n.samples["extended"][0].landed_state = ExtendedState.LANDED_STATE_IN_AIR
        self.assertFalse(n.prearm_ground_stable(n.samples, 101.7, 11.7))

    def test_bad_configs_rejected(self):
        cases = [("max_altitude_m", 1.0), ("setpoint_rate_hz", 2), ("pose_timeout_s", 0),
                 ("max_horizontal_speed_mps", float("nan")), ("accept_external_goals", "false")]
        for key, value in cases:
            with self.subTest(key=key), tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as f:
                c = copy.deepcopy(self.c)
                c[key] = value
                yaml.safe_dump(c, f)
                f.flush()
                with self.assertRaises(ValueError):
                    motion.load_config(f.name)

    def node(self):
        n = motion.Navigator.__new__(motion.Navigator)
        n.c, n.origin, n.clock_wall = self.c, [0, 0, 0], 100.0
        n.lock, n.samples = threading.RLock(), {}
        n.event = lambda *a, **k: None
        for key, cls in (("state", State), ("pose", PoseStamped), ("velocity", TwistStamped), ("extended", ExtendedState)):
            msg = cls()
            msg.header.stamp = rospy.Time.from_sec(10)
            n.samples[key] = (msg, 100.0)
        n.samples["state"][0].connected = True
        n.samples["pose"][0].header.frame_id = "map"
        return n

    def test_stale_receipt(self):
        n = self.node()
        self.assertFalse(n.fresh(n.samples, "pose", 103, 10))

    def test_stale_and_future_stamp(self):
        n = self.node()
        self.assertFalse(n.fresh(n.samples, "pose", 100, 13))
        self.assertFalse(n.fresh(n.samples, "pose", 100, 8))

    def test_nan_and_frame_rejected(self):
        n = self.node()
        n.samples["pose"][0].header.frame_id = "odom"
        self.assertEqual(n.healthy(n.samples, 100, 10), "POSE_FRAME_MISMATCH")
        n.samples["pose"][0].header.frame_id = "map"
        n.samples["pose"][0].pose.position.x = float("nan")
        self.assertEqual(n.healthy(n.samples, 100, 10), "NONFINITE_TELEMETRY")

    def test_altitude_and_clock(self):
        n = self.node()
        n.samples["pose"][0].pose.position.z = 3
        self.assertEqual(n.healthy(n.samples, 100, 10), "ALTITUDE_LIMIT")
        n.samples["pose"][0].pose.position.z = 0
        n.clock_wall = 90
        self.assertEqual(n.healthy(n.samples, 100, 10), "CLOCK_STALLED")

    def test_disconnected_state_rejected(self):
        n = self.node()
        n.samples["state"][0].connected = False
        self.assertEqual(n.healthy(n.samples, 100, 10), "FCU_DISCONNECTED_OR_STATE_STALE")

    def test_cancel_before_arm_does_not_touch_fcu(self):
        n = self.node()
        n.phase, n.owned = "PRESTREAM", False
        n.landing("CANCELLED")
        self.assertEqual((n.phase, n.result), ("DONE", 2))

    def test_blocked_service_does_not_block_caller(self):
        released = threading.Event()
        start = time.monotonic()
        worker = motion.Worker(lambda: released.wait(2))
        self.assertLess(time.monotonic() - start, 0.2)
        self.assertFalse(worker.done.is_set())
        released.set()
        self.assertTrue(worker.done.wait(1))

    def route_goal_node(self):
        n = self.node()
        n.c = motion.load_config(str(ROUTE_CONFIG))
        n.phase, n.active = "MOVE", {"name": "ROUTE_OLD", "position": [1, 0, 2.4], "hover_time_s": 0.0}
        n.queue, n.target, n.command = [], [1, 0, 2.4], [0, 0, 2.4]
        n.command_velocity = [0.1, 0.0, 0.0]
        n.replan_worker = n.replan_resume = n.replan_request = None
        n.pending_goal, n.route_plan = None, object()
        n.route_local, n.route_actual = [[0, 0, 2.4], [1, 0, 2.4]], []
        n.route_start_local = [0, 0, 2.4]
        n.transform = SimpleNamespace(world_to_local=lambda point: (point[0] + 3, point[1], point[2]))
        n.path_world_pub = n.path_local_pub = None
        n._publish_path = lambda *args: None
        n.samples["pose"][0].pose.position.z = 2.4
        n.samples["models"] = ({"iris": ((-1.0, 0.0, 2.4), 0.0)}, 100.0)
        return n

    @staticmethod
    def route_goal(sequence=7, x=2.0, y=-2.0):
        msg = PoseStamped()
        msg.header.seq = sequence
        msg.header.stamp = rospy.Time.from_sec(10.0)
        msg.header.frame_id = "gazebo_world"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, 0.0
        return msg

    def test_route_goal_replans_before_replacing_old_route(self):
        n = self.route_goal_node()
        n.pending_goal = self.route_goal()
        planned = ((-1.0, 0.0, 2.4), (-1.0, -2.0, 2.4), (2.0, -2.0, 2.4))
        with patch.object(motion, "replan_route", return_value=planned) as planner:
            n.accept_goal(10.0)
            self.assertEqual(n.phase, "REPLAN")
            self.assertEqual(n.target, [0.0, 0.0, 2.4])
            self.assertEqual(n.replan_resume[1]["name"], "ROUTE_OLD")
            self.assertTrue(n.replan_worker.done.wait(1))
            n.accept_goal(10.1)
        planner.assert_called_once()
        self.assertEqual(n.phase, "MOVE")
        self.assertEqual(n.active["name"], "external_7_0000")
        self.assertEqual(n.route_local[-1], [5.0, -2.0, 2.4])
        self.assertIsNone(n.replan_resume)

    def test_failed_route_goal_resumes_previous_target(self):
        n = self.route_goal_node()
        old_target = n.target[:]
        n.pending_goal = self.route_goal()
        with patch.object(motion, "replan_route", side_effect=motion.RouteError("REPLAN_GOAL_OCCUPIED")):
            n.accept_goal(10.0)
            self.assertTrue(n.replan_worker.done.wait(1))
            n.accept_goal(10.1)
        self.assertEqual(n.phase, "MOVE")
        self.assertEqual(n.active["name"], "ROUTE_OLD")
        self.assertEqual(n.target, old_target)
        self.assertIsNone(n.replan_resume)

    def test_newest_route_goal_discards_late_old_plan(self):
        n = self.route_goal_node()
        released = threading.Event()
        calls = []
        def planner(_route, _current, goal, _config):
            calls.append(tuple(goal))
            if len(calls) == 1:
                released.wait(1)
            return ((-1.0, 0.0, 2.4), (goal[0], goal[1], 2.4))
        with patch.object(motion, "replan_route", side_effect=planner):
            n.pending_goal = self.route_goal(sequence=7, x=1.0, y=1.0)
            n.accept_goal(10.0)
            n.pending_goal = self.route_goal(sequence=8, x=2.0, y=-2.0)
            released.set()
            self.assertTrue(n.replan_worker.done.wait(1))
            n.accept_goal(10.1)
            self.assertEqual(n.phase, "REPLAN")
            self.assertTrue(n.replan_worker.done.wait(1))
            n.accept_goal(10.2)
        self.assertEqual(calls, [(1.0, 1.0), (2.0, -2.0)])
        self.assertTrue(n.active["name"].startswith("external_8_"))

    def test_landing_requires_grounded_and_fresh_disarm(self):
        n = self.node()
        n.phase, n.cancel, n.owned = "AUTO_LAND", False, True
        n.landing_deadline, n.landing_started, n.result, n.airborne_seen = 150, 99, 0, True
        n.publish_enabled, n.workers = False, {}
        n.samples["state"][0].mode = "AUTO.LAND"
        n.samples["extended"][0].landed_state = ExtendedState.LANDED_STATE_IN_AIR
        n.tick(100, 10, 0.05)
        self.assertEqual(n.phase, "AUTO_LAND")
        n.samples["extended"][0].landed_state = ExtendedState.LANDED_STATE_ON_GROUND
        n.tick(100, 10, 0.05)
        self.assertEqual(n.phase, "DONE")


if __name__ == "__main__":
    unittest.main(verbosity=2)
