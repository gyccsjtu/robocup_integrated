#!/usr/bin/env python3
"""Single-writer PX4 simulation executor; ROS motion time, wall watchdogs."""
import fcntl
import json
import math
import os
import signal
import struct
import sys
import threading
import time
from datetime import datetime, timezone

import rospy
import yaml
from geometry_msgs.msg import PoseStamped, TwistStamped
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelProperties, GetModelState, GetWorldProperties
from mavros_msgs.msg import ExtendedState, State, ParamValue
from mavros_msgs.srv import CommandBool, SetMode, ParamGet, ParamSet
from nav_msgs.msg import Path
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, String

from robocup_navigation.astar import GridMap
from robocup_navigation.local_grid import LocalOccupancyGrid, OCCUPIED
from robocup_navigation.route_runtime import (GridSource, RouteError, SCENE_MODES,
                                              acceleration_limited_slew,
                                              check_scene_model_list,
                                              perception_depth_age_s,
                                              prepare_route,
                                              replan_with_grid_source,
                                              verify_runtime_models)


class FlightError(Exception):
    pass


def load_config(path):
    with open(path, encoding="utf-8") as stream:
        c = yaml.safe_load(stream)
    if not isinstance(c, dict):
        raise ValueError("configuration must be a mapping")
    numeric = ("setpoint_rate_hz takeoff_altitude_m max_altitude_m max_radius_m "
               "max_horizontal_speed_mps max_vertical_speed_mps landing_descent_speed_mps auto_land_speed_mps "
               "arrival_tolerance_m arrival_speed_mps settle_time_s landing_switch_altitude_m "
               "prestream_duration_s connection_timeout_s service_timeout_s mode_timeout_s "
               "arming_timeout_s mission_timeout_s waypoint_timeout_s landing_timeout_s "
               "pose_timeout_s state_timeout_s clock_timeout_s goal_max_age_s").split()
    for key in numeric:
        if isinstance(c[key], bool):
            raise ValueError(key + " must be numeric")
        c[key] = float(c[key])
        if not math.isfinite(c[key]) or c[key] <= 0:
            raise ValueError(key + " must be finite and positive")
    if not 10 <= c["setpoint_rate_hz"] <= 50 or c["prestream_duration_s"] < 2:
        raise ValueError("require 10-50 Hz setpoints and >= 2 ROS seconds prestream")
    if not 0 < c["landing_switch_altitude_m"] < c["takeoff_altitude_m"] < c["max_altitude_m"]:
        raise ValueError("require 0 < landing switch < takeoff < max altitude")
    if c["auto_land_speed_mps"] < 0.1:
        raise ValueError("auto_land_speed_mps must be >= 0.1 for PX4 land detector")
    c["yaw_rad"] = float(c["yaw_rad"])
    if not math.isfinite(c["yaw_rad"]):
        raise ValueError("yaw_rad must be finite")
    if not isinstance(c["accept_external_goals"], bool):
        raise ValueError("accept_external_goals must be a YAML boolean")
    c["route_mode"] = c.get("route_mode", False)
    if not isinstance(c["route_mode"], bool):
        raise ValueError("route_mode must be a YAML boolean")
    if c["frame_id"] != "map":
        raise ValueError("frame_id must be 'map' (ENU)")
    c["mavros_namespace"] = str(c.get("mavros_namespace", "/mavros")).rstrip("/")
    if not c["mavros_namespace"].startswith("/"):
        raise ValueError("mavros_namespace must be absolute, e.g. /mavros or /uav_1/mavros")
    c["vehicle_model_name"] = str(c.get("vehicle_model_name", "iris"))
    if not c["vehicle_model_name"]:
        raise ValueError("vehicle_model_name must be non-empty")
    if not isinstance(c["waypoints"], list) or len(c["waypoints"]) < 2:
        raise ValueError("at least two waypoints are required")
    for point in c["waypoints"]:
        for key in ("x", "y", "z", "hover_time_s"):
            point[key] = float(point[key])
            if not math.isfinite(point[key]):
                raise ValueError("waypoint values must be finite")
        if not 0.5 <= point["z"] < c["max_altitude_m"]:
            raise ValueError("waypoint height outside [0.5, max altitude)")
        if math.hypot(point["x"], point["y"]) > c["max_radius_m"]:
            raise ValueError("waypoint outside horizontal radius")
        if point["hover_time_s"] < 0 or not isinstance(point["name"], str):
            raise ValueError("invalid waypoint name/hover")
    if c["route_mode"]:
        required = ("scene_receipt_file", "world_file", "metadata_file", "route_start_world_xy",
                    "route_goal_world_xy", "route_altitude_m", "route_max_expansions",
                    "route_planning_timeout_s", "vehicle_radius_m", "horizontal_margin_m",
                    "vehicle_half_height_m", "vertical_margin_m", "minimum_clearance_m",
                    "transform_pair_tolerance_m", "transform_second_pair_min_motion_m",
                    "gazebo_model_tolerance_m", "gazebo_model_yaw_tolerance_rad",
                    "max_horizontal_accel_mps2", "max_vertical_accel_mps2",
                    "route_tracking_error_limit_m", "goal_hover_time_s",
                    "fixed_enu_yaw_rad", "heading_observation_limit_rad",
                    "planning_tracking_margin_m", "prearm_settle_time_s",
                    "prearm_max_vertical_speed_mps", "prearm_spawn_tolerance_m",
                    "takeoff_progress_timeout_s", "takeoff_progress_m")
        if any(key not in c for key in required):
            raise ValueError("route_mode configuration incomplete")
        for key in required:
            if key.endswith("_file"):
                if not isinstance(c[key], str) or not c[key]:
                    raise ValueError(key + " must be a non-empty path")
        c["scene_mode"] = c.get("scene_mode", "static")
        if c["scene_mode"] not in SCENE_MODES:
            raise ValueError("scene_mode must be one of " + "|".join(SCENE_MODES))
        c["perception_max_dynamic_models"] = c.get("perception_max_dynamic_models", 8)
        if isinstance(c["perception_max_dynamic_models"], bool) or \
                not isinstance(c["perception_max_dynamic_models"], int) or \
                c["perception_max_dynamic_models"] < 0:
            raise ValueError("perception_max_dynamic_models must be a non-negative integer")
        for key in ("route_altitude_m", "route_planning_timeout_s", "vehicle_radius_m",
                    "horizontal_margin_m", "vehicle_half_height_m", "vertical_margin_m",
                    "minimum_clearance_m", "transform_pair_tolerance_m",
                    "transform_second_pair_min_motion_m", "gazebo_model_tolerance_m",
                    "gazebo_model_yaw_tolerance_rad", "max_horizontal_accel_mps2",
                    "max_vertical_accel_mps2", "route_tracking_error_limit_m",
                    "goal_hover_time_s", "heading_observation_limit_rad",
                    "planning_tracking_margin_m", "prearm_settle_time_s",
                    "prearm_max_vertical_speed_mps", "prearm_spawn_tolerance_m",
                    "takeoff_progress_timeout_s", "takeoff_progress_m"):
            c[key] = float(c[key])
            if not math.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(key + " must be finite and positive")
        c["perception_grid_resolution_m"] = float(c.get("perception_grid_resolution_m", 0.25))
        c["perception_grid_side_m"] = float(c.get("perception_grid_side_m", 12.0))
        c["perception_z_min_m"] = float(c.get("perception_z_min_m", 0.8))
        c["perception_z_max_m"] = float(c.get("perception_z_max_m", 3.5))
        c["perception_update_hz"] = float(c.get("perception_update_hz", 5.0))
        c["perception_check_hz"] = float(c.get("perception_check_hz", 2.0))
        c["perception_lookahead_m"] = float(c.get("perception_lookahead_m", 5.0))
        c["perception_corridor_half_m"] = float(c.get("perception_corridor_half_m", 0.9))
        c["perception_trigger_frames"] = c.get("perception_trigger_frames", 1)
        if isinstance(c["perception_trigger_frames"], bool) or \
                not isinstance(c["perception_trigger_frames"], int) or \
                c["perception_trigger_frames"] < 1:
            raise ValueError("perception_trigger_frames must be an integer >= 1")
        c["perception_blocked_min_samples"] = c.get("perception_blocked_min_samples", 3)
        if isinstance(c["perception_blocked_min_samples"], bool) or \
                not isinstance(c["perception_blocked_min_samples"], int) or \
                c["perception_blocked_min_samples"] < 1:
            raise ValueError("perception_blocked_min_samples must be an integer >= 1")
        c["perception_inflation_m"] = float(c.get("perception_inflation_m",
                                                  c["vehicle_radius_m"] + c["horizontal_margin_m"]
                                                  + c["planning_tracking_margin_m"]))
        c["perception_depth_max_age_s"] = float(c.get("perception_depth_max_age_s", 5.0))
        if not math.isfinite(c["perception_depth_max_age_s"]) or c["perception_depth_max_age_s"] <= 0:
            raise ValueError("perception_depth_max_age_s must be finite and positive")
        c["perception_depth_stale_action"] = str(c.get("perception_depth_stale_action", "warn"))
        if c["perception_depth_stale_action"] not in ("warn", "abort"):
            raise ValueError("perception_depth_stale_action must be warn or abort")
        c["perception_fusion_max_age_s"] = float(c.get("perception_fusion_max_age_s", 0.5))
        if not math.isfinite(c["perception_fusion_max_age_s"]) or c["perception_fusion_max_age_s"] <= 0:
            raise ValueError("perception_fusion_max_age_s must be finite and positive")
        c["perception_threat_close_mps"] = float(c.get("perception_threat_close_mps", 0.40))
        if not math.isfinite(c["perception_threat_close_mps"]) or c["perception_threat_close_mps"] <= 0:
            raise ValueError("perception_threat_close_mps must be finite and positive")
        # Occupied-cell blob size that separates a compact vehicle from a
        # large static structure.  Measured in flight: a 0.4 m intruder is
        # 3-6 OCCUPIED cells within 1 m; boundary walls and their shadow band
        # are 12-19.  Geometry this large is a statics problem for A*, not a
        # traffic problem for the yield manoeuvre.
        c["perception_threat_blob_max_cells"] = int(c.get("perception_threat_blob_max_cells", 10))
        if c["perception_threat_blob_max_cells"] < 0:
            raise ValueError("perception_threat_blob_max_cells must be non-negative")
        c["perception_threat_cross_mps"] = float(c.get("perception_threat_cross_mps", 0.25))
        if not math.isfinite(c["perception_threat_cross_mps"]) or c["perception_threat_cross_mps"] <= 0:
            raise ValueError("perception_threat_cross_mps must be finite and positive")
        c["perception_evade_lateral_m"] = float(c.get("perception_evade_lateral_m", 1.5))
        if not math.isfinite(c["perception_evade_lateral_m"]) or c["perception_evade_lateral_m"] <= 0:
            raise ValueError("perception_evade_lateral_m must be finite and positive")
        c["perception_evade_max_hold_s"] = float(c.get("perception_evade_max_hold_s", 25.0))
        if not math.isfinite(c["perception_evade_max_hold_s"]) or c["perception_evade_max_hold_s"] <= 0:
            raise ValueError("perception_evade_max_hold_s must be finite and positive")
        c["perception_evade_grace_s"] = float(c.get("perception_evade_grace_s", 3.0))
        if not math.isfinite(c["perception_evade_grace_s"]) or c["perception_evade_grace_s"] < 0:
            raise ValueError("perception_evade_grace_s must be finite and non-negative")
        # Bounded retry for a TRANSIENT perception replan failure (e.g. the
        # goal cell still shadowed by a passing vehicle: the mark decay
        # clears it within perception_mark_decay_s).  Retrying keeps the
        # mission recoverable; the budget guarantees we still fail closed
        # instead of flying (or resuming) an unproven route.
        # A corridor replan needs 3 blocked samples (~1.5 s) while the
        # intruder detector needs a full 2 s window — left alone, the
        # corridor always wins and A*'s far bend point loses the race to an
        # on-line intruder (run 004104: 0.08 m contact).  When there is
        # provisional closing evidence, defer the corridor replan briefly so
        # the lateral hop can fire first.  Bounded: the corridor always gets
        # its turn once the budget expires.
        c["perception_corridor_defer_s"] = float(c.get("perception_corridor_defer_s", 3.0))
        if not math.isfinite(c["perception_corridor_defer_s"]) or c["perception_corridor_defer_s"] < 0:
            raise ValueError("perception_corridor_defer_s must be finite and non-negative")
        c["perception_replan_retry_s"] = float(c.get("perception_replan_retry_s", 12.0))
        if not math.isfinite(c["perception_replan_retry_s"]) or c["perception_replan_retry_s"] < 0:
            raise ValueError("perception_replan_retry_s must be finite and non-negative")
        c["perception_replan_retry_max"] = int(c.get("perception_replan_retry_max", 5))
        if c["perception_replan_retry_max"] < 0:
            raise ValueError("perception_replan_retry_max must be non-negative")
        c["perception_replan_retry_gap_s"] = float(c.get("perception_replan_retry_gap_s", 2.0))
        if not math.isfinite(c["perception_replan_retry_gap_s"]) or c["perception_replan_retry_gap_s"] < 0:
            raise ValueError("perception_replan_retry_gap_s must be finite and non-negative")
        c["perception_mark_decay_s"] = float(c.get("perception_mark_decay_s", 6.0))
        if not math.isfinite(c["perception_mark_decay_s"]) or c["perception_mark_decay_s"] <= 0:
            raise ValueError("perception_mark_decay_s must be finite and positive")
        c["depth_topic"] = str(c.get("depth_topic", "/camera_front/depth/image_raw"))
        c["depth_info_topic"] = str(c.get("depth_info_topic", "/camera_front/depth/camera_info"))
        for key in ("perception_grid_resolution_m", "perception_grid_side_m",
                    "perception_update_hz", "perception_check_hz",
                    "perception_lookahead_m", "perception_corridor_half_m",
                    "perception_inflation_m"):
            if not math.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(key + " must be finite and positive")
        if not 0 < c["perception_z_min_m"] < c["perception_z_max_m"]:
            raise ValueError("perception z band must satisfy 0 < z_min < z_max")
        if isinstance(c["route_max_expansions"], bool) or int(c["route_max_expansions"]) <= 0:
            raise ValueError("route_max_expansions must be positive")
        c["route_max_expansions"] = int(c["route_max_expansions"])
        for key in ("route_start_world_xy", "route_goal_world_xy"):
            if not isinstance(c[key], list) or len(c[key]) != 2:
                raise ValueError(key + " must contain [x, y]")
            c[key] = tuple(float(value) for value in c[key])
            if not all(math.isfinite(value) for value in c[key]):
                raise ValueError(key + " must be finite")
        if c["accept_external_goals"]:
            if not isinstance(c.get("external_goal_frame_id"), str) or not c["external_goal_frame_id"]:
                raise ValueError("external_goal_frame_id must be a non-empty string")
        c["fixed_enu_yaw_rad"] = float(c["fixed_enu_yaw_rad"])
        if not math.isfinite(c["fixed_enu_yaw_rad"]) or abs(c["fixed_enu_yaw_rad"]) > 1e-8:
            raise ValueError("phase-1 route transform requires fixed ENU yaw = 0")
        if abs(c["takeoff_altitude_m"] - c["route_altitude_m"]) > 1e-6:
            raise ValueError("route takeoff altitude must equal route altitude")
    return c


def slew(current, target, dt, xy_speed, z_speed):
    """Bound setpoint displacement per ROS second (not actual vehicle speed)."""
    dt = max(0.0, min(0.2, dt))
    dx, dy = target[0] - current[0], target[1] - current[1]
    distance = math.hypot(dx, dy)
    scale = min(1.0, xy_speed * dt / distance) if distance else 0.0
    dz = max(-z_speed * dt, min(z_speed * dt, target[2] - current[2]))
    return [current[0] + dx * scale, current[1] + dy * scale, current[2] + dz]


def takeoff_progress_problem(started, baseline_z, now, actual_z, timeout_s, minimum_climb_m):
    """Detect an armed PX4/Gazebo chain that produces no physical climb."""
    if actual_z >= baseline_z + minimum_climb_m:
        return ""
    return "TAKEOFF_NO_PHYSICAL_CLIMB" if now - started >= timeout_s else ""


class Worker:
    """One outstanding operation; service hangs cannot spawn endless threads."""
    def __init__(self, operation):
        self.done = threading.Event()
        self.result = self.error = None
        self.started = time.monotonic()
        self.finished = None
        def execute():
            try:
                self.result = operation()
            except Exception as exc:
                self.error = str(exc)
            finally:
                self.finished = time.monotonic()
                self.done.set()
        threading.Thread(target=execute, daemon=True).start()


class Navigator:
    # Namespacing defaults; __init__ overrides them from config.  Keeping them
    # at class level lets partially-constructed instances (test harnesses that
    # bypass __init__) still resolve these names.
    mavros_ns = "/mavros"
    model_name = "iris"

    def __init__(self, config):
        self.c = config
        self.mavros_ns = self.c["mavros_namespace"]
        self.model_name = self.c["vehicle_model_name"]
        self.lock = threading.RLock()
        self.samples = {}
        self.pending_goal = None
        self.replan_worker = self.replan_resume = self.replan_request = None
        self.cancel = self.owned = self.publish_enabled = False
        self.phase, self.reason, self.result = "CONNECTING", "", None
        self.target = self.command = self.origin = self.active = None
        self.route_plan = self.transform = self.route_start_local = None
        self.route_local = self.route_actual = []
        self.route_second_pair_pending = self.route_landing_confirmed = False
        self.route_last_clearance = self.route_last_tracking_error = None
        self._dynamic_models = ()
        self.route_world_current = []
        self.mission_goal_world = None
        self.local_grid = None
        self.grid_source = None
        self._depth = {"fx": None, "fy": None, "cx": None, "cy": None}
        self._depth_warned_encoding = False
        self._last_depth_fusion = 0.0
        self._last_depth_sim = None       # stamp of the last depth frame (sim s)
        self._depth_move_sim = None       # sim s when MOVE first began
        self._depth_stale_reported = False
        self._corridor_frozen_reported = False
        self._last_threat_check = 0.0
        self._threat_samples = []         # (sim_s, radial, cross, wx, wy)
        self._static_blocks = []          # world xy of blocks seen NOT closing
        self._evade = None                # closing-intruder yield state or None
        self._evade_entered_sim = None
        self._evade_resumed_sim = None    # sim time of last evade resume (grace)
        self._retry_goal_cell_clear = False
        self._replan_fail_streak = 0      # consecutive failed perception replans
        self._perception_retry = None     # bounded retry state or None
        self._corridor_defer_sim = None   # corridor yield-to-threat window
        self._last_decay = 0.0
        self._last_corridor_check = 0.0
        self._corridor_hits = 0
        self._last_perception_replan_version = None
        self._replan_seq = 0
        if self.c["route_mode"] and self.c["scene_mode"] == "perception":
            self.local_grid = LocalOccupancyGrid(
                resolution_m=self.c["perception_grid_resolution_m"],
                side_m=self.c["perception_grid_side_m"],
                z_min_m=self.c["perception_z_min_m"],
                z_max_m=self.c["perception_z_max_m"],
                unknown_is_obstacle=True)
        self.prearm_stable_since = self.prearm_last_problem = None
        self.arm_requested = False
        self.takeoff_watchdog = None
        self.command_velocity = [0.0, 0.0, 0.0]
        self.queue = []
        self.settled_since = None
        self.workers, self.last_request = {}, {}
        self.last_graph = self.last_log = 0.0
        self.started = self.clock_wall = time.monotonic()
        self.deadline = self.started + config["connection_timeout_s"]
        self.mission_deadline = float("inf")
        self.last_sim = self.phase_sim = rospy.Time.now().to_sec()
        self.airborne_seen = False
        self.original_params = {}
        self.param_index = 0
        self.param_names = ["COM_RCL_EXCEPT", "COM_OBL_ACT", "COM_OBL_RC_ACT", "COM_OF_LOSS_T",
                            "LNDMC_Z_VEL_MAX", "MPC_LAND_CRWL", "MPC_LAND_SPEED"]
        self.param_stage = "read"
        self.run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        # ROS private params survive rosrun exit. Only an explicit invocation
        # override may reuse a chosen path; otherwise generate a fresh run log.
        log_dir = next((arg.split(":=", 1)[1] for arg in sys.argv[1:]
                        if arg.startswith("_log_dir:=")), "/workspace/logs/flight/" + self.run_id)
        os.makedirs(log_dir, exist_ok=True)
        self.log = open(os.path.join(log_dir, "events.jsonl"), "x", buffering=1)
        self.status = rospy.Publisher("~status", String, queue_size=10, latch=True)
        self.pub = rospy.Publisher(self.mavros_ns + "/setpoint_position/local", PoseStamped, queue_size=1)
        self.path_world_pub = rospy.Publisher("~planned_path_world", Path, queue_size=1, latch=True)
        self.path_local_pub = rospy.Publisher("~planned_path_local", Path, queue_size=1, latch=True)
        self.path_actual_pub = rospy.Publisher("~actual_path_local", Path, queue_size=1)
        for key, topic, kind in (
                ("state", self.mavros_ns + "/state", State),
                ("pose", self.mavros_ns + "/local_position/pose", PoseStamped),
                ("velocity", self.mavros_ns + "/local_position/velocity_local", TwistStamped),
                ("extended", self.mavros_ns + "/extended_state", ExtendedState)):
            rospy.Subscriber(topic, kind, self.sample, callback_args=key, queue_size=1)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states, queue_size=1)
        rospy.Subscriber("~cancel", Empty, self.on_cancel, queue_size=1)
        rospy.Subscriber("~goal", PoseStamped, self.on_goal, queue_size=1)
        if self.c["route_mode"] and self.c["scene_mode"] == "perception":
            # Perception inputs are optional by scene contract; a missing
            # camera surfaces as a stale-corridor timeout, never a crash.
            rospy.Subscriber(self.c["depth_info_topic"], CameraInfo, self.camera_info_cb,
                             queue_size=1)
            rospy.Subscriber(self.c["depth_topic"], Image, self.depth_cb, queue_size=1,
                             buff_size=64 * 64 * 4 * 4)
        signal.signal(signal.SIGINT, self.on_signal)
        signal.signal(signal.SIGTERM, self.on_signal)
        self.event("START", config=config, log_dir=log_dir)

    @staticmethod
    def yaw(quaternion):
        """Yaw of an ENU quaternion; frame alignment is checked again in flight."""
        return math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                          1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))

    def sample(self, message, key):
        with self.lock:
            self.samples[key] = (message, time.monotonic())

    def model_states(self, message):
        observed = {}
        for name, pose in zip(message.name, message.pose):
            xyz = self.xyz(pose.position)
            if None not in xyz:
                observed[name] = (tuple(xyz), self.yaw(pose.orientation))
        with self.lock:
            self.samples["models"] = (observed, time.monotonic())

    # -- perception (scene_mode=perception) ----------------------------------

    def camera_info_cb(self, message):
        self._depth["fx"], self._depth["fy"] = message.K[0], message.K[4]
        self._depth["cx"], self._depth["cy"] = message.K[2], message.K[5]

    def depth_cb(self, message):
        """Fuse throttled depth frames into the world-frame local grid.

        Uses Gazebo-truth iris pose from /gazebo/model_states so grid cells
        share the route's world frame directly."""
        info = self._depth
        if info["fx"] is None or self.local_grid is None:
            return
        stamp = message.header.stamp.to_sec()
        # Fusion age guard: a frame is only fused at the CURRENT truth pose,
        # so a frame older than perception_fusion_max_age_s would paint its
        # obstacles at the wrong world position (drone displacement x latency
        # = misregistration).  Drop those; the staleness watchdog owns the
        # stream-health signal, and recovery only counts fusable frames so a
        # persistently late stream cannot flap STALE/RECOVERED.
        age = rospy.Time.now().to_sec() - stamp
        if stamp <= 0 or age < -0.1 or age > self.c["perception_fusion_max_age_s"]:
            return
        self.local_grid.note_sim(stamp)
        if stamp - self._last_depth_fusion < 1.0 / self.c["perception_update_hz"]:
            return
        if message.encoding != "32FC1":
            if not self._depth_warned_encoding:
                self._depth_warned_encoding = True
                self.event("PERCEPTION_DEPTH_ENCODING_UNSUPPORTED", encoding=message.encoding)
            return
        with self.lock:
            models = self.samples.get("models")
        if not models or self.model_name not in models[0]:
            return
        try:
            count = message.width * message.height
            ranges = struct.unpack("<%df" % count, bytes(message.data))
        except (struct.error, ValueError):
            return
        world, yaw = models[0][self.model_name]
        grid = self.local_grid
        try:
            grid.recenter(world[0], world[1])
            grid.update(ranges, message.width, message.height,
                        info["fx"], info["fy"], info["cx"], info["cy"],
                        world[0], world[1], world[2], yaw)
        except (ValueError, TypeError):
            return
        # Liveness counts only frames that actually FUSED: a stream of
        # wrong-encoding or undecodable frames must not keep the watchdog
        # happy while the grid silently stops updating (review finding).
        if self._depth_stale_reported:
            self._depth_stale_reported = False
            self._corridor_frozen_reported = False
            self.event("PERCEPTION_DEPTH_RECOVERED", lag_s=round(age, 2))
        self._last_depth_sim = stamp
        self._last_depth_fusion = stamp

    def _corridor_blocked_samples(self, world, route):
        """Count route samples within the lookahead whose probed cells are
        OCCUPIED in the live perception grid (world frame, planar)."""
        grid = self.local_grid
        lookahead = self.c["perception_lookahead_m"]
        half = self.c["perception_corridor_half_m"]
        step = max(grid.resolution, 0.4)
        points = [(point[0], point[1]) for point in route]
        # Nearest projection of the live position onto the route polyline.
        best = None
        px, py = world[0], world[1]
        for index in range(len(points) - 1):
            ax, ay = points[index]
            bx, by = points[index + 1]
            dx, dy = bx - ax, by - ay
            length_sq = dx * dx + dy * dy
            t = 0.0 if length_sq <= 1e-12 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
            qx, qy = ax + t * dx, ay + t * dy
            distance = math.hypot(px - qx, py - qy)
            if best is None or distance < best[0]:
                best = (distance, index, t, qx, qy)
        if best is None:
            return 0
        _, index, t, qx, qy = best
        # Remaining path: projection point, then the following vertices.
        remaining = [(px, py), (qx, qy)] + points[index + 1:]
        covered = 0.0
        blocked = 0
        probes = ((0.0, 0.0), (half, 0.0), (-half, 0.0), (0.0, half), (0.0, -half))
        for current, following in zip(remaining, remaining[1:]):
            dx, dy = following[0] - current[0], following[1] - current[1]
            length = math.hypot(dx, dy)
            if length <= 1e-9:
                continue
            walked = 0.0
            while walked < length and covered <= lookahead:
                walked += step
                covered += step
                if covered < 0.5:
                    continue
                ratio = min(1.0, walked / length)
                sx, sy = current[0] + ratio * dx, current[1] + ratio * dy
                for ox, oy in probes:
                    cell = grid.world_to_cell((sx + ox, sy + oy))
                    if cell is not None and grid.states[cell[1] * grid.width + cell[0]] == OCCUPIED:
                        blocked += 1
                        break
                if covered >= lookahead:
                    break
            if covered >= lookahead:
                break
        return blocked

    def check_perception_corridor(self, sim):
        """Throttled perception trigger: a repeatedly observed occupied
        corridor ahead self-preempts the current route with a replan to the
        same final goal on the live grid."""
        if self.local_grid is None or self.grid_source is None or not self.route_world_current:
            return
        if self.phase not in ("MOVE", "HOVER") or self.replan_worker is not None \
                or self.replan_resume is not None or self.pending_goal is not None:
            return
        if self._depth_stale_reported:
            # The live grid is frozen: replanning from a new position on an
            # incomplete map can route through never-scanned space (a wall
            # corner the camera never marked).  Keep the last LIVE-planned
            # route instead; abort mode already fail-safes via the watchdog.
            if not self._corridor_frozen_reported:
                self._corridor_frozen_reported = True
                self.event("PERCEPTION_CORRIDOR_FROZEN",
                           map_version=self.local_grid.map_version)
            return
        if sim - self._last_corridor_check < 1.0 / self.c["perception_check_hz"]:
            return
        self._last_corridor_check = sim
        with self.lock:
            models = self.samples.get("models")
        if not models or self.model_name not in models[0]:
            return
        blocked = self._corridor_blocked_samples(models[0][self.model_name][0], self.route_world_current)
        # A real blocking obstacle spans several corridor samples; a single
        # hit is corridor-edge geometry of already-known static walls (which
        # the planner has already routed around) and must not trigger a
        # replan storm.  Count consecutive checks, not lifetime hits.
        if blocked >= self.c["perception_blocked_min_samples"]:
            self._corridor_hits += 1
        else:
            self._corridor_hits = 0
        if self._corridor_hits >= self.c["perception_trigger_frames"]:
            # One replan per map_version: while the world snapshot is
            # unchanged the same replan would return the same route, and a
            # legal close pass (< corridor_half) re-triggers endlessly.
            # Suppressed while a closing-intruder yield is in progress: the
            # lateral hold replan owns the goal during that window.
            version = self.local_grid.map_version
            if version != self._last_perception_replan_version and self._evade is None:
                # Yield to the intruder detector while it has provisional
                # closing evidence: its lateral hop is the faster (and only
                # winning) response to on-line traffic, whereas an A* bypass
                # races the intruder to a bend point it usually loses.
                if self._provisional_closing():
                    if self._corridor_defer_sim is None:
                        self._corridor_defer_sim = sim
                        self.event("PERCEPTION_CORRIDOR_DEFERRED",
                                   reason="closing_threat_pending",
                                   budget_s=self.c["perception_corridor_defer_s"],
                                   blocked_samples=blocked)
                    if sim - self._corridor_defer_sim < self.c["perception_corridor_defer_s"]:
                        return
                self._corridor_defer_sim = None
                self._corridor_hits = 0
                self.trigger_perception_replan(sim, blocked)

    def check_depth_staleness(self, sim):
        """Perception watchdog: surface (or abort on) an observation stream
        whose last sim-stamped frame is older than
        perception_depth_max_age_s.  Age is measured in sim time, so a
        paused Gazebo cannot false-fire, and a live-but-delayed stream is
        capped by the same constant.  warn emits once per outage and keeps
        flying on the last known map; abort fails safe immediately."""
        if self.phase not in ("MOVE", "HOVER"):
            return
        age = perception_depth_age_s(self._last_depth_sim, sim, self._depth_move_sim)
        if age is None or age <= self.c["perception_depth_max_age_s"]:
            return
        if not self._depth_stale_reported:
            self._depth_stale_reported = True
            self.event("PERCEPTION_DEPTH_STALE", age_s=round(age, 2),
                       action=self.c["perception_depth_stale_action"])
        if self.c["perception_depth_stale_action"] == "abort":
            raise FlightError("PERCEPTION_DEPTH_STALE")

    def _nearest_ahead_block(self, world):
        """Nearest DYNAMIC OCCUPIED cell ahead of the drone (forward
        half-plane, +x being the nominal route direction) inside the
        lookahead horizon.

        NOTE: no ever_free filtering here — an airborne intruder approaches
        through cells that were never observed (UNKNOWN -> OCCUPIED), so a
        "was FREE" requirement would exclude exactly the threats this
        detector exists for.  Static-geometry false positives are handled
        by the closing-rate thresholds and the crossing gate instead.

        Returns ``(distance, world_x, world_y)`` or None when the way is
        clear.  The world position lets the caller reason about the intruder
        bearing and cross-track motion, not just the radial distance."""
        grid = self.local_grid
        if grid is None:
            return None
        lookahead = self.c["perception_lookahead_m"] + 1.5
        best = None
        resolution = grid.resolution
        origin_x, origin_y = grid.origin
        for cy in range(grid.height):
            wy = origin_y + (cy + 0.5) * resolution
            if wy < world[1] - lookahead or wy > world[1] + lookahead:
                continue
            row = cy * grid.width
            for cx in range(grid.width):
                if grid.states[row + cx] != OCCUPIED:
                    continue
                wx = origin_x + (cx + 0.5) * resolution
                if wx < world[0] - 1.0:  # nominal route runs +x: drop wake cells
                    continue
                d = math.hypot(wx - world[0], wy - world[1])
                if d > lookahead:
                    continue
                if best is None or d < best[0]:
                    best = (d, wx, wy)
        return best

    def _provisional_closing(self):
        """True while the accumulated threat samples already look like a
        closing intruder, before the full persistence window has elapsed.

        Used only to YIELD the corridor replan for a bounded moment — never
        to trigger an evade on its own (that still needs the full window)."""
        samples = getattr(self, "_threat_samples", None)
        if not samples or len(samples) < 2:
            return False
        (t0, r0, c0, _x0, _y0), (t1, r1, c1, _x1, _y1) = samples[0], samples[-1]
        span = t1 - t0
        if span <= 0.0:
            return False
        closing = (r0 - r1) / span
        crossing = (c0 - c1) / span
        return (closing >= self.c["perception_threat_close_mps"] * 0.6
                or crossing >= self.c["perception_threat_cross_mps"] * 0.6)

    def check_intruder_threat(self, sim):
        """Track the nearest obstacle ahead and detect a MOVING intruder.

        Two geometric signals, both measured against the drone's own motion:
          - radial closing: a static wall only closes at the drone's speed
            (<= its 0.3 m/s limit); a head-on intruder closes at the SUM of
            both speeds (>= ~0.5 m/s for same-speed traffic).
          - cross-track closing: an intruder CROSSING the route perpendicular
            closes radially at only ~0.4 m/s (may sit under the radial bar),
            but its distance from the route line shrinks at its full speed.
        The nearest-ahead-block either rate persistently exceeding its bar
        means the block is a vehicle moving toward the route: leave the line
        laterally and let it pass (a plain replan would race the A* bend
        point against the intruder and lose)."""
        if self.local_grid is None or not self.route_world_current:
            return
        # MOVE-only sampling (measured): allowing HOVER as well made the
        # detector fire far more often, and the extra yields pushed the drone
        # off route into ROUTE_TRACKING_ERROR_LIMIT (battery 013106: 4/6 vs
        # 5/6 MOVE-only).  A replan in flight still owns the goal.
        if self.phase != "MOVE" or self.replan_worker is not None \
                or self.replan_resume is not None or self.pending_goal is not None \
                or self._evade is not None:
            return
        if sim - self._last_threat_check < 1.0 / self.c["perception_check_hz"]:
            return
        self._last_threat_check = sim
        with self.lock:
            models = self.samples.get("models")
        if not models or self.model_name not in models[0]:
            return
        world = models[0][self.model_name][0]
        hit = self._nearest_ahead_block(world)
        if hit is None or hit[0] > self.c["perception_lookahead_m"] + 1.0:
            self._threat_samples = []
            return
        radial, bx, by = hit
        # Cross signal = the block's distance to the ORIGINAL route polyline
        # (fixed in the world), NOT its distance to the drone.  A crossing
        # intruder closes on the line at its own speed; A's own descent back
        # to the route line must never look like a crossing (review finding:
        # that false positive caused repeated evades and an off-goal landing).
        cross = self._distance_to_polyline((bx, by), self.route_plan.route_world)
        self._threat_samples.append((sim, radial, cross, bx, by))
        while len(self._threat_samples) > 1 and \
                sim - self._threat_samples[0][0] > 3.0:
            self._threat_samples.pop(0)
        if len(self._threat_samples) >= 3:
            first_sim, first_r, first_c, _, _ = self._threat_samples[0]
            last_sim, last_r, last_c, _, _ = self._threat_samples[-1]
            span = last_sim - first_sim
            if span < 2.0:
                return  # persistence window: 2 s of sustained closing
            reason = None
            closing_fast = (first_r - last_r >= 0.25 and
                            (first_r - last_r) / span >=
                            self.c["perception_threat_close_mps"])
            crossing_fast = (first_c - last_c >= 0.25
                             and (first_c - last_c) / span >=
                             self.c["perception_threat_cross_mps"]
                             and last_c <= self.c["perception_evade_lateral_m"] + 0.25)
            if closing_fast or crossing_fast:
                # Review guard: a block that sits at a position we previously
                # watched WITHOUT sustained closing is known-static geometry
                # (e.g. a boundary wall whose line-distance only appears to
                # converge because newly revealed cells sit closer to the
                # line).  The A* route already proves around it — suppressing
                # the evade here avoids yanking the drone off-route repeatedly
                # on its way to the goal.
                for sx, sy in self._static_blocks:
                    if math.hypot(bx - sx, by - sy) <= 1.0:
                        self._threat_samples = []
                        return
                # Occupancy-persistence guard: geometry that has held its
                # current cells far longer than any vehicle could (well past
                # the mark-decay horizon) is static — newly revealed wall
                # corner cells converge in line-distance as the drone
                # approaches, and must never be evaded.  A genuine mover
                # vacates each cell within a couple of seconds.
                static_age = self.local_grid.static_age_at((bx, by), 0.75)
                blob_cells = self.local_grid.occupied_count_at((bx, by), 1.0)
                blob_limit = self.c["perception_threat_blob_max_cells"]
                if static_age >= self.c["perception_mark_decay_s"] or \
                        (blob_limit > 0 and blob_cells >= blob_limit):
                    # Too long-lived or too large to be a vehicle: newly
                    # revealed wall/shadow cells near the goal converge in
                    # line-distance and used to yank the drone off route
                    # (measured blob: vehicle 3-6 cells, wall 12-19).
                    self._static_blocks.append((bx, by))
                    self._threat_samples = []
                    self.event("INTRUDER_THREAT_STATIC_SUPPRESSED",
                               static_age_s=round(static_age, 1),
                               blob_cells=blob_cells,
                               block_xy=[round(bx, 2), round(by, 2)],
                               reason="long_lived" if static_age >=
                               self.c["perception_mark_decay_s"] else "large_blob")
                    return
                reason = "closing" if closing_fast else "crossing"
                self._threat_samples = []
                self._start_intruder_evade(sim, reason, radial, cross,
                                           (bx, by))
            else:
                # Non-firing window: whatever is ahead is NOT closing faster
                # than the drone can account for — remember it as known
                # static geometry for future suppression.
                self._static_blocks.append((bx, by))
                trimmed = []
                for sx, sy in self._static_blocks:
                    if not any(math.hypot(sx - tx, sy - ty) <= 1.0
                               for tx, ty in trimmed):
                        trimmed.append((sx, sy))
                self._static_blocks = trimmed[-30:]

    def _distance_to_polyline(self, point, polyline):
        """Planar distance from ``point`` to the nearest segment of a world
        polyline (points may carry a z component; it is ignored)."""
        px, py = point[0], point[1]
        best = None
        for index in range(len(polyline) - 1):
            ax, ay = polyline[index][0], polyline[index][1]
            bx_, by_ = polyline[index + 1][0], polyline[index + 1][1]
            seg_dx, seg_dy = bx_ - ax, by_ - ay
            seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy
            if seg_len_sq < 1e-12:
                t = 0.0
            else:
                t = max(0.0, min(1.0, ((px - ax) * seg_dx + (py - ay) * seg_dy)
                                 / seg_len_sq))
            cx, cy = ax + t * seg_dx, ay + t * seg_dy
            d = math.hypot(px - cx, py - cy)
            if best is None or d < best:
                best = d
        return best

    @staticmethod
    def _grid_segment_free(grid, start, end):
        """Check the full segment against closed occupied cell rectangles.

        The supplied grid already includes vehicle clearance. Boundary contact
        is blocked as well; sparse point probes cannot prove a segment safe.
        """
        if any(not math.isfinite(v) for p in (start, end) for v in p[:2]):
            return False
        if grid.world_to_cell(start[:2]) is None or grid.world_to_cell(end[:2]) is None:
            return False
        for index, occupied in enumerate(grid.cells):
            if not occupied:
                continue
            cy, cx = divmod(index, grid.width)
            lower = (grid.origin[0] + cx * grid.resolution,
                     grid.origin[1] + cy * grid.resolution)
            enter, leave = 0.0, 1.0
            for axis in (0, 1):
                delta = end[axis] - start[axis]
                lo, hi = lower[axis], lower[axis] + grid.resolution
                if abs(delta) < 1e-12:
                    if start[axis] < lo or start[axis] > hi:
                        enter, leave = 1.0, 0.0
                        break
                else:
                    a, b = (lo - start[axis]) / delta, (hi - start[axis]) / delta
                    enter, leave = max(enter, min(a, b)), min(leave, max(a, b))
            if enter <= leave:
                return False
        return True

    def _start_intruder_evade(self, sim, reason, radial, cross, block_xy):
        """Yield laterally OFF the intruder's side of the route and hold there
        until it has passed, then resume.

        The yield is a SINGLE straight hop to a lateral point, committed
        directly (no A* worker): against a closing intruder every second of
        planning / micro-waypoint settling is closing distance, and the hop
        is one short segment that can be proven inline (SafetyMap + live
        grid + corridor probe)."""
        with self.lock:
            models = self.samples.get("models")
        if (not models or self.model_name not in models[0] or not self.route_world_current):
            return
        current = models[0][self.model_name][0]
        lateral = self.c["perception_evade_lateral_m"]
        # Keep the accepted mission goal separate from temporary yield routes.
        final_goal = self.mission_goal_world
        # Direction-aware yield: advance along the goal bearing while
        # stepping aside, so the hop keeps forward progress instead of a
        # 90-degree stop-turn.
        goal_dx = final_goal[0] - current[0]
        goal_dy = final_goal[1] - current[1]
        goal_len = math.hypot(goal_dx, goal_dy) or 1.0
        ux, uy = goal_dx / goal_len, goal_dy / goal_len
        lead = 1.0
        # Dodge AWAY from the intruder: prefer the side opposite its bearing.
        # Head-on traffic sits on the line (ambiguous) so default to +y.
        # perpendicular of the goal bearing, both sides
        px_, py_ = -uy, ux
        if block_xy[1] <= current[1]:
            preferred = (current[0] + ux * lead + px_ * lateral,
                         current[1] + uy * lead + py_ * lateral)
            alternate = (current[0] + ux * lead - px_ * lateral,
                         current[1] + uy * lead - py_ * lateral)
        else:
            preferred = (current[0] + ux * lead - px_ * lateral,
                         current[1] + uy * lead - py_ * lateral)
            alternate = (current[0] + ux * lead + px_ * lateral,
                         current[1] + uy * lead + py_ * lateral)
        chosen = None
        grid, _version = self.grid_source.snapshot()
        for candidate in (preferred, alternate):
            try:
                seg = (tuple(current[:3]), (candidate[0], candidate[1], current[2]))
                self.route_plan.safety.validate_route(
                    seg, tuple(self.route_plan.start_world),
                    self.c["max_radius_m"], self.c["max_altitude_m"])
                cell = grid.world_to_cell(candidate)
                if cell is None or not grid.is_free(cell):
                    continue
                if self._grid_segment_free(grid, current, candidate):
                    chosen = candidate
                    break
            except (RouteError, ValueError, TypeError):
                continue
        if chosen is None:
            # Review finding: continuing toward a confirmed closing threat
            # with no provable yield side violates fail-closed.  A bounded
            # fail-safe landing is the honest outcome here.
            self.event("INTRUDER_EVADE_FAILED", reason="no_free_lateral_side",
                       current_xy=[current[0], current[1]])
            raise FlightError("INTRUDER_EVADE_NO_SAFE_SIDE")
        route_world = [(current[0], current[1], current[2]),
                       (chosen[0], chosen[1], current[2])]
        route_local = [list(self.transform.world_to_local(point))
                       for point in route_world]
        self.route_start_local = route_local[0][:]
        self.route_local = route_local
        self.route_world_current = [list(point) for point in route_world]
        self._corridor_hits = 0
        self._threat_samples = []
        self._publish_path(self.path_world_pub, "gazebo_world", route_world)
        self._publish_path(self.path_local_pub, "mavros_local_enu", route_local)
        self.queue = [dict(name="intruder_evade_%04d" % self._replan_seq,
                           position=route_local[1][:],
                           world_position=list(route_world[1]),
                           hover_time_s=self.c["goal_hover_time_s"])]
        self._evade = {"lateral": chosen, "final": final_goal}
        self._evade_entered_sim = sim
        self.event("INTRUDER_EVADE_LATERAL", lateral_xy=[chosen[0], chosen[1]],
                   final_xy=[final_goal[0], final_goal[1]], reason=reason,
                   radial_m=round(radial, 2), cross_m=round(cross, 2),
                   blob_cells=self.local_grid.occupied_count_at(block_xy[:2], 1.0)
                   if self.local_grid is not None else None)
        self.next_waypoint(sim)

    def _evade_hold_tick(self, sim):
        """At the lateral hold point, resume once the intruder has passed
        (corridor toward the final goal clear, or it stopped closing, or the
        hold time-box expires) — otherwise keep waiting."""
        if self._evade is None:
            return
        with self.lock:
            models = self.samples.get("models")
        if not models or self.model_name not in models[0]:
            return
        world = models[0][self.model_name][0]
        final_goal = self._evade["final"]
        probe_route = [tuple(world[:2]), final_goal]
        blocked = self._corridor_blocked_samples(world, probe_route)
        hit = self._nearest_ahead_block(world)
        expired = self._evade_entered_sim is not None and \
            sim - self._evade_entered_sim > self.c["perception_evade_max_hold_s"]
        resume = (blocked < self.c["perception_blocked_min_samples"]) or expired
        if resume:
            self.event("INTRUDER_EVADE_RESUME", blocked_samples=blocked,
                       expired=bool(expired))
            # Resume MUST target the ORIGINAL final goal.  route_world_current
            # currently ends at the lateral yield point (the evade replan
            # committed it), so the default goal would strand the mission
            # there — an earlier review found exactly that.  Pass the stored
            # final goal explicitly; the commit re-checks the route endpoint.
            resume_goal = self._evade["final"]
            self._evade_resumed_sim = sim
            self._evade = None
            self._evade_entered_sim = None
            self.trigger_perception_replan(sim, max(1, blocked), resume_goal)
            return
        self.event("INTRUDER_EVADE_HOLD", blocked_samples=blocked,
                   gap_m=round(hit[0], 2) if hit is not None else None)
        self.transition("HOVER", self.c["waypoint_timeout_s"], sim)

    def trigger_perception_replan(self, sim, blocked, goal_world=None):
        with self.lock:
            models = self.samples.get("models")
        snapshot = self.snapshot()
        if (not models or self.model_name not in models[0] or "pose" not in snapshot
                or not self.route_world_current):
            return
        current_world = models[0][self.model_name][0]
        goal_world = tuple(self.route_world_current[-1][:2]) if goal_world is None \
            else tuple(goal_world[:2])
        if self.phase != "REPLAN":
            self.replan_resume = (self.phase, self.active, list(self.queue), self.target[:])
        self.target = list(self.xyz(snapshot["pose"][0].pose.position))
        self.command_velocity = [0.0, 0.0, 0.0]
        self.transition("REPLAN", self.c["route_planning_timeout_s"] + 1.0, sim)
        self._replan_seq += 1
        self._last_perception_replan_version = self.local_grid.map_version
        self.replan_request = ("perception", self._replan_seq, blocked)
        self.replan_worker = Worker(lambda: replan_with_grid_source(
            self.route_plan, current_world[:2], goal_world, self.c, self.grid_source))
        self.event("PERCEPTION_REPLAN_STARTED", blocked_samples=blocked,
                   goal_world_xy=[goal_world[0], goal_world[1]],
                   map_version=self.local_grid.map_version)

    def snapshot(self):
        with self.lock:
            return dict(self.samples)

    def on_cancel(self, _msg):
        self.cancel = True

    def on_signal(self, _signum, _frame):
        self.cancel = True  # Keep ROS alive until controlled landing completes.

    def on_goal(self, msg):
        with self.lock:
            self.pending_goal = msg

    @staticmethod
    def xyz(vector):
        return [float(v) if math.isfinite(v) else None for v in (vector.x, vector.y, vector.z)]

    def event(self, event, **extra):
        s = self.snapshot()
        pose, state, velocity, extended = [s.get(k, (None,))[0] for k in ("pose", "state", "velocity", "extended")]
        row = dict(utc=datetime.now(timezone.utc).isoformat(), run_id=self.run_id,
                   elapsed_s=time.monotonic() - self.started, ros_s=rospy.Time.now().to_sec(),
                   event=event, phase=self.phase, reason=self.reason, command=self.command, target=self.target,
                   connected=state.connected if state else None, mode=state.mode if state else None,
                   armed=state.armed if state else None, landed_state=extended.landed_state if extended else None,
                   position=self.xyz(pose.pose.position) if pose else None,
                   velocity=self.xyz(velocity.twist.linear) if velocity else None,
                   route_clearance_m=self.route_last_clearance,
                   route_tracking_error_m=self.route_last_tracking_error)
        row.update(extra)
        payload = json.dumps(row, ensure_ascii=False, allow_nan=False)
        self.log.write(payload + "\n")
        self.status.publish(String(data=payload))
        if event != "TELEMETRY":
            rospy.loginfo("[flight] %s phase=%s %s", event, self.phase, json.dumps(extra))

    def fresh(self, s, key, now, sim):
        if key not in s:
            return False
        msg, received = s[key]
        limit = self.c["pose_timeout_s"] if key in ("pose", "velocity") else self.c["state_timeout_s"]
        age = sim - msg.header.stamp.to_sec()
        return now - received <= limit and msg.header.stamp.to_sec() > 0 and -0.1 <= age <= limit

    def healthy(self, s, now, sim):
        # Clock stall first: a frozen Gazebo freezes every stream at once,
        # and the clock is the root cause worth reporting (pose/state
        # staleness is then a symptom, not the diagnosis).
        if now - self.clock_wall > self.c["clock_timeout_s"]:
            return "CLOCK_STALLED"
        if not self.fresh(s, "state", now, sim) or not s["state"][0].connected:
            return "FCU_DISCONNECTED_OR_STATE_STALE"
        for key in ("pose", "velocity"):
            if not self.fresh(s, key, now, sim):
                return key.upper() + "_STALE"
        p = s["pose"][0]
        if p.header.frame_id != self.c["frame_id"]:
            return "POSE_FRAME_MISMATCH"
        if None in self.xyz(p.pose.position) or None in self.xyz(s["velocity"][0].twist.linear):
            return "NONFINITE_TELEMETRY"
        if self.origin:
            pos = self.xyz(p.pose.position)
            if pos[2] - self.origin[2] > self.c["max_altitude_m"]:
                return "ALTITUDE_LIMIT"
            if math.hypot(pos[0] - self.origin[0], pos[1] - self.origin[1]) > self.c["max_radius_m"]:
                return "HORIZONTAL_LIMIT"
        return ""

    def transition(self, phase, timeout, sim):
        self.phase = phase
        self.deadline = time.monotonic() + timeout
        self.phase_sim, self.settled_since = sim, None
        if phase == "MOVE" and self._depth_move_sim is None:
            self._depth_move_sim = sim  # staleness clock start for a never-seen camera
        self.event("PHASE")

    def rpc(self, key, service, kind, **args):
        now = time.monotonic()
        worker = self.workers.get(key)
        if worker:
            if worker.done.is_set():
                self.event("SERVICE_RESULT", service=service, error=worker.error, response=str(worker.result))
                del self.workers[key]
            elif now - worker.started > self.c["service_timeout_s"]:
                raise FlightError("SERVICE_TIMEOUT:" + service)
            else:
                return
        if now - self.last_request.get(key, 0) < 2:
            return
        def call():
            rospy.wait_for_service(service, timeout=self.c["service_timeout_s"])
            return rospy.ServiceProxy(service, kind)(**args)
        self.workers[key] = Worker(call)
        self.last_request[key] = now
        self.event("SERVICE_REQUEST", service=service, arguments=args)

    def check_writers(self, now):
        worker = self.workers.get("graph")
        if worker:
            if worker.done.is_set():
                del self.workers["graph"]
                if worker.error:
                    raise FlightError("ROS_GRAPH_UNAVAILABLE:" + worker.error)
                code, _, graph = worker.result
                if code != 1:
                    raise FlightError("ROS_GRAPH_UNAVAILABLE")
                for topic, nodes in graph[0]:
                    control = topic.startswith((self.mavros_ns + "/setpoint_position/",
                                                self.mavros_ns + "/setpoint_velocity/",
                                                self.mavros_ns + "/setpoint_attitude/"))
                    control = control or topic in (self.mavros_ns + "/setpoint_raw/local",
                                                   self.mavros_ns + "/setpoint_raw/global",
                                                   self.mavros_ns + "/setpoint_raw/attitude",
                                                   self.mavros_ns + "/actuator_control")
                    others = [n for n in nodes if n != rospy.get_name()]
                    if control and others:
                        raise FlightError("CONTROL_CONFLICT:" + topic + ":" + str(others))
                self.last_graph = now
            elif now - worker.started > self.c["service_timeout_s"]:
                raise FlightError("ROS_GRAPH_TIMEOUT")
        elif now - self.last_graph > 1:
            self.workers["graph"] = Worker(lambda: rospy.get_master().getSystemState())

    def _gazebo_scene_worker(self):
        """Runs outside the 20 Hz loop; services may block or time out."""
        route = prepare_route(self.c, expected_container_id=os.environ.get("HOSTNAME"))
        rospy.wait_for_service("/gazebo/get_world_properties", timeout=self.c["service_timeout_s"])
        rospy.wait_for_service("/gazebo/get_model_state", timeout=self.c["service_timeout_s"])
        rospy.wait_for_service("/gazebo/get_model_properties", timeout=self.c["service_timeout_s"])
        worlds = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)()
        if not worlds.success:
            raise RouteError("GAZEBO_WORLD_PROPERTIES_FAILED:" + worlds.status_message)
        required = {model.name for model in route.expected_models} | {self.model_name, "training_ground"}
        dynamic = check_scene_model_list(required, worlds.model_names, self.c["scene_mode"],
                                         self.c["perception_max_dynamic_models"])
        state_service = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        properties_service = rospy.ServiceProxy("/gazebo/get_model_properties", GetModelProperties)
        observed = {}
        iris = None
        for name in sorted(required):
            if name != self.model_name:
                properties = properties_service(model_name=name)
                if not properties.success or not properties.is_static:
                    raise RouteError("GAZEBO_MODEL_NOT_STATIC:" + name)
            response = state_service(model_name=name, relative_entity_name="world")
            if not response.success:
                raise RouteError("GAZEBO_MODEL_STATE_FAILED:" + name + ":" + response.status_message)
            position = self.xyz(response.pose.position)
            if None in position:
                raise RouteError("GAZEBO_MODEL_STATE_NONFINITE:" + name)
            item = (tuple(position), self.yaw(response.pose.orientation))
            if name == self.model_name:
                iris = item
            else:
                observed[name] = item
        verify_runtime_models(route, observed, self.c["gazebo_model_tolerance_m"],
                              self.c["gazebo_model_yaw_tolerance_rad"])
        return route, observed, iris, dynamic

    def _publish_path(self, publisher, frame_id, points):
        message = Path()
        message.header.stamp, message.header.frame_id = rospy.Time.now(), frame_id
        for point in points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        publisher.publish(message)

    def begin_scene_validation(self):
        if "scene" not in self.workers:
            self.workers["scene"] = Worker(self._gazebo_scene_worker)
            self.event("ROUTE_SCENE_VALIDATION_STARTED")

    def finish_scene_validation(self, now, sim, snapshot):
        worker = self.workers.get("scene")
        if not worker:
            self.begin_scene_validation()
            return False
        if not worker.done.is_set():
            if now - worker.started > self.c["service_timeout_s"] + self.c["route_planning_timeout_s"]:
                raise FlightError("ROUTE_SCENE_VALIDATION_TIMEOUT")
            return False
        del self.workers["scene"]
        if worker.error:
            raise FlightError("ROUTE_SCENE_VALIDATION_FAILED:" + worker.error)
        route, _observed, iris, preflight_dynamic = worker.result
        models = snapshot.get("models")
        if not models or now - models[1] > self.c["pose_timeout_s"]:
            raise FlightError("GAZEBO_MODEL_STATES_STALE")
        allowed = {model.name for model in route.expected_models} | {self.model_name, "training_ground"}
        try:
            dynamic = check_scene_model_list(allowed, models[0], self.c["scene_mode"],
                                             self.c["perception_max_dynamic_models"])
        except (RouteError, ValueError) as exc:
            raise FlightError(str(exc))
        if dynamic or preflight_dynamic:
            self.event("GAZEBO_DYNAMIC_MODELS_OBSERVED",
                       names=sorted(set(dynamic) | set(preflight_dynamic)))
        verify_runtime_models(route, models[0], self.c["gazebo_model_tolerance_m"],
                              self.c["gazebo_model_yaw_tolerance_rad"])
        if math.hypot(iris[0][0] - route.start_world[0], iris[0][1] - route.start_world[1]) > 0.10:
            raise FlightError("ROUTE_SPAWN_START_MISMATCH")
        try:
            route.safety.validate_route((iris[0],) + route.route_world, route.start_world,
                                        self.c["max_radius_m"], self.c["max_altitude_m"])
            if not route.safety.landing_safe(iris[0]):
                raise RouteError("ACTUAL_SPAWN_LANDING_CORRIDOR_UNSAFE")
        except (RouteError, ValueError) as exc:
            raise FlightError("ROUTE_ACTUAL_TAKEOFF_CORRIDOR_INVALID:" + str(exc))
        pose = snapshot["pose"][0].pose
        local = tuple(self.xyz(pose.position))
        yaw_observation = self.yaw(pose.orientation) - iris[1]
        # Position axes are deliberately fixed ENU. PX4/MAVROS attitude yaw
        # can carry EKF/body-frame bias, so it is only an admission monitor;
        # moving world/local pairs prove the translational mapping.
        try:
            from robocup_navigation.route_safety import WorldLocalTransform
            if abs(math.atan2(math.sin(yaw_observation), math.cos(yaw_observation))) > self.c["heading_observation_limit_rad"]:
                raise RouteError("HEADING_OBSERVATION_OUT_OF_BOUNDS")
            self.transform = WorldLocalTransform(iris[0], local, yaw_rad=self.c["fixed_enu_yaw_rad"])
            if not self.transform.validate_pair(iris[0], local, self.c["transform_pair_tolerance_m"]):
                raise RouteError("TRANSFORM_INITIAL_PAIR_MISMATCH")
            self.route_local = [list(self.transform.world_to_local(point)) for point in route.route_world]
        except (ImportError, ValueError, RouteError) as exc:
            raise FlightError("ROUTE_TRANSFORM_INVALID:" + str(exc))
        self.route_plan = route
        self.route_world_current = [list(point) for point in route.route_world]
        self.mission_goal_world = tuple(route.route_world[-1][:2])
        if self.c["scene_mode"] == "perception":
            # Obstacle continuation into the unscanned FOV shadow is assumed
            # to last one clearance budget: inflation expressed in cells.
            self.grid_source = GridSource.from_occupancy(
                self.local_grid, base_grid=GridMap.from_metadata(route.metadata),
                inflation_m=self.c["perception_inflation_m"],
                shadow_cells=int(math.ceil(
                    self.c["perception_inflation_m"]
                    / self.c["perception_grid_resolution_m"])))
        else:
            self.grid_source = GridSource.from_metadata(route.metadata)
        # Tracking includes the vertical takeoff segment from the physical
        # spawn.  Online replans replace this anchor with the current pose.
        self.route_start_local = self.origin[:]
        self.route_second_pair_pending = True
        self._publish_path(self.path_world_pub, "gazebo_world", route.route_world)
        self._publish_path(self.path_local_pub, "mavros_local_enu", self.route_local)
        self.event("ROUTE_READY", launch_id=route.run_scene["launch_id"],
                   world_sha256=route.world_sha256, metadata_sha256=route.metadata_sha256,
                   route_world=[list(point) for point in route.route_world], route_local=self.route_local,
                   world_origin=list(iris[0]), local_origin=list(local),
                   transform_yaw_rad=self.c["fixed_enu_yaw_rad"], yaw_observation_rad=yaw_observation,
                   transform_assumption="fixed_ENU_axes; yaw is admission monitor only",
                   metadata_file=self.c["metadata_file"], world_file=self.c["world_file"],
                   planning_elapsed_s=route.planning_elapsed_s)
        return True

    def check_route_runtime(self, now, snapshot):
        if not self.c["route_mode"] or not self.route_plan:
            return ""
        models = snapshot.get("models")
        if not models or now - models[1] > self.c["pose_timeout_s"]:
            return "GAZEBO_MODEL_STATES_STALE"
        try:
            allowed = {model.name for model in self.route_plan.expected_models} | {self.model_name, "training_ground"}
            dynamic = check_scene_model_list(allowed, models[0], self.c["scene_mode"],
                                             self.c["perception_max_dynamic_models"])
            if dynamic != self._dynamic_models:
                self._dynamic_models = dynamic
                self.event("GAZEBO_DYNAMIC_MODELS_CHANGED", names=list(dynamic))
            verify_runtime_models(self.route_plan, models[0], self.c["gazebo_model_tolerance_m"],
                                  self.c["gazebo_model_yaw_tolerance_rad"])
            local = tuple(self.xyz(snapshot["pose"][0].pose.position))
            iris = models[0].get(self.model_name)
            if not iris:
                return "GAZEBO_IRIS_MISSING"
            if not self.transform.validate_planar_pair(iris[0], local, self.c["transform_pair_tolerance_m"]):
                return "TRANSFORM_CONTINUOUS_PAIR_MISMATCH"
            # This phase is explicitly Gazebo-only.  Collision clearance and
            # altitude use physical simulator truth; MAVROS local position is
            # still checked against it and remains the controller feedback.
            # On-ground EKF z can briefly be below the world ground plane.
            world = iris[0]
            if (world[2] + self.c["vehicle_half_height_m"] + self.c["vertical_margin_m"] >
                    self.c["max_altitude_m"] + 1e-9):
                return "ROUTE_ACTUAL_ALTITUDE_LIMIT"
            clearance = self.route_plan.safety.clearance(world)
            self.route_last_clearance = clearance
            if not self.route_plan.safety.point_safe(world):
                return "ROUTE_ACTUAL_POSITION_UNSAFE"
            if clearance < self.c["minimum_clearance_m"]:
                return "ROUTE_ACTUAL_CLEARANCE_LOW"
            self.route_last_tracking_error = self._distance_to_route(local)
            if self.route_last_tracking_error > self.c["route_tracking_error_limit_m"]:
                # A deliberate lateral evade intentionally leaves the original
                # route; that deviation IS the safety response, not a loss of
                # control, so it must not trip the route-tracking fail-safe.
                # Altitude/clearance checks above still apply.  A short grace
                # after RESUME covers the transient re-approach to the route.
                evading = self._evade is not None
                in_grace = (self._evade_resumed_sim is not None
                            and now - self._evade_resumed_sim < self.c["perception_evade_grace_s"])
                if not evading and not in_grace:
                    return "ROUTE_TRACKING_ERROR_LIMIT"
            self.route_actual.append(list(local))
            if len(self.route_actual) > 1000:
                self.route_actual.pop(0)
            self._publish_path(self.path_actual_pub, "mavros_local_enu", self.route_actual)
            if self.route_second_pair_pending and math.hypot(local[0] - self.origin[0], local[1] - self.origin[1]) >= self.c["transform_second_pair_min_motion_m"]:
                self.route_second_pair_pending = False
                self.event("ROUTE_TRANSFORM_SECOND_PAIR_CONFIRMED")
            return ""
        except (RouteError, ValueError) as exc:
            return "ROUTE_RUNTIME_VALIDATION_FAILED:" + str(exc)

    def _distance_to_route(self, point):
        segments = [self.route_start_local] + self.route_local
        if len(segments) < 2:
            return float("inf")
        nearest = float("inf")
        for begin, end in zip(segments, segments[1:]):
            direction = [end[index] - begin[index] for index in range(2)]
            length_squared = sum(value * value for value in direction)
            fraction = 0.0 if length_squared == 0 else max(0.0, min(1.0, sum(
                (point[index] - begin[index]) * direction[index] for index in range(2)) / length_squared))
            nearest = min(nearest, math.sqrt(sum((point[index] - begin[index] - fraction * direction[index]) ** 2
                                                 for index in range(2))))
        return nearest

    def route_landing_safe(self, snapshot):
        if not self.c["route_mode"] or not self.route_plan:
            return not self.c["route_mode"]
        try:
            models = snapshot.get("models")
            if not models or time.monotonic() - models[1] > self.c["pose_timeout_s"]:
                return False
            iris = models[0].get(self.model_name)
            if not iris:
                return False
            return bool(self.route_plan.safety.landing_safe(iris[0]))
        except (RouteError, ValueError, TypeError):
            return False

    def configure_fcu(self, now):
        if self.param_index >= len(self.param_names):
            return True
        name = self.param_names[self.param_index]
        worker = self.workers.get("param")
        if worker:
            if not worker.done.is_set():
                if now - worker.started > self.c["service_timeout_s"]:
                    raise FlightError("PARAM_TIMEOUT:" + name)
                return False
            del self.workers["param"]
            if worker.error or not worker.result.success:
                raise FlightError("PARAM_FAILED:" + name + ":" + str(worker.error))
            value = worker.result.value
            if self.param_stage == "read":
                self.original_params[name] = value
                desired = {"COM_RCL_EXCEPT": ParamValue(integer=value.integer | 4),
                           "COM_OBL_ACT": ParamValue(integer=0), "COM_OBL_RC_ACT": ParamValue(integer=4),
                           "COM_OF_LOSS_T": ParamValue(real=1.0),
                           "LNDMC_Z_VEL_MAX": ParamValue(real=min(value.real, self.c["auto_land_speed_mps"])),
                           "MPC_LAND_CRWL": ParamValue(real=min(value.real, self.c["auto_land_speed_mps"])),
                           "MPC_LAND_SPEED": ParamValue(real=self.c["auto_land_speed_mps"])}[name]
                self.desired_param = desired
                self.event("PARAM_ORIGINAL", param=name, integer=value.integer, real=value.real)
                self.param_stage = "write"
            else:
                # MAVROS ParamSet returns the FCU-confirmed parameter value.
                if value.integer != self.desired_param.integer or abs(value.real - self.desired_param.real) > 1e-4:
                    raise FlightError("PARAM_VERIFY_FAILED:" + name)
                self.event("PARAM_CONFIGURED", param=name, integer=value.integer, real=value.real)
                self.param_index += 1
                self.param_stage = "read"
            return False
        if self.param_stage == "read":
            def call():
                rospy.wait_for_service(self.mavros_ns + "/param/get", timeout=self.c["service_timeout_s"])
                return rospy.ServiceProxy(self.mavros_ns + "/param/get", ParamGet)(param_id=name)
        else:
            def call():
                rospy.wait_for_service(self.mavros_ns + "/param/set", timeout=self.c["service_timeout_s"])
                return rospy.ServiceProxy(self.mavros_ns + "/param/set", ParamSet)(param_id=name, value=self.desired_param)
        self.workers["param"] = Worker(call)
        return False

    def restore_fcu(self):
        s, now, sim = self.snapshot(), time.monotonic(), rospy.Time.now().to_sec()
        if not self.original_params:
            return
        if not self.fresh(s, "state", now, sim) or s["state"][0].armed or not s["state"][0].connected:
            self.event("PARAM_RESTORE_DEFERRED", detail="FCU is not confirmed connected and disarmed")
            return
        # Restore land speed before detector thresholds (PX4 otherwise clamps them).
        for name, value in reversed(list(self.original_params.items())):
            def restore(name=name, value=value):
                rospy.wait_for_service(self.mavros_ns + "/param/set", timeout=self.c["service_timeout_s"])
                return rospy.ServiceProxy(self.mavros_ns + "/param/set", ParamSet)(param_id=name, value=value)
            worker = Worker(restore)
            done = worker.done.wait(self.c["service_timeout_s"])
            ok = (done and worker.error is None and worker.result.success
                  and worker.result.value.integer == value.integer
                  and abs(worker.result.value.real - value.real) < 1e-4)
            self.event("PARAM_RESTORED" if ok else "PARAM_RESTORE_FAILED", param=name,
                       integer=value.integer, real=value.real)
            if not ok:
                self.result = 1
                break

    def prearm_ground_stable(self, snapshot, now, sim):
        """Require PX4 and Gazebo to agree on a settled ground state."""
        problem = ""
        extended = snapshot.get("extended")
        models = snapshot.get("models")
        if not extended or now - extended[1] > self.c["state_timeout_s"]:
            problem = "PREARM_EXTENDED_STATE_STALE"
        elif extended[0].landed_state != ExtendedState.LANDED_STATE_ON_GROUND:
            problem = "PREARM_NOT_ON_GROUND"
        elif not models or now - models[1] > self.c["pose_timeout_s"] or self.model_name not in models[0]:
            problem = "PREARM_GAZEBO_POSE_STALE"
        elif math.dist(models[0][self.model_name][0], self.transform.world_origin_xyz) > self.c["prearm_spawn_tolerance_m"]:
            problem = "PREARM_SPAWN_MOVED"
        elif abs(self.xyz(snapshot["velocity"][0].twist.linear)[2]) > self.c["prearm_max_vertical_speed_mps"]:
            problem = "PREARM_VERTICAL_ESTIMATE_UNSTABLE"
        if problem:
            if problem != self.prearm_last_problem:
                self.event("PREARM_WAIT", detail=problem)
            self.prearm_last_problem = problem
            self.prearm_stable_since = None
            return False
        if self.prearm_stable_since is None:
            self.prearm_stable_since = now
            self.prearm_last_problem = "PREARM_SETTLING"
            self.event("PREARM_STABILITY_STARTED")
            return False
        if now - self.prearm_stable_since < self.c["prearm_settle_time_s"]:
            return False
        if self.prearm_last_problem != "PREARM_READY":
            self.prearm_last_problem = "PREARM_READY"
            self.event("PREARM_READY")
        return True

    def next_waypoint(self, sim):
        self.active = self.queue.pop(0)
        self.target = self.active["position"][:]
        if self.c["route_mode"] and self.active.get("world_position"):
            snapshot = self.snapshot()
            models = snapshot.get("models")
            pose = snapshot.get("pose")
            if models and pose and models[0].get(self.model_name):
                world_z = models[0][self.model_name][0][2]
                local_z = self.xyz(pose[0].pose.position)[2]
                # Rebase z once at the waypoint boundary.  Keeping the target
                # fixed for the whole segment preserves the slew/braking
                # invariant even if PX4's estimator offset later drifts.
                self.target[2] = self.active["world_position"][2] - world_z + local_z
        self.transition("MOVE", self.c["waypoint_timeout_s"], sim)
        self.event("TARGET", name=self.active["name"])

    def accept_goal(self, sim):
        # Only one A* worker may exist.  A newer request remains in
        # pending_goal; when the current worker finishes its late result is
        # discarded before the newest request is planned.
        if self.replan_worker is not None:
            if not self.replan_worker.done.is_set():
                return
            worker, request = self.replan_worker, self.replan_request
            self.replan_worker = self.replan_request = None
            with self.lock:
                superseded = self.pending_goal is not None
            if superseded:
                self.event("REPLAN_RESULT_DISCARDED", goal_seq=request[0], detail="NEWER_GOAL_PENDING")
            elif worker.error:
                if isinstance(request, tuple) and request and request[0] == "perception":
                    self._perception_replan_retry_or_fail(sim, worker.error)
                    return
                self.event("GOAL_REJECTED", goal_seq=request[0], detail=worker.error)
                self._resume_after_replan(sim)
                return
            else:
                self._apply_replan(request, worker.result, sim)
                return
        with self.lock:
            msg, self.pending_goal = self.pending_goal, None
        if msg is None:
            return
        pos = self.xyz(msg.pose.position)
        age = sim - msg.header.stamp.to_sec()
        frame = self.c.get("external_goal_frame_id", "map") if self.c["route_mode"] else "map"
        valid = (self.c["accept_external_goals"] and self.phase in ("MOVE", "HOVER", "REPLAN")
                 and self.active["name"] != "TAKEOFF" and msg.header.frame_id == frame
                 and msg.header.stamp.to_sec() > 0 and 0 <= age <= self.c["goal_max_age_s"] and None not in pos)
        if valid:
            valid = (self.c["route_mode"] or
                     (0.5 <= pos[2] - self.origin[2] < self.c["max_altitude_m"]
                      and math.hypot(pos[0] - self.origin[0], pos[1] - self.origin[1]) <= self.c["max_radius_m"]))
        if not valid:
            self.event("GOAL_REJECTED", goal_seq=msg.header.seq, detail="GOAL_MESSAGE_INVALID")
            if self.phase == "REPLAN":
                self._resume_after_replan(sim)
            return
        if self.c["route_mode"]:
            snapshot = self.snapshot()
            models = snapshot.get("models")
            if not models or self.model_name not in models[0] or "pose" not in snapshot:
                self.event("GOAL_REJECTED", goal_seq=msg.header.seq, detail="REPLAN_POSE_UNAVAILABLE")
                if self.phase == "REPLAN":
                    self._resume_after_replan(sim)
                return
            current_world = models[0][self.model_name][0]
            current_local = self.xyz(snapshot["pose"][0].pose.position)
            if self.phase != "REPLAN":
                self.replan_resume = (self.phase, self.active, list(self.queue), self.target[:])
            self.target = current_local
            self.command_velocity = [0.0, 0.0, 0.0]
            self.transition("REPLAN", self.c["route_planning_timeout_s"] + 1.0, sim)
            self.replan_request = (msg.header.seq, tuple(pos[:2]))
            self.replan_worker = Worker(lambda: replan_with_grid_source(
                self.route_plan, current_world[:2], pos[:2], self.c, self.grid_source))
            self.event("REPLAN_STARTED", goal_seq=msg.header.seq,
                       goal_world_xy=pos[:2], requested_z=pos[2],
                       cruise_world_z=self.c["route_altitude_m"])
            return
        self.event("GOAL_PREEMPTED", name=self.active["name"])
        self.queue = [dict(name="external_" + str(msg.header.seq), position=pos, hover_time_s=2.0)]
        self.next_waypoint(sim)
        self.event("GOAL_ACCEPTED", goal_seq=msg.header.seq)

    def _resume_after_replan(self, sim):
        if self.replan_resume is None:
            raise FlightError("REPLAN_RESUME_STATE_MISSING")
        previous_phase, self.active, self.queue, self.target = self.replan_resume
        self.replan_resume = None
        self._evade = None  # a failed replan aborts any yield attempt
        self._evade_entered_sim = None
        self.transition(previous_phase if previous_phase in ("MOVE", "HOVER") else "MOVE",
                        self.c["waypoint_timeout_s"], sim)
        self.event("REPLAN_RESUMED_PREVIOUS_ROUTE")

    def _perception_replan_retry_or_fail(self, sim, detail, goal_world=None):
        """Handle a failed perception replan: retry while a bounded budget
        remains, then fail closed.

        A perception corridor already carries OCCUPIED evidence, so resuming
        the previous route is never allowed (review P0).  Some failures are
        nonetheless TRANSIENT — e.g. RESUME replans that hit
        REPLAN_GOAL_OCCUPIED while the goal cell is still shadowed by a
        departing vehicle; the mark decay clears it a few seconds later.
        Retrying inside a time/mcount budget recovers those, and the budget
        makes the outcome still honest: exhausted budget raises
        PERCEPTION_REPLAN_UNRESOLVABLE (bounded fail-safe, never a guess)."""
        # Flag a transient goal-cell shadow so the retry releases it before the
        # next replan snapshot (run vo1Bwp #3: a departing intruder's shadow at
        # the destination blocked every replan).
        self._retry_goal_cell_clear = "REPLAN_GOAL_OCCUPIED" in str(detail)
        goal = goal_world if goal_world is not None else self.mission_goal_world
        if goal is None and self._evade is not None:
            goal = self._evade.get("final")
        budget = self.c["perception_replan_retry_s"]
        max_retries = self.c["perception_replan_retry_max"]
        if goal is not None and budget > 0 and max_retries > 0:
            state = self._perception_retry
            if state is None:
                state = {"goal": tuple(goal[:2]), "deadline": sim + budget,
                         "count": 0, "next_sim": sim + self.c["perception_replan_retry_gap_s"]}
                self._perception_retry = state
            if state["count"] < max_retries and sim < state["deadline"]:
                state["count"] += 1
                state["next_sim"] = sim + self.c["perception_replan_retry_gap_s"]
                self.event("PERCEPTION_REPLAN_RETRY", attempt=state["count"],
                           max_attempts=max_retries, detail=str(detail),
                           goal_world_xy=[state["goal"][0], state["goal"][1]])
                return
        self._perception_retry = None
        self._perception_replan_failed(sim, detail)

    def _fire_pending_perception_retry(self, sim):
        """Re-issue a deferred perception replan (called from the tick)."""
        state = self._perception_retry
        if state is None or self.replan_worker is not None \
                or self.replan_request is not None or self._evade is not None:
            return
        if sim < state["next_sim"]:
            return
        goal = state["goal"]
        # Keep the retry state across fires so the attempt counter actually
        # accumulates.  Nulling it here previously reset count to 1 every cycle,
        # defeating max_retries and letting a blocked goal cell spin into an
        # unbounded replan storm (run vo1Bwp #3).  Only advance the fire cursor.
        state["next_sim"] = sim + self.c["perception_replan_retry_gap_s"]
        # A goal cell blocked only by a transient depth shadow from a departing
        # vehicle must not strand the mission: the destination was reachable
        # when first planned, so release it before the retry replan snapshots
        # the grid.  Static geometry stays enforced by the base SafetyMap merge.
        if getattr(self, "_retry_goal_cell_clear", False) and self.local_grid is not None:
            self.local_grid.clear_cell(goal)
        self.event("PERCEPTION_REPLAN_RETRY_FIRED", attempt=state["count"],
                   goal_world_xy=[goal[0], goal[1]])
        self.trigger_perception_replan(sim, 1, goal)

    def _perception_replan_failed(self, sim, detail):
        """A blocked route cannot be resumed without a successful proof."""
        self.event("PERCEPTION_REPLAN_FAILED", detail=detail)
        self._evade = None
        self._evade_entered_sim = None
        raise FlightError("PERCEPTION_REPLAN_UNRESOLVABLE:" + str(detail))

    def _apply_replan(self, request, plan_result, sim):
        points, map_version = plan_result
        route_world = tuple(points)
        if isinstance(request, tuple) and request and request[0] == "perception":
            sequence = "perception_%04d" % request[1]
        else:
            sequence = request
        try:
            route_local = [list(self.transform.world_to_local(point)) for point in route_world]
        except (RouteError, ValueError, TypeError) as exc:
            detail = "REPLAN_TRANSFORM_INVALID:" + str(exc)
            if isinstance(request, tuple) and request and request[0] == "perception":
                self._perception_replan_retry_or_fail(sim, detail)
                return
            self.event("GOAL_REJECTED", goal_seq=sequence, detail=detail)
            self._resume_after_replan(sim)
            return
        if not route_local:
            detail = "REPLAN_EMPTY_ROUTE"
            if isinstance(request, tuple) and request and request[0] == "perception":
                self._perception_replan_retry_or_fail(sim, detail)
                return
            self.event("GOAL_REJECTED", goal_seq=sequence, detail=detail)
            self._resume_after_replan(sim)
            return
        previous_name = self.replan_resume[1]["name"] if self.replan_resume else None
        self.replan_resume = None
        # Review guard: a perception replan whose target was the ORIGINAL
        # final goal must actually end there.  (An evade-lateral replan is
        # exempt while _evade is set — its endpoint is the yield point by
        # design.)  Without this, a lost goal regresses the mission into a
        # "MISSION_COMPLETE" at the yield point, far from the real goal.
        if isinstance(request, tuple) and request and request[0] == "perception" \
                and self._evade is None:
            expected_goal = self.mission_goal_world
            got_goal = tuple(route_world[-1][:2])
            if math.hypot(got_goal[0] - expected_goal[0],
                          got_goal[1] - expected_goal[1]) > 1e-6:
                self.event("REPLAN_ENDPOINT_NOT_FINAL_GOAL",
                           expected_xy=[expected_goal[0], expected_goal[1]],
                           got_xy=[got_goal[0], got_goal[1]])
                raise FlightError("REPLAN_ENDPOINT_NOT_FINAL_GOAL")
        # A successful perception replan (incl. a retry) clears any pending
        # bounded-retry state so a later unrelated failure starts fresh.
        self._perception_retry = None
        self._retry_goal_cell_clear = False
        self.route_start_local = route_local[0][:]
        self.route_local = route_local
        self.route_world_current = [list(point) for point in route_world]
        self._corridor_hits = 0
        self._publish_path(self.path_world_pub, "gazebo_world", route_world)
        self._publish_path(self.path_local_pub, "mavros_local_enu", route_local)
        # Thin the dense A* cell path: consecutive waypoints closer than
        # PERCEPTION_WAYPOINT_MIN_GAP_M cost a full settle cycle each while
        # adding nothing (run 234626 crawled through 2 cm hops as an
        # intruder closed in).  Always keep the final goal waypoint.
        min_gap = self.c.get("waypoint_min_gap_m", 0.5)
        kept_local, kept_world = [], []
        for index, point in enumerate(route_local):
            is_final = index == len(route_local) - 1
            if kept_local and not is_final:
                last_pt = kept_local[-1]
                if math.hypot(point[0] - last_pt[0], point[1] - last_pt[1]) < min_gap:
                    continue
            kept_local.append(point)
            kept_world.append(route_world[index])
        route_local, route_world = kept_local, kept_world
        self.queue = [dict(name="external_%s_%04d" % (sequence, index), position=point[:],
                           world_position=list(kept_world[index]),
                           hover_time_s=self.c["goal_hover_time_s"] if index == len(kept_local) - 1 else 0.0)
                      for index, point in enumerate(kept_local)]
        # Fresh-start: the route was planned from the position captured at
        # trigger time, but inertia carries the drone past it while the plan
        # runs.  Re-anchoring to the live position avoids a wasteful (and,
        # against a closing intruder, fatal) backtrack to the stale start.
        # REVIEW GUARD: blindly skipping waypoints breaks the SafetyMap proof
        # of the original route — the connector "live position -> kept
        # waypoint" is unproven.  Trim only to the first waypoint whose
        # connector is proven by BOTH the static SafetyMap and the live
        # perception grid; if none is, keep the full queue (the backtrack is
        # slower but proven).  Waypoint positions are never re-anchored, so
        # every retained leg keeps its original proof.
        try:
            snapshot = self.snapshot()
            live = tuple(self.xyz(snapshot["pose"][0].pose.position))
        except (KeyError, TypeError, ValueError):
            live = None
        if live is not None and len(self.queue) > 1:
            models = snapshot.get("models") or {}
            iris = models.get(self.model_name) if isinstance(models, dict) else None
            best = min(range(len(self.queue)),
                       key=lambda i: sum((a - b) ** 2 for a, b in
                                         zip(live, self.queue[i]["position"])))
            for candidate in range(best, len(self.queue)):
                wp = self.queue[candidate]["world_position"]
                if not wp:
                    continue
                try:
                    live_w = (iris[0][0], iris[0][1], wp[2])
                    self.route_plan.safety.validate_route(
                        (live_w, tuple(wp[:3])),
                        tuple(self.route_plan.start_world),
                        self.c["max_radius_m"], self.c["max_altitude_m"])
                    probe = self._corridor_blocked_samples(
                        live_w, [tuple(live_w[:2]), tuple(wp[:2])])
                    ok = (probe == 0)
                except (RouteError, ValueError, TypeError):
                    ok = False
                if ok:
                    self.queue = self.queue[candidate:]
                    break
        if not (isinstance(request, tuple) and request and request[0] == "perception"):
            self.mission_goal_world = tuple(route_world[-1][:2])
            self._evade = None  # an external goal supersedes any yielding
            self._evade_entered_sim = None
            self._perception_retry = None
        self.event("GOAL_PREEMPTED", name=previous_name)
        self.next_waypoint(sim)
        if isinstance(request, tuple) and request and request[0] == "perception":
            self._replan_fail_streak = 0
            self._perception_retry = None
        self.event("GOAL_ACCEPTED", goal_seq=sequence, map_version=map_version,
                   route_world=[list(point) for point in route_world], route_local=route_local)

    def update_active_waypoint(self, snapshot, extended, now, sim):
        position = self.xyz(snapshot["pose"][0].pose.position)
        speed = math.sqrt(sum(v * v for v in self.xyz(snapshot["velocity"][0].twist.linear)))
        distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(position, self.target)))
        stable = distance <= self.c["arrival_tolerance_m"] and speed <= self.c["arrival_speed_mps"]
        self.settled_since = (self.settled_since if self.settled_since is not None else sim) if stable else None
        if self.c["route_mode"]:
            iris = snapshot["models"][0].get(self.model_name)
            if not iris:
                raise FlightError("GAZEBO_IRIS_MISSING")
            if iris[0][2] >= self.transform.world_origin_xyz[2] + self.c["takeoff_progress_m"]:
                self.airborne_seen = True
            if self.active["name"] == "TAKEOFF" and self.takeoff_watchdog:
                problem = takeoff_progress_problem(
                    self.takeoff_watchdog[0], self.takeoff_watchdog[1], now, iris[0][2],
                    self.c["takeoff_progress_timeout_s"], self.c["takeoff_progress_m"])
                if problem:
                    raise FlightError(problem)
        elif extended and extended.landed_state == ExtendedState.LANDED_STATE_IN_AIR:
            self.airborne_seen = True
        duration = self.c["settle_time_s"] if self.phase == "MOVE" else self.active["hover_time_s"]
        if self.settled_since is not None and sim - self.settled_since >= duration:
            if self.phase == "MOVE":
                self.event("ARRIVED", name=self.active["name"], distance_m=distance, speed_mps=speed)
                self.transition("HOVER", self.c["waypoint_timeout_s"], sim)
            else:
                self.event("HOVER_DONE", name=self.active["name"], distance_m=distance)
                if self.queue:
                    self.next_waypoint(sim)
                elif self._evade is not None:
                    # At the lateral yield point: re-check the corridor toward
                    # the final goal each settle cycle; resume when clear.
                    self._evade_hold_tick(sim)
                else:
                    self.landing("MISSION_COMPLETE", normal=True)

    def landing(self, reason, normal=False):
        if self.phase in ("DESCEND", "AUTO_LAND", "DONE"):
            return
        self.reason = reason
        if not self.owned:
            self.result = 2 if reason == "CANCELLED" else 1
            self.phase = "DONE"
            self.event("ABORTED_BEFORE_ARM")
            return
        self.result = 0 if normal else (2 if reason == "CANCELLED" else 1)
        s, sim, now = self.snapshot(), rospy.Time.now().to_sec(), time.monotonic()
        self.route_landing_confirmed = self.route_landing_safe(s)
        if self.c["route_mode"] and not self.route_landing_confirmed:
            # AUTO.LAND remains a bounded PX4 failsafe request, but cannot be
            # described as safe without a fresh verified vertical corridor.
            normal, self.result = False, 1
            self.event("LANDING_UNCONFIRMED", detail="ROUTE_LANDING_CORRIDOR_UNVERIFIED")
        if self.fresh(s, "pose", now, sim) and None not in self.xyz(s["pose"][0].pose.position):
            self.command = self.xyz(s["pose"][0].pose.position)
        self.target = self.command[:] if self.command else None
        # A new landing target starts a new bounded trajectory.  In an
        # abnormal landing AUTO.LAND takes control; retaining the previous
        # route setpoint velocity can move the hold target away from the
        # verified descent corridor before the mode switch completes.
        self.command_velocity = [0.0, 0.0, 0.0]
        self.landing_deadline = now + self.c["landing_timeout_s"]
        self.landing_started = now
        if normal:
            self.target[2] = min(self.command[2], self.origin[2] + self.c["landing_switch_altitude_m"])
        self.transition("DESCEND" if normal else "AUTO_LAND", self.c["landing_timeout_s"], sim)
        self.event("LANDING_START")

    def tick(self, now, sim, dt):
        s = self.snapshot()
        state, extended = s.get("state", (None,))[0], s.get("extended", (None,))[0]
        if self.cancel and self.phase not in ("DESCEND", "AUTO_LAND", "DONE"):
            self.landing("CANCELLED")
        if self.phase == "DONE":
            return
        if self.phase not in ("DESCEND", "AUTO_LAND"):
            self.check_writers(now)
            # Health (clock stall first) BEFORE wall-clock deadlines: a paused
            # Gazebo freezes every stream while wall deadlines keep ticking,
            # so a long pause must be reported as CLOCK_STALLED, never as a
            # bogus MISSION/phase timeout.
            if self.phase != "CONNECTING":
                problem = self.healthy(s, now, sim)
                if problem:
                    raise FlightError(problem)
            if now > self.mission_deadline:
                raise FlightError("MISSION_TIMEOUT")
            if now > self.deadline:
                if self.phase == "PRESTREAM" and self.prearm_last_problem:
                    raise FlightError("PREARM_STABILITY_TIMEOUT:" + self.prearm_last_problem)
                raise FlightError(self.phase + "_TIMEOUT")
            if self.phase != "CONNECTING":
                problem = self.check_route_runtime(now, s)
                if problem:
                    raise FlightError(problem)
                if self.c["route_mode"] and self.c["scene_mode"] == "perception":
                    # Threat check FIRST: a confirmed closing intruder must
                    # take the immediate lateral hop, not queue behind a
                    # corridor replan whose A* bend point loses the race to
                    # an on-line approach (run 234626 lesson).
                    if self._perception_retry is not None:
                        self._fire_pending_perception_retry(sim)
                    self.check_intruder_threat(sim)
                    self.check_perception_corridor(sim)
                    self.check_depth_staleness(sim)
                    if sim - self._last_decay > 1.0:
                        self._last_decay = sim
                        self.local_grid.decay_stale(self.c["perception_mark_decay_s"])
                    if sim - getattr(self, "_dbg_last", 0.0) > 5.0:
                        self._dbg_last = sim
                        summary = self.local_grid.summary() if self.local_grid else None
                        samples = getattr(self, "_threat_samples", [])
                        self.event("GRID_SUMMARY", counts=summary,
                                   last_depth_sim=self._last_depth_sim,
                                   depth_move_sim=self._depth_move_sim,
                                   threat_n=len(samples),
                                   threat_last_d=round(samples[-1][1], 2) if samples else None,
                                   sim_s=round(sim, 2))
                if self.phase in ("ARM_REQUEST", "MOVE", "HOVER", "REPLAN") and state.mode != "OFFBOARD":
                    raise FlightError("OFFBOARD_LOST:" + state.mode)
                if self.phase in ("MOVE", "HOVER", "REPLAN") and not state.armed:
                    raise FlightError("UNEXPECTED_DISARM")
        if self.phase == "CONNECTING":
            if self.healthy(s, now, sim) or not self.fresh(s, "extended", now, sim) or not self.last_graph:
                return
            if state.armed or extended.landed_state != ExtendedState.LANDED_STATE_ON_GROUND:
                raise FlightError("REFUSE_ALREADY_ARMED_OR_AIRBORNE")
            self.origin = self.xyz(s["pose"][0].pose.position)
            if self.c["route_mode"] and not self.finish_scene_validation(now, sim, s):
                return
            self.command, self.target = self.origin[:], self.origin[:]
            if self.c["route_mode"]:
                # First endpoint is vertical takeoff. Every A* segment then
                # settles, so no corner is cut while acceleration limits mature.
                self.queue = [dict(name="TAKEOFF", position=self.route_local[0][:],
                                   world_position=list(self.route_plan.route_world[0]), hover_time_s=0.0)]
                self.queue.extend(dict(name="ROUTE_%04d" % index, position=point[:],
                                       world_position=list(self.route_plan.route_world[index]),
                                       hover_time_s=self.c["goal_hover_time_s"] if index == len(self.route_local) - 1 else 0.0)
                                  for index, point in enumerate(self.route_local[1:], 1))
            else:
                self.queue = [dict(name="TAKEOFF", position=[self.origin[0], self.origin[1],
                                                           self.origin[2] + self.c["takeoff_altitude_m"]], hover_time_s=1.0)]
                for p in self.c["waypoints"]:
                    self.queue.append(dict(name=p["name"], position=[self.origin[i] + p[k]
                                            for i, k in enumerate(("x", "y", "z"))], hover_time_s=p["hover_time_s"]))
            self.mission_deadline = now + self.c["mission_timeout_s"]
            self.transition("CONFIGURE", self.c["connection_timeout_s"], sim)
        elif self.phase == "CONFIGURE":
            if state.armed:
                raise FlightError("UNEXPECTED_EXTERNAL_ARM")
            if self.configure_fcu(now):
                self.publish_enabled = True
                self.transition("PRESTREAM", self.c["mode_timeout_s"], sim)
        elif self.phase == "PRESTREAM":
            if state.armed:
                raise FlightError("UNEXPECTED_EXTERNAL_ARM")
            prearm_ready = (self.prearm_ground_stable(s, now, sim)
                            if self.c["route_mode"] else True)
            if (prearm_ready and sim - self.phase_sim >= self.c["prestream_duration_s"]
                    and self.pub.get_num_connections()):
                self.transition("MODE_REQUEST", self.c["mode_timeout_s"], sim)
        elif self.phase == "MODE_REQUEST":
            if state.armed:
                raise FlightError("UNEXPECTED_EXTERNAL_ARM")
            if state.mode == "OFFBOARD":
                self.event("OFFBOARD_CONFIRMED")
                self.transition("ARM_REQUEST", self.c["arming_timeout_s"], sim)
            else:
                self.rpc("mode", self.mavros_ns + "/set_mode", SetMode, base_mode=0, custom_mode="OFFBOARD")
        elif self.phase == "ARM_REQUEST":
            if state.armed:
                if not self.arm_requested:
                    raise FlightError("UNEXPECTED_EXTERNAL_ARM")
                self.event("ARMED_CONFIRMED")
                iris = s["models"][0].get(self.model_name) if self.c["route_mode"] else None
                if self.c["route_mode"] and not iris:
                    raise FlightError("GAZEBO_IRIS_MISSING")
                self.takeoff_watchdog = (now, iris[0][2]) if iris else None
                self.next_waypoint(sim)
            else:
                if self.c["route_mode"] and not self.prearm_ground_stable(s, now, sim):
                    raise FlightError("PREARM_STATE_CHANGED:" + str(self.prearm_last_problem))
                self.owned = True  # Recover even when arming succeeded but its ACK was lost.
                self.arm_requested = True
                self.rpc("arm", self.mavros_ns + "/cmd/arming", CommandBool, value=True)
        elif self.phase == "REPLAN":
            self.accept_goal(sim)
        elif self.phase in ("MOVE", "HOVER"):
            self.accept_goal(sim)
            if self.phase != "REPLAN":
                self.update_active_waypoint(s, extended, now, sim)
        elif self.phase in ("DESCEND", "AUTO_LAND"):
            if now > self.landing_deadline:
                self.result, self.phase, self.publish_enabled = 1, "DONE", False
                self.event("LANDING_UNCONFIRMED", detail="Deadline expired; setpoints stopped for PX4 failsafe")
                return
            if self.phase == "DESCEND":
                problem = self.healthy(s, now, sim)
                reached = not problem and s["pose"][0].pose.position.z <= self.target[2] + self.c["arrival_tolerance_m"]
                if problem or state.mode != "OFFBOARD" or reached or now > self.landing_deadline - self.c["landing_timeout_s"] / 2:
                    self.transition("AUTO_LAND", self.landing_deadline - now, sim)
            if self.phase == "AUTO_LAND":
                fresh_state = self.fresh(s, "state", now, sim) and state.connected
                grounded = self.fresh(s, "extended", now, sim) and extended.landed_state == ExtendedState.LANDED_STATE_ON_GROUND
                arm_worker = self.workers.get("arm")
                arm_pending = arm_worker is not None and not arm_worker.done.is_set()
                # A late arming ACK must not let pre-arming telemetry prove landing.
                confirmed_after = max(self.landing_started, arm_worker.finished or now) if arm_worker else self.landing_started
                new_confirmation = (fresh_state and grounded and s["state"][1] > confirmed_after
                                    and s["extended"][1] > self.landing_started)
                if new_confirmation and not state.armed and not arm_pending:
                    if self.result == 0 and not self.airborne_seen:
                        self.result = 1
                    self.phase, self.publish_enabled = "DONE", False
                    self.event("LANDED", result=self.result)
                    return
                if fresh_state and state.mode == "AUTO.LAND":
                    self.publish_enabled = False
                else:
                    try:
                        self.rpc("land", self.mavros_ns + "/set_mode", SetMode, base_mode=0, custom_mode="AUTO.LAND")
                    except FlightError:
                        self.publish_enabled = False
                if not fresh_state or not self.fresh(s, "pose", now, sim):
                    self.publish_enabled = False
        if self.publish_enabled:
            z_speed = self.c["landing_descent_speed_mps"] if self.phase == "DESCEND" else self.c["max_vertical_speed_mps"]
            if self.c["route_mode"]:
                next_command, next_velocity = acceleration_limited_slew(
                    self.command, self.target, self.command_velocity, dt,
                    self.c["max_horizontal_speed_mps"], z_speed,
                    self.c["max_horizontal_accel_mps2"], self.c["max_vertical_accel_mps2"])
                # The pure helper returns immutable tuples for deterministic
                # testing; the publisher applies the final altitude clamp.
                self.command, self.command_velocity = list(next_command), list(next_velocity)
            else:
                self.command = slew(self.command, self.target, dt, self.c["max_horizontal_speed_mps"], z_speed)
            self.command[2] = min(self.command[2], self.origin[2] + self.c["max_altitude_m"])
            msg = PoseStamped()
            msg.header.stamp, msg.header.frame_id = rospy.Time.now(), "map"
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.command
            msg.pose.orientation.z, msg.pose.orientation.w = math.sin(self.c["yaw_rad"] / 2), math.cos(self.c["yaw_rad"] / 2)
            self.pub.publish(msg)

    def run(self):
        previous_state = None
        try:
            while self.phase != "DONE" and not rospy.is_shutdown():
                now, sim = time.monotonic(), rospy.Time.now().to_sec()
                dt = sim - self.last_sim
                if dt > 0:
                    self.clock_wall = now
                if dt < -0.05 and self.phase not in ("CONNECTING", "DESCEND", "AUTO_LAND"):
                    self.landing("CLOCK_REWOUND")
                self.last_sim = sim
                try:
                    self.tick(now, sim, dt)
                except Exception as exc:
                    self.event("FAILURE", detail=type(exc).__name__ + ":" + str(exc))
                    if self.phase in ("DESCEND", "AUTO_LAND"):
                        self.publish_enabled = False
                        if now > self.landing_deadline:
                            self.phase, self.result = "DONE", 1
                    else:
                        self.landing(str(exc))
                state = self.snapshot().get("state", (None,))[0]
                values = (state.connected, state.mode, state.armed) if state else None
                if values != previous_state:
                    previous_state = values
                    self.event("STATE_CHANGED")
                if now - self.last_log >= (0.1 if self.c["route_mode"] else 0.5):
                    self.event("TELEMETRY")
                    self.last_log = now
                time.sleep(max(0.0, 1 / self.c["setpoint_rate_hz"] - (time.monotonic() - now)))
            if self.phase != "DONE":
                self.result = 1
            self.restore_fcu()
            self.event("RESULT", code=self.result, succeeded=self.result == 0)
            return self.result if self.result is not None else 1
        finally:
            self.log.close()


def main():
    # Lock BEFORE ROS init: duplicate launches must not evict the active node.
    with open("/tmp/robocup_single_uav_controller.lock", "a") as lease:
        try:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("CONTROL_CONFLICT: another RoboCup controller holds the lock", flush=True)
            return 3
        rospy.init_node("uav_navigation", disable_signals=True)
        if not rospy.get_param("/use_sim_time", False):
            rospy.logfatal("Simulation only: /use_sim_time must be true")
            return 2
        try:
            config = load_config(rospy.get_param("~config_file"))
        except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError) as exc:
            rospy.logfatal("CONFIG_INVALID: %s", exc)
            return 2
        return Navigator(config).run()


if __name__ == "__main__":
    raise SystemExit(main())
