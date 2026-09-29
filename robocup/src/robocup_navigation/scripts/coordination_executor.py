#!/usr/bin/env python3
"""Dev-harness executor: one process per UAV (WorkBuddy access layer).

Turns a coordination ROUTE_GRANT into real MAVROS setpoint flight and reports
VEHICLE_STATE / COMMAND_ACK from *measured facts*, never from an internal phase.

Ownership boundary (AGENTS.md):
    This file is transport and actuation only. It contains NO clearance,
    release, timeout-action or route-version semantics -- those live in the
    Codex core (robocup_navigation.coordination).

What it deliberately refuses to do:
    * never fills an unknown safety proof with a default `true`. If a clearance
      or grid source is not configured the executor emits no ROUTE_OFFER at
      all, so the core cannot grant (fail-closed).
    * never lets an older command overwrite a newer one: (epoch, route_version)
      must move forward, otherwise the message is dropped and never ACKed.
    * never reports COMPLETED from a waypoint index; completion is arrival at
      the final point AND a sustained measured stop.

Usage:
    ROS_NAMESPACE=/uav_1 rosrun robocup_navigation coordination_executor.py \
        _uav_id:=uav_1 _mavros_ns:=/uav_1/mavros _task_id:=task_1 ...
"""
import copy
import json
import math
import os
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, State as MavrosState
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import CommandBool, ParamSet, SetMode
from std_msgs.msg import String

try:
    from robocup_navigation.coordination_adapter import SustainedStop
except ImportError:  # pragma: no cover - host-side import fallback
    SustainedStop = None

try:
    from robocup_navigation.coordination.route_planner import (
        load_obstacles, plan_route)
except ImportError:  # pragma: no cover - host-side import fallback
    load_obstacles = plan_route = None

try:
    from robocup_navigation.coordination.route_planner import (
        load_obstacles, plan_route)
except ImportError:  # pragma: no cover - host-side import fallback
    load_obstacles = plan_route = None


def _dist(a, b):
    return math.dist(tuple(a[:3]), tuple(b[:3]))


def _seg_point_distance(p, a, b):
    """Distance from p to segment ab (3D), used for obstacle clearance."""
    ax, ay, az = a[:3]
    bx, by, bz = b[:3]
    px, py, pz = p[:3]
    dx, dy, dz = bx - ax, by - ay, bz - az
    len2 = dx * dx + dy * dy + dz * dz
    if len2 <= 1e-9:
        return _dist(p, a)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy + (pz - az) * dz) / len2))
    return math.dist((px, py, pz), (ax + t * dx, ay + t * dy, az + t * dz))


def _route_clearance(points, obstacles):
    """Minimum clearance of a polyline against known obstacles.

    obstacles: list of {xyz: [x,y,z], radius_m: float}. An empty list is NOT
    treated as "infinitely clear": the caller must say so explicitly via
    ~clearance_source.
    """
    if not obstacles:
        return None
    worst = float("inf")
    for i in range(len(points) - 1):
        for ob in obstacles:
            c = _seg_point_distance(ob["xyz"], points[i], points[i + 1])
            worst = min(worst, c - float(ob.get("radius_m", 0.0)))
    return max(0.0, worst)


class Executor(object):
    """Per-UAV actuator. Single writer: the timer owns MAVROS output."""

    IDLE_MODES = ("IDLE", "HOLDING", "LANDED")

    def p(self, name, default):
        """Param lookup: ROS param wins, then ~config_file, then the default."""
        return rospy.get_param("~" + name, self.cfg.get(name, default))

    def __init__(self):
        cfg_path = rospy.get_param("~config_file", "")
        self.cfg = json.load(open(cfg_path)) if cfg_path else {}
        self.uav_id = self.p("uav_id", "uav_1")
        ns_hint = self.p("mavros_ns", "")
        self.mavros_ns = ns_hint or ("/%s/mavros" % self.uav_id)

        self.takeoff_alt = float(self.p("takeoff_altitude_m", 2.4))
        self.setpoint_hz = float(self.p("setpoint_rate_hz", 20.0))
        self.state_hz = float(self.p("state_rate_hz", 10.0))
        self.arrival_tol = float(self.p("arrival_tolerance_m", 0.6))
        self.wp_tol = float(self.p("waypoint_tolerance_m", 0.8))
        self.stop_speed = float(self.p("stop_speed_mps", 0.25))
        self.stop_hold = float(self.p("stop_hold_s", 1.0))
        self.stop_drift = float(self.p("stop_drift_m", 0.05))
        self.max_fresh = float(self.p("state_freshness_s", 0.5))
        self.cruise_speed = float(self.p("cruise_speed_mps", 1.0))
        # MAVROS setpoint header frame (NOT the coordination protocol frame,
        # which is world_enu). Must match MAVROS ~local_frame, default "map".
        self.sp_frame = self.p("setpoint_frame_id", "map")

        # --- safety-evidence sources (both must be configured to offer) ----
        self.clearance_source = self.p("clearance_source", "none")
        self.grid_source = self.p("grid_source", "none")
        self.obstacles = self.p("obstacles", []) or []
        self.required_clearance_m = float(self.p("required_clearance_m", 0.8))
        self.tracking_bound_m = float(self.p("tracking_bound_m", 0.5))
        self.empty_world_clearance_m = float(self.p("empty_world_clearance_m", 25.0))
        # --- A* detour route (off by default, preserves prior behaviour) ---
        # When enabled the ROUTE_OFFER points follow an A* detour around the
        # buildings in the metadata, and obstacles are loaded from the same
        # metadata so clearance becomes a real measurement rather than a
        # constant. Not needed for empty-world runs.
        self.route_planner = self.p("route_planner", "straight")
        self.metadata_path = self.p("metadata_path", "")
        self.route_inflate_m = float(self.p("route_inflate_m", 0.5))
        # --- A* 绕障路线（默认关闭，保持原有直角折线行为）---------------
        # 打开后，ROUTE_OFFER 的 points 由 A* 生成，绕开 metadata 里的建筑；
        # 同时 obstacles 自动从 metadata 加载，使 clearance 是**真实测量**
        # 而非常量。空世界（clearance_source=empty_world）下无需打开。
        self.route_planner = self.p("route_planner", "straight")
        self.metadata_path = self.p("metadata_path", "")
        self.route_inflate_m = float(self.p("route_inflate_m", 0.5))
        self._route_plan_cache = None

        # --- task this UAV is a candidate for ------------------------------
        self.task_id = self.p("task_id", "")
        self.target_id = self.p("target_id", "")
        self.target_xyz = self.p("target_xyz", [])

        self.log_dir = rospy.get_param(
            "~log_dir", os.path.join(os.getcwd(), "coord_logs", self.uav_id))
        os.makedirs(self.log_dir, exist_ok=True)
        self.ev = open(os.path.join(self.log_dir, "executor_events.jsonl"), "a", buffering=1)

        # --- live state ----------------------------------------------------
        self._lock = threading.Lock()
        self.state = MavrosState()
        self.ext = ExtendedState()
        self.local = [0.0, 0.0, 0.0]
        self.spawn = [float(v) for v in self.p("spawn_xyz", [0.0, 0.0, 0.0])]
        self.pos = list(self.spawn)
        self.vel = [0.0, 0.0, 0.0]
        self.last_pose_s = 0.0
        self.mode = "IDLE"
        self.armed = False

        self.active = None            # accepted command
        self.points = []              # remaining waypoints of active command
        self.last_epoch = 0
        self.last_version = 0
        self.command_seq = 0
        self.rejections = []

        self.setpoint = [0.0, 0.0, 0.0]
        self._rc_failsafe_relaxed = False
        self._stop_reported = False
        self.stopper = (SustainedStop(self.stop_speed, self.stop_hold,
                                      self.stop_drift, self.max_fresh)
                        if SustainedStop else None)

        # --- ROS plumbing --------------------------------------------------
        self.sp_pub = rospy.Publisher(self.mavros_ns + "/setpoint_position/local",
                                      PoseStamped, queue_size=1)
        self.vs_pub = rospy.Publisher("/%s/coordination/vehicle_state" % self.uav_id,
                                      String, queue_size=50)
        self.ack_pub = rospy.Publisher("/%s/coordination/command_ack" % self.uav_id,
                                       String, queue_size=50)
        self.offer_pub = rospy.Publisher("/coordination/route_offer", String, queue_size=50)

        rospy.Subscriber(self.mavros_ns + "/state", MavrosState, self._on_state)
        rospy.Subscriber(self.mavros_ns + "/local_position/pose", PoseStamped, self._on_pose)
        rospy.Subscriber(self.mavros_ns + "/local_position/velocity_local",
                         TwistStamped, self._on_vel)
        rospy.Subscriber(self.mavros_ns + "/extended_state", ExtendedState, self._on_ext)
        rospy.Subscriber("/coordination/commands", String, self._on_command)

        self.set_mode = rospy.ServiceProxy(self.mavros_ns + "/set_mode", SetMode)
        self.arm = rospy.ServiceProxy(self.mavros_ns + "/cmd/arming", CommandBool)

        self.event("EXECUTOR_START", mavros_ns=self.mavros_ns,
                   clearance_source=self.clearance_source, grid_source=self.grid_source)

    # ------------------------------------------------------------------ log
    def event(self, name, **kw):
        rec = dict(ts=time.time(), sim_s=rospy.get_time() if not rospy.is_shutdown() else 0.0,
                   uav_id=self.uav_id, event=name)
        rec.update(kw)
        self.ev.write(json.dumps(rec, sort_keys=True) + "\n")

    # -------------------------------------------------------------- inputs
    def _relax_rc_failsafe(self):
        """DEV HARNESS ONLY: keep the RC-loss failsafe from blocking OFFBOARD.

        SITL has no RC receiver, so PX4 declares RC loss shortly after boot and
        latches a failsafe mode (AUTO.LOITER with NAV_RCL_ACT=1, AUTO.RTL with
        NAV_RCL_ACT=2). While a failsafe is latched PX4 **refuses the OFFBOARD
        switch entirely** -- the command is accepted (mode_sent true) but the
        mode never changes, so the vehicle can never arm or take off.

        The decisive fix is COM_RCL_EXCEPT=7 (exempt stick, switch and mode from
        the RC-loss check), which keeps the vehicle out of the failsafe state in
        the first place. Stretching the timeout alone is NOT enough.

        SITL-only: on real hardware RC loss must stay a real failsafe.
        """
        try:
            setp = rospy.ServiceProxy(self.mavros_ns + "/param/set", ParamSet)
            setp(param_id="COM_RCL_EXCEPT", value=ParamValue(integer=7, real=0.0))
            setp(param_id="NAV_RCL_ACT", value=ParamValue(integer=0, real=0.0))
            setp(param_id="COM_RC_LOSS_T", value=ParamValue(integer=0, real=60.0))
            self.event("RC_FAILSAFE_RELAXED", com_rcl_except=7,
                       nav_rcl_act=0, com_rc_loss_t=60.0)
        except Exception as exc:  # noqa: BLE001
            self.event("RC_FAILSAFE_RELAX_FAILED",
                       error=type(exc).__name__ + ":" + str(exc)[:120])

    def _on_state(self, msg):
        with self._lock:
            self.state = msg
            self.armed = bool(msg.armed)

    def _on_ext(self, msg):
        with self._lock:
            self.ext = msg

    def _on_pose(self, msg):
        with self._lock:
            # local_position is relative to THIS vehicle's EKF origin, so every
            # UAV would otherwise report ~ (0,0,0) and the core would see the
            # whole fleet stacked on one point (and every route "in conflict").
            self.local = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            self.pos = [self.spawn[i] + self.local[i] for i in range(3)]
            self.last_pose_s = time.monotonic()

    def _on_vel(self, msg):
        with self._lock:
            self.vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]

    def _fresh(self):
        return (time.monotonic() - self.last_pose_s) <= self.max_fresh

    # ------------------------------------------------------------- commands
    def _on_command(self, msg):
        try:
            env = json.loads(msg.data)
            data = env.get("data", {})
        except (ValueError, AttributeError):
            return
        if data.get("uav_id") != self.uav_id:
            return
        kind = env.get("kind")
        if kind == "ROUTE_GRANT":
            self._accept_grant(env, data)
        elif kind in ("HOLD_REQUEST", "LAND_REQUEST", "CANCEL_REQUEST"):
            self._accept_directive(kind, env, data)

    def _stale(self, epoch, version):
        """True when this command must not overwrite what we already carry."""
        if self.active is None:
            return epoch < self.last_epoch
        if epoch < self.last_epoch:
            return True
        if epoch == self.last_epoch and version <= self.last_version:
            return True
        return False

    def _accept_grant(self, env, data):
        epoch = int(data.get("epoch", 0))
        version = int(data.get("route_version", 0))
        if self._stale(epoch, version):
            self.rejections.append(dict(kind="ROUTE_GRANT", epoch=epoch,
                                        route_version=version, reason="STALE"))
            self.event("COMMAND_REJECTED", kind="ROUTE_GRANT", epoch=epoch,
                       route_version=version, reason="STALE")
            return
        pts = [list(p) for p in data.get("points", [])]
        if len(pts) < 2:
            self.rejections.append(dict(kind="ROUTE_GRANT", reason="EMPTY_ROUTE"))
            return
        with self._lock:
            self.active = dict(command_id=data.get("command_id"), task_id=data.get("task_id"),
                               epoch=epoch, route_version=version,
                               offer_id=data.get("offer_id"))
            self.points = pts[1:]
            self.last_epoch = epoch
            self.last_version = version
            self.mode = "EXECUTING"
        self._last_cmd = data.get("command_id") or ""
        self._stop_reported = False
        # ACCEPTED means "received and accepted", not "already flying".
        self._ack("ACCEPTED", "ACCEPTED_AFTER_FRESHNESS_CHECK", data)
        self.event("COMMAND_ACCEPTED", command_id=data.get("command_id"),
                   epoch=epoch, route_version=version)

    def _accept_directive(self, kind, env, data):
        epoch = int(data.get("epoch", 0))
        version = int(data.get("route_version", 0))
        if self._stale(epoch, version):
            self.rejections.append(dict(kind=kind, reason="STALE"))
            self.event("COMMAND_REJECTED", kind=kind, reason="STALE")
            return
        with self._lock:
            self.last_epoch = max(self.last_epoch, epoch)
            self.last_version = max(self.last_version, version)
            if kind == "HOLD_REQUEST":
                self.active = None
                self.points = []
                self.mode = "HOLDING"
                self.setpoint = list(self.pos)
            elif kind in ("LAND_REQUEST", "CANCEL_REQUEST"):
                self.active = None
                self.points = []
                self.mode = "LANDING"
                self.setpoint = [self.pos[0], self.pos[1], 0.0]
            self._last_cmd = data.get("command_id") or getattr(self, "_last_cmd", "")
            self._stop_reported = False
        self._ack("ACCEPTED", "ACCEPTED_" + kind, data)

    # ----------------------------------------------------------------- acks
    def _ack(self, status, reason, data):
        self.command_seq += 1
        msg = dict(schema_version=1, source_id=self.uav_id, seq=self.command_seq,
                   kind="COMMAND_ACK",
                   data=dict(uav_id=self.uav_id,
                             command_id=data.get("command_id") or "unspecified",
                             task_id=data.get("task_id"),
                             epoch=data.get("epoch", 0),
                             route_version=data.get("route_version", 0),
                             status=status, reason=reason))
        self.ack_pub.publish(String(json.dumps(msg["data"], sort_keys=True)))
        self.event("ACK", status=status, reason=reason,
                   command_id=data.get("command_id"), epoch=data.get("epoch"))

    # --------------------------------------------------------------- offers
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

    def _build_offer(self):
        """Emit a ROUTE_OFFER only when every safety proof is actually known."""
        if not self.task_id or not self.target_xyz:
            return None
        if self.clearance_source not in ("map", "empty_world") or \
                self.grid_source not in ("static_substitute",):
            # Unknown proof -> no offer -> the core cannot grant. Fail closed.
            return None
        with self._lock:
            start = list(self.pos)
        goal = [float(self.target_xyz[0]), float(self.target_xyz[1]),
                float(self.target_xyz[2]) if len(self.target_xyz) > 2 else self.takeoff_alt]
        # Start the offered route at the vehicle's ACTUAL position, not at a
        # nominal altitude: the core checks the reported position against the
        # reserved route every lease renewal and raises AUTHORIZATION_CONFLICT
        # as soon as it drifts past tracking_bound_m.
        points = self._route_points(start, goal)
        if points is None:
            return None
        obstacles = self._obstacles()
        if self.clearance_source == "empty_world" and not obstacles:
            clearance = self.empty_world_clearance_m
        else:
            clearance = _route_clearance(points, obstacles)
        if clearance is None:
            return None
        return dict(schema_version=1, source_id=self.uav_id,
                    kind="ROUTE_OFFER",
                    data=dict(offer_id="offer-%s-%d" % (self.uav_id, int(time.time() * 1000)),
                              uav_id=self.uav_id, task_id=self.task_id,
                              frame_id="world_enu",
                              points=points,
                              map_revision=int(self.p("map_revision", 1)),
                              valid_until_sim_s=rospy.get_time() + 30.0,
                              clearance_m=clearance,
                              tracking_bound_m=self.tracking_bound_m,
                              static_safe=clearance >= self.required_clearance_m,
                              grid_safe=(self.grid_source == "static_substitute"),
                              waiting_points=[]))

    # ----------------------------------------------------------------- loop
    def _publish_setpoint(self):
        with self._lock:
            if self.mode == "EXECUTING" and self.points:
                nxt = self.points[0]
                if _dist(self.pos, nxt) <= self.wp_tol:
                    self.points.pop(0)
                    nxt = self.points[0] if self.points else self.setpoint
                self.setpoint = list(nxt)
            sp = list(self.setpoint)
        # setpoint_position/local is in THIS vehicle's local frame, while
        # self.setpoint is world_enu (routes come from the core in world_enu).
        local_sp = [sp[i] - self.spawn[i] for i in range(3)]
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        # Careful: this is the MAVROS setpoint header, NOT the coordination
        # protocol frame (which is `world_enu`). MAVROS transforms into its
        # local frame (~local_frame, default "map"); an unknown frame_id makes
        # it drop the message, so PX4 never sees a setpoint stream and then
        # refuses the OFFBOARD switch. The working single-UAV node uses "map".
        msg.header.frame_id = self.sp_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = local_sp
        msg.pose.orientation.w = 1.0
        self.sp_pub.publish(msg)

    def _arrived_and_holding(self):
        """Route finished and we are at the target -> HOLDING.

        Without this the executor stays in EXECUTING forever, SustainedStop
        (which only accepts HOLDING/LANDED/IDLE) can never confirm the stop,
        and COMPLETED is never reported -- so the core never marks the task
        complete.
        """
        with self._lock:
            act, pts, mode = self.active, list(self.points), self.mode
            pos = list(self.pos)
        if not act or pts or mode != "EXECUTING" or not self.target_xyz:
            return
        goal = [float(self.target_xyz[0]), float(self.target_xyz[1]),
                float(self.target_xyz[2]) if len(self.target_xyz) > 2 else self.takeoff_alt]
        if _dist(pos, goal) <= self.arrival_tol:
            with self._lock:
                self.mode = "HOLDING"
            self.event("ARRIVED", xyz=[round(v, 2) for v in pos])

    def _publish_state(self):
        self._arrived_and_holding()
        with self._lock:
            speed = math.sqrt(sum(v * v for v in self.vel))
            stopped = self.stopper.update(
                self.uav_id, sim_s=rospy.get_time(), wall_s=time.monotonic(),
                position=list(self.pos), velocity=list(self.vel), mode=self.mode,
                active_command_id=(self.active or {}).get("command_id"),
                observed_epoch=self.last_epoch, route_version=self.last_version,
                command_id=(self.active or {}).get("command_id"),
                epoch=self.last_epoch, expected_route_version=self.last_version
            ) if self.stopper else False
            pos, vel = list(self.pos), list(self.vel)
            armed, mode = self.armed, self.mode
        rec = dict(schema_version=1, source_id=self.uav_id, kind="VEHICLE_STATE",
                   data=dict(uav_id=self.uav_id, frame_id="world_enu", xyz=pos,
                             velocity_xyz=vel,
                             health="OK" if self._fresh() else "DEGRADED",
                             mode=mode, armed=armed,
                             active_command_id=(self.active or {}).get("command_id"),
                             observed_epoch=self.last_epoch,
                             route_version=self.last_version,
                             map_revision=int(self.p("map_revision", 1))))
        self.vs_pub.publish(String(json.dumps(rec["data"], sort_keys=True)))

        # STARTED / STOPPED / COMPLETED are measured, never inferred from phase.
        with self._lock:
            act = self.active
            pts = list(self.points)
        if act and mode == "EXECUTING" and speed > self.stop_speed:
            if not getattr(self, "_started_sent", False):
                self._started_sent = True
                self._ack("STARTED", "MOTION_MEASURED", act)
        if stopped:
            if mode == "HOLDING" and act is None and getattr(self, "_last_cmd", "") \
                    and not self._stop_reported:
                # Report the measured stop ONCE per command. Repeating it is an
                # invalid ACK transition and only pollutes the core's state
                # machine (it was firing at the state rate).
                self._stop_reported = True
                self._ack("STOPPED", "SUSTAINED_STOP_MEASURED",
                          dict(command_id=self._last_cmd,
                               epoch=self.last_epoch, route_version=self.last_version))
            elif act and not pts and self.target_xyz:
                goal = [float(self.target_xyz[0]), float(self.target_xyz[1]),
                        float(self.target_xyz[2]) if len(self.target_xyz) > 2
                        else self.takeoff_alt]
                if _dist(pos, goal) <= self.arrival_tol:
                    self._ack("COMPLETED", "ARRIVED_AND_STOPPED", act)
                    self.active = None
                    self.mode = "HOLDING"
                    self._started_sent = False

    def spin(self):
        rate = rospy.Rate(self.setpoint_hz)
        last_state = 0.0
        last_offer = 0.0
        last_arm = 0.0
        offboard_ok = False
        while not rospy.is_shutdown():
            self._publish_setpoint()
            now = time.monotonic()
            if now - last_state > 1.0 / self.state_hz:
                last_state = now
                self._publish_state()
            if now - last_offer > 2.0:
                last_offer = now
                off = self._build_offer()
                if off is None:
                    self.event("OFFER_WITHHELD",
                               clearance_source=self.clearance_source,
                               grid_source=self.grid_source,
                               reason="UNKNOWN_SAFETY_PROOF")
                else:
                    self.offer_pub.publish(String(json.dumps(off["data"], sort_keys=True)))
                    self.event("OFFER_SENT", offer_id=off["data"]["offer_id"],
                               clearance_m=off["data"]["clearance_m"],
                               static_safe=off["data"]["static_safe"],
                               grid_safe=off["data"]["grid_safe"])
            with self._lock:
                connected = self.state.connected
                cur_mode = self.state.mode
                armed = self.armed
            if connected and not self._rc_failsafe_relaxed:
                # SITL has no RC, so PX4 declares RC loss ~0.5 s after boot and
                # latches AUTO.RTL; while in RTL it refuses OFFBOARD and nothing
                # would ever take off. Relax the failsafe once, on connect.
                self._rc_failsafe_relaxed = True
                self._relax_rc_failsafe()
            if connected and not offboard_ok and now - last_arm > 1.5:
                last_arm = now
                # Aim at the hover point BEFORE switching to OFFBOARD, otherwise
                # the vehicle would just hold its ground position forever.
                with self._lock:
                    if self._fresh():
                        self.setpoint = [self.pos[0], self.pos[1], self.takeoff_alt]
                    ready = self._fresh()
                if not ready:
                    rate.sleep()
                    continue
                try:
                    if cur_mode != "OFFBOARD":
                        # mode_sent only means the command left; wait for the
                        # mode to actually change before asking to arm.
                        r1 = self.set_mode(base_mode=0, custom_mode="OFFBOARD")
                        self.event("OFFBOARD_REQUESTED",
                                   mode_sent=bool(getattr(r1, "mode_sent", False)),
                                   current_mode=cur_mode)
                    elif not armed:
                        r2 = self.arm(value=True)
                        self.event("ARM_REQUESTED",
                                   success=bool(getattr(r2, "success", False)),
                                   result=getattr(r2, "result", None))
                        if bool(getattr(r2, "success", False)):
                            offboard_ok = True
                            self.event("OFFBOARD_ARMED")
                    else:
                        offboard_ok = True
                except Exception as exc:  # noqa: BLE001
                    self.event("OFFBOARD_RETRY",
                               error=type(exc).__name__ + ":" + str(exc)[:120])
            rate.sleep()


def main():
    rospy.init_node("coordination_executor")
    Executor().spin()


if __name__ == "__main__":
    main()
