#!/usr/bin/env python3
"""ROS wrapper for the coordination core -- DEVELOPMENT HARNESS, not六机 E2E.

Scope and honesty (interface v0 §5, AGENTS.md):
  * This node is WorkBuddy's transport/plumbing only. It never reinterprets
    clearance, release, timeout or route-version semantics.
  * Without the official multi-UAV launch it is a *dev harness*. A TODO skeleton
    must never be recorded as "six-UAV E2E passed".
  * The single-UAV node's ~goal topic is NOT a ROUTE_GRANT: it must never be
    turned into a flight command without the executor gate authorising it.
  * The coordinator is single-writer: ROS callbacks only enqueue; ONE timer
    thread drives the core. Do not let callbacks touch the core directly.

Transport: JSON strings over std_msgs/String. Message payloads must already
conform to schema v1 (see docs/coordination_interface_v0.md); the adapter stamps
only run_id/seq/sim_s/source_id.

Wiring summary (topics):
  sub  ~<uav>/coordination/vehicle_state   (JSON)  -> VEHICLE_STATE
  sub  ~<uav>/coordination/command_ack     (JSON)  -> COMMAND_ACK
  sub  /coordination/route_offer           (JSON)  -> ROUTE_OFFER   [planner]
  sub  /coordination/resource_clear        (JSON)  -> RESOURCE_CLEAR[verifier]
  sub  /coordination/cancel_task           (JSON)  -> CANCEL_TASK
  sub  /coordination/target_report         (JSON)  -> TARGET_REPORT [perception]
  pub  /coordination/commands              (JSON)  every core output, full stream
  pub  /coordination/events                (JSON)  coord_events mirror
  pub  ~<uav>/coordination/grant           (JSON)  this UAV's ROUTE_GRANT / stop

TODO(official): once the competition multi-UAV launch is frozen, replace the
dev-harness bring-up with it and enable the per-UAV isolation checks (controller
flock, ROS node names, control-topic conflict detection, TF, spawn transforms,
MAVLink id/port, model allow-list) before claiming six-UAV E2E.
"""
import collections
import json
import os
import signal
import sys
import threading
import time

import rospy
from std_msgs.msg import String

from robocup_navigation.coordination_adapter import CoreBus, GateFanout, PhysicalMonitor
from robocup_navigation.coordination.protocol import CoordinationError

# Dev-harness limits. Real numbers MUST come from the scenario config; these are
# placeholders and are NOT competition/PX4 safety parameters.
DEV_LIMITS = dict(min_separation_m=1.0, arrival_tolerance_m=0.15, mission_timeout_s=180.0,
                  deadlock_timeout_s=60.0, max_monitor_gap_s=1.1, state_timeout_s=3.0,
                  lease_s=3.0, stop_speed_mps=0.05, required_clearance_m=0.6,
                  tracking_bound_m=0.2, position_tolerance_m=0.05, nominal_speed_mps=0.3)


class CoordinationHarness:
    def __init__(self):
        run_id = rospy.get_param("~run_id", "run_%d" % int(time.time()))
        self.fleet = rospy.get_param("~fleet_ids", ["uav_%d" % i for i in range(1, 7)])
        test_case = rospy.get_param("~test_case_id", "dev_harness")
        limits = rospy.get_param("~limits", DEV_LIMITS)
        log_dir = rospy.get_param("~log_dir", os.path.join(os.getcwd(), "coord_logs", run_id))
        os.makedirs(log_dir, exist_ok=True)
        self.run_id = run_id
        self.tick_hz = float(rospy.get_param("~tick_hz", 10.0))
        # Read-only introspection of the core's PUBLIC state, for
        # diagnosing why scheduling produces nothing. It never changes
        # the core and is off unless explicitly enabled.
        self.debug_core = bool(rospy.get_param("~debug_core", False))
        self._last_dbg = -1.0

        # run_id must be globally unique (interface v0 §1); refuse to reuse a log.
        self.events_path = os.path.join(log_dir, "coord_events.jsonl")
        self.commands_path = os.path.join(log_dir, "commands.jsonl")
        self.events_file = open(self.events_path, "x", buffering=1, encoding="utf-8")
        self.commands_file = open(self.commands_path, "x", buffering=1, encoding="utf-8")

        tasks = rospy.get_param("~tasks", {})       # {task_id: {target_id, xyz}}
        self.tasks = tasks
        roles = {u: ["VEHICLE_STATE", "COMMAND_ACK"] for u in self.fleet}
        roles.update(planner=["ROUTE_OFFER"], verifier=["RESOURCE_CLEAR"],
                     clock=["TICK"], perception=["TARGET_REPORT"])
        # The core measures its mission/deadlock budgets from start_sim_s. With
        # a simulator that has been running for a while, the first message can
        # arrive at sim_s ~ 2400 s; seeding the core with 0.0 makes
        # `now - progress` exceed deadlock_timeout_s on the very first TICK, so
        # the core halts and never schedules anything. Seed from the live clock.
        try:
            rospy.wait_for_message("/clock", rospy.AnyMsg, timeout=10.0)
        except Exception:  # noqa: BLE001 - a wall-clock-only master still works
            pass
        start_sim_s = rospy.Time.now().to_sec()
        start_wall_s = time.monotonic()
        rospy.loginfo("[coordination] start_sim_s=%.3f start_wall_s=%.3f",
                      start_sim_s, start_wall_s)
        self.bus = CoreBus(run_id, self.fleet, tasks, limits, roles, test_case,
                           start_sim_s=start_sim_s, start_wall_s=start_wall_s)
        self.fan = GateFanout(run_id, self.fleet, limits["position_tolerance_m"],
                              max(1.0, limits["lease_s"]))
        self.monitor = PhysicalMonitor(self.fleet, limits["min_separation_m"],
                                       limits["arrival_tolerance_m"],
                                       limits["max_monitor_gap_s"],
                                       start_sim_s=start_sim_s)

        # Dev-harness placeholders for the executor fan-out.  TODO(executor):
        # replace with the LOCAL map revision and each UAV's MEASURED world_xyz.
        self._positions = {u: [0.0, 0.0, 0.0] for u in self.fleet}
        # Only UAVs that have actually reported may be monitored:/n        # the zero defaults are placeholders, and sampling them made
        # the whole fleet look co-located (separation 0 -> COLLISION).
        self._reported = set()
        self._arrival_seen = set()
        self._map_revision = 1

        # Single-writer queue: callbacks enqueue, the timer drains.
        self._q = collections.deque()
        self._lock = threading.Lock()

        self.cmd_pub = rospy.Publisher("/coordination/commands", String, queue_size=200)
        self.event_pub = rospy.Publisher("/coordination/events", String, queue_size=200)

        self._subs = []
        for u in self.fleet:
            self._subs.append(rospy.Subscriber("/%s/coordination/vehicle_state" % u, String,
                                               self._in(u, "VEHICLE_STATE"), queue_size=50))
            self._subs.append(rospy.Subscriber("/%s/coordination/command_ack" % u, String,
                                               self._in(u, "COMMAND_ACK"), queue_size=50))
        self._subs.append(rospy.Subscriber("/coordination/route_offer", String,
                                           self._in("planner", "ROUTE_OFFER"), queue_size=50))
        self._subs.append(rospy.Subscriber("/coordination/resource_clear", String,
                                           self._in("verifier", "RESOURCE_CLEAR"), queue_size=50))
        self._subs.append(rospy.Subscriber("/coordination/cancel_task", String,
                                           self._in("planner", "CANCEL_TASK"), queue_size=50))
        self._subs.append(rospy.Subscriber("/coordination/target_report", String,
                                           self._in("perception", "TARGET_REPORT"), queue_size=50))
        self._timer = rospy.Timer(rospy.Duration(1.0 / self.tick_hz), self._tick)

        rospy.loginfo("[coordination] dev harness up run_id=%s fleet=%s log=%s",
                      run_id, self.fleet, log_dir)

        # Without this the process is killed mid-run and the core never emits
        # RUN_FINISHED, so the verdict can only ABSTAIN.
        rospy.on_shutdown(self.finish)
        for _sig in (signal.SIGTERM, signal.SIGINT):
            try:
                signal.signal(_sig, lambda *_a: (self.finish(), rospy.signal_shutdown("signal")))
            except Exception:  # noqa: BLE001
                pass

    def _in(self, source_id, kind):
        def cb(msg):
            try:
                data = json.loads(msg.data)
            except ValueError as exc:
                rospy.logerr("[coordination] bad JSON for %s/%s: %s", source_id, kind, exc)
                return
            if kind == "VEHICLE_STATE" and isinstance(data.get("xyz"), list):
                with self._lock:
                    self._positions[data.get("uav_id", source_id)] = list(data["xyz"])
                    self._reported.add(data.get("uav_id", source_id))
            with self._lock:
                self._q.append((source_id, kind, data))
        return cb

    def _tick(self, _evt=None):
        self.process_queue()
        # TICK drives deadline checks; also covers gaps when no packet arrives.
        self._submit("clock", "TICK", {})
        self._sample_monitor()

    def _sample_monitor(self):
        """Feed the independent monitor from measured positions only."""
        with self._lock:
            if len(self._reported) < len(self.fleet):
                return  # never turn a placeholder into a measurement
            positions = {u: list(p) for u, p in self._positions.items()
                         if u in self._reported}
        targets = {}
        try:
            snap = self.bus.core.snapshot()
            for tid, t in (snap.get("tasks") or {}).items():
                owner = t.get("owner") or t.get("owner_uav_id")
                if not owner or tid not in self.tasks:
                    continue
                # "Arrival error" is the error AT ARRIVAL. Feeding the target on
                # every sample makes the run's maximum distance-to-target (i.e.
                # the transit distance) the "arrival error", so a target farther
                # away than arrival_tolerance_m can never pass. Record it once,
                # at the sample where the task completes.
                if t.get("complete") and tid not in self._arrival_seen:
                    self._arrival_seen.add(tid)
                    targets[owner] = list(self.tasks[tid]["xyz"])
        except Exception:  # noqa: BLE001 - monitoring must never break the loop
            targets = {}
        self.monitor.sample(rospy.Time.now().to_sec(), positions, targets, progressed=True)
        self._debug_core(positions)

    def _debug_core(self, positions):
        if not self.debug_core:
            return
        now = rospy.Time.now().to_sec()
        if now - self._last_dbg < 8.0:
            return
        self._last_dbg = now
        try:
            core = self.bus.core
            rospy.loginfo("[coord-debug] halted=%s finished=%s offers=%d vehicles=%d "
                          "tasks=%s", core.halted, core.finished, len(core.offers),
                          len(core.vehicles),
                          {k: v["status"] for k, v in core.tasks.items()})
            rospy.loginfo("[coord-debug] now=%.2f progress=%.2f started=%.2f deadlock_t=%.1f",
                          core.now, core.progress, core.started,
                          core.limits["deadlock_timeout_s"])
            for (u, tid), o in list(core.offers.items()):
                rospy.loginfo("[coord-debug] %s/%s valid_until=%.1f now=%.1f safe=%s "
                              "conflict=%s stopped=%s fresh=%s", u, tid,
                              o["valid_until_sim_s"], core.now,
                              core._offer_safe(o), core._conflict(o["points"], u),
                              core._stopped(u), core._fresh(u))
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn("[coord-debug] failed: %s", exc)

    def finish(self, *_args):
        """Let the core close the run so the verdict has a RUN_FINISHED."""
        if getattr(self, "_finished", False):
            return
        self._finished = True
        try:
            self._sample_monitor()
            self.bus.finish(self.monitor.report())
            self._flush_events()
        except Exception as exc:  # noqa: BLE001
            rospy.logerr("[coordination] finish failed: %s", exc)
        rospy.loginfo("[coordination] run finished; log=%s", self.log_dir)

    def process_queue(self):
        while True:
            with self._lock:
                if not self._q:
                    return
                item = self._q.popleft()
            self._submit(*item)

    def _submit(self, source_id, kind, data):
        sim_s = rospy.Time.now().to_sec()
        wall_s = time.monotonic()
        try:
            outputs = self.bus.submit(source_id, kind, data, sim_s, wall_s)
        except CoordinationError as exc:
            rospy.logwarn("[coordination] core rejected %s from %s: %s", kind, source_id, exc)
            self._flush_events()
            return
        with self._lock:
            positions = dict(self._positions)
        for m in outputs:
            payload = json.dumps(m, sort_keys=True, allow_nan=False)
            self.commands_file.write(payload + "\n")
            self.cmd_pub.publish(String(payload))
            # Fan-out needs the LOCAL map revision and each UAV's MEASURED
            # world_xyz.  The placeholders below keep the harness runnable; they
            # are NOT flight-worthy and Gazebo truth must never reach the core.
            self.fan.deliver([m], sim_s, wall_s, self._map_revision,
                             lambda u: positions.get(u, [0.0, 0.0, 0.0]))
        self._flush_events()

    def _flush_events(self):
        for e in self.bus.drain_events():
            line = json.dumps(e, sort_keys=True, allow_nan=False)
            self.events_file.write(line + "\n")
            self.event_pub.publish(String(line))


def main():
    rospy.init_node("coordination", anonymous=False)
    CoordinationHarness()
    rospy.spin()


if __name__ == "__main__":
    sys.exit(main())
