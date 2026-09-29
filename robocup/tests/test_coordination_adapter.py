"""Host tests for the WorkBuddy coordination adapter (no ROS, no rospy).

Run:  PYTHONPATH=src/robocup_navigation/src python tests/test_coordination_adapter.py
"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/robocup_navigation/src"))

from robocup_navigation.coordination_adapter import (  # noqa: E402
    COORDINATOR_SOURCE, CoreBus, GateFanout, InputSeq, PhysicalMonitor,
)
from robocup_navigation.coordination.protocol import CoordinationError  # noqa: E402

FLEET = ["uav_1", "uav_2"]
LIMITS = dict(min_separation_m=1., arrival_tolerance_m=.15, mission_timeout_s=180.,
              deadlock_timeout_s=60., max_monitor_gap_s=1.1, state_timeout_s=3.,
              lease_s=3., stop_speed_mps=.05, required_clearance_m=.6,
              tracking_bound_m=.2, position_tolerance_m=.05, nominal_speed_mps=.3)
TASKS = {"task1": dict(target_id="target1", xyz=[3., 0., 2.]),
         "task2": dict(target_id="target2", xyz=[3., 4., 2.])}


def make_bus(run_id="run_test"):
    roles = {u: ["VEHICLE_STATE", "COMMAND_ACK"] for u in FLEET}
    roles.update(planner=["ROUTE_OFFER"], verifier=["RESOURCE_CLEAR"], clock=["TICK"])
    return CoreBus(run_id, FLEET, TASKS, LIMITS, roles, "adapter-test")


class Harness:
    """Minimal two-UAV driver exercising bus + gates like a real adapter."""

    def __init__(self):
        self.bus = make_bus()
        self.fan = GateFanout("run_test", FLEET, .05, 3.)
        self.pos = {"uav_1": [0., 0., 2.], "uav_2": [0., 4., 2.]}
        self.sim, self.wall = 0., 0.
        self.all_outputs = []
        self.grants = {}

    def step(self, source, kind, data, dt=0.0):
        self.sim += dt
        self.wall += dt
        outs = self.bus.submit(source, kind, data, self.sim, self.wall)
        self.all_outputs.extend(outs)
        acts = self.fan.deliver(outs, self.sim, self.wall, 1, lambda u: self.pos[u])
        for u, entries in acts.items():
            for k, a in entries:
                if k == "ROUTE_GRANT":
                    self.grants[u] = a
        return outs, acts

    def bring_up(self):
        for u in FLEET:
            self.step(u, "VEHICLE_STATE", dict(uav_id=u, frame_id="world_enu", xyz=self.pos[u],
                     velocity_xyz=[0., 0., 0.], health="OK", mode="IDLE", armed=True,
                     active_command_id=None, observed_epoch=0, route_version=0, map_revision=1))
        for i, u in enumerate(FLEET):
            self.step("planner", "ROUTE_OFFER", dict(offer_id="offer%d" % i, uav_id=u,
                     task_id="task%d" % (i + 1), frame_id="world_enu",
                     points=[self.pos[u], TASKS["task%d" % (i + 1)]["xyz"]], map_revision=1,
                     valid_until_sim_s=100., clearance_m=1., tracking_bound_m=.1,
                     static_safe=True, grid_safe=True, waiting_points=[]))
        self.step("clock", "TICK", {}, dt=1.0)
        return self.grants


class InputSeqTests(unittest.TestCase):
    def test_monotonic_per_source_starting_at_one(self):
        s = InputSeq()
        self.assertEqual([s.next("uav_1"), s.next("uav_1"), s.next("uav_2"), s.next("uav_1")],
                         [1, 2, 1, 3])


class CoreBusTests(unittest.TestCase):
    def test_outputs_are_core_stamped_not_rewritten(self):
        h = Harness()
        h.bring_up()
        self.assertEqual(len(h.grants), 2)
        seqs = [m["seq"] for m in h.all_outputs]
        self.assertEqual(seqs, list(range(1, len(seqs) + 1)))  # core's own monotonic seq
        self.assertTrue(all(m["source_id"] == COORDINATOR_SOURCE for m in h.all_outputs))
        self.assertTrue(all(m["schema_version"] == 1 and m["run_id"] == "run_test"
                            for m in h.all_outputs))


class GateFanoutTests(unittest.TestCase):
    def test_full_stream_grants_both_uavs(self):
        h = Harness()
        h.bring_up()
        self.assertEqual(set(h.grants), {"uav_1", "uav_2"})
        self.assertIn("command_id", h.grants["uav_1"])

    def test_filtering_by_uav_breaks_sequence(self):
        """Delivering only uav_1's messages must fail its sequence check."""
        h = Harness()
        h.bring_up()
        fan2 = GateFanout("run_test", FLEET, .05, 3.)
        filtered = [m for m in h.all_outputs
                    if (m.get("data") or {}).get("uav_id") == "uav_1"]
        acts = fan2.deliver(filtered, h.sim, h.wall, 1, lambda u: h.pos[u])
        kinds = [k for entries in acts.values() for k, _ in entries]
        self.assertIn("GATE_ERROR", kinds)

    def test_check_flags_expired_lease(self):
        h = Harness()
        h.bring_up()
        # Jump well past lease_s=3 without renewal: gate must demand a stop.
        result = h.fan.check(h.sim + 10.0, h.wall + 10.0, 1)
        self.assertTrue(any(v is True for v in result.values()))

    def test_stale_grant_refused(self):
        """A grant that does not advance route_version must be refused."""
        roles = {"uav_1": ["VEHICLE_STATE", "COMMAND_ACK"],
                 "planner": ["ROUTE_OFFER"], "clock": ["TICK"]}
        bus = CoreBus("run_stale", ["uav_1"],
                      {"task1": dict(target_id="target1", xyz=[3., 0., 2.])},
                      LIMITS, roles, "stale-test")
        fan = GateFanout("run_stale", ["uav_1"], .05, 3.)
        pos, clock = {"uav_1": [0., 0., 2.]}, {"sim": 0.}

        def send(source, kind, data, dt=0.0):
            clock["sim"] += dt
            outs = bus.submit(source, kind, data, clock["sim"], clock["sim"])
            fan.deliver(outs, clock["sim"], clock["sim"], 1, lambda u: pos[u])
            return outs

        send("uav_1", "VEHICLE_STATE", dict(uav_id="uav_1", frame_id="world_enu", xyz=pos["uav_1"],
             velocity_xyz=[0., 0., 0.], health="OK", mode="IDLE", armed=True,
             active_command_id=None, observed_epoch=0, route_version=0, map_revision=1))
        send("planner", "ROUTE_OFFER", dict(offer_id="o1", uav_id="uav_1", task_id="task1",
             frame_id="world_enu", points=[pos["uav_1"], [3., 0., 2.]], map_revision=1,
             valid_until_sim_s=100., clearance_m=1., tracking_bound_m=.1, static_safe=True,
             grid_safe=True, waiting_points=[]))
        outs = send("clock", "TICK", {}, dt=1.0)
        grants = [m["data"] for m in outs if m["kind"] == "ROUTE_GRANT"]
        self.assertEqual(len(grants), 1)
        stale = dict(schema_version=1, run_id="run_stale", source_id="coordinator_commands",
                     seq=max(int(m["seq"]) for m in outs) + 1, sim_s=clock["sim"],
                     kind="ROUTE_GRANT", data=dict(grants[0], route_version=1, epoch=1))
        acts = fan.deliver([stale], clock["sim"], clock["sim"], 1, lambda u: pos[u])
        msgs = [a for entries in acts.values() for k, a in entries if k == "GATE_ERROR"]
        self.assertTrue(any("NOT_AUTHORIZED" in m or "OLD_EPOCH" in m for m in msgs),
                        "stale grant was not refused: %s" % msgs)


class PhysicalMonitorTests(unittest.TestCase):
    def test_report_keys_match_interface(self):
        m = PhysicalMonitor(FLEET, 1., .15, 1.1)
        m.sample(0., {"uav_1": [0., 0., 2.], "uav_2": [0., 4., 2.]})
        self.assertEqual(set(m.report()), {
            "coverage_start_sim_s", "coverage_end_sim_s", "max_gap_s", "collision_count",
            "min_separation_m", "max_arrival_error_m", "max_no_progress_s"})

    def test_detects_separation_and_collision(self):
        m = PhysicalMonitor(FLEET, 1., .15, 1.1, collision_separation_m=0.2)
        m.sample(0., {"uav_1": [0., 0., 2.], "uav_2": [0., 4., 2.]})
        m.sample(1., {"uav_1": [0., 0., 2.], "uav_2": [0., 0.1, 2.]})
        r = m.report()
        self.assertLess(r["min_separation_m"], 1.0)
        self.assertGreaterEqual(r["collision_count"], 1)

    def test_gap_beyond_budget_fails_coverage(self):
        m = PhysicalMonitor(FLEET, 1., .15, 1.1)
        m.sample(0., {"uav_1": [0., 0., 2.], "uav_2": [0., 4., 2.]})
        m.sample(5., {"uav_1": [0., 0., 2.], "uav_2": [0., 4., 2.]})  # 5s gap > 1.1s
        self.assertGreater(m.report()["max_gap_s"], 1.1)
        self.assertFalse(m.coverage_ok())

    def test_arrival_error_and_no_progress(self):
        m = PhysicalMonitor(FLEET, 1., .15, 5.)
        m.sample(0., {"uav_1": [0., 0., 2.], "uav_2": [0., 4., 2.]},
                 task_targets={"uav_1": [3., 0., 2.]}, progressed=True)
        m.sample(2., {"uav_1": [0., 0., 2.], "uav_2": [0., 4., 2.]},
                 task_targets={"uav_1": [3., 0., 2.]}, progressed=False)
        m.sample(9., {"uav_1": [0., 0., 2.], "uav_2": [0., 4., 2.]},
                 task_targets={"uav_1": [3., 0., 2.]}, progressed=False)
        r = m.report()
        self.assertAlmostEqual(r["max_arrival_error_m"], 3.0)
        self.assertGreaterEqual(r["max_no_progress_s"], 9.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
