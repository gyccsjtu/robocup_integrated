"""Host tests for the clarified adapter components (map_revision / waiting
points / sustained stop / position cache).  No ROS.

Run:  PYTHONPATH=src/robocup_navigation/src python tests/test_coordination_adapter_components.py
"""
import sys
import threading
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/robocup_navigation/src"))

from robocup_navigation.coordination_adapter import (  # noqa: E402
    FREE, OCCUPIED, UNKNOWN, MapRevisionProvider, OccupancyView, PositionCache,
    SustainedStop, WaitingPointProvider,
)

RES = 0.25
K_Z = 8                      # z = 2.0 m


def corridor(extra=None):
    """FREE corridor along x in [-3, 3] at y=0, z=2, plus optional overrides."""
    cells = {}
    for i in range(-12, 13):
        cells[(i, 0, K_Z)] = FREE
    cells.update(extra or {})
    return cells


def view(cells):
    return OccupancyView(cells, RES, (0.0, 0.0, 0.0))


class MapRevisionTests(unittest.TestCase):
    def test_content_change_bumps_same_content_does_not(self):
        m = MapRevisionProvider()
        rev, changed = m.apply(corridor())
        self.assertEqual(changed, len(corridor()))
        self.assertGreaterEqual(rev, 1)
        rev2, changed2 = m.apply(corridor())        # identical rewrite
        self.assertEqual(changed2, 0)
        self.assertEqual(rev2, rev)
        rev3, _ = m.apply({(-12, 0, K_Z): UNKNOWN})  # FREE -> UNKNOWN
        self.assertEqual(rev3, rev + 1)

    def test_clear_and_recenter_and_snapshot_atomic(self):
        m = MapRevisionProvider(corridor())
        r0 = m.revision
        self.assertEqual(m.recenter(), r0 + 1)
        cells, rev = m.snapshot()
        self.assertEqual(len(cells), len(corridor()))
        self.assertEqual(rev, r0 + 1)
        self.assertEqual(m.clear(), r0 + 2)
        self.assertEqual(m.clear(), r0 + 2)         # already empty -> no bump
        self.assertEqual(m.snapshot(), ({}, r0 + 2))

    def test_concurrent_apply_is_consistent(self):
        m = MapRevisionProvider()
        def worker(base):
            for i in range(200):
                m.apply({(base + i, 1, 1): OCCUPIED})
        threads = [threading.Thread(target=worker, args=(b,)) for b in (1000, 5000, 9000)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        cells, rev = m.snapshot()
        self.assertEqual(len(cells), 600)            # no lost update
        self.assertEqual(rev, 600)                   # one bump per real change


class WaitingPointTests(unittest.TestCase):
    def setUp(self):
        self.p = WaitingPointProvider(vehicle_radius_m=0.4, required_clearance_m=0.6,
                                      max_tracking_bound_m=0.2, sample_step_m=0.25)
        self.kw = dict(bottleneck_boxes=[], tracking_bound_m=0.15, sim_s=10.0,
                       valid_until_sim_s=100.0)

    def test_valid_candidate_is_fully_proved(self):
        out = self.p.candidates(view=view(corridor()), revision=3, current_xyz=[-3.0, 0.0, 2.0],
                                proposed_xyz_list=[[-1.0, 0.0, 2.0]], **self.kw)
        self.assertEqual(len(out), 1)
        c = out[0]
        self.assertTrue(c["connector_safe"] and c["static_safe"] and c["grid_safe"])
        self.assertTrue(c["outside_bottleneck"])
        self.assertGreaterEqual(c["clearance_m"], 0.6)
        self.assertEqual(c["map_revision"], 3)
        self.assertGreater(c["valid_until_sim_s"], 10.0)

    def test_unknown_or_occupied_on_connector_refuses(self):
        for bad in (UNKNOWN, OCCUPIED):
            cells = corridor({(-6, 0, K_Z): bad})
            out = self.p.candidates(view=view(cells), revision=1, current_xyz=[-3.0, 0.0, 2.0],
                                    proposed_xyz_list=[[-1.0, 0.0, 2.0]], **self.kw)
            self.assertEqual(out, [], "state %r must refuse the candidate" % bad)

    def test_missing_bounds_or_geometry_refuses(self):
        base = dict(view=view(corridor()), revision=1, current_xyz=[-3.0, 0.0, 2.0],
                    proposed_xyz_list=[[-1.0, 0.0, 2.0]])
        cases = [dict(self.kw, tracking_bound_m=None),
                 dict(self.kw, tracking_bound_m=0.9),          # over budget
                 dict(self.kw, bottleneck_boxes=None),         # no channel geometry
                 dict(self.kw, valid_until_sim_s=10.0)]        # not strictly later
        for kw in cases:
            self.assertEqual(self.p.candidates(**base, **kw), [], "must refuse: %r" % kw)

    def test_candidate_inside_bottleneck_is_refused(self):
        box = dict(min=[-1.5, -0.5, 1.5], max=[-0.5, 0.5, 2.5])
        out = self.p.candidates(view=view(corridor()), revision=1, current_xyz=[-3.0, 0.0, 2.0],
                                proposed_xyz_list=[[-1.0, 0.0, 2.0]], bottleneck_boxes=[box],
                                tracking_bound_m=0.15, sim_s=10.0, valid_until_sim_s=100.0)
        self.assertEqual(out, [])


class SustainedStopTests(unittest.TestCase):
    def setUp(self):
        self.s = SustainedStop(stop_speed_mps=0.05, settle_time_s=1.0,
                              position_tolerance_m=0.05, state_timeout_s=0.5)
        self.ident = dict(active_command_id="c1", observed_epoch=2, route_version=4,
                          command_id="c1", epoch=2, expected_route_version=4)

    def feed(self, sim, wall, pos=(0.0, 0.0, 2.0), vel=(0.0, 0.0, 0.0), mode="HOLDING"):
        return self.s.update("uav_1", sim_s=sim, wall_s=wall, position=list(pos),
                             velocity=list(vel), mode=mode, **self.ident)

    def test_single_frame_is_not_enough(self):
        self.assertFalse(self.feed(0.0, 0.0))
        self.assertFalse(self.feed(0.5, 0.5))      # only 0.5 s of settle
        self.assertTrue(self.feed(1.0, 1.0))       # 1.0 s reached

    def test_motion_resets_and_sim_pause_does_not_accrue(self):
        self.assertFalse(self.feed(0.0, 0.0))
        self.assertFalse(self.feed(0.5, 0.5, vel=(0.3, 0.0, 0.0)))   # moving -> reset
        self.assertFalse(self.feed(0.5, 1.0))                          # wall moved, sim frozen
        self.assertFalse(self.feed(0.5, 1.5))
        self.assertFalse(self.feed(1.0, 2.0))                          # 0.5 s sim only
        self.assertTrue(self.feed(1.5, 2.5))

    def test_drift_restarts_window(self):
        self.assertFalse(self.feed(0.0, 0.0))
        self.assertFalse(self.feed(0.5, 0.5))
        self.assertFalse(self.feed(1.0, 1.0, pos=(0.2, 0.0, 2.0)))     # drift -> restart (anchor 0.2)
        self.assertFalse(self.feed(1.5, 1.5, pos=(0.2, 0.0, 2.0)))     # 0.5 s of the new window
        self.assertTrue(self.feed(2.0, 2.0, pos=(0.2, 0.0, 2.0)))      # 1.0 s reached

    def test_wrong_command_identity_rejected(self):
        bad = dict(self.ident, command_id="other")
        self.assertFalse(self.s.update("uav_1", sim_s=0.0, wall_s=0.0, position=[0., 0., 2.],
                                       velocity=[0., 0., 0.], mode="HOLDING", **bad))

    def test_arrival_uses_3d_tolerance(self):
        self.assertTrue(self.s.arrival_ok([3.0, 0.0, 2.0], [3.0, 0.0, 2.0], 0.15))
        self.assertFalse(self.s.arrival_ok([3.0, 0.0, 2.0], [3.0, 0.0, 0.0], 0.15))


class PositionCacheTests(unittest.TestCase):
    def test_missing_is_none_never_zero(self):
        c = PositionCache(max_age_s=2.0)
        self.assertIsNone(c.get("uav_9", 0.0, 0.0))

    def test_staleness_flag(self):
        c = PositionCache(max_age_s=2.0)
        c.update("uav_1", [1.0, 2.0, 3.0], sim_s=5.0, wall_s=5.0)
        fresh = c.get("uav_1", 6.0, 6.0)
        self.assertFalse(fresh["stale"])
        old = c.get("uav_1", 9.0, 9.0)
        self.assertTrue(old["stale"])
        self.assertEqual(old["xyz"], [1.0, 2.0, 3.0])   # reported, not extrapolated


if __name__ == "__main__":
    unittest.main(verbosity=2)
