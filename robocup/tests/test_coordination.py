"""Deterministic core acceptance tests; no ROS, NumPy or network required."""
import copy
import json
import math
import random
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/robocup_navigation/src'))
from robocup_navigation.coordination import Coordinator, CoordinationError
from robocup_navigation.coordination.geometry import segment_distance, matching
from robocup_navigation.coordination.verdict import verdict
from robocup_navigation.coordination.executor import ExecutorGate


LIMITS = dict(min_separation_m=1., arrival_tolerance_m=.15, mission_timeout_s=300.,
              deadlock_timeout_s=60., max_monitor_gap_s=.2, state_timeout_s=5.,
              lease_s=3., stop_speed_mps=.05, required_clearance_m=.6,
              tracking_bound_m=.2, position_tolerance_m=.05, nominal_speed_mps=.3)


class Harness:
    def __init__(self, n=2, positions=None, goals=None, limits=None):
        self.fleet = ['uav_%d' % (i + 1) for i in range(n)]
        self.positions = positions or [[0., i * 10., 2.] for i in range(n)]
        goals = goals or [[5., i * 10., 2.] for i in range(n)]
        tasks = {'t%d' % i: dict(target_id='target%d' % i, xyz=goals[i]) for i in range(n)}
        roles = {u: ['VEHICLE_STATE', 'COMMAND_ACK'] for u in self.fleet}
        roles.update(planner=['ROUTE_OFFER'], sensor=['TARGET_REPORT'],
                     verifier=['RESOURCE_CLEAR'], operator=['TICK', 'CANCEL_TASK'])
        self.core = Coordinator('test-run', self.fleet, tasks, limits or LIMITS, roles, 'unit')
        self.seq, self.time, self.wall = {}, 0., 0.
        for i, u in enumerate(self.fleet):
            self.state(u, self.positions[i])

    def msg(self, source, kind, data):
        return dict(schema_version=1, run_id='test-run', source_id=source,
                    seq=self.seq.get(source, 0) + 1, sim_s=self.time, kind=kind, data=data)

    def send(self, source, kind, data):
        m = self.msg(source, kind, data)
        self.seq[source] = m['seq']
        return self.core.receive(m, self.wall)

    def state(self, u, pos, **kw):
        data = dict(uav_id=u, frame_id='world_enu', xyz=list(pos), velocity_xyz=[0., 0., 0.],
                    health='OK', mode='IDLE', armed=True, active_command_id=None,
                    observed_epoch=0, route_version=0, map_revision=1)
        data.update(kw)
        return self.send(u, 'VEHICLE_STATE', data)

    def offer(self, u, tid, points=None, **kw):
        data = dict(offer_id='%s-%s' % (u, tid), uav_id=u, task_id=tid, frame_id='world_enu',
                    points=points or [self.core.vehicles[u]['xyz'], self.core.tasks[tid]['xyz']],
                    map_revision=1, valid_until_sim_s=100., clearance_m=.8,
                    tracking_bound_m=.1, static_safe=True, grid_safe=True, waiting_points=[])
        data.update(kw)
        self.send('planner', 'ROUTE_OFFER', data)
        return data

    def tick(self, sim=None, wall=None):
        if sim is not None:
            self.time = sim
        if wall is not None:
            self.wall = wall
        return self.send('operator', 'TICK', {})

    def grants(self):
        return [m['data'] for m in self.tick() if m['kind'] == 'ROUTE_GRANT']

    def ack(self, g, status):
        return self.send(g['uav_id'], 'COMMAND_ACK', dict(uav_id=g['uav_id'],
            command_id=g['command_id'], task_id=g['task_id'], epoch=g['epoch'],
            route_version=g['route_version'], status=status, reason='TEST'))

    def stopped_state(self, g, pos, **kw):
        return self.state(g['uav_id'], pos, active_command_id=g['command_id'],
                          observed_epoch=g['epoch'], route_version=g['route_version'],
                          mode=kw.pop('mode', 'HOLDING'), **kw)

    def clear(self, g, pos):
        rid = self.core.tasks[g['task_id']]['reservation']
        return self.send('verifier', 'RESOURCE_CLEAR', dict(uav_id=g['uav_id'],
            reservation_id=rid, epoch=g['epoch'], route_version=g['route_version'],
            frame_id='world_enu', xyz=pos, map_revision=1))

    def complete(self, g):
        self.ack(g, 'ACCEPTED')
        self.ack(g, 'STARTED')
        goal = self.core.tasks[g['task_id']]['xyz']
        self.stopped_state(g, goal)
        self.ack(g, 'COMPLETED')
        landed = [goal[0], goal[1], 0.]
        self.stopped_state(g, landed, mode='LANDED', armed=False)
        self.clear(g, landed)


class GeometryTests(unittest.TestCase):
    def test_crossing_and_head_on(self):
        self.assertEqual(segment_distance((0, 0, 0), (2, 0, 0), (1, -1, 0), (1, 1, 0)), 0.)
        self.assertEqual(segment_distance((0, 0, 0), (2, 0, 0), (2, 0, 0), (0, 0, 0)), 0.)

    def test_degenerate_and_vertical(self):
        self.assertAlmostEqual(segment_distance((0, 0, 0), (0, 0, 0), (1, -1, 0), (1, 1, 0)), 1.)
        self.assertAlmostEqual(segment_distance((0, 0, 0), (2, 0, 0), (1, -1, 2), (1, 1, 2)), 2.)

    def test_global_matching_beats_greedy(self):
        costs = {('a', 'x'): 1., ('a', 'y'): 2., ('b', 'x'): 2., ('b', 'y'): 100.}
        self.assertEqual(set(matching(['a', 'b'], ['x', 'y'], costs)), {('a', 'y'), ('b', 'x')})

    def test_infeasible_is_not_assigned(self):
        self.assertEqual(matching(['a', 'b'], ['x', 'y'], {('a', 'x'): 1}), [('a', 'x')])

    def test_segment_distance_randomized_symmetry_and_sample_bounds(self):
        rng = random.Random(20260910)
        for _ in range(150):
            a, b, c, d = [[rng.uniform(-10, 10) for _ in range(3)] for _ in range(4)]
            distance = segment_distance(a, b, c, d)
            self.assertAlmostEqual(distance, segment_distance(c, d, a, b), places=8)
            self.assertAlmostEqual(distance, segment_distance(b, a, d, c), places=8)
            for i in range(11):
                p = [a[k] + (b[k] - a[k]) * i / 10 for k in range(3)]
                for j in range(11):
                    q = [c[k] + (d[k] - c[k]) * j / 10 for k in range(3)]
                    self.assertLessEqual(distance, math.dist(p, q) + 1e-8)


class CoordinatorTests(unittest.TestCase):
    def test_six_nonintersecting_routes_granted_and_completed(self):
        h = Harness(6)
        for i, u in enumerate(h.fleet):
            h.offer(u, 't%d' % i)
        gs = h.grants()
        self.assertEqual(len(gs), 6)
        self.assertEqual(len({g['target_id'] for g in gs}), 6)
        for g in gs:
            h.complete(g)
        self.assertTrue(all(t['status'] == 'COMPLETED' and t['owner'] is None for t in h.core.tasks.values()))
        self.assertTrue(all(r['state'] == 'RELEASED' for r in h.core.reservations.values()))

    def test_crossing_routes_serialized(self):
        h = Harness(2, [[-3., 0., 2.], [0., -3., 2.]], [[3., 0., 2.], [0., 3., 2.]])
        for i, u in enumerate(h.fleet):
            h.offer(u, 't%d' % i)
        gs = h.grants()
        self.assertEqual(len(gs), 1)
        h.complete(gs[0])
        self.assertEqual(len(h.grants()), 1)

    def test_head_on_swap_waits_instead_of_flying_through_idle_peer(self):
        h = Harness(2, [[-3., 0., 2.], [3., 0., 2.]], [[3., 0., 2.], [-3., 0., 2.]])
        for i, u in enumerate(h.fleet):
            h.offer(u, 't%d' % i)
        self.assertEqual(h.grants(), [])
        h.tick(61., 61.)
        self.assertIn('DEADLOCK', h.core.failures)

    def test_one_target_has_one_owner(self):
        h = Harness()
        for u in h.fleet:
            h.offer(u, 't0')
        self.assertEqual(len(h.grants()), 1)
        self.assertEqual(h.grants(), [])

    def test_unknown_vehicle_blocks_all_grants(self):
        h = Harness()
        h.offer('uav_1', 't0')
        h.core.vehicles.pop('uav_2')
        self.assertEqual(h.grants(), [])

    def test_unsafe_expired_revision_and_clearance_offers_rejected(self):
        for kw in (dict(static_safe=False), dict(grid_safe=False), dict(map_revision=2),
                   dict(valid_until_sim_s=0.), dict(clearance_m=.1), dict(tracking_bound_m=1.)):
            h = Harness(1)
            h.offer('uav_1', 't0', **kw)
            self.assertEqual(h.grants(), [], kw)

    def test_lease_expiry_quarantines_without_release(self):
        h = Harness(1)
        h.offer('uav_1', 't0')
        g = h.grants()[0]
        h.tick(3., 3.)
        t = h.core.tasks['t0']
        self.assertEqual(t['status'], 'QUARANTINED')
        self.assertEqual(h.core.reservations[t['reservation']]['state'], 'QUARANTINED')
        self.assertIsNotNone(t['owner'])
        with self.assertRaisesRegex(CoordinationError, 'ACK_SUPERSEDED'):
            h.ack(g, 'ACCEPTED')

    def test_late_heartbeat_does_not_resurrect_lease(self):
        h = Harness(1)
        h.offer('uav_1', 't0')
        g = h.grants()[0]
        h.time, h.wall = 4., 4.
        h.stopped_state(g, [0., 0., 2.])
        self.assertEqual(h.core.tasks['t0']['status'], 'QUARANTINED')

    def test_pause_uses_local_wall_timeout(self):
        h = Harness(1)
        h.offer('uav_1', 't0')
        h.grants()
        h.tick(0., 6.)
        self.assertEqual(h.core.tasks['t0']['status'], 'QUARANTINED')

    def test_completion_requires_accept_start_and_position(self):
        h = Harness(1)
        h.offer('uav_1', 't0')
        g = h.grants()[0]
        with self.assertRaisesRegex(CoordinationError, 'ACK_ORDER'):
            h.ack(g, 'COMPLETED')
        h.ack(g, 'ACCEPTED')
        h.ack(g, 'STARTED')
        h.stopped_state(g, [0., 0., 2.])
        h.ack(g, 'COMPLETED')
        self.assertIn('ARRIVAL_ERROR', h.core.failures)

    def test_stop_does_not_release_occupied_route(self):
        h = Harness(1)
        h.offer('uav_1', 't0')
        g = h.grants()[0]
        h.ack(g, 'ACCEPTED')
        h.stopped_state(g, [0., 0., 2.])
        h.ack(g, 'STOPPED')
        h.stopped_state(g, [0., 0., 2.])
        with self.assertRaisesRegex(CoordinationError, 'RESOURCE_STILL_OCCUPIED'):
            h.clear(g, [0., 0., 2.])

    def test_map_change_revokes_route(self):
        h = Harness(1)
        h.offer('uav_1', 't0')
        g = h.grants()[0]
        h.stopped_state(g, [0., 0., 2.], map_revision=2)
        self.assertEqual(h.core.tasks['t0']['status'], 'REVOKING')

    def test_failed_vehicle_takeover_only_after_stop_and_clear(self):
        h = Harness(2, [[0., 0., 2.], [0., 10., 2.]])
        h.offer('uav_1', 't0')
        g = h.grants()[0]
        h.stopped_state(g, [0., 0., 2.], health='FAILED')
        cancel = next(m['data'] for m in reversed(h.core.outbox) if m['kind'] == 'CANCEL_REQUEST')
        h.ack(cancel, 'ACCEPTED')
        h.stopped_state(cancel, [0., 0., 2.], health='FAILED')
        h.ack(cancel, 'STOPPED')
        self.assertEqual(h.grants(), [])
        h.stopped_state(cancel, [0., 0., 0.], health='FAILED', mode='LANDED', armed=False)
        h.clear(cancel, [0., 0., 0.])
        h.offer('uav_2', 't0')
        new = h.grants()
        self.assertEqual(len(new), 1)
        self.assertEqual(new[0]['uav_id'], 'uav_2')
        self.assertGreater(new[0]['epoch'], g['epoch'])

    def test_cancel_retains_reservation_and_blocks_reassignment(self):
        h = Harness(1)
        h.offer('uav_1', 't0')
        h.grants()
        h.send('operator', 'CANCEL_TASK', dict(task_id='t0', reason='USER'))
        self.assertEqual(h.core.tasks['t0']['status'], 'REVOKING')
        self.assertEqual(h.grants(), [])

    def test_protocol_duplicate_identity_sequence_and_nonfinite(self):
        h = Harness(1)
        m = h.msg('operator', 'TICK', {})
        h.core.receive(m, 0.)
        before = h.core.snapshot()
        self.assertEqual(h.core.receive(m, 0.), [])
        self.assertEqual(h.core.snapshot(), before)
        wrong = copy.deepcopy(m)
        wrong['sim_s'] = 1.
        with self.assertRaisesRegex(CoordinationError, 'SEQ_CONTENT_CONFLICT'):
            h.core.receive(wrong, 1.)
        for key, value in [('schema_version', True), ('sim_s', math.nan), ('run_id', 'old-run'), ('seq', 8)]:
            bad = copy.deepcopy(m)
            bad[key] = value
            with self.assertRaises(CoordinationError):
                h.core.receive(bad, 1.)
        self.assertEqual(h.core.snapshot(), before)

    def test_wait_candidates_require_current_proof(self):
        h = Harness(1)
        o = h.offer('uav_1', 't0')
        p = dict(point_id='w', xyz=[0., 2., 2.], map_revision=1, valid_until_sim_s=5.,
                 clearance_m=.8, tracking_bound_m=.1, static_safe=True, grid_safe=True,
                 connector_safe=True, outside_bottleneck=True)
        o['waiting_points'] = [p]
        self.assertEqual(len(h.core.safe_waiting_points(o)), 1)
        p['outside_bottleneck'] = False
        self.assertEqual(h.core.safe_waiting_points(o), [])

    def test_outputs_do_not_alias_internal_reservations(self):
        h = Harness(1)
        h.offer('uav_1', 't0')
        g = h.grants()[0]
        g['points'][0][0] = 1000
        self.assertEqual(next(iter(h.core.reservations.values()))['points'][0][0], 0.)


class VerdictTests(unittest.TestCase):
    def log(self):
        h = Harness(6)
        for i, u in enumerate(h.fleet):
            h.offer(u, 't%d' % i)
        for g in h.grants():
            h.complete(g)
        monitor = dict(coverage_start_sim_s=0., coverage_end_sim_s=0., max_gap_s=0.,
                       collision_count=0, min_separation_m=9., max_arrival_error_m=0., max_no_progress_s=0.)
        self.assertEqual(h.core.finish(monitor), (True, 'ACCEPTANCE_PASSED'))
        return h.core.drain_events()

    def run_verdict(self, events, tail=''):
        with tempfile.TemporaryDirectory() as d:
            Path(d, 'coord_events.jsonl').write_text(''.join(json.dumps(e) + '\n' for e in events) + tail, encoding='utf-8')
            return verdict({'run_dir': d})

    def test_complete_six_vehicle_evidence_passes(self):
        self.assertEqual(self.run_verdict(self.log())[0], True)

    def test_missing_and_malformed_evidence_abstains(self):
        self.assertIsNone(verdict({'run_dir': '/nonexistent'})[0])
        events = self.log()
        self.assertIsNone(self.run_verdict(events[:-1])[0])
        self.assertIsNone(self.run_verdict(events, '{broken\n')[0])
        self.assertIsNone(self.run_verdict(events[:3] + events[4:])[0])

    def test_claimed_pass_with_collision_fails(self):
        events = self.log()
        events[-1]['data']['details']['monitor_report']['collision_count'] = 1
        self.assertEqual(self.run_verdict(events), (False, 'COLLISION'))
        self.assertEqual(self.run_verdict(events, '{broken\n'), (False, 'COLLISION'))

    def test_missing_monitor_and_nan_abstain(self):
        events = self.log()
        events[-1]['data']['details']['monitor_report'] = None
        self.assertIsNone(self.run_verdict(events)[0])
        events = self.log()
        events[-1]['data']['details']['monitor_report']['max_gap_s'] = math.nan
        self.assertIsNone(self.run_verdict(events)[0])

    def test_duplicate_identical_is_idempotent(self):
        events = self.log()
        self.assertEqual(self.run_verdict(events[:2] + [events[1]] + events[2:])[0], True)

    def test_missing_task_events_abstain(self):
        events = self.log()
        for e in events:
            if e['data']['event'] == 'TASK_COMPLETED':
                e['data']['event'] = 'COMMAND_ACKED'
        self.assertIsNone(self.run_verdict(events)[0])

    def test_missing_clearance_cannot_pass(self):
        events = self.log()
        for e in events:
            if e['data']['event'] == 'RESERVATION_CHANGED' and e['data']['details']['state'] == 'RELEASED':
                e['data']['details']['state'] = 'OCCUPIED'
        self.assertIsNone(self.run_verdict(events)[0])


class ExecutorGateTests(unittest.TestCase):
    def prepared(self):
        h = Harness(1)
        h.offer('uav_1', 't0')
        h.grants()
        gate = ExecutorGate('test-run', 'uav_1', .05, 5.)
        outputs = h.core.drain_outputs()
        for m in outputs:
            gate.receive(m, 0., 0., 1, [0., 0., 2.])
        return h, gate, outputs

    def test_complete_authorization_chain(self):
        h, gate, outputs = self.prepared()
        self.assertIsNotNone(gate.active)
        self.assertIsNone(gate.receive(outputs[-1], 0., 0., 1, [0., 0., 2.]))
        self.assertFalse(gate.stop_required)

    def test_lease_expires_even_if_transport_alive(self):
        _, gate, _ = self.prepared()
        self.assertTrue(gate.check(3., 1., 1))
        self.assertIn(('t0', 1), gate.closed_epochs)

    def test_local_map_change_stops_before_next_core_roundtrip(self):
        _, gate, _ = self.prepared()
        self.assertTrue(gate.check(0., 0., 2))

    def test_unknown_run_or_missing_reservation_never_flys(self):
        _, _, outputs = self.prepared()
        gate = ExecutorGate('test-run', 'uav_1', .05, 5.)
        with self.assertRaises(CoordinationError):
            gate.receive(outputs[-1], 0., 0., 1, [0., 0., 2.])
        self.assertIsNone(gate.active)
        bad = copy.deepcopy(outputs[0])
        bad['run_id'] = 'previous-run'
        with self.assertRaises(CoordinationError):
            gate.receive(bad, 0., 0., 1, [0., 0., 2.])

    def test_out_of_order_revokes_existing_authorization(self):
        _, gate, outputs = self.prepared()
        with self.assertRaises(CoordinationError):
            gate.receive(outputs[0], 0., 0., 1, [0., 0., 2.])
        self.assertTrue(gate.stop_required)


if __name__ == '__main__':
    unittest.main(verbosity=2)
