"""Execute the actual Navigator methods without importing ROS packages."""
import ast
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/robocup_navigation/src'))
from robocup_navigation.astar import GridMap
from robocup_navigation.local_grid import LocalOccupancyGrid, OCCUPIED
from robocup_navigation.route_runtime import RouteError

source = ROOT / 'src/robocup_navigation/scripts/uav_navigation_node.py'
tree = ast.parse(source.read_text(encoding='utf-8'))
node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Navigator')
class FlightError(Exception):
    pass
scope = dict(math=math, OCCUPIED=OCCUPIED, FlightError=FlightError, RouteError=RouteError)
exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), 'exec'), scope)
Navigator = scope['Navigator']


class NavigationRegressions(unittest.TestCase):
    def make_nav(self):
        n = Navigator.__new__(Navigator)
        n.local_grid = LocalOccupancyGrid(side_m=12)
        n.c = dict(perception_lookahead_m=5, perception_corridor_half_m=.9,
                   perception_blocked_min_samples=3, perception_evade_max_hold_s=25,
                   perception_replan_retry_s=12., perception_replan_retry_max=5,
                   perception_replan_retry_gap_s=2.)
        n.event = lambda *a, **kw: None
        return n

    def test_straight_corridor_keeps_endpoint(self):
        n = self.make_nav()
        for x in (1.2, 1.6, 2.0, 2.4, 2.8):
            n.local_grid.mark(x, 0, OCCUPIED)
        self.assertGreater(n._corridor_blocked_samples((0, 0), [(0, 0), (4, 0)]), 0)

    def test_bend_is_not_replaced_by_diagonal(self):
        n = self.make_nav()
        for y in (1.2, 1.6, 2.0):
            n.local_grid.mark(0, y, OCCUPIED)
        self.assertGreater(n._corridor_blocked_samples((0, 0), [(0, 0), (0, 3), (4, 3)]), 0)

    def test_full_segment_rejects_middle_and_corner_contact(self):
        cells = [0] * 25
        cells[12] = 1
        grid = GridMap(5, 5, 1., (0., 0.), cells, 'map')
        self.assertFalse(Navigator._grid_segment_free(grid, (.5, 2.5), (4.5, 2.5)))
        self.assertFalse(Navigator._grid_segment_free(grid, (.5, .5), (2., 2.)))
        self.assertTrue(Navigator._grid_segment_free(grid, (.5, .5), (4.5, .5)))
        self.assertFalse(Navigator._grid_segment_free(grid, (-1., .5), (4., .5)))

    def test_failure_never_resumes_blocked_route(self):
        n = self.make_nav()
        n._resume_after_replan = lambda sim: self.fail('resumed unsafe route')
        with self.assertRaisesRegex(FlightError, 'PERCEPTION_REPLAN_UNRESOLVABLE'):
            n._perception_replan_failed(10., 'NO_PATH')

    def test_evade_resumes_saved_goal(self):
        n = self.make_nav()
        n.lock = threading.RLock()
        n.samples = {'models': ({'iris': ((0., 0., 2.), 0.)}, 0.)}
        n._evade = {'final': (7., 4.), 'lateral': (0., 1.)}
        n._evade_entered_sim = 1.
        n._nearest_ahead_block = lambda world: None
        calls = []
        n.trigger_perception_replan = lambda sim, blocked, goal: calls.append(goal)
        n._evade_hold_tick(2.)
        self.assertEqual(calls, [(7., 4.)])
        self.assertIsNone(n._evade)

    def test_evade_hop_emits_goal_and_metrics(self):
        """Exercise the real evade path — it must not reference names that
        only exist in the caller (a NameError here only surfaced in flight)."""
        n = self.make_nav()
        n.c['perception_evade_lateral_m'] = 1.5
        n.c['max_radius_m'] = 20.
        n.c['max_altitude_m'] = 6.
        n.lock = threading.RLock()
        n.samples = {'models': ({'iris': ((0., 0., 2.), 0.)}, 0.)}
        n.route_world_current = [(0., 0., 2.), (7., 4., 2.)]
        n.mission_goal_world = (7., 4.)
        n.local_grid.mark(2.0, 0.0, OCCUPIED)
        n.grid_source = SimpleNamespace(
            snapshot=lambda: (n.local_grid.to_grid_map(unknown_is_obstacle=False), 0))
        n.route_plan = SimpleNamespace(
            route_world=[(0., 0., 2.), (7., 4., 2.)],
            start_world=(0., 0., 2.),
            safety=SimpleNamespace(validate_route=lambda *a, **kw: None))
        n._corridor_blocked_samples = lambda world, route: 0
        n.c['goal_hover_time_s'] = 1.
        n.transform = SimpleNamespace(world_to_local=lambda p: p)
        n.path_world_pub = n.path_local_pub = None
        n._publish_path = lambda *args: None
        n._replan_seq = 0
        n.route_start_local = [0., 0., 2.]
        n.route_local = [(0., 0., 2.), (7., 4., 2.)]
        n.active = {'name': 'prev'}
        events = []
        n.event = lambda name, **kw: events.append((name, kw))
        n.next_waypoint = lambda sim: None
        n._start_intruder_evade(10., 'closing', 3.0, 0.1, (2., 0.))
        fired = [e for e in events if e[0] == 'INTRUDER_EVADE_LATERAL']
        self.assertEqual(len(fired), 1)
        payload = fired[0][1]
        self.assertEqual(payload['final_xy'], [7., 4.])
        self.assertIsNotNone(payload.get('blob_cells'))
        self.assertIsNotNone(n._evade)

    def test_transient_failure_retries_toward_mission_goal(self):
        """A transient replan failure (goal cell still shadowed by a departing
        vehicle) must be retried against the MISSION goal, not resumed.  The
        bounded-retry state must persist across fires so the attempt counter
        accumulates — a prior bug nulled it on every fire, defeating
        max_retries and spinning an unbounded replan storm."""
        n = self.make_nav()
        n._resume_after_replan = lambda sim: self.fail('resumed unsafe route')
        n.mission_goal_world = (7., 4.)
        n._evade = None
        n._perception_retry = None
        n._perception_replan_retry_or_fail(10., 'REPLAN_GOAL_OCCUPIED')
        # no raise: deferred into the bounded retry budget
        self.assertEqual(n._perception_retry['goal'], (7., 4.))
        self.assertEqual(n._perception_retry['count'], 1)
        # a goal-occupied failure arms the transient goal-cell release
        self.assertTrue(n._retry_goal_cell_clear)
        fired = []
        n.replan_worker = None
        n.replan_request = None
        n.trigger_perception_replan = lambda sim, blocked, goal: fired.append(goal)
        n._fire_pending_perception_retry(11.)          # before the gap: quiet
        self.assertEqual(fired, [])
        n._fire_pending_perception_retry(12.5)         # after the gap: refire
        self.assertEqual(fired, [(7., 4.)])
        # state persists across the fire so the bound actually accumulates
        self.assertIsNotNone(n._perception_retry)
        self.assertEqual(n._perception_retry['count'], 1)
        self.assertEqual(n._perception_retry['goal'], (7., 4.))
        self.assertGreater(n._perception_retry['next_sim'], 12.5)
        # a second failure increments the counter instead of resetting it
        n._perception_replan_retry_or_fail(13., 'REPLAN_GOAL_OCCUPIED')
        self.assertEqual(n._perception_retry['count'], 2)

    def test_replan_storm_is_bounded_across_fires(self):
        """Regression: an unresolved goal-occupied replan must not spin an
        unbounded retry storm.  A prior bug reset the attempt counter on every
        fire, so the max_retries bound never engaged.  Repeated fail->fire
        cycles must still hit max_retries and fail closed."""
        n = self.make_nav()
        n._resume_after_replan = lambda sim: self.fail('resumed unsafe route')
        n.mission_goal_world = (7., 4.)
        n._evade = None
        n._perception_retry = None
        n.c['perception_replan_retry_max'] = 3
        n.replan_worker = None
        n.replan_request = None
        n.trigger_perception_replan = lambda sim, blocked, goal: None
        sim = 10.0
        with self.assertRaisesRegex(FlightError, 'PERCEPTION_REPLAN_UNRESOLVABLE'):
            for _ in range(6):  # more cycles than max_retries
                n._perception_replan_retry_or_fail(sim, 'REPLAN_GOAL_OCCUPIED')
                n._fire_pending_perception_retry(sim + 2.5)
                sim += 2.5
        self.assertIsNone(n._perception_retry)

    def test_retry_budget_exhausted_fails_closed(self):
        n = self.make_nav()
        n._resume_after_replan = lambda sim: self.fail('resumed unsafe route')
        n.mission_goal_world = (7., 4.)
        n._evade = None
        n._perception_retry = None
        n.c['perception_replan_retry_max'] = 2
        for _ in range(2):
            n._perception_replan_retry_or_fail(10., 'REPLAN_GOAL_OCCUPIED')
        with self.assertRaisesRegex(FlightError, 'PERCEPTION_REPLAN_UNRESOLVABLE'):
            n._perception_replan_retry_or_fail(11., 'REPLAN_GOAL_OCCUPIED')
        self.assertIsNone(n._perception_retry)

    def test_no_goal_never_retries(self):
        """Without an authoritative goal there is nothing safe to retry."""
        n = self.make_nav()
        n._resume_after_replan = lambda sim: self.fail('resumed unsafe route')
        n.mission_goal_world = None
        n._evade = None
        n._perception_retry = None
        with self.assertRaisesRegex(FlightError, 'PERCEPTION_REPLAN_UNRESOLVABLE'):
            n._perception_replan_retry_or_fail(10., 'NO_PATH')

    def test_external_goal_becomes_perception_resume_goal(self):
        n = self.make_nav()
        n.c['goal_hover_time_s'] = 1.
        n.mission_goal_world = (3., 0.)
        n.route_plan = SimpleNamespace(route_world=[(3., 0., 2.)])
        n.transform = SimpleNamespace(world_to_local=lambda p: p)
        n.replan_resume = None
        n._evade = None
        n.path_world_pub = n.path_local_pub = None
        n._publish_path = lambda *args: None
        n.snapshot = lambda: {}
        n.next_waypoint = lambda sim: None
        new_route = ((7., 4., 2.),)
        n._apply_replan((42, (7., 4.)), (new_route, 0), 1.)
        self.assertEqual(n.mission_goal_world, (7., 4.))
        # Startup RoutePlan still ends at the old goal. Perception must use G2.
        n._apply_replan(('perception', 1, 3), (new_route, 0), 2.)
        self.assertEqual(tuple(n.queue[-1]['world_position'][:2]), (7., 4.))


if __name__ == '__main__':
    unittest.main()
