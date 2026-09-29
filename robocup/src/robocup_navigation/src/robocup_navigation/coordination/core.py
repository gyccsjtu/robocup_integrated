"""Single-writer fleet coordinator: assignment, fencing and spatial reservations.

Routes are reserved in their entirety until verified clear. This deliberately
does not extrapolate a future release from an arrival estimate. Transport,
map construction, ROS and physical safety monitors belong to adapters.
"""
import copy
import hashlib
import json
import math

from .geometry import matching, route_distance, route_length
from .protocol import CoordinationError, number, string, validate, xyz


DEFAULT_KEYS = {
    'min_separation_m', 'arrival_tolerance_m', 'mission_timeout_s',
    'deadlock_timeout_s', 'max_monitor_gap_s', 'state_timeout_s',
    'lease_s', 'stop_speed_mps', 'required_clearance_m',
    'tracking_bound_m', 'position_tolerance_m', 'nominal_speed_mps',
}


class Coordinator:
    """Pure deterministic core. Call receive(message, received_wall_s) serially.

    fleet: logical UAV ids, at most six.
    tasks: {task_id: {'target_id': str, 'xyz': [x,y,z]}}.
    roles: {source_id: [permitted input kinds]}; vehicle reports and ACKs
    additionally require source_id == uav_id. Source authentication is external.
    Inputs, outputs and public snapshots are copied to prevent alias mutation.
    """

    def __init__(self, run_id, fleet, tasks, limits, roles, test_case_id,
                 start_sim_s=0., start_wall_s=0.):
        if not string(run_id) or not string(test_case_id) or not number(start_sim_s) or not number(start_wall_s):
            raise CoordinationError('CONFIG_RUN')
        if not isinstance(fleet, (list, tuple)) or not 1 <= len(fleet) <= 6 or len(set(fleet)) != len(fleet):
            raise CoordinationError('CONFIG_FLEET')
        if any(u not in ['uav_%d' % i for i in range(1, 7)] for u in fleet):
            raise CoordinationError('CONFIG_UAV_ID')
        if set(limits) != DEFAULT_KEYS or any(not number(v) or v <= 0 for v in limits.values()):
            raise CoordinationError('CONFIG_LIMITS')
        if limits['position_tolerance_m'] > limits['tracking_bound_m']:
            raise CoordinationError('CONFIG_START_OUTSIDE_TRACKING_BOUND')
        if not tasks or any(not string(k) or not isinstance(v, dict) or
                            set(v) != {'target_id', 'xyz'} or not string(v['target_id']) or
                            not xyz(v['xyz']) for k, v in tasks.items()):
            raise CoordinationError('CONFIG_TASKS')
        if len({v['target_id'] for v in tasks.values()}) != len(tasks):
            raise CoordinationError('CONFIG_DUPLICATE_TARGET')
        if not isinstance(roles, dict) or any(not string(k) or not isinstance(v, (list, tuple))
                                             for k, v in roles.items()):
            raise CoordinationError('CONFIG_ROLES')
        self.run_id, self.fleet = run_id, tuple(sorted(fleet))
        self.limits, self.roles = copy.deepcopy(limits), copy.deepcopy(roles)
        self.tasks = {k: dict(copy.deepcopy(v), status='PENDING', epoch=0, owner=None,
                              stopped=False, stop_seq=None, lease=0., version=0,
                              reservation=None, command=None, complete=False, cancel=False)
                      for k, v in tasks.items()}
        self.vehicles, self.offers, self.reservations, self.commands = {}, {}, {}, {}
        self.retired = set()
        self.sequences, self.observations = {}, {}
        self.now, self.wall, self.started = start_sim_s, start_wall_s, start_sim_s
        self.progress = start_sim_s
        self.halted = self.finished = False
        self.failures, self.outbox, self.events = [], [], []
        self.output_seq = self.event_seq = self.counter = 0
        digest = hashlib.sha256(json.dumps(dict(fleet=fleet, tasks=tasks, limits=limits, roles=roles),
                                           sort_keys=True).encode()).hexdigest()
        self._event('RUN_STARTED', details=dict(fleet_ids=list(self.fleet),
                    required_task_ids=sorted(tasks), test_case_id=test_case_id,
                    config_digest=digest, limits=copy.deepcopy(limits)))

    def _message(self, source, seq, kind, data):
        return dict(schema_version=1, run_id=self.run_id, source_id=source,
                    seq=seq, sim_s=self.now, kind=kind, data=copy.deepcopy(data))

    def _emit(self, kind, **data):
        self.output_seq += 1
        msg = self._message('coordinator_commands', self.output_seq, kind, data)
        self.outbox.append(msg)
        return msg

    def _event(self, event, uav=None, task=None, reason='OK', details=None):
        self.event_seq += 1
        self.events.append(self._message('coordinator', self.event_seq, 'EVENT',
                           dict(event=event, uav_id=uav, task_id=task,
                                reason=reason, details=details or {})))

    def drain_outputs(self):
        result, self.outbox = copy.deepcopy(self.outbox), []
        return result

    def drain_events(self):
        result, self.events = copy.deepcopy(self.events), []
        return result

    def snapshot(self):
        return copy.deepcopy(dict(tasks=self.tasks, reservations=self.reservations,
                                  halted=self.halted, failures=self.failures))

    def _fresh(self, u):
        s = self.vehicles.get(u)
        return s is not None and self.wall - s['wall'] <= self.limits['state_timeout_s'] and \
            0 <= self.now - s['sim'] <= self.limits['state_timeout_s']

    def _stopped(self, u):
        return self._fresh(u) and math.dist(self.vehicles[u]['velocity_xyz'], (0, 0, 0)) <= self.limits['stop_speed_mps']

    def _lock(self, tid, state, reason):
        t = self.tasks[tid]
        data = dict(task_id=tid, target_id=t['target_id'], owner_uav_id=t['owner'],
                    epoch=t['epoch'], state=state, lease_until_sim_s=t['lease'], reason=reason)
        self._emit('TARGET_LOCK', **data)
        self._event('LOCK_CHANGED', t['owner'], tid, reason, data)

    def _reservation(self, rid, state):
        r = self.reservations[rid]
        r['state'] = state
        data = {k: r[k] for k in ('reservation_id', 'uav_id', 'task_id', 'epoch', 'route_version',
                                 'resource_id', 'enter_after_sim_s', 'expected_exit_sim_s')}
        data['state'] = state
        self._emit('RESERVATION', **data)
        self._event('RESERVATION_CHANGED', r['uav_id'], r['task_id'], details=data)

    def _revoke(self, tid, reason, quarantine=False):
        t = self.tasks[tid]
        if t['owner'] is None or t['status'] in ('COMPLETED', 'CANCELLED'):
            return
        if t['status'] in ('REVOKING', 'QUARANTINED'):
            return
        t['status'], t['stopped'], t['stop_seq'] = ('QUARANTINED' if quarantine else 'REVOKING'), False, None
        self._lock(tid, t['status'], reason)
        if t['reservation']:
            self._reservation(t['reservation'], 'QUARANTINED')
        self.counter += 1
        cid = 'stop-%d' % self.counter
        self.commands[cid] = dict(task=tid, kind='CANCEL_REQUEST', epoch=t['epoch'],
                                  version=t['version'], uav=t['owner'], ack=None)
        t['command'] = cid
        self._emit('CANCEL_REQUEST', command_id=cid, uav_id=t['owner'], task_id=tid,
                   epoch=t['epoch'], route_version=t['version'], reason=reason)

    def _violation(self, reason):
        if reason not in self.failures:
            self.failures.append(reason)
            self._event('SAFETY_VIOLATION', reason=reason)
        self.halted = True
        for tid in self.tasks:
            self._revoke(tid, reason, quarantine=True)

    def receive(self, message, received_wall_s):
        """Malformed/unauthorized/out-of-order input raises without mutation.

        Semantic rejections consume a valid sequence (caller must not retry it
        with changed data). Duplicate delivery has no effects or outputs.
        """
        canonical = validate(message)
        m = copy.deepcopy(message)
        if self.finished:
            raise CoordinationError('RUN_FINISHED')
        if m['run_id'] != self.run_id:
            raise CoordinationError('RUN_MISMATCH')
        src, kind, d = m['source_id'], m['kind'], m['data']
        if kind not in self.roles.get(src, ()):
            raise CoordinationError('SOURCE_NOT_AUTHORIZED')
        if 'uav_id' in d and d['uav_id'] not in self.fleet:
            raise CoordinationError('UNKNOWN_UAV')
        if kind in ('VEHICLE_STATE', 'COMMAND_ACK') and src != d['uav_id']:
            raise CoordinationError('UAV_IDENTITY_MISMATCH')
        prev = self.sequences.get(src)
        if prev and m['seq'] == prev[0]:
            if canonical != prev[1]:
                raise CoordinationError('SEQ_CONTENT_CONFLICT')
            return []
        if m['seq'] != (prev[0] + 1 if prev else 1):
            raise CoordinationError('SEQ_GAP_OR_OLD')
        if not number(received_wall_s) or received_wall_s < self.wall:
            raise CoordinationError('RECEIVE_CLOCK_INVALID')
        if prev and m['sim_s'] < prev[2]:
            # Delayed source stamps are not a global clock rewind. Reject them.
            raise CoordinationError('SOURCE_TIME_REWOUND')
        if kind == 'TICK' and m['sim_s'] < self.now:
            self.halted = True
            for tid in self.tasks:
                self._revoke(tid, 'CLOCK_REWOUND', quarantine=True)
            raise CoordinationError('CLOCK_REWOUND_NEW_RUN_REQUIRED')
        if m['sim_s'] < self.now - self.limits['state_timeout_s']:
            raise CoordinationError('MESSAGE_STALE')
        self.sequences[src] = (m['seq'], canonical, m['sim_s'])
        self.now, self.wall = max(self.now, m['sim_s']), received_wall_s
        offset = len(self.outbox)
        # Expiry precedes renewal: a late heartbeat cannot resurrect an epoch.
        self._watchdogs()
        if kind == 'VEHICLE_STATE':
            self._state(d, m)
        elif kind == 'TARGET_REPORT':
            self._target(d)
        elif kind == 'ROUTE_OFFER':
            if d['task_id'] not in self.tasks:
                raise CoordinationError('UNKNOWN_TASK')
            self.offers[d['uav_id'], d['task_id']] = d
        elif kind == 'COMMAND_ACK':
            self._ack(d)
        elif kind == 'RESOURCE_CLEAR':
            self._clear(d)
        elif kind == 'CANCEL_TASK':
            if d['task_id'] not in self.tasks:
                raise CoordinationError('UNKNOWN_TASK')
            t = self.tasks[d['task_id']]
            t['cancel'] = True
            if t['owner'] is None:
                t['status'] = 'CANCELLED'
            else:
                self._revoke(d['task_id'], d['reason'])
        self._watchdogs()
        if kind == 'TICK' and not self.halted:
            self._schedule()
        return copy.deepcopy(self.outbox[offset:])

    def _state(self, d, m):
        u = d['uav_id']
        old = self.vehicles.get(u)
        self.vehicles[u] = dict(d, wall=self.wall, sim=m['sim_s'], seq=m['seq'])
        active = next(((tid, t) for tid, t in self.tasks.items() if t['owner'] == u and
                       t['status'] not in ('COMPLETED', 'CANCELLED')), None)
        if active:
            tid, t = active
            if d['health'] != 'OK':
                self._revoke(tid, 'VEHICLE_UNHEALTHY', quarantine=True)
            elif d['observed_epoch'] == t['epoch'] and d['route_version'] == t['version'] and \
                    d['active_command_id'] == t['command'] and t['status'] in ('ASSIGNED', 'EXECUTING'):
                t['lease'] = self.now + self.limits['lease_s']
                self._lock(tid, 'HELD', 'RENEWED')
                r = self.reservations[t['reservation']]
                if route_distance(r['points'], [d['xyz'], d['xyz']]) > self.limits['tracking_bound_m']:
                    self._violation('AUTHORIZATION_CONFLICT')
                if d['map_revision'] != r['map_revision']:
                    self._revoke(tid, 'MAP_CHANGED')
            elif d['mode'] == 'EXECUTING' and d['active_command_id'] != t['command']:
                # During revocation, old movement is possible and stays reserved.
                if t['status'] not in ('REVOKING', 'QUARANTINED'):
                    self._violation('STALE_COMMAND_EXECUTED')
            if old and t['status'] == 'EXECUTING':
                # Count monotone distance along the route, not straight-line
                # distance to goal (a valid detour initially moves away).
                points = self.reservations[t['reservation']]['points']
                best, walked = None, 0.
                for a, b in zip(points, points[1:]):
                    delta = [b[i] - a[i] for i in range(3)]
                    length = math.dist(a, b)
                    fraction = max(0., min(1., sum((d['xyz'][i] - a[i]) * delta[i] for i in range(3)) /
                                          (length * length))) if length else 0.
                    q = [a[i] + fraction * delta[i] for i in range(3)]
                    item = (math.dist(d['xyz'], q), walked + fraction * length)
                    best = item if best is None or item < best else best
                    walked += length
                if best[1] - t.get('route_progress', 0.) > self.limits['position_tolerance_m']:
                    t['route_progress'], self.progress = best[1], self.now
        elif d['mode'] == 'EXECUTING':
            self._violation('STALE_COMMAND_EXECUTED')
        for other in self.fleet:
            if other != u and self._fresh(other) and math.dist(d['xyz'], self.vehicles[other]['xyz']) < self.limits['min_separation_m']:
                self._violation('SEPARATION_BREACH')

    def _target(self, d):
        key = d['target_id'], d['observation_id']
        if key in self.observations:
            if self.observations[key] != d:
                raise CoordinationError('OBSERVATION_CONFLICT')
            return
        self.observations[key] = d
        if d['confidence'] < 1.:
            return  # v1 consumes confirmed reports; upstream owns fusion.
        for tid, t in self.tasks.items():
            if t['target_id'] == d['target_id'] and t['status'] != 'COMPLETED':
                moved = math.dist(t['xyz'], d['xyz']) > self.limits['arrival_tolerance_m']
                t['xyz'] = d['xyz']
                if moved and t['owner']:
                    self._revoke(tid, 'TARGET_MOVED')

    def _offer_safe(self, offer):
        u, tid = offer['uav_id'], offer['task_id']
        if not self._stopped(u):
            return False
        s, t, l = self.vehicles[u], self.tasks[tid], self.limits
        return (s['health'] == 'OK' and s['mode'] in ('IDLE', 'HOLDING', 'LANDED') and
                offer['valid_until_sim_s'] > self.now and offer['map_revision'] == s['map_revision'] and
                offer['static_safe'] and offer['grid_safe'] and
                offer['clearance_m'] >= l['required_clearance_m'] and
                offer['tracking_bound_m'] <= l['tracking_bound_m'] and
                math.dist(offer['points'][0], s['xyz']) <= l['position_tolerance_m'] and
                math.dist(offer['points'][-1], t['xyz']) <= l['arrival_tolerance_m'])

    def _conflict(self, points, owner):
        margin = self.limits['min_separation_m'] + 2 * self.limits['tracking_bound_m']
        for r in self.reservations.values():
            if r['state'] != 'RELEASED' and r['uav_id'] != owner and route_distance(points, r['points']) <= margin:
                return True
        for u, s in self.vehicles.items():
            if u != owner and route_distance(points, [s['xyz'], s['xyz']]) <= margin:
                return True
        return False

    def safe_waiting_points(self, offer):
        """Filter trusted planner candidates; never synthesize a wait point."""
        from .protocol import fields, SPECS
        fields(offer, SPECS['ROUTE_OFFER'])
        if offer['uav_id'] not in self.fleet or not self._fresh(offer['uav_id']):
            return []
        current = self.vehicles[offer['uav_id']]
        return copy.deepcopy([p for p in offer['waiting_points']
                              if p['static_safe'] and p['grid_safe'] and p['connector_safe'] and
                              p['outside_bottleneck'] and p['map_revision'] == current['map_revision'] and
                              p['valid_until_sim_s'] > self.now and
                              p['clearance_m'] >= self.limits['required_clearance_m'] and
                              p['tracking_bound_m'] <= self.limits['tracking_bound_m'] and
                              not self._conflict([current['xyz'], p['xyz']], offer['uav_id'])])

    def _schedule(self):
        # An unobserved/lost aircraft could be anywhere: no new motion grants.
        if any(t['status'] in ('REVOKING', 'QUARANTINED') for t in self.tasks.values()):
            return
        if not all(u in self.retired or (self._fresh(u) and self.vehicles[u]['health'] == 'OK') for u in self.fleet):
            return
        busy = {t['owner'] for t in self.tasks.values() if t['owner'] is not None}
        pending = [tid for tid, t in self.tasks.items() if t['status'] == 'PENDING']
        available = [u for u in self.fleet if u not in busy and u not in self.retired]
        costs = {(u, tid): route_length(o['points']) / self.limits['nominal_speed_mps']
                 for (u, tid), o in self.offers.items() if u in available and tid in pending and self._offer_safe(o)}
        for u, tid in matching(available, pending, costs):
            offer = self.offers[u, tid]
            if self._conflict(offer['points'], u):
                continue  # stay in current safe state; retry on a future TICK
            t = self.tasks[tid]
            t.update(owner=u, epoch=t['epoch'] + 1, version=t['version'] + 1,
                     status='ASSIGNED', stopped=False, stop_seq=None, complete=False,
                     lease=self.now + self.limits['lease_s'], route_progress=0.)
            self.counter += 1
            rid, cid = 'route-%d' % self.counter, 'fly-%d' % self.counter
            t['reservation'], t['command'] = rid, cid
            self.reservations[rid] = dict(reservation_id=rid, uav_id=u, task_id=tid,
                epoch=t['epoch'], route_version=t['version'], resource_id=rid,
                enter_after_sim_s=self.now,
                expected_exit_sim_s=self.now + costs[u, tid], points=copy.deepcopy(offer['points']),
                map_revision=offer['map_revision'], valid_until_sim_s=offer['valid_until_sim_s'])
            self.commands[cid] = dict(task=tid, kind='ROUTE_GRANT', epoch=t['epoch'],
                                      version=t['version'], uav=u, ack=None)
            self._lock(tid, 'HELD', 'ASSIGNED')
            self._emit('TASK_ASSIGN', task_id=tid, target_id=t['target_id'], uav_id=u,
                       epoch=t['epoch'], lease_until_sim_s=t['lease'])
            self._event('TASK_ASSIGNED', u, tid)
            self._reservation(rid, 'RESERVED')
            msg = self._emit('ROUTE_GRANT', command_id=cid, task_id=tid, target_id=t['target_id'],
                uav_id=u, epoch=t['epoch'], route_version=t['version'], offer_id=offer['offer_id'],
                map_revision=offer['map_revision'], frame_id='world_enu', points=offer['points'],
                reservation_ids=[rid], valid_until_sim_s=offer['valid_until_sim_s'])
            self._event('ROUTE_GRANTED', u, tid, details=msg['data'])

    def _ack(self, d):
        c = self.commands.get(d['command_id'])
        if not c or (d['uav_id'], d['task_id'], d['epoch'], d['route_version']) != \
                (c['uav'], c['task'], c['epoch'], c['version']):
            raise CoordinationError('ACK_FENCE_MISMATCH')
        t = self.tasks[c['task']]
        if d['command_id'] != t['command']:
            raise CoordinationError('ACK_SUPERSEDED')
        status = d['status']
        if c['ack'] == status:
            return
        transitions = {None: {'ACCEPTED', 'REJECTED'}, 'ACCEPTED': {'STARTED', 'STOPPED', 'REJECTED'},
                       'STARTED': {'STOPPED', 'COMPLETED', 'REJECTED'}}
        if status not in transitions.get(c['ack'], set()):
            raise CoordinationError('ACK_ORDER')
        if status == 'COMPLETED' and c['kind'] != 'ROUTE_GRANT':
            raise CoordinationError('STOP_COMMAND_CANNOT_COMPLETE_TASK')
        if status in ('STOPPED', 'COMPLETED'):
            s = self.vehicles.get(c['uav'])
            if not self._stopped(c['uav']) or s['mode'] not in ('HOLDING', 'LANDED', 'IDLE') or s['observed_epoch'] != c['epoch'] or \
                    s['route_version'] != c['version'] or s['active_command_id'] != d['command_id']:
                raise CoordinationError('ACK_NO_STOP_EVIDENCE')
            if status == 'COMPLETED' and math.dist(s['xyz'], t['xyz']) > self.limits['arrival_tolerance_m']:
                self._violation('ARRIVAL_ERROR')
                return
            t['stopped'], t['stop_seq'] = True, s['seq']
            if status == 'COMPLETED':
                t['complete'], t['status'] = True, 'COMPLETED'
                self.progress = self.now
                self._event('TASK_COMPLETED', c['uav'], c['task'])
            elif t['status'] not in ('REVOKING', 'QUARANTINED'):
                t['status'] = 'REVOKING'
                self._lock(c['task'], 'REVOKING', 'STOPPED')
        c['ack'] = status
        if status == 'STARTED' and c['kind'] == 'ROUTE_GRANT':
            t['status'] = 'EXECUTING'
            self._reservation(t['reservation'], 'OCCUPIED')
        if status == 'REJECTED':
            self._revoke(c['task'], 'COMMAND_REJECTED', quarantine=True)
        self._event('COMMAND_ACKED', c['uav'], c['task'], d['reason'], d)

    def _clear(self, d):
        r = self.reservations.get(d['reservation_id'])
        if not r or (d['uav_id'], d['epoch'], d['route_version']) != \
                (r['uav_id'], r['epoch'], r['route_version']):
            raise CoordinationError('CLEAR_FENCE_MISMATCH')
        if r['state'] == 'RELEASED':
            return
        t, u = self.tasks[r['task_id']], r['uav_id']
        s = self.vehicles.get(u)
        if not t['stopped'] or not self._stopped(u) or s['mode'] not in ('HOLDING', 'LANDED', 'IDLE') or s['seq'] <= t['stop_seq'] or \
                s['observed_epoch'] != r['epoch'] or s['route_version'] != r['route_version'] or \
                d['map_revision'] != s['map_revision'] or \
                math.dist(d['xyz'], s['xyz']) > self.limits['position_tolerance_m']:
            raise CoordinationError('CLEAR_NO_FRESH_EVIDENCE')
        # Must be physically OUTSIDE even if landed: a ground-level reservation
        # cannot be released merely because motors are disarmed.
        if route_distance(r['points'], [s['xyz'], s['xyz']]) <= \
                self.limits['min_separation_m'] + 2 * self.limits['tracking_bound_m']:
            raise CoordinationError('RESOURCE_STILL_OCCUPIED')
        self._reservation(r['reservation_id'], 'RELEASED')
        self._lock(r['task_id'], 'RELEASED', 'VERIFIED_CLEAR')
        if s['health'] != 'OK' and s['mode'] == 'LANDED' and not s['armed']:
            # Explicit stopped + clear + grounded evidence permits task
            # takeover. This aircraft cannot be reactivated in this run.
            self.retired.add(u)
        t.update(owner=None, reservation=None, command=None,
                 status='COMPLETED' if t['complete'] else ('CANCELLED' if t['cancel'] else 'PENDING'))
        self.progress = self.now
        self.offers = {k: v for k, v in self.offers.items() if k[1] != r['task_id']}

    def _watchdogs(self):
        for tid, t in self.tasks.items():
            if t['owner'] and t['status'] in ('ASSIGNED', 'EXECUTING'):
                r = self.reservations[t['reservation']]
                if not self._fresh(t['owner']):
                    self._revoke(tid, 'HEARTBEAT_TIMEOUT', quarantine=True)
                elif self.now >= t['lease']:
                    self._revoke(tid, 'LEASE_EXPIRED', quarantine=True)
                elif self.now >= r['valid_until_sim_s']:
                    self._revoke(tid, 'ROUTE_EXPIRED')
        incomplete = any(t['status'] != 'COMPLETED' for t in self.tasks.values())
        if incomplete and self.now - self.started > self.limits['mission_timeout_s']:
            self._violation('MISSION_TIMEOUT')
        elif incomplete and self.now - self.progress > self.limits['deadlock_timeout_s']:
            self._violation('DEADLOCK')

    def finish(self, monitor_report=None):
        """Close log only after no new grants are desired. Never invent evidence.

        No monitor report => ABSTAIN; caller must supply independent coverage.
        This does not stop a real aircraft: adapter owns shutdown/landing.
        """
        if self.finished:
            raise CoordinationError('RUN_FINISHED')
        from .verdict import evaluate_terminal
        self.halted = True
        landed = all(self._fresh(u) and self.vehicles[u]['mode'] == 'LANDED' and
                     not self.vehicles[u]['armed'] for u in self.fleet)
        details = dict(outcome='ABSTAIN', failures=list(self.failures),
                       completed_task_ids=sorted(k for k, t in self.tasks.items() if t['complete']),
                       all_landed_disarmed=landed, evidence_complete=monitor_report is not None,
                       monitor_report=copy.deepcopy(monitor_report))
        ok, reason = evaluate_terminal(self.limits, self.started, self.now, list(self.tasks),
                                       len(self.fleet), details)
        if any(r['state'] != 'RELEASED' for r in self.reservations.values()) and ok is True:
            ok, reason = None, 'RESERVATIONS_NOT_CLEARED'
            details['evidence_complete'] = False
        details['outcome'] = 'PASS' if ok is True else ('FAIL' if ok is False else 'ABSTAIN')
        self._event('RUN_FINISHED', reason=reason, details=details)
        self.finished = True
        return ok, reason
