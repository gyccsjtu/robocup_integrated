"""Local command fencing. Transport must broadcast the complete output stream.

This gate grants permission only. It never proves a physical stop or emits an
ACK; the single-UAV controller must actually execute and report that action.
"""
import copy
import json

from .protocol import CoordinationError, fields, integer, nullable, number, string, xyz, enum
from .geometry import segment_distance


SPECS = {
    'TASK_ASSIGN': dict(task_id=string, target_id=string, uav_id=string, epoch=integer, lease_until_sim_s=number),
    'ROUTE_GRANT': dict(command_id=string, task_id=string, target_id=string, uav_id=string,
                        epoch=integer, route_version=integer, offer_id=string, map_revision=integer,
                        frame_id=enum('world_enu'), points=lambda x: isinstance(x, list) and len(x) >= 2 and all(xyz(p) for p in x),
                        reservation_ids=lambda x: isinstance(x, list) and bool(x) and all(string(p) for p in x),
                        valid_until_sim_s=number),
    'TARGET_LOCK': dict(task_id=string, target_id=string, owner_uav_id=string, epoch=integer,
                        state=enum('HELD', 'REVOKING', 'QUARANTINED', 'RELEASED'),
                        lease_until_sim_s=number, reason=string),
    'RESERVATION': dict(reservation_id=string, uav_id=string, task_id=string, epoch=integer,
                        route_version=integer, resource_id=string,
                        state=enum('RESERVED', 'OCCUPIED', 'QUARANTINED', 'RELEASED'),
                        enter_after_sim_s=number, expected_exit_sim_s=number),
}
for _kind in ('HOLD_REQUEST', 'LAND_REQUEST', 'CANCEL_REQUEST'):
    SPECS[_kind] = dict(command_id=string, uav_id=string, task_id=nullable, epoch=integer,
                       route_version=integer, reason=string)


class ExecutorGate:
    def __init__(self, run_id, uav_id, start_tolerance_m, heartbeat_timeout_s):
        if not string(run_id) or not string(uav_id) or not number(start_tolerance_m) or \
                start_tolerance_m <= 0 or not number(heartbeat_timeout_s) or heartbeat_timeout_s <= 0:
            raise CoordinationError('GATE_CONFIG')
        self.run_id, self.uav_id = run_id, uav_id
        self.tolerance, self.timeout = start_tolerance_m, heartbeat_timeout_s
        self.seq, self.last, self.sim, self.wall = 0, None, 0., 0.
        self.checked_wall = 0.
        self.epochs, self.versions, self.locks, self.reservations = {}, {}, {}, {}
        self.active = None
        self.stop_required = False
        self.closed_epochs = set()
        self.assignment = None

    def check(self, sim_s, wall_s, map_revision):
        """Call every executor tick. True requires the adapter to stop safely.

        No later lease renewal can revive a locally expired authorization.
        """
        if not number(sim_s) or not number(wall_s) or not integer(map_revision) or sim_s < self.sim or wall_s < self.checked_wall:
            self.stop_required = True
            raise CoordinationError('GATE_CLOCK_OR_MAP')
        if self.active:
            d = self.active
            lease = self.locks.get(d['task_id'])
            if (not lease or lease['state'] != 'HELD' or sim_s >= lease['lease_until_sim_s'] or
                    sim_s >= d['valid_until_sim_s'] or wall_s - self.wall > self.timeout or
                    d['map_revision'] != map_revision):
                self.stop_required = True
                self.closed_epochs.add((d['task_id'], d['epoch']))
        self.sim = sim_s
        self.checked_wall = wall_s
        return self.stop_required

    def receive(self, msg, sim_s, wall_s, map_revision, position):
        """Returns a copied actionable ROUTE_GRANT/stop request or None.

        All output messages, including those for other UAVs, must be delivered
        in sequence. On any protocol error the adapter must fail closed; it
        cannot skip a missing sequence to regain liveness.
        """
        try:
            return self._receive(msg, sim_s, wall_s, map_revision, position)
        except (CoordinationError, TypeError, ValueError, KeyError) as exc:
            self.stop_required = True
            if self.active:
                self.closed_epochs.add((self.active['task_id'], self.active['epoch']))
            raise CoordinationError('GATE_REJECTED:' + str(exc)) from exc

    def _receive(self, msg, sim_s, wall_s, map_revision, position):
        self.check(sim_s, wall_s, map_revision)
        if not xyz(position) or not isinstance(msg, dict) or set(msg) != {
                'schema_version', 'run_id', 'source_id', 'seq', 'sim_s', 'kind', 'data'}:
            raise CoordinationError('GATE_ENVELOPE')
        if type(msg['schema_version']) is not int or msg['schema_version'] != 1 or \
                msg['run_id'] != self.run_id or msg['source_id'] != 'coordinator_commands' or \
                not number(msg['sim_s']) or msg['sim_s'] > sim_s or not integer(msg['seq']):
            raise CoordinationError('GATE_ID_OR_TIME')
        kind = msg['kind']
        if kind not in SPECS:
            raise CoordinationError('GATE_KIND')
        fields(msg['data'], SPECS[kind])
        canonical = json.dumps(msg, sort_keys=True, allow_nan=False)
        if msg['seq'] == self.seq and canonical == self.last:
            return None
        if msg['seq'] != self.seq + 1:
            raise CoordinationError('GATE_SEQUENCE')
        self.seq, self.last, self.wall = msg['seq'], canonical, wall_s
        d = copy.deepcopy(msg['data'])
        if d.get('uav_id', d.get('owner_uav_id')) != self.uav_id:
            return None
        tid, epoch = d['task_id'], d['epoch']
        if epoch < self.epochs.get(tid, 0):
            raise CoordinationError('GATE_OLD_EPOCH')
        if kind == 'TARGET_LOCK':
            self.epochs[tid] = epoch
            self.locks[tid] = d
            if d['state'] != 'HELD':
                self.closed_epochs.add((tid, epoch))
                if self.active and self.active['task_id'] == tid:
                    self.stop_required = True
            return None
        if kind == 'RESERVATION':
            self.reservations[d['reservation_id']] = d
            return None
        if kind == 'TASK_ASSIGN':
            self.assignment = d
            return None
        if kind == 'ROUTE_GRANT':
            lock = self.locks.get(tid)
            if self.active or (tid, epoch) in self.closed_epochs or not lock or lock['state'] != 'HELD' or \
                    lock['epoch'] != epoch or sim_s >= lock['lease_until_sim_s'] or \
                    sim_s >= d['valid_until_sim_s'] or d['map_revision'] != map_revision or \
                    d['route_version'] <= self.versions.get(tid, 0) or \
                    segment_distance(position, position, d['points'][0], d['points'][0]) > self.tolerance:
                raise CoordinationError('GATE_ROUTE_NOT_AUTHORIZED')
            if not self.assignment or any(self.assignment[k] != d[k] for k in ('task_id', 'target_id', 'uav_id', 'epoch')):
                raise CoordinationError('GATE_ASSIGNMENT_MISMATCH')
            for rid in d['reservation_ids']:
                r = self.reservations.get(rid)
                if not r or r['state'] != 'RESERVED' or any(r[k] != d[k] for k in ('uav_id', 'task_id', 'epoch', 'route_version')) or sim_s < r['enter_after_sim_s']:
                    raise CoordinationError('GATE_RESERVATION_MISSING')
            self.active, self.stop_required = d, False
            self.versions[tid] = d['route_version']
            return copy.deepcopy(d)
        # Stop requests do not authorize movement to a new location.
        self.stop_required = True
        self.closed_epochs.add((tid, epoch))
        return d

    def confirm_stopped(self):
        """Adapter calls only after measured stop; resource release is separate."""
        if self.active:
            self.closed_epochs.add((self.active['task_id'], self.active['epoch']))
        self.active = None
