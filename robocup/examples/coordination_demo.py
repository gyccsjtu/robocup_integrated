"""Six-vehicle kinematic example, not PX4/Gazebo flight evidence.

Run: python examples/coordination_demo.py --out <new run directory>
Each vehicle follows a separate three-metre lane then descends outside the
reserved cruise corridor. An independent toy monitor supplies measurements.
"""
import argparse
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/robocup_navigation/src'))
from robocup_navigation.coordination import Coordinator
from robocup_navigation.coordination.executor import ExecutorGate
from robocup_navigation.coordination.verdict import verdict


def run(out):
    out.mkdir(parents=True, exist_ok=False)
    fleet = ['uav_%d' % i for i in range(1, 7)]
    positions = {u: [0., i * 4., 2.] for i, u in enumerate(fleet)}
    tasks = {'task%d' % i: dict(target_id='target%d' % i, xyz=[3., i * 4., 2.]) for i in range(6)}
    limits = dict(min_separation_m=1., arrival_tolerance_m=.15, mission_timeout_s=180.,
                  deadlock_timeout_s=60., max_monitor_gap_s=1.1, state_timeout_s=3.,
                  lease_s=3., stop_speed_mps=.05, required_clearance_m=.6,
                  tracking_bound_m=.2, position_tolerance_m=.05, nominal_speed_mps=.3)
    roles = {u: ['VEHICLE_STATE', 'COMMAND_ACK'] for u in fleet}
    roles.update(planner=['ROUTE_OFFER'], verifier=['RESOURCE_CLEAR'], clock=['TICK'])
    core = Coordinator(out.name, fleet, tasks, limits, roles, 'six-lane-kinematic-demo')
    gates = {u: ExecutorGate(out.name, u, .05, 3.) for u in fleet}
    seq, grants = {}, {}
    sim = 0.
    event_file = out / 'coord_events.jsonl'
    command_file = out / 'commands.jsonl'
    with event_file.open('x', encoding='utf-8') as events, command_file.open('x', encoding='utf-8') as commands:
        def flush():
            for e in core.drain_events():
                events.write(json.dumps(e, allow_nan=False) + '\n')
            for m in core.drain_outputs():
                commands.write(json.dumps(m, allow_nan=False) + '\n')
                for u, gate in gates.items():
                    action = gate.receive(m, sim, sim, 1, positions[u])
                    if action and m['kind'] == 'ROUTE_GRANT':
                        grants[u] = action

        def send(source, kind, data):
            seq[source] = seq.get(source, 0) + 1
            core.receive(dict(schema_version=1, run_id=out.name, source_id=source,
                              seq=seq[source], sim_s=sim, kind=kind, data=data), sim)
            flush()

        def state(u, mode, velocity, armed=True):
            g = grants.get(u)
            send(u, 'VEHICLE_STATE', dict(uav_id=u, frame_id='world_enu', xyz=positions[u],
                velocity_xyz=velocity, health='OK', mode=mode, armed=armed,
                active_command_id=g['command_id'] if g else None,
                observed_epoch=g['epoch'] if g else 0,
                route_version=g['route_version'] if g else 0, map_revision=1))

        def ack(u, status):
            g = grants[u]
            send(u, 'COMMAND_ACK', dict(uav_id=u, command_id=g['command_id'], task_id=g['task_id'],
                epoch=g['epoch'], route_version=g['route_version'], status=status, reason='SIMULATED'))

        flush()
        for i, u in enumerate(fleet):
            state(u, 'IDLE', [0., 0., 0.])
            send('planner', 'ROUTE_OFFER', dict(offer_id='offer%d' % i, uav_id=u, task_id='task%d' % i,
                frame_id='world_enu', points=[positions[u], tasks['task%d' % i]['xyz']], map_revision=1,
                valid_until_sim_s=100., clearance_m=1., tracking_bound_m=.1,
                static_safe=True, grid_safe=True, waiting_points=[]))
        send('clock', 'TICK', {})
        if len(grants) != 6:
            raise RuntimeError('Expected six simultaneous grants')
        for u in fleet:
            ack(u, 'ACCEPTED')
            ack(u, 'STARTED')
        separation = float('inf')
        for step in range(1, 21):
            sim = float(step)
            for u in fleet:
                if step <= 10:
                    positions[u][0] = min(3., step * .3)
                    state(u, 'EXECUTING' if step < 10 else 'HOLDING', [.3, 0., 0.] if step < 10 else [0., 0., 0.])
                    if step == 10:
                        ack(u, 'COMPLETED')
                        gates[u].confirm_stopped()
                else:
                    # Adapter-owned verified descent, outside cruise grants.
                    positions[u][2] = max(0., 2. - (step - 10) * .2)
                    state(u, 'LANDING' if step < 20 else 'LANDED',
                          [0., 0., -.2] if step < 20 else [0., 0., 0.], armed=step < 20)
            separation = min(separation, min(math.dist(positions[a], positions[b])
                              for i, a in enumerate(fleet) for b in fleet[i + 1:]))
            send('clock', 'TICK', {})
        for u in fleet:
            g = grants[u]
            send('verifier', 'RESOURCE_CLEAR', dict(uav_id=u, reservation_id=g['reservation_ids'][0],
                epoch=g['epoch'], route_version=g['route_version'], frame_id='world_enu',
                xyz=positions[u], map_revision=1))
        monitor = dict(coverage_start_sim_s=0., coverage_end_sim_s=sim, max_gap_s=1.,
                       collision_count=0, min_separation_m=separation, max_arrival_error_m=0.,
                       max_no_progress_s=10.)
        core.finish(monitor)
        flush()
    result = verdict({'run_dir': str(out)})
    print(json.dumps(dict(test='KINEMATIC_ONLY', fleet=6, duration_sim_s=sim,
                          min_separation_m=separation, verdict=result), ensure_ascii=False))
    return 0 if result[0] is True else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    raise SystemExit(run(parser.parse_args().out))
