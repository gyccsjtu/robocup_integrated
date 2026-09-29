"""One verdict implementation shared by the core and WorkBuddy collectors."""
import json
import os

from .protocol import integer, number, string


FAILURES = {'COLLISION', 'SEPARATION_BREACH', 'DEADLOCK', 'MISSION_TIMEOUT',
            'ARRIVAL_ERROR', 'AUTHORIZATION_CONFLICT', 'STALE_COMMAND_EXECUTED'}
EVENTS = {'RUN_STARTED', 'TASK_ASSIGNED', 'LOCK_CHANGED', 'ROUTE_GRANTED',
          'COMMAND_ACKED', 'RESERVATION_CHANGED', 'SAFETY_VIOLATION',
          'TASK_COMPLETED', 'RUN_FINISHED'}
LIMITS = {'min_separation_m', 'arrival_tolerance_m', 'mission_timeout_s',
          'deadlock_timeout_s', 'max_monitor_gap_s'}


def unique_strings(value):
    return isinstance(value, list) and all(string(x) for x in value) and len(set(value)) == len(value)


def evaluate_terminal(limits, start, end, required, fleet_size, d):
    if not isinstance(d, dict) or set(d) != {'outcome', 'failures', 'completed_task_ids',
                                            'all_landed_disarmed', 'evidence_complete', 'monitor_report'}:
        return None, 'TERMINAL_SCHEMA'
    if not unique_strings(d['failures']) or not set(d['failures']) <= FAILURES:
        return None, 'FAILURE_SCHEMA'
    if d['failures']:
        return False, d['failures'][0]
    if d['outcome'] not in ('PASS', 'FAIL', 'ABSTAIN') or \
            type(d['evidence_complete']) is not bool or type(d['all_landed_disarmed']) is not bool or \
            not unique_strings(d['completed_task_ids']):
        return None, 'TERMINAL_SCHEMA'
    if not isinstance(limits, dict) or not LIMITS <= set(limits) or \
            any(not number(limits[k]) or limits[k] <= 0 for k in LIMITS) or \
            not number(start) or not number(end) or end < start:
        return None, 'LIMITS_OR_TIME_INVALID'
    m = d['monitor_report']
    keys = {'coverage_start_sim_s', 'coverage_end_sim_s', 'max_gap_s', 'collision_count',
            'min_separation_m', 'max_arrival_error_m', 'max_no_progress_s'}
    if not isinstance(m, dict) or set(m) != keys or not integer(m['collision_count']) or \
            any(not number(m[k]) for k in keys - {'min_separation_m'}) or \
            (not number(m['min_separation_m']) and not (fleet_size < 2 and m['min_separation_m'] is None)):
        return None, 'MONITOR_MISSING_OR_INVALID'
    violations = [('COLLISION', m['collision_count'] > 0),
                  ('SEPARATION_BREACH', fleet_size >= 2 and m['min_separation_m'] < limits['min_separation_m']),
                  ('ARRIVAL_ERROR', m['max_arrival_error_m'] > limits['arrival_tolerance_m']),
                  ('DEADLOCK', m['max_no_progress_s'] > limits['deadlock_timeout_s']),
                  ('MISSION_TIMEOUT', end - start > limits['mission_timeout_s'])]
    for reason, failed in violations:
        if failed:
            return False, reason
    if not d['evidence_complete'] or m['coverage_start_sim_s'] > start or \
            m['coverage_end_sim_s'] < end or m['coverage_start_sim_s'] > m['coverage_end_sim_s'] or \
            m['max_gap_s'] > limits['max_monitor_gap_s']:
        return None, 'MONITOR_COVERAGE_INCOMPLETE'
    if set(d['completed_task_ids']) != set(required) or not d['all_landed_disarmed']:
        return False, 'TASKS_OR_LANDING_INCOMPLETE'
    return True, 'ACCEPTANCE_PASSED'


def event_valid(e):
    if not isinstance(e, dict) or set(e) != {'schema_version', 'run_id', 'source_id', 'seq', 'sim_s', 'kind', 'data'}:
        return False
    if type(e['schema_version']) is not int or e['schema_version'] != 1 or e['kind'] != 'EVENT' or \
            not string(e['run_id']) or not string(e['source_id']) or \
            not integer(e['seq']) or e['seq'] < 1 or not number(e['sim_s']):
        return False
    d = e['data']
    return isinstance(d, dict) and set(d) == {'event', 'uav_id', 'task_id', 'reason', 'details'} and \
        isinstance(d['event'], str) and d['event'] in EVENTS and string(d['reason']) and \
        (d['uav_id'] is None or string(d['uav_id'])) and \
        (d['task_id'] is None or string(d['task_id'])) and isinstance(d['details'], dict)


def verdict(run):
    """WorkBuddy hook: verdict({'run_dir': ...}) -> (bool | None, reason).

    Reads the complete coordination stream; old navigation RESULT=0 alone
    is intentionally insufficient. Does not trust a claimed terminal PASS.
    """
    try:
        with open(os.path.join(run['run_dir'], 'coord_events.jsonl'), encoding='utf-8') as f:
            lines = list(f)
    except (OSError, KeyError, TypeError):
        return None, 'MISSING_COORD_EVENTS'
    events, invalid, seen = [], False, {}
    for line in lines:
        if not line.strip():
            continue
        try:
            e = json.loads(line)
            if not event_valid(e):
                invalid = True
                continue
            key = e['run_id'], e['source_id'], e['seq']
            raw = line.rstrip('\r\n')
            if key in seen:
                invalid |= seen[key] != raw
                continue
            seen[key] = raw
            events.append(e)
        except (ValueError, TypeError):
            invalid = True
    starts = [e for e in events if e['data']['event'] == 'RUN_STARTED']
    if len(starts) != 1:
        return None, 'RUN_START_MISSING_OR_DUPLICATE'
    start = starts[0]
    # Only a same-run same-producer violation is authoritative for this log.
    relevant = [e for e in events if (e['run_id'], e['source_id']) == (start['run_id'], start['source_id'])]
    for e in relevant:
        if e['data']['event'] == 'SAFETY_VIOLATION' and e['data']['reason'] in FAILURES:
            return False, e['data']['reason']
    sd = start['data']['details']
    for e in relevant:
        if e['data']['event'] == 'RUN_FINISHED' and isinstance(sd.get('fleet_ids'), list):
            known, why = evaluate_terminal(sd.get('limits'), start['sim_s'], e['sim_s'],
                                           sd.get('required_task_ids', []), len(sd['fleet_ids']),
                                           e['data']['details'])
            if known is False and why in FAILURES:
                return False, why
    if invalid or len(relevant) != len(events) or not events or events[0] != start:
        return None, 'EVENT_EVIDENCE_INVALID'
    if any(e['seq'] != i + 1 for i, e in enumerate(events)) or \
            any(b['sim_s'] < a['sim_s'] for a, b in zip(events, events[1:])):
        return None, 'EVENT_SEQUENCE_OR_TIME_INVALID'
    ends = [e for e in events if e['data']['event'] == 'RUN_FINISHED']
    if len(ends) != 1 or ends[0] != events[-1]:
        return None, 'RUN_FINISH_MISSING_OR_INVALID'
    s, d = start['data']['details'], ends[0]['data']['details']
    if set(s) != {'fleet_ids', 'required_task_ids', 'test_case_id', 'config_digest', 'limits'} or \
            not unique_strings(s['fleet_ids']) or not 1 <= len(s['fleet_ids']) <= 6 or \
            not unique_strings(s['required_task_ids']) or not s['required_task_ids'] or \
            not string(s['test_case_id']) or not isinstance(s['config_digest'], str) or \
            len(s['config_digest']) != 64 or any(c not in '0123456789abcdef' for c in s['config_digest']):
        return None, 'RUN_START_SCHEMA'
    ok, reason = evaluate_terminal(s['limits'], start['sim_s'], ends[0]['sim_s'],
                                   s['required_task_ids'], len(s['fleet_ids']), d)
    if ok is True:
        completed = {e['data']['task_id'] for e in events if e['data']['event'] == 'TASK_COMPLETED'}
        if completed != set(s['required_task_ids']):
            return None, 'TASK_COMPLETION_EVIDENCE_MISSING'
        # Completion is not clearance. Reconstruct reservations rather than
        # trusting a terminal success string with missing release records.
        from .executor import SPECS
        from .protocol import fields, CoordinationError
        reservations, granted_tasks = {}, set()
        for e in events:
            name, detail = e['data']['event'], e['data']['details']
            try:
                if name == 'RESERVATION_CHANGED':
                    fields(detail, SPECS['RESERVATION'])
                    rid = detail['reservation_id']
                    if rid not in reservations and detail['state'] != 'RESERVED':
                        return None, 'RESERVATION_HISTORY_MISSING'
                    reservations[rid] = detail
                elif name == 'ROUTE_GRANTED':
                    fields(detail, SPECS['ROUTE_GRANT'])
                    if detail['uav_id'] not in s['fleet_ids'] or detail['task_id'] not in s['required_task_ids']:
                        return None, 'GRANT_ID_INVALID'
                    for rid in detail['reservation_ids']:
                        r = reservations.get(rid)
                        if not r or r['state'] != 'RESERVED' or any(r[k] != detail[k] for k in ('uav_id', 'task_id', 'epoch', 'route_version')):
                            return None, 'GRANT_RESERVATION_MISSING'
                    granted_tasks.add(detail['task_id'])
            except (CoordinationError, TypeError, ValueError, KeyError):
                return None, 'AUTHORIZATION_EVIDENCE_INVALID'
        if granted_tasks != set(s['required_task_ids']) or any(r['state'] != 'RELEASED' for r in reservations.values()):
            return None, 'RESERVATION_CLEARANCE_EVIDENCE_MISSING'
        if d['outcome'] != 'PASS':
            return None, 'TERMINAL_OUTCOME_CONFLICT'
    return ok, reason
