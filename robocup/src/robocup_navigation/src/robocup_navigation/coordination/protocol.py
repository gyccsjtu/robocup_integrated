"""Strict frozen schema-v1 input validation. No ROS dependencies."""
import json
import math


class CoordinationError(ValueError):
    pass


def number(value, minimum=0.):
    return type(value) in (int, float) and math.isfinite(value) and value >= minimum


def integer(value):
    return type(value) is int and value >= 0


def string(value):
    return isinstance(value, str) and bool(value)


def nullable(value):
    return value is None or string(value)


def xyz(value):
    return isinstance(value, list) and len(value) == 3 and all(number(x, -math.inf) for x in value)


def enum(*values):
    return lambda x: x in values and isinstance(x, str)


def fields(data, spec):
    if not isinstance(data, dict) or set(data) - set(spec) - {'extensions'} or set(spec) - set(data):
        raise CoordinationError('SCHEMA_FIELDS')
    if 'extensions' in data and not isinstance(data['extensions'], dict):
        raise CoordinationError('SCHEMA_EXTENSIONS')
    for key, check in spec.items():
        if not check(data[key]):
            raise CoordinationError('SCHEMA_VALUE:' + key)


COMMON = dict(uav_id=string, frame_id=enum('world_enu'))
WAIT = dict(point_id=string, xyz=xyz, map_revision=integer, valid_until_sim_s=number,
            clearance_m=number, tracking_bound_m=number, static_safe=lambda x: type(x) is bool,
            grid_safe=lambda x: type(x) is bool, connector_safe=lambda x: type(x) is bool,
            outside_bottleneck=lambda x: type(x) is bool)


def waiting(value):
    if not isinstance(value, list):
        return False
    for item in value:
        fields(item, WAIT)
    return len({x['point_id'] for x in value}) == len(value)


SPECS = {
    'VEHICLE_STATE': dict(COMMON, xyz=xyz, velocity_xyz=xyz,
                          health=enum('OK', 'DEGRADED', 'FAILED'),
                          mode=enum('IDLE', 'EXECUTING', 'HOLDING', 'LANDING', 'LANDED'),
                          armed=lambda x: type(x) is bool, active_command_id=nullable,
                          observed_epoch=integer, route_version=integer, map_revision=integer),
    'TARGET_REPORT': dict(target_id=string, frame_id=enum('world_enu'), xyz=xyz,
                          confidence=lambda x: number(x) and x <= 1, observation_id=string),
    'ROUTE_OFFER': dict(COMMON, offer_id=string, task_id=string,
                        points=lambda x: isinstance(x, list) and len(x) >= 2 and all(xyz(p) for p in x),
                        map_revision=integer, valid_until_sim_s=number, clearance_m=number,
                        tracking_bound_m=number, static_safe=lambda x: type(x) is bool,
                        grid_safe=lambda x: type(x) is bool, waiting_points=waiting),
    'COMMAND_ACK': dict(uav_id=string, command_id=string, task_id=nullable,
                        epoch=integer, route_version=integer,
                        status=enum('ACCEPTED', 'STARTED', 'STOPPED', 'COMPLETED', 'REJECTED'), reason=string),
    'RESOURCE_CLEAR': dict(COMMON, reservation_id=string, epoch=integer,
                           route_version=integer, xyz=xyz, map_revision=integer),
    'CANCEL_TASK': dict(task_id=string, reason=string),
    'TICK': {},
}


def validate(message):
    if not isinstance(message, dict) or set(message) != {
            'schema_version', 'run_id', 'source_id', 'seq', 'sim_s', 'kind', 'data'}:
        raise CoordinationError('SCHEMA_ENVELOPE')
    if type(message['schema_version']) is not int or message['schema_version'] != 1:
        raise CoordinationError('SCHEMA_VERSION')
    if not all(string(message[k]) for k in ('run_id', 'source_id', 'kind')):
        raise CoordinationError('SCHEMA_ID')
    if not integer(message['seq']) or message['seq'] < 1 or not number(message['sim_s']):
        raise CoordinationError('SCHEMA_TIME_OR_SEQ')
    if message['kind'] not in SPECS:
        raise CoordinationError('SCHEMA_KIND')
    fields(message['data'], SPECS[message['kind']])
    if message['kind'] == 'TICK' and message['data']:
        raise CoordinationError('SCHEMA_TICK')
    try:
        return json.dumps(message, sort_keys=True, separators=(',', ':'), allow_nan=False)
    except (ValueError, TypeError) as exc:
        raise CoordinationError('SCHEMA_JSON') from exc
