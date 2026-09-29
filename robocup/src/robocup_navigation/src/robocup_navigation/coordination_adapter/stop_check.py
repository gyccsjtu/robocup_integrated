"""Sustained-stop / settle tracking that the core deliberately does NOT do.

Clarifications §3: the core only inspects the LATEST velocity.  A single
low-velocity frame must never be reported as STOPPED/COMPLETED.  The adapter
keeps the continuous settle window (``limits.settle_time_s``) and only then
lets the executor emit STOPPED/COMPLETED.

Rules implemented:
  * state must be FRESH in both sim and wall time;
  * 3-D speed norm <= stop_speed_mps;
  * mode in {HOLDING, LANDED, IDLE};
  * active_command_id / observed_epoch / route_version must match the command;
  * the UAV must stay within position_tolerance_m of the window's anchor,
    otherwise the timer RESETS;
  * a paused simulation (sim time not advancing) does not accrue settle time.
COMPLETED additionally needs the final 3-D distance to the task xyz to be
<= arrival_tolerance_m (not an intermediate waypoint, not a string).
"""
import math


class SustainedStop:
    def __init__(self, stop_speed_mps, settle_time_s, position_tolerance_m, state_timeout_s):
        self.stop_speed = float(stop_speed_mps)
        self.settle_s = float(settle_time_s)
        self.position_tol = float(position_tolerance_m)
        self.state_timeout = float(state_timeout_s)
        self._w = {}   # uav_id -> dict(start_sim, anchor_xyz, last_sim, last_wall)

    def update(self, uav_id, *, sim_s, wall_s, position, velocity, mode,
               active_command_id, observed_epoch, route_version,
               command_id, epoch, expected_route_version):
        """Feed one VEHICLE_STATE.  Returns True when the settle window is met."""
        ok = (mode in ("HOLDING", "LANDED", "IDLE")
              and active_command_id == command_id
              and observed_epoch == epoch
              and route_version == expected_route_version
              and self._speed(velocity) <= self.stop_speed)
        with_state = self._w.get(uav_id)
        if not ok:
            self._w.pop(uav_id, None)
            return False
        if with_state is None:
            self._w[uav_id] = dict(start_sim=sim_s, anchor=list(position),
                                   last_sim=sim_s, last_wall=wall_s)
            return False
        w = with_state
        # freshness: both clocks must move forward, and wall gap bounded
        if sim_s < w["last_sim"] or wall_s < w["last_wall"] or wall_s - w["last_wall"] > self.state_timeout:
            self._w.pop(uav_id, None)
            return False
        if math.dist(position, w["anchor"]) > self.position_tol:
            # drifted while "stopping": restart the window from here
            w.update(start_sim=sim_s, anchor=list(position), last_sim=sim_s, last_wall=wall_s)
            return False
        w["last_sim"], w["last_wall"] = sim_s, wall_s
        return (sim_s - w["start_sim"]) >= self.settle_s

    def arrival_ok(self, position, target_xyz, arrival_tolerance_m):
        return math.dist(position, target_xyz) <= float(arrival_tolerance_m)

    def reset(self, uav_id=None):
        if uav_id is None:
            self._w.clear()
        else:
            self._w.pop(uav_id, None)

    @staticmethod
    def _speed(velocity):
        return math.sqrt(sum(float(v) ** 2 for v in velocity))
