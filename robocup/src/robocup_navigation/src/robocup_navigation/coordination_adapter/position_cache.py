"""Display-only position cache with explicit staleness (clarifications §4).

``Coordinator.snapshot()`` exposes only ``tasks``, ``reservations``, ``halted``
and ``failures`` -- no positions.  For visualisation the adapter may cache REAL
telemetry positions converted to ``world_enu``, but it must expose the timestamp
and a staleness flag.

Rules:
  * missing position -> report MISSING, never 0.0 or an extrapolated value;
  * stale position -> still report it, but flagged;
  * this cache is for DISPLAY ONLY.  Acceptance monitoring must come from an
    independent measurement stream, never from this cache, from the authorised
    route, or from TASK_COMPLETED.
"""


class PositionCache:
    def __init__(self, max_age_s):
        self.max_age = float(max_age_s)
        self._p = {}

    def update(self, uav_id, xyz, sim_s, wall_s):
        self._p[uav_id] = dict(xyz=[float(v) for v in xyz], sim_s=float(sim_s),
                               wall_s=float(wall_s))

    def get(self, uav_id, sim_s, wall_s):
        """Return {xyz, sim_s, wall_s, age_s, stale} or None if MISSING."""
        rec = self._p.get(uav_id)
        if rec is None:
            return None
        age = max(float(sim_s) - rec["sim_s"], float(wall_s) - rec["wall_s"])
        return dict(xyz=list(rec["xyz"]), sim_s=rec["sim_s"], wall_s=rec["wall_s"],
                    age_s=age, stale=age > self.max_age)

    def drop(self, uav_id):
        self._p.pop(uav_id, None)
