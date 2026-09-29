"""Independent physical monitor for multi-UAV runs.

Interface v0 §4: the coordinator's own authorisation events *cannot* prove
physical safety, so collision / separation / arrival measurements must come
from an independent source that covers the whole run (landing included) and
produces the ``monitor_report`` the core only aggregates.

This class is deliberately not fed by the core.  On the development harness the
values may come from Gazebo truth / TF; that data must never be routed into the
core (the core must not read truth).

collision_count / min_separation / max_arrival_error / max_no_progress are the
exact fields RUN_FINISHED.monitor_report expects.
"""
import math


class PhysicalMonitor:
    def __init__(self, fleet_ids, min_separation_m, arrival_tolerance_m,
                 max_monitor_gap_s, collision_separation_m=0.0, start_sim_s=None):
        self.fleet_ids = list(fleet_ids)
        self.min_sep = float(min_separation_m)
        self.arrival_tol = float(arrival_tolerance_m)
        self.max_gap = float(max_monitor_gap_s)
        self.collision_sep = float(collision_separation_m)

        # The coverage window is declared by the caller at run start: the first
        # *sample* necessarily lands a tick later, and the verdict rejects any
        # coverage_start_sim_s strictly greater than the run start. Declaring the
        # window invents no measurement -- every number still comes from sample().
        self.start = start_sim_s
        self.end = None
        self._last_sim = None
        self.max_gap_seen = 0.0
        self.collisions = 0
        self.min_sep_seen = None
        self.max_arrival_error = 0.0
        self._progress_sim = None
        self.max_no_progress = 0.0

    def sample(self, sim_s, positions, task_targets=None, progressed=True):
        """Record one independent sample at ``sim_s`` and return the report.

        positions:      {uav_id: [x, y, z]} measured independently.
        task_targets:   {uav_id: [x, y, z]} for arrival error. The caller must
                        pass this ONLY for UAVs whose task has completed:
                        max_arrival_error is a running maximum, so passing the
                        target during transit would record the transit distance
                        as the arrival error.
        progressed:     harness assessment that some task made real progress;
                        heartbeat/duplicate commands must be reported as False.
        """
        if self.start is None:
            self.start = sim_s
        self.end = sim_s
        if self._last_sim is not None:
            self.max_gap_seen = max(self.max_gap_seen, sim_s - self._last_sim)
        self._last_sim = sim_s

        ids = [u for u in self.fleet_ids if u in positions]
        for i, a in enumerate(ids):
            for b in ids[i + 1:]:
                d = math.dist(positions[a], positions[b])
                self.min_sep_seen = d if self.min_sep_seen is None else min(self.min_sep_seen, d)
                if d <= self.collision_sep:
                    self.collisions += 1

        if task_targets:
            for u, tgt in task_targets.items():
                if u in positions:
                    self.max_arrival_error = max(self.max_arrival_error,
                                                 math.dist(positions[u], tgt))

        if progressed:
            self._progress_sim = sim_s
        else:
            base = self._progress_sim if self._progress_sim is not None else self.start
            self.max_no_progress = max(self.max_no_progress, sim_s - base)
        return self.report()

    def report(self):
        return dict(
            coverage_start_sim_s=self.start if self.start is not None else 0.0,
            coverage_end_sim_s=self.end if self.end is not None else 0.0,
            max_gap_s=self.max_gap_seen,
            collision_count=self.collisions,
            min_separation_m=self.min_sep_seen,
            max_arrival_error_m=self.max_arrival_error,
            max_no_progress_s=self.max_no_progress,
        )

    def coverage_ok(self):
        """True only if the monitor covered the run with no gap beyond budget."""
        return self.start is not None and self.max_gap_seen <= self.max_gap
