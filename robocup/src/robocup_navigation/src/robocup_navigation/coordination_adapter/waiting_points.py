"""Waiting-point candidate computation WITH proofs (clarifications §1).

Division of labour: Codex owns the rules; this adapter computes candidates and
their proofs from the local map.  The core only re-checks conflicts with other
aircraft -- and, per §1, has NO automatic yield authorisation.

Refusal over guessing: if any required proof is missing or fails, the candidate
is dropped.  An empty result means "no proved waiting point", never "pick any
FREE cell".

Proofs implemented:
  * ``clearance_m``  -- conservative lower bound over the waiting point AND the
    whole connector from the current pose, minus the body budget.  A FREE cell
    is NOT sufficient clearance.
  * ``tracking_bound_m`` -- the executor-declared upper bound (positioning +
    tracking + stop overshoot).  The CURRENT instantaneous tracking error must
    never be used here; unknown bound -> no candidate.
  * ``connector_safe`` -- the ENTIRE connector is checked on one snapshot.
  * ``outside_bottleneck`` -- the full dwell envelope must not intrude the
    planner-identified bottleneck/entry geometry.  No geometry -> no candidate.
  * ``static_safe`` / ``grid_safe`` -- actual check results (never constants).
"""
import math

from .map_revision import FREE, OCCUPIED, UNKNOWN

BIG = 1.0e9


class OccupancyView:
    """Immutable geometric view over one map snapshot.

    ``cells`` maps integer index (i, j, k) -> state; world = origin + index*res.
    """

    def __init__(self, cells, resolution, origin=(0.0, 0.0, 0.0)):
        self.cells = dict(cells)
        self.resolution = float(resolution)
        self.origin = tuple(float(v) for v in origin)
        self._obstacles = [self._world(idx) for idx, st in self.cells.items() if st != FREE]
        self._occupied = [self._world(idx) for idx, st in self.cells.items() if st == OCCUPIED]

    def _world(self, index):
        return tuple(self.origin[a] + index[a] * self.resolution for a in range(3))

    def is_free(self, xyz):
        """FREE only on an explicit FREE cell; UNKNOWN is never free."""
        index = tuple(int(round((xyz[a] - self.origin[a]) / self.resolution)) for a in range(3))
        return self.cells.get(index) == FREE

    def nearest_obstacle_distance(self, xyz):
        """Distance to the nearest OCCUPIED/UNKNOWN cell centre (BIG if none)."""
        if not self._obstacles:
            return BIG
        return min(math.dist(xyz, o) for o in self._obstacles)

    def has_occupied_on(self, samples):
        return any(not self.is_free(s) and self._nearest_is_occupied(s) for s in samples)

    def _nearest_is_occupied(self, xyz):
        return any(math.dist(xyz, o) < self.resolution for o in self._occupied)


def sample_segment(a, b, step_m):
    """Sample [a, b] inclusively with spacing <= step_m (endpoints included)."""
    length = math.dist(a, b)
    n = max(1, int(math.ceil(length / float(step_m))))
    return [tuple(a[i] + (b[i] - a[i]) * t / n for i in range(3)) for t in range(n + 1)]


class WaitingPointProvider:
    def __init__(self, vehicle_radius_m, required_clearance_m, max_tracking_bound_m,
                 sample_step_m=0.25):
        self.radius = float(vehicle_radius_m)
        self.required_clearance = float(required_clearance_m)
        self.max_tb = float(max_tracking_bound_m)
        self.step = float(sample_step_m)

    def candidates(self, *, view, revision, current_xyz, proposed_xyz_list,
                   bottleneck_boxes, tracking_bound_m, sim_s, valid_until_sim_s):
        """Return only fully proved candidates; [] when anything is unproved."""
        if tracking_bound_m is None or not (0.0 < tracking_bound_m <= self.max_tb):
            return []                       # unknown / out-of-budget envelope
        if not (valid_until_sim_s > sim_s):
            return []                       # already expired
        if bottleneck_boxes is None:
            return []                       # no channel geometry -> cannot prove

        envelope = self.radius + float(tracking_bound_m)
        result = []
        for i, p in enumerate(proposed_xyz_list):
            if not self._outside_bottleneck(p, envelope, bottleneck_boxes):
                continue
            samples = sample_segment(tuple(current_xyz), tuple(p), self.step)
            clearance = min(view.nearest_obstacle_distance(s) for s in samples) - self.radius
            if clearance < self.required_clearance:
                continue
            static_safe = not view.has_occupied_on(samples)          # real check
            grid_safe = all(view.is_free(s) for s in samples)        # real check
            if not (static_safe and grid_safe):
                continue
            result.append(dict(
                point_id="wp_%d_%d" % (revision, i), xyz=list(p), map_revision=revision,
                valid_until_sim_s=float(valid_until_sim_s), clearance_m=float(clearance),
                tracking_bound_m=float(tracking_bound_m), static_safe=True,
                grid_safe=True, connector_safe=True, outside_bottleneck=True))
        return result

    def _outside_bottleneck(self, xyz, envelope, boxes):
        for box in boxes:
            lo, hi = box["min"], box["max"]
            if all(lo[a] - envelope <= xyz[a] <= hi[a] + envelope for a in range(3)):
                return False
        return True
