"""Per-UAV authoritative map snapshot + revision (interface v0 §2, clarif. §2).

Two rules drive this module:

  * ATOMICITY.  Security-relevant proofs, the planner and the executor gate must
    all reference the SAME counter and the SAME snapshot.  Callers therefore use
    :meth:`snapshot` (which returns the grid copy and its revision together) and
    must never read the counter separately.
  * WHAT BUMPs THE REVISION.  Any safety-relevant content change
    (FREE/OCCUPIED/UNKNOWN transitions), window recenter/reset, or geometry
    change.  Rewriting identical content keeps the revision; refreshing an
    observation timestamp is not a content change.

Simulation states: FREE=0, OCCUPIED=1, UNKNOWN=2 (unknown is NOT free).
"""
import threading

FREE, OCCUPIED, UNKNOWN = 0, 1, 2

VALID_STATES = (FREE, OCCUPIED, UNKNOWN)


class MapRevisionError(ValueError):
    pass


class MapRevisionProvider:
    """Owns one UAV's occupancy grid and its content revision."""

    def __init__(self, initial=None):
        self._lock = threading.RLock()
        self._cells = {}
        self._rev = 0
        if initial:
            self.apply(initial)

    # -- writers ------------------------------------------------------------
    def apply(self, cell_states):
        """Write {cell_index: state}.  Bumps revision only on real change.

        Returns (revision, changed_count).
        """
        for index, state in cell_states.items():
            if state not in VALID_STATES:
                raise MapRevisionError("bad state %r at %r" % (state, index))
        with self._lock:
            changed = 0
            for index, state in cell_states.items():
                if self._cells.get(index) != state:
                    self._cells[index] = state
                    changed += 1
            if changed:
                self._rev += 1
            return self._rev, changed

    def clear(self):
        """Clear every cell (e.g. a fresh observation window).  Returns revision."""
        with self._lock:
            if self._cells:
                self._cells = {}
                self._rev += 1
            return self._rev

    def recenter(self):
        """Window recenter / geometry change: always safety relevant."""
        with self._lock:
            self._rev += 1
            return self._rev

    def reset(self, cells, geometry_changed=True):
        """Full replacement (restart / reset).  Bumps on content or geometry."""
        with self._lock:
            if geometry_changed or self._cells != dict(cells):
                self._cells = dict(cells)
                self._rev += 1
            return self._rev

    # -- reader -------------------------------------------------------------
    def snapshot(self):
        """Atomic read: returns (cells_copy, revision).  Use this, not .revision."""
        with self._lock:
            return dict(self._cells), self._rev

    @property
    def revision(self):
        """Informational only -- never pair it with a separately read grid."""
        with self._lock:
            return self._rev
