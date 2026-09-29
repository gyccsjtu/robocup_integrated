"""WorkBuddy-owned ROS-side adapter for the Codex coordination core (schema v1).

Package layout:
    bus.py             single-writer front end + gate fan-out
    monitor.py         independent physical monitor (fed by the test harness)
    map_revision.py    atomic per-UAV map snapshot + content revision
    waiting_points.py  waiting-point candidates WITH proofs (or empty set)
    stop_check.py      continuous settle/stop tracking the core does not do
    position_cache.py  display-only position cache with staleness
    node.py            ROS node wrapper (imports rospy; not needed for host tests)

Only the core (``robocup_navigation.coordination``) owns safety semantics.
Nothing in here may reinterpret release/clearance/timeout rules.
"""
from .bus import COORDINATOR_SOURCE, CoreBus, GateFanout, InputSeq
from .map_revision import FREE, OCCUPIED, UNKNOWN, MapRevisionProvider
from .monitor import PhysicalMonitor
from .position_cache import PositionCache
from .stop_check import SustainedStop
from .waiting_points import OccupancyView, WaitingPointProvider, sample_segment

__all__ = [
    "COORDINATOR_SOURCE", "CoreBus", "GateFanout", "InputSeq", "PhysicalMonitor",
    "MapRevisionProvider", "FREE", "OCCUPIED", "UNKNOWN",
    "WaitingPointProvider", "OccupancyView", "sample_segment",
    "SustainedStop", "PositionCache",
]
