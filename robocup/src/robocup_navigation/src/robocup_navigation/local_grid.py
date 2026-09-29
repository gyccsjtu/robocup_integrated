"""Body-camera depth image -> local world-frame occupancy grid.

Core geometry module for perception-mode replanning (M3).  Deliberately free
of ROS/numpy dependencies so it is unit-testable anywhere; the ROS plumbing
lives in the navigation node.
"""
import math
import threading

# Semantics
# ---------
# - A sliding square window in the ENU ``map`` frame, centred near the drone,
#   snapped to the resolution so the origin only moves in whole-cell steps.
# - Cell states are tracked exactly; ``to_grid_map`` folds them into the
#   A* ``GridMap`` convention (0 = free, 1 = blocked).  UNKNOWN cells fold to
#   blocked when ``unknown_is_obstacle`` is set, which is the perception-mode
#   default: the drone never plans through space it has not observed.
# - Depth hits below the obstacle band (ground returns) mark their cell FREE:
#   the ray passed through that column below the band without hitting anything.
# - Hits inside the band mark their cell OCCUPIED.
# - Hits above the band and out-of-range pixels leave cells UNKNOWN.

FREE = 0
OCCUPIED = 2
UNKNOWN = 3

_STATE_NAME = {FREE: "free", OCCUPIED: "occupied", UNKNOWN: "unknown"}


class LocalGridError(ValueError):
    """Invalid configuration or update input."""


def _positive(value, name):
    if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
        raise LocalGridError(name + "_INVALID")
    return float(value)


def _finite(value, name):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise LocalGridError(name + "_INVALID")
    value = float(value)
    if value != value or value in (float("inf"), float("-inf")):
        raise LocalGridError(name + "_NOT_FINITE")
    return value


class LocalOccupancyGrid(object):
    def __init__(self, resolution_m=0.25, side_m=10.0, near_m=0.5, far_m=18.0,
                 z_min_m=0.8, z_max_m=3.5, unknown_is_obstacle=True):
        self.resolution = _positive(resolution_m, "RESOLUTION")
        self.side_m = _positive(side_m, "SIDE")
        self.near_m = _positive(near_m, "NEAR")
        self.far_m = _positive(far_m, "FAR")
        if self.near_m >= self.far_m:
            raise LocalGridError("NEAR_GE_FAR")
        self.z_min_m = _finite(z_min_m, "Z_MIN")
        self.z_max_m = _finite(z_max_m, "Z_MAX")
        if self.z_min_m >= self.z_max_m:
            raise LocalGridError("Z_MIN_GE_Z_MAX")
        self.unknown_is_obstacle = bool(unknown_is_obstacle)

        self.width = int(round(self.side_m / self.resolution))
        self.height = self.width
        if self.width < 3:
            raise LocalGridError("GRID_TOO_SMALL")
        self.map_version = 0
        # content_revision bumps on EVERY state mutation (mark/free/reset),
        # unlike map_version which only moves on recenter.  Planning commits
        # compare the pair so a recenter-stable but content-changing grid
        # cannot pass a freshness check; corridor dedup keeps using
        # map_version alone (content churn must not open a replan storm).
        self.content_revision = 0
        self.lock = threading.Lock()
        self.states = [UNKNOWN] * (self.width * self.height)
        # ever_free[i]: cell i was observed FREE at least once.  A later
        # OCCUPIED write on an ever_free cell means a NEW object appeared in
        # already-observed space (dynamic candidate); an OCCUPIED write on a
        # never-free cell is geometry revealed for the first time by the FOV
        # edge (just as likely static) and must not be tracked as a moving
        # threat.  This is the minimal static/dynamic separation the
        # crossing-intruder detector needs.
        self.ever_free = [False] * (self.width * self.height)
        # Per-cell sim timestamp of the last OCCUPIED write, plus the current
        # frame stamp (via note_sim).  Powers decay_stale(): a moving object
        # leaves OCCUPIED trail cells the camera can never re-observe (too
        # close, below the vertical FOV) — without decay they become
        # permanent phantom obstacles that block every replan.
        self.sim_now = 0.0
        self.last_marked = [0.0] * (self.width * self.height)
        # marked_since[i]: sim time when cell i entered its CURRENT continuous
        # OCCUPIED streak.  Static geometry stays marked for as long as the
        # camera sees it (minutes); a moving object occupies a given cell for
        # only a couple of seconds before its trail decays.  The intruder
        # detector uses this to refuse evading geometry that has provably sat
        # still longer than any vehicle could.
        self.marked_since = [None] * (self.width * self.height)
        # Start centred on the world origin, snapped to whole cells.
        half = self.width * self.resolution / 2.0
        self.origin = [(round(-half / self.resolution)) * self.resolution,
                       (round(-half / self.resolution)) * self.resolution]

    # -- window management -------------------------------------------------
    def recenter(self, center_x, center_y):
        """Move the window so it is centred on the drone, snapping the origin
        to whole cells.  A moved window resets all cells to UNKNOWN and bumps
        the map version; a window whose snapped origin is unchanged is a no-op.
        """
        center_x = _finite(center_x, "CENTER_X")
        center_y = _finite(center_y, "CENTER_Y")
        half = self.width * self.resolution / 2.0
        new_ox = (round((center_x - half) / self.resolution)) * self.resolution
        new_oy = (round((center_y - half) / self.resolution)) * self.resolution
        with self.lock:
            if [new_ox, new_oy] != [self.origin[0], self.origin[1]]:
                self.origin[0] = new_ox
                self.origin[1] = new_oy
                self.states = [UNKNOWN] * (self.width * self.height)
                self.ever_free = [False] * (self.width * self.height)
                self.last_marked = [0.0] * (self.width * self.height)
                self.marked_since = [None] * (self.width * self.height)
                self.map_version += 1
                self.content_revision += 1

    def mark(self, x, y, state):
        """Write one cell (last write wins).  Returns True when the write
        changed traversability (an OCCUPIED cell appeared or vanished).
        NOTE: this does NOT bump ``map_version`` — depth-noise flicker at
        height-band boundaries would inflate it every frame.  Version only
        moves on window re-alignment (recenter), which is the coarse but
        stable "world changed" signal the replan dedup relies on."""
        cell = self.world_to_cell((x, y))
        if cell is not None:
            index = cell[1] * self.width + cell[0]
            with self.lock:
                changed = (self.states[index] == OCCUPIED) != (state == OCCUPIED)
                self.states[index] = state
                if state == FREE:
                    self.ever_free[index] = True
                if state == OCCUPIED:
                    if self.states[index] != OCCUPIED or \
                            self.marked_since[index] is None:
                        self.marked_since[index] = self.sim_now
                    self.last_marked[index] = self.sim_now
                elif self.states[index] == OCCUPIED:
                    self.marked_since[index] = None
                if self.states[index] != UNKNOWN:
                    self.content_revision += 1
                return changed
        return False

    def note_sim(self, sim_s):
        """Stamp the grid with the current frame's sim time (call before
        update so mark() can timestamp OCCUPIED writes)."""
        self.sim_now = _finite(sim_s, "NOTE_SIM")

    def decay_stale(self, max_age_s):
        """Demote OCCUPIED cells not re-marked within ``max_age_s`` to
        UNKNOWN and return how many were demoted.

        This is what lets a MOVED obstacle go: its trail cells cannot be
        re-observed (the object is gone and near-range ground is below the
        vertical FOV), so without decay they would block every future
        replan.  Demoted cells fold as UNKNOWN — with the verified base map
        attached, static scene geometry stays blocked through the merge, so
        decay can never open a hole in a real wall."""
        max_age_s = _finite(max_age_s, "DECAY_AGE")
        demoted = 0
        with self.lock:
            for index in range(len(self.states)):
                if self.states[index] == OCCUPIED and                         self.sim_now - self.last_marked[index] > max_age_s:
                    self.states[index] = UNKNOWN
                    self.ever_free[index] = False
                    self.marked_since[index] = None
                    self.content_revision += 1
                    demoted += 1
        return demoted

    def clear_cell(self, world_xy):
        """Force one cell FREE (observed-clear).

        Used to release a destination cell whose only obstruction is a
        transient depth shadow from a departing vehicle: the destination was
        reachable when first planned, so a stale mark there must not block
        mission completion.  Static geometry is still enforced by the base
        SafetyMap merge inside ``GridSource``, so this cannot open a hole in a
        real wall."""
        cell = self.world_to_cell(world_xy)
        if cell is None:
            return False
        index = cell[1] * self.width + cell[0]
        with self.lock:
            self.states[index] = FREE
            self.ever_free[index] = True
            self.marked_since[index] = None
            self.content_revision += 1
        return True

    def occupied_count_at(self, world_xy, radius_m):
        """Number of OCCUPIED cells within ``radius_m`` of ``world_xy``.

        Used to tell a large static structure (a wall, or a wall plus its
        shadow band — many cells) from a compact vehicle (a 0.4 m box is
        only a handful of cells).  Returns -1 when outside the window."""
        cell = self.world_to_cell(world_xy)
        if cell is None:
            return -1
        cx, cy = cell
        reach = max(0, int(math.ceil(radius_m / self.resolution)))
        count = 0
        for yy in range(max(0, cy - reach), min(self.height, cy + reach + 1)):
            for xx in range(max(0, cx - reach), min(self.width, cx + reach + 1)):
                if self.states[yy * self.width + xx] == OCCUPIED:
                    count += 1
        return count

    def static_age_at(self, world_xy, radius_m):
        """Longest continuous-OCCUPIED streak among cells within
        ``radius_m`` of ``world_xy``, in seconds (0.0 when none is
        currently occupied).  Returns -1.0 when the location falls
        outside the window."""
        cell = self.world_to_cell(world_xy)
        if cell is None:
            return -1.0
        cx, cy = cell
        reach = max(0, int(math.ceil(radius_m / self.resolution)))
        best = 0.0
        for yy in range(max(0, cy - reach), min(self.height, cy + reach + 1)):
            for xx in range(max(0, cx - reach), min(self.width, cx + reach + 1)):
                index = yy * self.width + xx
                if self.states[index] == OCCUPIED and \
                        self.marked_since[index] is not None:
                    age = self.sim_now - self.marked_since[index]
                    if age > best:
                        best = age
        return best

    # -- A* GridMap compatibility ------------------------------------------
    def world_to_cell(self, point):
        x = _finite(point[0], "POINT_X")
        y = _finite(point[1], "POINT_Y")
        x_max = self.origin[0] + self.width * self.resolution
        y_max = self.origin[1] + self.height * self.resolution
        if not (self.origin[0] <= x < x_max and self.origin[1] <= y < y_max):
            return None
        return (int((x - self.origin[0]) / self.resolution),
                int((y - self.origin[1]) / self.resolution))

    def cell_to_world(self, cell):
        return (self.origin[0] + (cell[0] + 0.5) * self.resolution,
                self.origin[1] + (cell[1] + 0.5) * self.resolution)

    def to_grid_map(self, unknown_is_obstacle=None, obstacle_shadow_cells=0):
        """Fold into an A* GridMap.  OCCUPIED -> 1, FREE -> 0 and
        UNKNOWN -> 1/0 per ``unknown_is_obstacle``.  When the argument is
        omitted the grid's own configured policy applies; passing False
        explicitly folds UNKNOWN as free so a caller can merge the window
        over a static base map (UNKNOWN then means "ask the base map").

        ``obstacle_shadow_cells`` (only meaningful when UNKNOWN folds as
        free) promotes UNKNOWN cells within this many 4-connected steps of
        an OCCUPIED cell to OCCUPIED before folding.  Rationale: the depth
        camera cannot see how far an obstacle continues past the scan edge
        (FOV boundary), so the planner must assume it continues for at
        least one clearance budget instead of planning a route that grazes
        the real obstacle corner through unscanned shadow."""
        if isinstance(obstacle_shadow_cells, bool) or \
                not isinstance(obstacle_shadow_cells, int) or \
                obstacle_shadow_cells < 0:
            raise LocalGridError("SHADOW_CELLS_INVALID")
        from robocup_navigation.astar import GridMap
        with self.lock:
            unknown_block = 1 if (self.unknown_is_obstacle if unknown_is_obstacle is None
                                  else bool(unknown_is_obstacle)) else 0
            cells = [1 if s == OCCUPIED else (unknown_block if s == UNKNOWN else 0)
                     for s in self.states]
            if obstacle_shadow_cells > 0 and unknown_block == 0:
                width = self.width
                height = self.height
                frontier = [index for index, s in enumerate(self.states)
                            if s == OCCUPIED]
                shadowed = set(frontier)
                for _ in range(obstacle_shadow_cells):
                    grown = []
                    for index in frontier:
                        cy, cx = divmod(index, width)
                        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                            nx, ny = cx + dx, cy + dy
                            if 0 <= nx < width and 0 <= ny < height:
                                neighbour = ny * width + nx
                                if neighbour not in shadowed and \
                                        self.states[neighbour] == UNKNOWN:
                                    shadowed.add(neighbour)
                                    grown.append(neighbour)
                    for index in grown:
                        cells[index] = 1
                    frontier = grown
                    if not frontier:
                        break
            return GridMap(self.width, self.height, self.resolution,
                           tuple(self.origin), cells, "map")

    # -- depth update -------------------------------------------------------
    def update(self, depth, width, height, fx, fy, cx, cy,
               drone_x, drone_y, drone_z, yaw,
               cam_dx=0.12, cam_dy=0.0, cam_dz=0.04, max_points=4096):
        """Fuse one depth frame.

        ``depth`` is a row-major sequence of per-pixel ranges in metres
        (32FC1 encoding, already unpacked).  The camera is assumed rigidly
        mounted at ``(cam_dx, cam_dy, cam_dz)`` in the body frame with its
        optical axis along body +X, camera +Y left, +Z up (Gazebo body-frame
        camera convention).  ``yaw`` is the ENU heading in radians.
        """
        width = int(width)
        height = int(height)
        if width <= 0 or height <= 0 or len(depth) != width * height:
            raise LocalGridError("DEPTH_SHAPE_MISMATCH")
        fx = _positive(fx, "FX")
        fy = _positive(fy, "FY")
        cx = _finite(cx, "CX")
        cy = _finite(cy, "CY")
        drone_x = _finite(drone_x, "DRONE_X")
        drone_y = _finite(drone_y, "DRONE_Y")
        drone_z = _finite(drone_z, "DRONE_Z")
        yaw = _finite(yaw, "YAW")

        # Camera pose in world frame: translate by drone pos, rotate by yaw.
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        cam_wx = drone_x + cos_y * cam_dx - sin_y * cam_dy
        cam_wy = drone_y + sin_y * cam_dx + cos_y * cam_dy
        cam_wz = drone_z + cam_dz

        # Marked-cell dedup within this frame.
        seen = set()
        # Subsample if the frame has more candidate pixels than the budget.
        step = 1
        total = width * height
        if total > max_points:
            step = int(total / max_points) + 1

        for index in range(0, total, step):
            d = depth[index]
            if d != d or d < self.near_m or d > self.far_m:
                continue
            u = index % width
            v = index // width
            # Ray in the camera frame (X forward, Y left, Z up).
            rx = fx
            ry = cx - u
            rz = cy - v
            norm = math.sqrt(rx * rx + ry * ry + rz * rz)
            if norm <= 0:
                continue
            # Direction in the body frame == camera frame (identity mount).
            bx = rx / norm
            by = ry / norm
            bz = rz / norm
            # Hit point in world frame: rotate body->world by yaw.
            hx = cam_wx + d * (cos_y * bx - sin_y * by)
            hy = cam_wy + d * (sin_y * bx + cos_y * by)
            hz = cam_wz + d * bz
            if hz < self.z_min_m:
                state = FREE
            elif hz <= self.z_max_m:
                state = OCCUPIED
            else:
                state = UNKNOWN
            key = (hx, hy)
            if state != UNKNOWN and key not in seen:
                seen.add(key)
                self.mark(hx, hy, state)

    def summary(self):
        counts = {name: 0 for name in _STATE_NAME.values()}
        for s in self.states:
            counts[_STATE_NAME[s]] += 1
        return counts
