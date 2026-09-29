"""Strict A* adapter for robocup_training_worlds metadata v1 grids.

This module is intentionally independent from ROS and from the training-world
generator's Python internals.  It consumes the versioned JSON contract only.
It plans geometry; it never publishes a setpoint or claims flight success.
"""

from dataclasses import dataclass
import hashlib
import heapq
import json
import math


SCHEMA = "robocup_training_worlds/metadata/v1"


class MapError(ValueError):
    """Malformed or unsupported map metadata."""


@dataclass(frozen=True)
class PlanResult:
    success: bool
    reason: str
    cells: tuple
    points: tuple
    path_length_m: float
    expanded_nodes: int
    start_cell: tuple
    goal_cell: tuple


class GridMap:
    def __init__(self, width, height, resolution, origin, cells, frame_id="map"):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = origin
        self.cells = cells
        self.frame_id = frame_id

    @classmethod
    def from_metadata(cls, metadata):
        if not isinstance(metadata, dict):
            raise MapError("METADATA_NOT_OBJECT")
        if metadata.get("schema") != SCHEMA:
            raise MapError("UNSUPPORTED_SCHEMA:%s" % metadata.get("schema"))
        grid = metadata.get("grid")
        if not isinstance(grid, dict):
            raise MapError("GRID_MISSING")
        if grid.get("encoding") != "rle_pairs":
            raise MapError("UNSUPPORTED_GRID_ENCODING:%s" % grid.get("encoding"))
        width = _positive_int(grid.get("width"), "GRID_WIDTH")
        height = _positive_int(grid.get("height"), "GRID_HEIGHT")
        if width * height > 2_000_000:
            raise MapError("GRID_TOO_LARGE")
        resolution = _finite_float(grid.get("resolution_m"), "GRID_RESOLUTION")
        if resolution <= 0:
            raise MapError("GRID_RESOLUTION_NOT_POSITIVE")
        origin = grid.get("origin")
        if not isinstance(origin, list) or len(origin) != 2:
            raise MapError("GRID_ORIGIN_INVALID")
        origin = (_finite_float(origin[0], "GRID_ORIGIN_X"),
                  _finite_float(origin[1], "GRID_ORIGIN_Y"))
        cells = decode_rle(grid.get("data"), width * height)
        frame = metadata.get("frame")
        if not isinstance(frame, dict) or frame.get("convention") != "ENU" or frame.get("units") != "m":
            raise MapError("FRAME_CONVENTION_INVALID")
        frame_id = grid.get("frame_id")
        if frame_id != "map" or frame.get("frame_id") != frame_id:
            raise MapError("FRAME_MISMATCH:%s" % frame_id)
        return cls(width, height, resolution, origin, cells, frame_id)

    def in_bounds(self, cell):
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def is_free(self, cell):
        return self.in_bounds(cell) and self.cells[cell[1] * self.width + cell[0]] == 0

    def world_to_cell(self, point):
        x = _finite_float(point[0], "POINT_X")
        y = _finite_float(point[1], "POINT_Y")
        x_max = self.origin[0] + self.width * self.resolution
        y_max = self.origin[1] + self.height * self.resolution
        if not (self.origin[0] <= x < x_max and self.origin[1] <= y < y_max):
            return None
        return (int(math.floor((x - self.origin[0]) / self.resolution)),
                int(math.floor((y - self.origin[1]) / self.resolution)))

    def cell_to_world(self, cell):
        if not self.in_bounds(cell):
            raise MapError("CELL_OUT_OF_BOUNDS")
        return (self.origin[0] + (cell[0] + 0.5) * self.resolution,
                self.origin[1] + (cell[1] + 0.5) * self.resolution)


def _positive_int(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise MapError("%s_INVALID" % name)
    return value


def _finite_float(value, name):
    if isinstance(value, bool):
        raise MapError("%s_INVALID" % name)
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise MapError("%s_INVALID" % name)
    if not math.isfinite(result):
        raise MapError("%s_NONFINITE" % name)
    return result


def decode_rle(runs, expected_count):
    if not isinstance(runs, list):
        raise MapError("GRID_DATA_MISSING")
    cells = bytearray()
    for index, run in enumerate(runs):
        if not isinstance(run, list) or len(run) != 2:
            raise MapError("RLE_RUN_INVALID:%d" % index)
        value, count = run
        if isinstance(value, bool) or not isinstance(value, int) or value not in (0, 1):
            raise MapError("RLE_VALUE_INVALID:%d" % index)
        if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
            raise MapError("RLE_COUNT_INVALID:%d" % index)
        if len(cells) + count > expected_count:
            raise MapError("RLE_TOO_LONG")
        cells.extend([value] * count)
    if len(cells) != expected_count:
        raise MapError("RLE_SIZE_MISMATCH:%d!=%d" % (len(cells), expected_count))
    return bytes(cells)


def load_metadata(path):
    with open(path, "rb") as stream:
        raw = stream.read()
    try:
        metadata = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise MapError("METADATA_JSON_INVALID:%s" % exc)
    return metadata, hashlib.sha256(raw).hexdigest()


def metadata_endpoints(metadata, goal_id=None, require_outdoor_goal=True):
    spawn = metadata.get("spawn")
    if not isinstance(spawn, dict) or not _point_valid(spawn.get("center")):
        raise MapError("SPAWN_MISSING_OR_INVALID")
    goals = metadata.get("goal_candidates")
    if not isinstance(goals, list) or not goals:
        raise MapError("GOAL_CANDIDATES_MISSING")
    if any(not isinstance(goal, dict) for goal in goals):
        raise MapError("GOAL_CANDIDATE_NOT_OBJECT")
    selected = None
    if goal_id is None:
        selected = next((goal for goal in goals if goal.get("outdoors") is True), goals[0])
    else:
        selected = next((goal for goal in goals if goal.get("id") == goal_id), None)
    if selected is None:
        raise MapError("GOAL_ID_NOT_FOUND:%s" % goal_id)
    # The generator's `valid` includes its BFS result. A* must establish
    # reachability itself, including for deliberately disconnected maps.
    if require_outdoor_goal and (selected.get("outdoors") is not True or
                                 selected.get("inside_obstacle") is not None):
        raise MapError("GOAL_NOT_OUTDOORS:%s" % selected.get("id"))
    if not _point_valid(selected.get("center")):
        raise MapError("GOAL_CENTER_INVALID:%s" % selected.get("id"))
    return tuple(spawn["center"]), tuple(selected["center"]), selected.get("id")


def _point_valid(point):
    return (isinstance(point, list) and len(point) == 2 and
            all(not isinstance(v, bool) and isinstance(v, (int, float)) and
                math.isfinite(float(v)) for v in point))


def plan(grid, start_world, goal_world, connectivity=8,
         prevent_corner_cutting=True, max_expansions=250000):
    if connectivity not in (4, 8):
        raise ValueError("connectivity must be 4 or 8")
    if not isinstance(prevent_corner_cutting, bool):
        raise ValueError("prevent_corner_cutting must be boolean")
    if isinstance(max_expansions, bool) or not isinstance(max_expansions, int) or max_expansions <= 0:
        raise ValueError("max_expansions must be a positive integer")
    start = grid.world_to_cell(start_world)
    goal = grid.world_to_cell(goal_world)
    if start is None:
        return _failure("START_OUT_OF_BOUNDS", start, goal)
    if goal is None:
        return _failure("GOAL_OUT_OF_BOUNDS", start, goal)
    if not grid.is_free(start):
        return _failure("START_OCCUPIED", start, goal)
    if not grid.is_free(goal):
        return _failure("GOAL_OCCUPIED", start, goal)
    if start == goal:
        point = grid.cell_to_world(start)
        return PlanResult(True, "SUCCEEDED", (start,), (point,), 0.0, 0, start, goal)

    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    if connectivity == 8:
        directions += [(1, 1), (1, -1), (-1, 1), (-1, -1)]
    diagonal_cost = math.sqrt(2.0)
    open_heap = []
    sequence = 0
    g_score = {start: 0.0}
    came_from = {}
    start_h = _heuristic(start, goal, connectivity)
    heapq.heappush(open_heap, (start_h, start_h, sequence, start))
    closed = set()
    expanded = 0

    while open_heap:
        _f, _h, _sequence, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            cells = _reconstruct(came_from, current)
            points = tuple(grid.cell_to_world(cell) for cell in cells)
            return PlanResult(True, "SUCCEEDED", cells, points,
                              g_score[current] * grid.resolution,
                              expanded, start, goal)
        if expanded >= max_expansions:
            return _failure("EXPANSION_LIMIT", start, goal, expanded)
        closed.add(current)
        expanded += 1

        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            if not grid.is_free(neighbor) or neighbor in closed:
                continue
            diagonal = dx != 0 and dy != 0
            if diagonal and prevent_corner_cutting:
                if not grid.is_free((current[0] + dx, current[1])):
                    continue
                if not grid.is_free((current[0], current[1] + dy)):
                    continue
            step = diagonal_cost if diagonal else 1.0
            tentative = g_score[current] + step
            if tentative + 1e-12 >= g_score.get(neighbor, float("inf")):
                continue
            came_from[neighbor] = current
            g_score[neighbor] = tentative
            h_score = _heuristic(neighbor, goal, connectivity)
            sequence += 1
            heapq.heappush(open_heap, (tentative + h_score, h_score, sequence, neighbor))

    return _failure("NO_PATH", start, goal, expanded)


def _heuristic(cell, goal, connectivity):
    dx, dy = abs(cell[0] - goal[0]), abs(cell[1] - goal[1])
    if connectivity == 4:
        return float(dx + dy)
    smaller, larger = min(dx, dy), max(dx, dy)
    return larger + (math.sqrt(2.0) - 1.0) * smaller


def _reconstruct(came_from, current):
    result = [current]
    while current in came_from:
        current = came_from[current]
        result.append(current)
    result.reverse()
    return tuple(result)


def _failure(reason, start, goal, expanded=0):
    return PlanResult(False, reason, (), (), 0.0, expanded, start, goal)


def render_ascii(grid, result):
    if grid.width > 120 or grid.height > 120:
        raise ValueError("ASCII_GRID_TOO_LARGE")
    route = set(result.cells)
    rows = []
    for iy in range(grid.height - 1, -1, -1):
        row = []
        for ix in range(grid.width):
            cell = (ix, iy)
            char = "." if grid.is_free(cell) else "#"
            if cell in route:
                char = "*"
            if cell == result.start_cell:
                char = "S"
            if cell == result.goal_cell:
                char = "G"
            row.append(char)
        rows.append("".join(row))
    return "\n".join(rows) + "\n"
