#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Deterministic Gazebo Classic training-city generator core.

Scope
-----
This module only produces an environment (SDF world) plus machine-readable
metadata. It never talks to ROS, never publishes MAVROS setpoints and never
spawns a vehicle. Flight commands remain the exclusive responsibility of the
single controller node in ``robocup_navigation``.

Provenance
----------
Official rules (2026, sections 2.4 / 2.5) fix: a 200 m x 100 m urban area,
buildings / lamp posts / similar objects, 6 UAVs below 6 m, 6 outdoor targets,
house and lamp positions randomised before each attempt. Everything else here
(road width, block sizes, building counts and heights, lamp spacing, spawn and
goal placement, the x[-100,100] / y[-50,50] convention) is a team training
assumption and must not be presented as an official parameter.

Implementation notes
--------------------
* Standard library only, except PyYAML for the config file.
* Deterministic: the same seed reproduces byte-identical ``.world`` and
  ``.json`` output (no timestamps unless ``--emit-timestamp`` is used).
"""

import json
import math
import os
import random
import sys
import xml.etree.ElementTree as ElementTree
from collections import deque

try:
    import yaml
except ImportError:  # pragma: no cover - the container image ships PyYAML
    yaml = None

GENERATOR_VERSION = "1.0.0"
SCHEMA_VERSION = "robocup_training_worlds/metadata/v1"
SDF_VERSION = "1.5"

UNIT_CATEGORIES = (
    "empty",
    "single_wall",
    "wall_with_gap",
    "u_shape",
    "narrow_corridor",
    "blocked",
    "no_path",
    "goal_in_obstacle",
)

CITY_PRESETS = ("unit", "small", "full")

EPS = 1e-9
ROUND_DIGITS = 6
# Walls that must seal a passage stop this far short of the neighbouring wall.
# The gap is impassable once the safety margin is applied, but the two solids
# never touch, so the strict "no overlapping obstacles" check stays meaningful.
SEAL_GAP = 0.05


# ---------------------------------------------------------------------------
# small helpers
# ---------------------------------------------------------------------------

def _r(value, digits=ROUND_DIGITS):
    return round(float(value) + 0.0, digits)


def fmt(value):
    """Deterministic, human-readable SDF number."""
    v = float(value)
    if v == 0.0:
        return "0"
    text = "%.6f" % v
    text = text.rstrip("0").rstrip(".")
    return text if text not in ("", "-", "-0") else "0"


def _pose(values):
    return " ".join(fmt(v) for v in values)


def deep_merge(base, override):
    merged = dict(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


# ---------------------------------------------------------------------------
# 2-D geometry (axis-aligned / rotated convex polygons and circles)
# ---------------------------------------------------------------------------

def rect_polygon(center_x, center_y, size_x, size_y, yaw=0.0):
    dx, dy = float(size_x) / 2.0, float(size_y) / 2.0
    corners = [(-dx, -dy), (dx, -dy), (dx, dy), (-dx, dy)]
    c, s = math.cos(yaw), math.sin(yaw)
    return [(center_x + x * c - y * s, center_y + x * s + y * c) for x, y in corners]


def point_segment_distance(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom <= EPS:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / denom
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def point_in_polygon(px, py, polygon):
    inside = False
    count = len(polygon)
    for i in range(count):
        ax, ay = polygon[i]
        bx, by = polygon[(i + 1) % count]
        if (ay > py) != (by > py):
            crossing = ax + (py - ay) * (bx - ax) / (by - ay)
            if px < crossing:
                inside = not inside
    return inside


def polygon_signed_distance(px, py, polygon):
    """Negative inside the polygon, positive outside."""
    count = len(polygon)
    best = min(
        point_segment_distance(px, py, polygon[i][0], polygon[i][1],
                               polygon[(i + 1) % count][0], polygon[(i + 1) % count][1])
        for i in range(count)
    )
    return -best if point_in_polygon(px, py, polygon) else best


def polygon_bbox(polygon):
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    return min(xs), min(ys), max(xs), max(ys)


def _project(polygon, axis_x, axis_y):
    values = [x * axis_x + y * axis_y for x, y in polygon]
    return min(values), max(values)


def polygons_overlap(a, b):
    """Separating-axis test for convex polygons."""
    for polygon in (a, b):
        count = len(polygon)
        for i in range(count):
            ax, ay = polygon[i]
            bx, by = polygon[(i + 1) % count]
            nx, ny = -(by - ay), (bx - ax)
            if nx == 0.0 and ny == 0.0:
                continue
            min_a, max_a = _project(a, nx, ny)
            min_b, max_b = _project(b, nx, ny)
            if max_a < min_b or max_b < min_a:
                return False
    return True


def polygon_distance(a, b):
    """0.0 when the convex polygons touch or overlap."""
    if polygons_overlap(a, b):
        return 0.0
    best = float("inf")
    for source, target in ((a, b), (b, a)):
        count = len(target)
        for px, py in source:
            for i in range(count):
                ax, ay = target[i]
                bx, by = target[(i + 1) % count]
                best = min(best, point_segment_distance(px, py, ax, ay, bx, by))
    return best


# ---------------------------------------------------------------------------
# obstacles
# ---------------------------------------------------------------------------

class Obstacle(object):
    """A static world obstacle with a 2-D footprint and a vertical extent."""

    __slots__ = ("id", "type", "shape", "center", "size", "radius", "yaw",
                 "height", "z_min", "blocking", "group", "material", "polygon")

    def __init__(self, oid, otype, shape, center, size=None, yaw=0.0, radius=None,
                 height=1.0, z_min=0.0, blocking=True, group=None, material=None):
        if shape not in ("box", "cylinder"):
            raise ValueError("unsupported obstacle shape: %s" % shape)
        self.id = oid
        self.type = otype
        self.shape = shape
        self.center = (_r(center[0]), _r(center[1]))
        self.yaw = _r(yaw)
        self.height = _r(height)
        self.z_min = _r(z_min)
        self.blocking = bool(blocking)
        self.group = group
        self.material = tuple(float(v) for v in (material or (0.7, 0.7, 0.7, 1.0)))
        if shape == "box":
            if size is None:
                raise ValueError("box obstacles require a size")
            self.size = (_r(size[0]), _r(size[1]))
            self.radius = None
            self.polygon = rect_polygon(center[0], center[1], size[0], size[1], yaw)
        else:
            if radius is None:
                raise ValueError("cylinder obstacles require a radius")
            self.size = None
            self.radius = _r(radius)
            self.polygon = None
        if self.height <= 0:
            raise ValueError("obstacle height must be positive")

    # -- constructors ------------------------------------------------------

    @classmethod
    def box(cls, oid, otype, center, size, height, yaw=0.0, z_min=0.0,
            blocking=True, group=None, material=None):
        return cls(oid, otype, "box", center, size=size, yaw=yaw, height=height,
                   z_min=z_min, blocking=blocking, group=group, material=material)

    @classmethod
    def cylinder(cls, oid, otype, center, radius, height, z_min=0.0,
                 blocking=True, group=None, material=None):
        return cls(oid, otype, "cylinder", center, radius=radius, height=height,
                   z_min=z_min, blocking=blocking, group=group, material=material)

    # -- geometry ----------------------------------------------------------

    @property
    def z_max(self):
        return _r(self.z_min + self.height)

    def bbox(self):
        if self.polygon is not None:
            return polygon_bbox(self.polygon)
        return (self.center[0] - self.radius, self.center[1] - self.radius,
                self.center[0] + self.radius, self.center[1] + self.radius)

    def signed_distance(self, x, y):
        if self.polygon is not None:
            return polygon_signed_distance(x, y, self.polygon)
        return math.hypot(x - self.center[0], y - self.center[1]) - self.radius

    def clearance_at(self, x, y):
        """Distance to the obstacle surface; 0 when the point is inside."""
        return max(0.0, self.signed_distance(x, y))

    def distance_to(self, other):
        if self.polygon is not None and other.polygon is not None:
            return polygon_distance(self.polygon, other.polygon)
        if self.polygon is None and other.polygon is None:
            return max(0.0, math.hypot(self.center[0] - other.center[0],
                                       self.center[1] - other.center[1])
                       - self.radius - other.radius)
        polygon, circle = (self.polygon, other) if self.polygon is not None else (other.polygon, self)
        distance = polygon_signed_distance(circle.center[0], circle.center[1], polygon)
        if distance <= 0.0:
            return 0.0
        return max(0.0, distance - circle.radius)

    # -- serialisation -----------------------------------------------------

    def to_metadata(self):
        x0, y0, x1, y1 = self.bbox()
        entry = {
            "id": self.id,
            "type": self.type,
            "shape": self.shape,
            "center": [self.center[0], self.center[1]],
            "yaw": self.yaw,
            "height": self.height,
            "z_min": self.z_min,
            "z_max": self.z_max,
            "blocking": self.blocking,
            "group": self.group,
            "bbox": [_r(x0), _r(y0), _r(x1), _r(y1)],
        }
        if self.polygon is not None:
            entry["size"] = [self.size[0], self.size[1]]
            entry["footprint"] = {
                "kind": "polygon",
                "points": [[_r(x), _r(y)] for x, y in self.polygon],
            }
        else:
            entry["radius"] = self.radius
            entry["footprint"] = {
                "kind": "circle",
                "center": [self.center[0], self.center[1]],
                "radius": self.radius,
            }
        return entry

    @classmethod
    def from_metadata(cls, entry):
        material = None
        if entry["shape"] == "box":
            return cls.box(entry["id"], entry["type"], entry["center"], entry["size"],
                           entry["height"], yaw=entry.get("yaw", 0.0),
                           z_min=entry.get("z_min", 0.0), blocking=entry.get("blocking", True),
                           group=entry.get("group"), material=material)
        return cls.cylinder(entry["id"], entry["type"], entry["center"], entry["radius"],
                            entry["height"], z_min=entry.get("z_min", 0.0),
                            blocking=entry.get("blocking", True), group=entry.get("group"),
                            material=material)


# ---------------------------------------------------------------------------
# occupancy grid + independent BFS connectivity check
# ---------------------------------------------------------------------------

class OccupancyGrid(object):
    """Inflated 2-D grid used only for validation and for planner input.

    The connectivity check is deliberately a plain 4-connected flood fill on
    this grid - it is NOT the A* under test, so a planner bug cannot hide a
    map bug.
    """

    def __init__(self, bounds, resolution):
        self.resolution = float(resolution)
        self.x_min, self.y_min, self.x_max, self.y_max = [float(v) for v in bounds]
        self.width = max(1, int(round((self.x_max - self.x_min) / self.resolution)))
        self.height = max(1, int(round((self.y_max - self.y_min) / self.resolution)))
        self.cells = bytearray(self.width * self.height)

    # -- indexing ----------------------------------------------------------

    def center(self, ix, iy):
        return (self.x_min + (ix + 0.5) * self.resolution,
                self.y_min + (iy + 0.5) * self.resolution)

    def index_of(self, x, y):
        ix = int(math.floor((x - self.x_min) / self.resolution))
        iy = int(math.floor((y - self.y_min) / self.resolution))
        ix = 0 if ix < 0 else (self.width - 1 if ix >= self.width else ix)
        iy = 0 if iy < 0 else (self.height - 1 if iy >= self.height else iy)
        return ix, iy

    def is_free(self, ix, iy):
        return self.cells[iy * self.width + ix] == 0

    def is_inside(self, ix, iy):
        return 0 <= ix < self.width and 0 <= iy < self.height

    # -- rasterisation -----------------------------------------------------

    def mark(self, obstacle, margin):
        reach = float(margin) + self.resolution
        x0, y0, x1, y1 = obstacle.bbox()
        ix0, iy0 = self.index_of(x0 - reach, y0 - reach)
        ix1, iy1 = self.index_of(x1 + reach, y1 + reach)
        for iy in range(iy0, iy1 + 1):
            py = self.y_min + (iy + 0.5) * self.resolution
            base = iy * self.width
            for ix in range(ix0, ix1 + 1):
                px = self.x_min + (ix + 0.5) * self.resolution
                if obstacle.signed_distance(px, py) <= margin:
                    self.cells[base + ix] = 1

    # -- analysis ----------------------------------------------------------

    def flood_fill(self, ix, iy):
        """4-connected flood fill; diagonals must not squeeze through corners."""
        visited = bytearray(self.width * self.height)
        start = iy * self.width + ix
        if not self.is_inside(ix, iy) or self.cells[start]:
            return visited
        visited[start] = 1
        queue = deque([(ix, iy)])
        while queue:
            cx, cy = queue.popleft()
            for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                if not self.is_inside(nx, ny):
                    continue
                index = ny * self.width + nx
                if visited[index] or self.cells[index]:
                    continue
                visited[index] = 1
                queue.append((nx, ny))
        return visited

    def free_ratio(self):
        free = self.cells.count(0)
        return free / float(self.width * self.height)

    # -- encoding ----------------------------------------------------------

    def rle(self):
        runs = []
        previous = int(self.cells[0])
        count = 1
        for value in self.cells[1:]:
            value = int(value)
            if value == previous:
                count += 1
            else:
                runs.append([previous, count])
                previous, count = value, 1
        runs.append([previous, count])
        return runs


def decode_rle(runs, width, height):
    cells = bytearray(width * height)
    index = 0
    for value, count in runs:
        for _ in range(count):
            if index >= width * height:
                raise ValueError("run-length data exceeds the declared grid size")
            cells[index] = int(value)
            index += 1
    if index != width * height:
        raise ValueError("run-length data does not fill the declared grid")
    return cells


# ---------------------------------------------------------------------------
# configuration
# ---------------------------------------------------------------------------

def default_config_path():
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "config", "training_city.yaml"))


def load_config(path=None):
    path = path or default_config_path()
    if yaml is None:
        raise RuntimeError("PyYAML is required to read %s" % path)
    with open(path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("training city configuration must be a mapping")
    return config


def preset_config(config, preset):
    if preset not in config.get("presets", {}):
        raise ValueError("unknown preset %r (available: %s)"
                         % (preset, ", ".join(sorted(config["presets"]))))
    return deep_merge(config["presets"][preset], {"safety": config["safety"]})


# ---------------------------------------------------------------------------
# generation: unit categories
# ---------------------------------------------------------------------------

def _unit_goal(bounds):
    x_min, _, x_max, _ = bounds
    width = x_max - x_min
    return (x_min + 0.8 * width, (bounds[1] + bounds[3]) / 2.0)


def _unit_structure(category, rng, bounds, preset, goal_center, material_wall):
    """Unit-preset obstacles. Fixed categories, coordinates vary with the seed.

    Walls stop `SEAL_GAP` short of the boundary wall instead of touching it:
    a touching pair would be reported as an overlap, while a 5 cm gap is
    already impassable once the safety margin is applied.
    """
    x_min, y_min, x_max, y_max = bounds
    width, height = x_max - x_min, y_max - y_min
    cx, cy = (x_min + x_max) / 2.0, (y_min + y_max) / 2.0
    thickness = float(preset["wall_thickness_m"])
    wall_height = float(preset["obstacle_height_m"])
    boundary_thickness = float(preset["boundary"]["thickness_m"])
    lo = y_min + boundary_thickness + SEAL_GAP
    hi = y_max - boundary_thickness - SEAL_GAP
    right = x_max - boundary_thickness - SEAL_GAP
    group = "unit_structure"
    obstacles = []

    def wall(oid, center, size):
        return Obstacle.box(oid, "wall", center, size, wall_height,
                            group=group, material=material_wall)

    if category == "empty":
        return obstacles

    if category in ("single_wall", "goal_in_obstacle"):
        wall_x = cx + rng.uniform(-0.08, 0.08) * width
        length = rng.uniform(0.45, 0.62) * (hi - lo)
        anchored_low = rng.random() < 0.5
        center_y = lo + length / 2.0 if anchored_low else hi - length / 2.0
        obstacles.append(wall("unit_wall_0000", (wall_x, center_y), (thickness, length)))
        if category == "goal_in_obstacle":
            # Negative case: the goal candidate sits inside a solid block.
            obstacles.append(Obstacle.box("unit_block_0000", "building", goal_center,
                                          (2.0, 2.0), wall_height, group=group,
                                          material=material_wall))
        return obstacles

    if category == "wall_with_gap":
        wall_x = cx + rng.uniform(-0.08, 0.08) * width
        gap = rng.uniform(1.8, 2.6)
        gap_y = cy + rng.uniform(-0.2, 0.2) * height
        lower_end, upper_start = gap_y - gap / 2.0, gap_y + gap / 2.0
        if lower_end - lo > 0.3:
            obstacles.append(wall("unit_wall_0000", (wall_x, (lo + lower_end) / 2.0),
                                  (thickness, lower_end - lo)))
        if hi - upper_start > 0.3:
            obstacles.append(wall("unit_wall_0001", (wall_x, (upper_start + hi) / 2.0),
                                  (thickness, hi - upper_start)))
        return obstacles

    if category == "u_shape":
        # The U is centred on the goal and opens towards the spawn side (-x).
        max_half = right - thickness - goal_center[0]
        half = max(0.9, min(rng.uniform(1.0, 1.4), max_half))
        ux = goal_center[0]
        arm_length = 2.0 * half + thickness
        obstacles.append(wall("unit_wall_0000", (ux + half + thickness / 2.0, cy),
                              (thickness, arm_length)))
        obstacles.append(wall("unit_wall_0001", (ux, cy + half + thickness / 2.0),
                              (arm_length, thickness)))
        obstacles.append(wall("unit_wall_0002", (ux, cy - half - thickness / 2.0),
                              (arm_length, thickness)))
        return obstacles

    if category == "narrow_corridor":
        half = rng.uniform(0.9, 1.3)
        start_x = cx - rng.uniform(0.5, 1.5)
        span = right - start_x
        center_x = (start_x + right) / 2.0
        obstacles.append(wall("unit_wall_0000", (center_x, cy + half + thickness / 2.0),
                              (span, thickness)))
        obstacles.append(wall("unit_wall_0001", (center_x, cy - half - thickness / 2.0),
                              (span, thickness)))
        return obstacles

    if category in ("blocked", "no_path"):
        wall_x = cx + rng.uniform(-0.08, 0.08) * width
        span = hi - lo
        obstacles.append(wall("unit_wall_0000", (wall_x, cy), (thickness, span)))
        return obstacles

    raise ValueError("unknown unit category: %s" % category)


# ---------------------------------------------------------------------------
# generation: random city (small / full)
# ---------------------------------------------------------------------------

def _axis_layout(lo, hi, border, road_width, target):
    """Split one axis into alternating blocks and roads; returns (blocks, roads)."""
    span = (hi - border) - (lo + border)
    count = max(1, int(round(span / (target + road_width))))
    block = (span - count * road_width) / float(count)
    blocks, roads = [], []
    position = lo + border
    for _ in range(count):
        blocks.append((position, position + block))
        position += block
        roads.append((position, position + road_width))
        position += road_width
    roads.pop()  # the trailing strip is the far border road
    strips = [(lo, lo + border)] + roads + [(hi - border, hi)]
    return blocks, strips, block


def _city_layout(rng, bounds, preset, materials):
    x_min, y_min, x_max, y_max = bounds
    border = float(preset["border_road_m"])
    road_width = float(preset["road_width_m"])
    x_blocks, x_strips, block_x = _axis_layout(x_min, x_max, border, road_width,
                                               float(preset["block_target_x_m"]))
    y_blocks, y_strips, block_y = _axis_layout(y_min, y_max, border, road_width,
                                               float(preset["block_target_y_m"]))

    obstacles = []
    buildings = []
    counter = [0]

    def next_id(prefix):
        counter[0] += 1
        return "%s_%04d" % (prefix, counter[0])

    # --- buildings inside the blocks -------------------------------------
    inset = float(preset["building_inset_m"])
    gap = float(preset["building_gap_m"])
    low_count, high_count = preset["buildings_per_block"]
    size_x = preset["building_size_x_m"]
    size_y = preset["building_size_y_m"]
    heights = preset["building_height_m"]
    for by0, by1 in y_blocks:
        for bx0, bx1 in x_blocks:
            ax0, ax1 = bx0 + inset, bx1 - inset
            ay0, ay1 = by0 + inset, by1 - inset
            if ax1 - ax0 < 1.0 or ay1 - ay0 < 1.0:
                continue
            wanted = rng.randint(int(low_count), int(high_count))
            for _ in range(wanted):
                for _attempt in range(40):
                    wx = min(rng.uniform(size_x[0], size_x[1]), ax1 - ax0)
                    wy = min(rng.uniform(size_y[0], size_y[1]), ay1 - ay0)
                    if wx < 1.0 or wy < 1.0:
                        break
                    center_x = rng.uniform(ax0 + wx / 2.0, ax1 - wx / 2.0)
                    center_y = rng.uniform(ay0 + wy / 2.0, ay1 - wy / 2.0)
                    candidate = Obstacle.box(next_id("building"), "building",
                                             (center_x, center_y), (wx, wy),
                                             rng.uniform(heights[0], heights[1]),
                                             material=materials["building"])
                    if all(candidate.distance_to(other) >= gap for other in buildings):
                        buildings.append(candidate)
                        break
    obstacles.extend(buildings)

    # --- lamp posts along the road strips --------------------------------
    spacing = float(preset["lamp_spacing_m"])
    offset = float(preset["lamp_offset_from_curb_m"])
    lamp_radius = float(preset["lamp_radius_m"])
    lamp_height = float(preset["lamp_height_m"])
    lamp_clearance = float(preset["lamp_clearance_m"])
    lamps = []

    def try_lamp(px, py):
        candidate = Obstacle.cylinder(next_id("lamp_post"), "lamp_post", (px, py),
                                      lamp_radius, lamp_height,
                                      material=materials["lamp_post"])
        for building in buildings:
            if candidate.distance_to(building) < lamp_clearance:
                return
        for placed in lamps:
            if candidate.distance_to(placed) < lamp_radius:
                return
        lamps.append(candidate)

    for sx0, sx1 in x_strips:
        centre = (sx0 + sx1) / 2.0
        sides = [centre] if (sx1 - sx0) < 2.0 * offset else [centre - offset, centre + offset]
        y = y_min + border + rng.uniform(0.0, spacing)
        while y <= y_max - border:
            for side in sides:
                try_lamp(side, y)
            y += spacing
    for sy0, sy1 in y_strips:
        centre = (sy0 + sy1) / 2.0
        sides = [centre] if (sy1 - sy0) < 2.0 * offset else [centre - offset, centre + offset]
        x = x_min + border + rng.uniform(0.0, spacing)
        while x <= x_max - border:
            for side in sides:
                try_lamp(x, side)
            x += spacing
    obstacles.extend(lamps)

    return obstacles, x_strips, y_strips


# ---------------------------------------------------------------------------
# generation: shared pieces
# ---------------------------------------------------------------------------

def _boundary_walls(bounds, boundary, materials):
    x_min, y_min, x_max, y_max = bounds
    thickness = float(boundary["thickness_m"])
    height = float(boundary["height_m"])
    half = thickness / 2.0
    material = materials["boundary_wall"]
    specs = (
        ("boundary_north", ((x_min + x_max) / 2.0, y_max - half), (x_max - x_min, thickness)),
        ("boundary_south", ((x_min + x_max) / 2.0, y_min + half), (x_max - x_min, thickness)),
        ("boundary_east", (x_max - half, (y_min + y_max) / 2.0), (thickness, y_max - y_min)),
        ("boundary_west", (x_min + half, (y_min + y_max) / 2.0), (thickness, y_max - y_min)),
    )
    return [Obstacle.box(oid, "boundary_wall", center, size, height,
                         group="boundary", material=material) for oid, center, size in specs]


def _clearance(x, y, obstacles, limit):
    """Minimum distance to any blocking obstacle, early-exited below `limit`."""
    best = float("inf")
    for obstacle in obstacles:
        if not obstacle.blocking:
            continue
        x0, y0, x1, y1 = obstacle.bbox()
        if x < x0 - best or x > x1 + best or y < y0 - best or y > y1 + best:
            continue
        distance = obstacle.clearance_at(x, y)
        if distance <= 0.0:
            return 0.0
        if distance < best:
            best = distance
            if best < limit:
                return best
    return best


def _candidate_stride(grid, target=3000):
    """Subsample free cells so the clearance scan stays cheap on large maps."""
    total = grid.width * grid.height
    step = int(round(math.sqrt(total / float(target))))
    return max(1, step)


def _free_candidates(grid, obstacles, stride):
    result = []
    for iy in range(0, grid.height, stride):
        for ix in range(0, grid.width, stride):
            if grid.is_free(ix, iy):
                result.append(grid.center(ix, iy))
    return result


def _choose_spawn(rng, preset, grid, obstacles, safety, road_polygons):
    spawn = preset["spawn"]
    radius = float(spawn.get("radius_m", 1.0))
    if spawn.get("mode") == "fixed":
        center = (float(spawn["center"][0]), float(spawn["center"][1]))
        clearance = _clearance(center[0], center[1], obstacles, safety["spawn_clearance_m"])
        return {"mode": "fixed", "center": [_r(center[0]), _r(center[1])],
                "radius_m": _r(radius), "clearance_m": _r(clearance), "yaw": 0.0}

    stride = _candidate_stride(grid)
    candidates = _free_candidates(grid, obstacles, stride)
    on_road = [point for point in candidates
               if any(polygon_signed_distance(point[0], point[1], polygon) <= 0.0
                      for polygon in road_polygons)]
    pools = [on_road, candidates] if on_road else [candidates]
    for pool in pools:
        usable = [point for point in pool
                  if _clearance(point[0], point[1], obstacles, safety["spawn_clearance_m"])
                  >= safety["spawn_clearance_m"] - 1e-6]
        if usable:
            usable.sort()
            chosen = usable[rng.randrange(len(usable))]
            clearance = _clearance(chosen[0], chosen[1], obstacles, 0.0)
            return {"mode": "auto", "center": [_r(chosen[0]), _r(chosen[1])],
                    "radius_m": _r(radius), "clearance_m": _r(clearance), "yaw": 0.0}
    return {"mode": "auto", "center": [0.0, 0.0], "radius_m": _r(radius),
            "clearance_m": 0.0, "yaw": 0.0}


def _choose_goals(rng, preset, grid, obstacles, safety, spawn, count):
    goals = []
    if count <= 0:
        return goals
    stride = _candidate_stride(grid)
    candidates = _free_candidates(grid, obstacles, stride)
    rng.shuffle(candidates)
    min_spawn = float(safety["goal_min_spawn_distance_m"])
    min_separation = float(safety["goal_min_separation_m"])
    radius = float(safety["goal_clearance_m"])
    for point in candidates:
        if len(goals) >= count:
            break
        if _clearance(point[0], point[1], obstacles, radius) < radius - 1e-6:
            continue
        if math.hypot(point[0] - spawn["center"][0], point[1] - spawn["center"][1]) < min_spawn:
            continue
        if any(math.hypot(point[0] - goal[0], point[1] - goal[1]) < min_separation
               for goal in goals):
            continue
        goals.append((point[0], point[1]))
    if not goals:
        goals.append((spawn["center"][0], spawn["center"][1]))
    return goals


def _goal_entries(grid, obstacles, goals, reachable_flags, safety, expect_goal_valid):
    entries = []
    for index, ((x, y), reachable) in enumerate(zip(goals, reachable_flags)):
        clearance = _clearance(x, y, obstacles, 0.0)
        outdoors = clearance >= float(safety["goal_clearance_m"]) - 1e-6
        inside = None
        for obstacle in obstacles:
            if obstacle.blocking and obstacle.signed_distance(x, y) < 0.0:
                inside = obstacle.id
                break
        entries.append({
            "id": "goal_%04d" % index,
            "center": [_r(x), _r(y)],
            "radius_m": _r(safety["goal_clearance_m"]),
            "clearance_m": _r(clearance),
            "outdoors": bool(outdoors),
            "inside_obstacle": inside,
            "valid": bool(outdoors and reachable),
            "reachable": bool(reachable),
            "expected_valid": bool(expect_goal_valid),
        })
    return entries


# ---------------------------------------------------------------------------
# validation
# ---------------------------------------------------------------------------

def _connectivity(grid, spawn_cell, goal_cells):
    visited = grid.flood_fill(*spawn_cell)
    flags = []
    for ix, iy in goal_cells:
        if not grid.is_inside(ix, iy):
            flags.append(False)
        else:
            flags.append(bool(visited[iy * grid.width + ix]) and grid.is_free(ix, iy))
    return visited, flags


def validate_metadata(metadata, rebuild_grid=True):
    """Re-check a metadata document. Returns (problems, checks)."""
    problems = []
    checks = {}
    bounds = metadata["bounds"]
    x_min, y_min, x_max, y_max = (float(bounds["x_min"]), float(bounds["y_min"]),
                                  float(bounds["x_max"]), float(bounds["y_max"]))
    safety = metadata["safety"]
    margin = float(safety["safety_margin_m"])
    obstacles = [Obstacle.from_metadata(entry) for entry in metadata["obstacles"]]
    expectations = metadata.get("expectations", {})
    expect_reachable = bool(expectations.get("all_goals_reachable", True))
    expect_goal_valid = bool(expectations.get("goals_valid", True))

    # 1. margins must be sane
    if margin < float(safety["robot_radius_m"]):
        problems.append("SAFETY_MARGIN_SMALLER_THAN_ROBOT_RADIUS")

    # 2. every obstacle stays inside the bounds
    outside = []
    for obstacle in obstacles:
        x0, y0, x1, y1 = obstacle.bbox()
        if x0 < x_min - 1e-6 or y0 < y_min - 1e-6 or x1 > x_max + 1e-6 or y1 > y_max + 1e-6:
            outside.append(obstacle.id)
    checks["obstacles_within_bounds"] = not outside
    if outside:
        problems.append("OBSTACLE_OUTSIDE_BOUNDS:" + ",".join(sorted(outside)))

    # 3. no two obstacles overlap (same group may touch: boundary corners,
    #    sealed U-shape joints)
    overlaps = []
    for i in range(len(obstacles)):
        for j in range(i + 1, len(obstacles)):
            left, right = obstacles[i], obstacles[j]
            if left.group is not None and left.group == right.group:
                continue
            if left.distance_to(right) <= 1e-6:
                overlaps.append("%s~%s" % (left.id, right.id))
    checks["no_obstacle_overlap"] = not overlaps
    if overlaps:
        problems.append("OBSTACLE_OVERLAP:" + ",".join(sorted(overlaps)[:5]))

    # 4. obstacles cover the planning altitude band
    altitude = float(metadata["planning_altitude_m"])
    required_top = altitude + float(safety["vertical_margin_m"])
    too_low = []
    for obstacle in obstacles:
        if not obstacle.blocking:
            continue
        if obstacle.z_min > 1e-6 or obstacle.z_max < required_top - 1e-6:
            too_low.append(obstacle.id)
    checks["obstacles_cover_altitude_band"] = not too_low
    if too_low:
        problems.append("OBSTACLE_TOO_LOW:" + ",".join(sorted(too_low)[:5]))

    # 5. spawn safety
    spawn = metadata["spawn"]
    spawn_clearance = _clearance(spawn["center"][0], spawn["center"][1], obstacles, 0.0)
    required_spawn = float(safety["spawn_clearance_m"])
    checks["spawn_clear_of_obstacles"] = spawn_clearance >= required_spawn - 1e-6
    if not checks["spawn_clear_of_obstacles"]:
        problems.append("SPAWN_TOO_CLOSE:%s" % fmt(spawn_clearance))

    # 6. goal candidates outdoors. The stored flags are recomputed from the
    #    geometry instead of being trusted, so a tampered metadata document
    #    cannot claim an indoor goal is outdoors.
    indoor = []
    required_goal = float(safety["goal_clearance_m"])
    for goal in metadata["goal_candidates"]:
        x, y = goal["center"]
        clearance = _clearance(x, y, obstacles, 0.0)
        computed_outdoors = clearance >= required_goal - 1e-6
        if abs(clearance - float(goal["clearance_m"])) > 1e-5:
            problems.append("GOAL_CLEARANCE_MISMATCH:" + goal["id"])
        if computed_outdoors != bool(goal["outdoors"]):
            problems.append("GOAL_OUTDOOR_FLAG_MISMATCH:" + goal["id"])
        stored_inside = None
        for obstacle in obstacles:
            if obstacle.blocking and obstacle.signed_distance(x, y) < 0.0:
                stored_inside = obstacle.id
                break
        if stored_inside != goal.get("inside_obstacle"):
            problems.append("GOAL_INSIDE_FLAG_MISMATCH:" + goal["id"])
        if not computed_outdoors:
            indoor.append(goal["id"])
    checks["goal_candidates_outdoors"] = (not indoor) if expect_goal_valid else True
    if expect_goal_valid and indoor:
        problems.append("GOAL_INSIDE_OBSTACLE:" + ",".join(indoor))
    if not expect_goal_valid and not indoor:
        problems.append("NEGATIVE_CASE_INVALID:goal_was_expected_inside_an_obstacle")

    # 7. connectivity via an independent BFS on a rebuilt inflated grid
    grid_info = metadata["grid"]
    grid = OccupancyGrid((x_min, y_min, x_max, y_max), float(grid_info["resolution_m"]))
    if rebuild_grid:
        for obstacle in obstacles:
            if obstacle.blocking:
                grid.mark(obstacle, margin)
        rebuilt = grid.cells
        if "data" in grid_info:
            stored = decode_rle(grid_info["data"], grid_info["width"], grid_info["height"])
            checks["grid_matches_metadata"] = bytes(stored) == bytes(rebuilt)
            if not checks["grid_matches_metadata"]:
                problems.append("GRID_MISMATCH")
    else:
        stored = decode_rle(grid_info["data"], grid_info["width"], grid_info["height"])
        grid.cells = stored
    checks["free_cell_ratio"] = _r(grid.free_ratio())
    if grid.free_ratio() < float(safety["min_free_cell_ratio"]):
        problems.append("TOO_FEW_FREE_CELLS:%s" % fmt(grid.free_ratio()))

    spawn_cell = grid.index_of(spawn["center"][0], spawn["center"][1])
    goal_cells = [grid.index_of(goal["center"][0], goal["center"][1])
                  for goal in metadata["goal_candidates"]]
    _visited, flags = _connectivity(grid, spawn_cell, goal_cells)
    for goal, flag in zip(metadata["goal_candidates"], flags):
        if goal["reachable"] != flag:
            problems.append("GOAL_REACHABILITY_MISMATCH:" + goal["id"])
    all_reachable = all(flags) if flags else False
    checks["all_goals_reachable"] = all_reachable
    checks["expect_all_goals_reachable"] = expect_reachable
    if all_reachable != expect_reachable:
        problems.append("CONNECTIVITY_MISMATCH:expected_reachable=%s observed=%s"
                        % (expect_reachable, all_reachable))
    return problems, checks


# ---------------------------------------------------------------------------
# SDF rendering
# ---------------------------------------------------------------------------

def _material_lines(material, indent):
    r, g, b, a = material
    pad = " " * indent
    return [
        "%s<material>" % pad,
        "%s  <ambient>%s</ambient>" % (pad, _pose((r * 0.45, g * 0.45, b * 0.45, a))),
        "%s  <diffuse>%s</diffuse>" % (pad, _pose((r, g, b, a))),
        "%s  <specular>0.04 0.04 0.04 1</specular>" % pad,
        "%s  <emissive>0 0 0 1</emissive>" % pad,
        "%s</material>" % pad,
    ]


def _inertia_lines(shape, size, radius, height, indent):
    pad = " " * indent
    mass = 1.0
    if shape == "box":
        sx, sy = size
        ixx = mass / 12.0 * (sy * sy + height * height)
        iyy = mass / 12.0 * (sx * sx + height * height)
        izz = mass / 12.0 * (sx * sx + sy * sy)
    else:
        ixx = iyy = mass / 12.0 * (3.0 * radius * radius + height * height)
        izz = mass * 0.5 * radius * radius
    return [
        "%s<inertial>" % pad,
        "%s  <mass>%s</mass>" % (pad, fmt(mass)),
        "%s  <inertia>" % pad,
        "%s    <ixx>%s</ixx><ixy>0</ixy><ixz>0</ixz>" % (pad, fmt(ixx)),
        "%s    <iyy>%s</iyy><iyz>0</iyz>" % (pad, fmt(iyy)),
        "%s    <izz>%s</izz>" % (pad, fmt(izz)),
        "%s  </inertia>" % pad,
        "%s</inertial>" % pad,
    ]


def _geometry_lines(obstacle, indent):
    pad = " " * indent
    if obstacle.shape == "box":
        body = "%s  <box><size>%s</size></box>" % (
            pad, _pose((obstacle.size[0], obstacle.size[1], obstacle.height)))
    else:
        body = "%s  <cylinder><radius>%s</radius><length>%s</length></cylinder>" % (
            pad, fmt(obstacle.radius), fmt(obstacle.height))
    return ["%s<geometry>" % pad, body, "%s</geometry>" % pad]


def _render_obstacle(obstacle, indent=4):
    pad = " " * indent
    lines = ["%s<model name=\"%s\">" % (pad, obstacle.id),
             "%s  <static>true</static>" % pad,
             "%s  <pose>%s</pose>" % (pad, _pose((obstacle.center[0], obstacle.center[1],
                                                  0.0, 0.0, 0.0, obstacle.yaw))),
             "%s  <link name=\"link\">" % pad]
    lines.append("%s    <pose>%s</pose>" % (pad, _pose((0.0, 0.0, obstacle.z_min + obstacle.height / 2.0,
                                                        0.0, 0.0, 0.0))))
    for tag in ("collision", "visual"):
        lines.append("%s    <%s name=\"%s\">" % (pad, tag, tag))
        lines.extend(_geometry_lines(obstacle, indent + 6))
        if tag == "visual":
            lines.extend(_material_lines(obstacle.material, indent + 6))
        else:
            lines.append("%s      <surface>" % pad)
            lines.append("%s        <friction><ode><mu>1</mu><mu2>1</mu2></ode></friction>" % pad)
            lines.append("%s        <contact><ode/></contact>" % pad)
            lines.append("%s      </surface>" % pad)
        lines.append("%s    </%s>" % (pad, tag))
    lines.extend(_inertia_lines(obstacle.shape, obstacle.size, obstacle.radius,
                                obstacle.height, indent + 4))
    lines.append("%s  </link>" % pad)
    lines.append("%s</model>" % pad)
    return lines


def render_sdf(world_name, obstacles, config):
    generator = config["generator"]
    ground = generator["ground"]
    light = generator["light"]
    physics = generator["physics"]
    lines = ["<?xml version=\"1.0\" ?>",
             "<sdf version=\"%s\">" % generator["sdf_version"],
             "  <world name=\"%s\">" % world_name,
             "",
             "    <!-- Self-contained training world: no online model repository,",
             "         no remote URIs, no vehicle and no plugins. The iris is",
             "         spawned by the upstream PX4 launch, never by this file. -->",
             "",
             "    <light type=\"directional\" name=\"training_sun\">",
             "      <pose>0 0 30 0 0 0</pose>",
             "      <cast_shadows>true</cast_shadows>",
             "      <diffuse>%s</diffuse>" % _pose(light["diffuse"]),
             "      <specular>%s</specular>" % _pose(light["specular"]),
             "      <direction>%s</direction>" % _pose(light["direction"]),
             "    </light>",
             ""]
    if ground["mode"] == "inline":
        size = fmt(ground["size_m"])
        lines += [
            "    <model name=\"training_ground\">",
            "      <static>true</static>",
            "      <pose>0 0 0 0 0 0</pose>",
            "      <link name=\"link\">",
            "        <collision name=\"collision\">",
            "          <geometry><plane><normal>0 0 1</normal><size>%s %s</size></plane></geometry>" % (size, size),
            "          <surface>",
            "            <friction><ode><mu>1</mu><mu2>1</mu2></ode></friction>",
            "            <contact><ode/></contact>",
            "          </surface>",
            "        </collision>",
            "        <visual name=\"visual\">",
            "          <geometry><plane><normal>0 0 1</normal><size>%s %s</size></plane></geometry>" % (size, size),
        ]
        lines += _material_lines(tuple(ground["colour"]), 10)
        lines += [
            "          <cast_shadows>false</cast_shadows>",
            "        </visual>",
            "      </link>",
            "    </model>",
            "",
        ]
    else:
        lines += ["    <include><uri>model://ground_plane</uri></include>", ""]

    for obstacle in obstacles:
        lines += _render_obstacle(obstacle)
        lines.append("")

    lines += [
        "    <physics name=\"default_physics\" default=\"0\" type=\"ode\">",
        "      <gravity>0 0 -9.8066</gravity>",
        "      <ode>",
        "        <solver><type>quick</type><iters>10</iters><sor>1.3</sor>"
        "<use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling></solver>",
        "        <constraints><cfm>0</cfm><erp>0.2</erp>"
        "<contact_max_correcting_vel>100</contact_max_correcting_vel>"
        "<contact_surface_layer>0.001</contact_surface_layer></constraints>",
        "      </ode>",
        "      <max_step_size>%s</max_step_size>" % fmt(physics["max_step_size"]),
        "      <real_time_factor>%s</real_time_factor>" % fmt(physics["real_time_factor"]),
        "      <real_time_update_rate>%s</real_time_update_rate>" % fmt(physics["real_time_update_rate"]),
        "      <magnetic_field>6.0e-6 2.3e-5 -4.2e-5</magnetic_field>",
        "    </physics>",
        "  </world>",
        "</sdf>",
        "",
    ]
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# top-level generation
# ---------------------------------------------------------------------------

def build_world(preset, seed, config, category=None, world_name=None,
                emit_grid=None, timestamp=None):
    preset_settings = preset_config(config, preset)
    materials = config["materials"]
    safety = dict(config["safety"])
    bounds_dict = preset_settings["bounds"]
    bounds = [float(bounds_dict["x_min"]), float(bounds_dict["y_min"]),
              float(bounds_dict["x_max"]), float(bounds_dict["y_max"])]
    generator = config["generator"]
    rng = random.Random(int(seed))

    if preset_settings["mode"] == "category":
        category = category or preset_settings.get("category") or "single_wall"
        if category not in config["unit_categories"]:
            raise ValueError("unknown unit category: %s" % category)
        expectation = config["unit_categories"][category]
        goal_centers = [_unit_goal(bounds)]
    else:
        category = None
        expectation = {"expect_reachable": True, "expect_goal_valid": True}
        goal_centers = []

    obstacles = []
    if preset_settings.get("boundary", {}).get("enabled", True):
        obstacles += _boundary_walls(bounds, preset_settings["boundary"], materials)

    road_polygons = []
    if preset_settings["mode"] == "category":
        obstacles += _unit_structure(category, rng, bounds, preset_settings,
                                     goal_centers[0], materials["wall"])
    else:
        city_obstacles, x_strips, y_strips = _city_layout(rng, bounds, preset_settings, materials)
        obstacles += city_obstacles
        for strip_x0, strip_x1 in x_strips:
            road_polygons.append(rect_polygon((strip_x0 + strip_x1) / 2.0,
                                              (bounds[1] + bounds[3]) / 2.0,
                                              strip_x1 - strip_x0, bounds[3] - bounds[1]))
        for strip_y0, strip_y1 in y_strips:
            road_polygons.append(rect_polygon((bounds[0] + bounds[2]) / 2.0,
                                              (strip_y0 + strip_y1) / 2.0,
                                              bounds[2] - bounds[0], strip_y1 - strip_y0))

    resolution = float(preset_settings["grid_resolution_m"])
    grid = OccupancyGrid(bounds, resolution)
    for obstacle in obstacles:
        if obstacle.blocking:
            grid.mark(obstacle, float(safety["safety_margin_m"]))

    spawn = _choose_spawn(rng, preset_settings, grid, obstacles, safety, road_polygons)
    if preset_settings["mode"] != "category":
        goal_centers = _choose_goals(rng, preset_settings, grid, obstacles, safety,
                                     spawn, int(preset_settings["goal_candidates"]))

    spawn_cell = grid.index_of(spawn["center"][0], spawn["center"][1])
    goal_cells = [grid.index_of(x, y) for x, y in goal_centers]
    _visited, reachable_flags = _connectivity(grid, spawn_cell, goal_cells)

    grid_info = {
        "resolution_m": _r(resolution),
        "width": grid.width,
        "height": grid.height,
        "origin": [_r(bounds[0]), _r(bounds[1])],
        "frame_id": "map",
        "inflation_m": _r(safety["safety_margin_m"]),
        "encoding": "rle_pairs",
        "note": "1 = blocked after inflation by the safety margin; 0 = free.",
    }
    if emit_grid is None:
        emit_grid = bool(generator.get("emit_grid", True))
    if emit_grid:
        grid_info["data"] = grid.rle()

    metadata = {
        "schema": SCHEMA_VERSION,
        "generator": {
            "name": "generate_training_city.py",
            "version": GENERATOR_VERSION,
            "config": "config/training_city.yaml",
            "preset": preset,
            "category": category,
            "seed": int(seed),
        },
        "world_name": world_name,
        "sdf_version": generator["sdf_version"],
        "frame": {"frame_id": "map", "convention": "ENU", "units": "m", "ground_z": 0.0},
        "bounds": {key: _r(value) for key, value in bounds_dict.items()},
        "planning_altitude_m": _r(preset_settings["planning_altitude_m"]),
        "safety": {key: _r(value) if isinstance(value, (int, float)) else value
                   for key, value in safety.items()},
        "expectations": {
            "all_goals_reachable": bool(expectation["expect_reachable"]),
            "goals_valid": bool(expectation["expect_goal_valid"]),
        },
        "obstacles": [obstacle.to_metadata() for obstacle in obstacles],
        "spawn": spawn,
        "goal_candidates": _goal_entries(grid, obstacles, goal_centers, reachable_flags,
                                         safety, expectation["expect_goal_valid"]),
        "grid": grid_info,
        "connectivity": {
            "method": "bfs_4connected_on_inflated_grid",
            "resolution_m": _r(resolution),
            "free_cell_ratio": _r(grid.free_ratio()),
            "goals_total": len(goal_centers),
            "goals_reachable": int(sum(1 for flag in reachable_flags if flag)),
        },
        "assumptions": [
            "Road width, block sizes, building counts / heights and lamp spacing are team training assumptions.",
            "Building / lamp positions are randomised here because the official randomisation script is not published.",
            "The spawn and goal candidates below are training artefacts, NOT official competition spawn points.",
            "Coordinates use frame 'map' (ENU, metres); the full-preset origin x[-100,100] y[-50,50] is a team convention.",
        ],
    }
    if timestamp:
        metadata["generated_utc"] = timestamp

    problems, checks = validate_metadata(metadata)
    metadata["validation"] = {
        "ok": not problems,
        "problems": problems,
        "checks": checks,
    }
    sdf = render_sdf(world_name, obstacles, config)
    return sdf, metadata


def make_world_name(config, preset, seed, category=None):
    prefix = config["generator"]["world_name_prefix"]
    suffix = "_%s" % category if category else ""
    return "%s_%s%s_s%s" % (prefix, preset, suffix, seed)
