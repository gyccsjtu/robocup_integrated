"""Continuous, geometry-based safety checks for planned UAV routes.

The A* planner consumes an inflated occupancy grid.  This module deliberately
does not: it validates the original obstacle geometry in the versioned training
metadata so that grid resolution and grid inflation cannot hide a collision.

Only grounded, axis-aligned boxes and circular footprints are accepted in this
unit-only implementation.  Grounded geometry is a useful restriction here:
every horizontal route position has a meaningful vertical landing corridor.
"""

from dataclasses import dataclass
import math
from numbers import Real


SCHEMA = "robocup_training_worlds/metadata/v1"

# The defaults are based on the verified Iris dimensions supplied for this
# stage: sqrt(0.13**2 + 0.22**2) + 0.128 ~= 0.3835 m.  Round up to 0.40 m.
DEFAULT_VEHICLE_RADIUS_M = 0.40
DEFAULT_HORIZONTAL_MARGIN_M = 0.20
DEFAULT_VEHICLE_HALF_HEIGHT_M = 0.15
DEFAULT_VERTICAL_MARGIN_M = 0.10

_EPS = 1.0e-9
_GEOMETRY_TOL = 1.0e-6


class SafetyError(ValueError):
    """Malformed metadata or a route that cannot be proven safe."""


def _error(code, detail=None):
    message = code if detail is None else "%s:%s" % (code, detail)
    raise SafetyError(message)


def _finite_number(value, name):
    """Return a strict finite real number, rejecting strings and booleans."""

    if isinstance(value, bool) or not isinstance(value, Real):
        _error("%s_INVALID" % name)
    result = float(value)
    if not math.isfinite(result):
        _error("%s_NONFINITE" % name)
    return result


def _finite_vector(value, name, length):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        _error("%s_INVALID" % name)
    return tuple(_finite_number(component, "%s_%d" % (name, index))
                 for index, component in enumerate(value))


def _optional_finite_number(entry, key, name, default):
    if key not in entry:
        return default
    return _finite_number(entry[key], name)


def _close(left, right, tolerance=_GEOMETRY_TOL):
    return abs(left - right) <= tolerance


def _axis_aligned_quarter(yaw, name):
    """Return the nearest quarter-turn, or reject a rotated box."""

    quarter_turn = math.pi / 2.0
    quarter = int(round(yaw / quarter_turn))
    if not _close(yaw, quarter * quarter_turn, 1.0e-8):
        _error("%s_NOT_AXIS_ALIGNED" % name)
    return quarter % 4


def _finite_bbox(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        _error("%s_INVALID" % name)
    x_min, y_min, x_max, y_max = (
        _finite_number(component, "%s_%d" % (name, index))
        for index, component in enumerate(value)
    )
    if not x_min < x_max or not y_min < y_max:
        _error("%s_ORDER_INVALID" % name)
    return x_min, y_min, x_max, y_max


def _finite_point2(value, name):
    return _finite_vector(value, name, 2)


def _rect_signed_distance(x, y, x_min, y_min, x_max, y_max):
    """Signed distance to an AABB: negative inside the obstacle."""

    if x < x_min:
        dx = x_min - x
    elif x > x_max:
        dx = x - x_max
    else:
        dx = -min(x - x_min, x_max - x)

    if y < y_min:
        dy = y_min - y
    elif y > y_max:
        dy = y - y_max
    else:
        dy = -min(y - y_min, y_max - y)

    outside = math.hypot(max(dx, 0.0), max(dy, 0.0))
    inside = min(max(dx, dy), 0.0)
    return outside + inside


def _rect_point_distance(x, y, x_min, y_min, x_max, y_max):
    dx = max(x_min - x, 0.0, x - x_max)
    dy = max(y_min - y, 0.0, y - y_max)
    return math.hypot(dx, dy)


def _point_segment_distance(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    denominator = dx * dx + dy * dy
    if denominator <= _EPS * _EPS:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / denominator
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _segment_intersects_rect(ax, ay, bx, by,
                             x_min, y_min, x_max, y_max):
    """Inclusive Liang-Barsky test for a segment and an AABB."""

    dx, dy = bx - ax, by - ay
    lower, upper = 0.0, 1.0
    for coefficient, constant in (
            (-dx, ax - x_min), (dx, x_max - ax),
            (-dy, ay - y_min), (dy, y_max - ay)):
        if abs(coefficient) <= _EPS:
            if constant < -_EPS:
                return False
            continue
        ratio = constant / coefficient
        if coefficient < 0.0:
            if ratio > upper + _EPS:
                return False
            lower = max(lower, ratio)
        else:
            if ratio < lower - _EPS:
                return False
            upper = min(upper, ratio)
    return lower <= upper + _EPS


def _segment_rect_distance(ax, ay, bx, by,
                           x_min, y_min, x_max, y_max):
    """Exact minimum Euclidean distance between a segment and an AABB."""

    if _segment_intersects_rect(ax, ay, bx, by,
                                x_min, y_min, x_max, y_max):
        return 0.0

    endpoints = (
        _rect_point_distance(ax, ay, x_min, y_min, x_max, y_max),
        _rect_point_distance(bx, by, x_min, y_min, x_max, y_max),
    )
    corners = (
        (x_min, y_min), (x_min, y_max),
        (x_max, y_min), (x_max, y_max),
    )
    corner_distances = tuple(
        _point_segment_distance(cx, cy, ax, ay, bx, by)
        for cx, cy in corners
    )
    return min(endpoints + corner_distances)


def _boundary_signed_distance(x, y, x_min, y_min, x_max, y_max):
    """Signed distance to the inside of the rectangular flight boundary."""

    if x_min <= x <= x_max and y_min <= y <= y_max:
        return min(x - x_min, x_max - x, y - y_min, y_max - y)
    return -_rect_point_distance(x, y, x_min, y_min, x_max, y_max)


def _distance2(left, right):
    return math.sqrt(sum((a - b) * (a - b)
                         for a, b in zip(left, right)))


@dataclass(frozen=True)
class _Box:
    obstacle_id: str
    x_min: float
    y_min: float
    x_max: float
    y_max: float
    z_min: float
    z_max: float

    def signed_distance(self, x, y):
        return _rect_signed_distance(x, y, self.x_min, self.y_min,
                                     self.x_max, self.y_max)

    def segment_distance(self, start, end):
        return _segment_rect_distance(start[0], start[1], end[0], end[1],
                                      self.x_min, self.y_min,
                                      self.x_max, self.y_max)


@dataclass(frozen=True)
class _Circle:
    obstacle_id: str
    center_x: float
    center_y: float
    radius: float
    z_min: float
    z_max: float

    def signed_distance(self, x, y):
        return math.hypot(x - self.center_x, y - self.center_y) - self.radius

    def segment_distance(self, start, end):
        return (_point_segment_distance(self.center_x, self.center_y,
                                        start[0], start[1], end[0], end[1])
                - self.radius)


def _obstacle_context(entry, index):
    obstacle_id = entry.get("id", "obstacle_%04d" % index)
    if not isinstance(obstacle_id, str) or not obstacle_id:
        _error("OBSTACLE_ID_INVALID", index)
    if "blocking" in entry and not isinstance(entry["blocking"], bool):
        _error("OBSTACLE_BLOCKING_INVALID", obstacle_id)
    if "type" in entry and not isinstance(entry["type"], str):
        _error("OBSTACLE_TYPE_INVALID", obstacle_id)
    if "group" in entry and entry["group"] is not None and not isinstance(entry["group"], str):
        _error("OBSTACLE_GROUP_INVALID", obstacle_id)
    return obstacle_id


def _vertical_extent(entry, ground_z, obstacle_id):
    if "z_min" in entry:
        z_min = _finite_number(entry["z_min"], "OBSTACLE_Z_MIN")
    else:
        # Metadata v1 serialises z_min.  A missing value is accepted only as
        # the unambiguous grounded default used by the generator.
        z_min = ground_z

    height = None
    if "height" in entry:
        height = _finite_number(entry["height"], "OBSTACLE_HEIGHT")
        if height <= 0.0:
            _error("OBSTACLE_HEIGHT_NOT_POSITIVE", obstacle_id)

    if "z_max" in entry:
        z_max = _finite_number(entry["z_max"], "OBSTACLE_Z_MAX")
    elif height is not None:
        z_max = z_min + height
    else:
        _error("OBSTACLE_VERTICAL_EXTENT_MISSING", obstacle_id)

    if not math.isfinite(z_max):
        _error("OBSTACLE_Z_MAX_NONFINITE", obstacle_id)
    if height is not None and not _close(z_max - z_min, height):
        _error("OBSTACLE_HEIGHT_MISMATCH", obstacle_id)
    if not _close(z_min, ground_z, 1.0e-9):
        _error("OBSTACLE_NOT_GROUNDED", obstacle_id)
    if z_max <= ground_z:
        _error("OBSTACLE_Z_MAX_NOT_ABOVE_GROUND", obstacle_id)
    return z_min, z_max


def _validate_inside_bounds(bbox, bounds, obstacle_id):
    x_min, y_min, x_max, y_max = bbox
    if (x_min < bounds["x_min"] - _EPS or
            y_min < bounds["y_min"] - _EPS or
            x_max > bounds["x_max"] + _EPS or
            y_max > bounds["y_max"] + _EPS):
        _error("OBSTACLE_OUT_OF_BOUNDS", obstacle_id)


def _validate_polygon_footprint(footprint, bbox, obstacle_id):
    if not isinstance(footprint, dict) or footprint.get("kind") != "polygon":
        _error("OBSTACLE_FOOTPRINT_UNSUPPORTED", obstacle_id)
    points = footprint.get("points")
    if not isinstance(points, (list, tuple)) or len(points) != 4:
        _error("OBSTACLE_POLYGON_INVALID", obstacle_id)
    parsed = [_finite_point2(point, "OBSTACLE_FOOTPRINT_POINT")
              for point in points]
    expected = (
        (bbox[0], bbox[1]), (bbox[0], bbox[3]),
        (bbox[2], bbox[1]), (bbox[2], bbox[3]),
    )
    unmatched = list(expected)
    for point in parsed:
        match = next((index for index, candidate in enumerate(unmatched)
                      if _close(point[0], candidate[0]) and
                      _close(point[1], candidate[1])), None)
        if match is None:
            _error("OBSTACLE_POLYGON_NOT_AXIS_ALIGNED", obstacle_id)
        unmatched.pop(match)


def _parse_box(entry, bounds, ground_z, obstacle_id):
    if "yaw" in entry:
        yaw = _finite_number(entry["yaw"], "OBSTACLE_YAW")
    else:
        yaw = 0.0
    quarter = _axis_aligned_quarter(yaw, "OBSTACLE_YAW")

    bbox = (_finite_bbox(entry["bbox"], "OBSTACLE_BBOX")
            if "bbox" in entry else None)
    center = (_finite_point2(entry["center"], "OBSTACLE_CENTER")
              if "center" in entry else None)
    size = (_finite_point2(entry["size"], "OBSTACLE_SIZE")
            if "size" in entry else None)
    if size is not None and (size[0] <= 0.0 or size[1] <= 0.0):
        _error("OBSTACLE_SIZE_NOT_POSITIVE", obstacle_id)

    if center is None and bbox is not None:
        center = ((bbox[0] + bbox[2]) / 2.0,
                  (bbox[1] + bbox[3]) / 2.0)
    if center is None:
        _error("OBSTACLE_CENTER_MISSING", obstacle_id)
    if bbox is not None and size is None:
        bbox_center = ((bbox[0] + bbox[2]) / 2.0,
                       (bbox[1] + bbox[3]) / 2.0)
        if not all(_close(left, right)
                   for left, right in zip(center, bbox_center)):
            _error("OBSTACLE_CENTER_MISMATCH", obstacle_id)

    if size is not None:
        world_size = (size[1], size[0]) if quarter % 2 else size
        derived_bbox = (
            center[0] - world_size[0] / 2.0,
            center[1] - world_size[1] / 2.0,
            center[0] + world_size[0] / 2.0,
            center[1] + world_size[1] / 2.0,
        )
        if bbox is not None and any(not _close(left, right)
                                    for left, right in zip(bbox, derived_bbox)):
            _error("OBSTACLE_BBOX_MISMATCH", obstacle_id)
        bbox = derived_bbox
    elif bbox is None:
        _error("OBSTACLE_SIZE_OR_BBOX_MISSING", obstacle_id)

    if "footprint" in entry:
        _validate_polygon_footprint(entry["footprint"], bbox, obstacle_id)

    z_min, z_max = _vertical_extent(entry, ground_z, obstacle_id)
    _validate_inside_bounds(bbox, bounds, obstacle_id)
    return _Box(obstacle_id, bbox[0], bbox[1], bbox[2], bbox[3],
                z_min, z_max)


def _parse_circle(entry, bounds, ground_z, obstacle_id):
    if "yaw" in entry:
        _finite_number(entry["yaw"], "OBSTACLE_YAW")

    footprint = entry.get("footprint")
    if footprint is not None:
        if not isinstance(footprint, dict) or footprint.get("kind") != "circle":
            _error("OBSTACLE_FOOTPRINT_UNSUPPORTED", obstacle_id)

    top_center = (_finite_point2(entry["center"], "OBSTACLE_CENTER")
                  if "center" in entry else None)
    footprint_center = None
    if isinstance(footprint, dict) and "center" in footprint:
        footprint_center = _finite_point2(footprint["center"],
                                          "OBSTACLE_FOOTPRINT_CENTER")
    center = top_center or footprint_center
    if center is None:
        _error("OBSTACLE_CENTER_MISSING", obstacle_id)
    if top_center is not None and footprint_center is not None:
        if not all(_close(left, right)
                   for left, right in zip(top_center, footprint_center)):
            _error("OBSTACLE_CENTER_MISMATCH", obstacle_id)

    top_radius = (_finite_number(entry["radius"], "OBSTACLE_RADIUS")
                  if "radius" in entry else None)
    footprint_radius = None
    if isinstance(footprint, dict) and "radius" in footprint:
        footprint_radius = _finite_number(footprint["radius"],
                                          "OBSTACLE_FOOTPRINT_RADIUS")
    radius = top_radius if top_radius is not None else footprint_radius
    if radius is None:
        _error("OBSTACLE_RADIUS_MISSING", obstacle_id)
    if top_radius is not None and footprint_radius is not None and not _close(top_radius, footprint_radius):
        _error("OBSTACLE_RADIUS_MISMATCH", obstacle_id)
    if radius <= 0.0:
        _error("OBSTACLE_RADIUS_NOT_POSITIVE", obstacle_id)

    derived_bbox = (center[0] - radius, center[1] - radius,
                    center[0] + radius, center[1] + radius)
    if "bbox" in entry:
        bbox = _finite_bbox(entry["bbox"], "OBSTACLE_BBOX")
        if any(not _close(left, right)
               for left, right in zip(bbox, derived_bbox)):
            _error("OBSTACLE_BBOX_MISMATCH", obstacle_id)
    else:
        bbox = derived_bbox

    z_min, z_max = _vertical_extent(entry, ground_z, obstacle_id)
    _validate_inside_bounds(bbox, bounds, obstacle_id)
    return _Circle(obstacle_id, center[0], center[1], radius, z_min, z_max)


def _parse_obstacle(entry, index, bounds, ground_z):
    if not isinstance(entry, dict):
        _error("OBSTACLE_NOT_OBJECT", index)
    obstacle_id = _obstacle_context(entry, index)
    shape = entry.get("shape")
    if shape == "box":
        return _parse_box(entry, bounds, ground_z, obstacle_id)
    if shape in ("circle", "cylinder"):
        return _parse_circle(entry, bounds, ground_z, obstacle_id)
    _error("UNSUPPORTED_OBSTACLE_GEOMETRY", obstacle_id)


class SafetyMap:
    """Validate continuous routes against original grounded map geometry."""

    def __init__(self, metadata,
                 vehicle_radius_m=DEFAULT_VEHICLE_RADIUS_M,
                 horizontal_margin_m=DEFAULT_HORIZONTAL_MARGIN_M,
                 vehicle_half_height_m=DEFAULT_VEHICLE_HALF_HEIGHT_M,
                 vertical_margin_m=DEFAULT_VERTICAL_MARGIN_M):
        self.vehicle_radius_m = _finite_number(vehicle_radius_m,
                                               "VEHICLE_RADIUS")
        self.horizontal_margin_m = _finite_number(horizontal_margin_m,
                                                  "HORIZONTAL_MARGIN")
        self.vehicle_half_height_m = _finite_number(vehicle_half_height_m,
                                                    "VEHICLE_HALF_HEIGHT")
        self.vertical_margin_m = _finite_number(vertical_margin_m,
                                                "VERTICAL_MARGIN")
        if self.vehicle_radius_m <= 0.0:
            _error("VEHICLE_RADIUS_NOT_POSITIVE")
        if self.horizontal_margin_m < 0.0:
            _error("HORIZONTAL_MARGIN_NEGATIVE")
        if self.vehicle_half_height_m < 0.0:
            _error("VEHICLE_HALF_HEIGHT_NEGATIVE")
        if self.vertical_margin_m < 0.0:
            _error("VERTICAL_MARGIN_NEGATIVE")

        if not isinstance(metadata, dict):
            _error("METADATA_NOT_OBJECT")
        if metadata.get("schema") != SCHEMA:
            _error("UNSUPPORTED_SCHEMA", metadata.get("schema"))

        frame = metadata.get("frame")
        if not isinstance(frame, dict):
            _error("FRAME_INVALID")
        if (frame.get("frame_id") != "map" or
                frame.get("convention") != "ENU" or
                frame.get("units") != "m"):
            _error("FRAME_INVALID")
        self.frame_id = "map"
        self.ground_z = _finite_number(frame.get("ground_z"), "GROUND_Z")

        bounds = metadata.get("bounds")
        if not isinstance(bounds, dict):
            _error("BOUNDS_INVALID")
        required_bounds = ("x_min", "x_max", "y_min", "y_max")
        if any(key not in bounds for key in required_bounds):
            _error("BOUNDS_MISSING")
        parsed_bounds = {
            key: _finite_number(bounds[key], "BOUND_%s" % key.upper())
            for key in required_bounds
        }
        if (not parsed_bounds["x_min"] < parsed_bounds["x_max"] or
                not parsed_bounds["y_min"] < parsed_bounds["y_max"]):
            _error("BOUNDS_ORDER_INVALID")
        self.bounds = parsed_bounds

        # The grid is deliberately not consumed.  If it is present, only its
        # frame identity is relevant; inflation remains A* planning data.
        if "grid" in metadata:
            grid = metadata["grid"]
            if not isinstance(grid, dict):
                _error("GRID_INVALID")
            if "frame_id" in grid and grid["frame_id"] != self.frame_id:
                _error("GRID_FRAME_INVALID")

        obstacles = metadata.get("obstacles")
        if not isinstance(obstacles, list):
            _error("OBSTACLES_INVALID")
        parsed_obstacles = []
        obstacle_ids = set()
        for index, entry in enumerate(obstacles):
            obstacle = _parse_obstacle(entry, index, self.bounds,
                                       self.ground_z)
            if obstacle.obstacle_id in obstacle_ids:
                _error("DUPLICATE_OBSTACLE_ID", obstacle.obstacle_id)
            obstacle_ids.add(obstacle.obstacle_id)
            parsed_obstacles.append(obstacle)
        self.obstacles = tuple(parsed_obstacles)

    @property
    def required_horizontal_clearance_m(self):
        """Center-to-geometry distance required by safe route checks."""

        return self.vehicle_radius_m + self.horizontal_margin_m

    def clearance(self, point_xyz):
        """Return signed horizontal clearance after subtracting vehicle radius.

        The horizontal margin is intentionally not subtracted here.  Positive
        values are remaining free space outside the vehicle body; a negative
        value means the body overlaps a boundary or obstacle.
        """

        point = _finite_vector(point_xyz, "POINT", 3)
        x, y = point[0], point[1]
        clearances = [
            _boundary_signed_distance(
                x, y,
                self.bounds["x_min"], self.bounds["y_min"],
                self.bounds["x_max"], self.bounds["y_max"])
            - self.vehicle_radius_m
        ]
        clearances.extend(
            obstacle.signed_distance(x, y) - self.vehicle_radius_m
            for obstacle in self.obstacles
        )
        return float(min(clearances))

    def point_safe(self, point_xyz):
        """Return whether one finite point has safe altitude and horizontal clearance."""

        try:
            point = _finite_vector(point_xyz, "POINT", 3)
        except SafetyError:
            return False
        if point[2] < self.ground_z:
            return False
        return self.clearance(point) + _EPS >= self.horizontal_margin_m

    def landing_safe(self, point_xyz):
        """Return whether a vertical descent at the point has a clear corridor.

        Every accepted obstacle starts at ``ground_z`` and has positive height.
        Therefore an obstacle footprint blocks a descent at every altitude in
        the corridor; the ground plane itself is not included as an obstacle.
        The same horizontal body-plus-margin test is used at the landing point.
        """

        try:
            point = _finite_vector(point_xyz, "POINT", 3)
        except SafetyError:
            return False
        if point[2] < self.ground_z:
            return False
        return self.clearance(point) + _EPS >= self.horizontal_margin_m

    def _validate_route_point(self, point, origin, max_radius, max_altitude,
                              index):
        if point[2] < self.ground_z:
            _error("ROUTE_BELOW_GROUND", index)

        top_of_vehicle = point[2] + self.vehicle_half_height_m + self.vertical_margin_m
        if not math.isfinite(top_of_vehicle):
            _error("ROUTE_ALTITUDE_NONFINITE", index)
        if top_of_vehicle > max_altitude + _EPS:
            _error("ROUTE_ALTITUDE_LIMIT", index)

        if self.clearance(point) + _EPS < self.horizontal_margin_m:
            _error("ROUTE_HORIZONTAL_CLEARANCE", index)

        radial_distance = math.hypot(point[0] - origin[0],
                                     point[1] - origin[1])
        if radial_distance + self.required_horizontal_clearance_m > max_radius + _EPS:
            _error("ROUTE_RADIUS_LIMIT", index)

        # Keep this explicit even though the accepted grounded geometry makes
        # it equivalent to the horizontal test.  It documents and freezes the
        # invariant needed by a future path executor.
        if not self.landing_safe(point):
            _error("ROUTE_LANDING_CORRIDOR", index)

    def _validate_route_segment(self, start, end, index):
        required = self.required_horizontal_clearance_m
        for obstacle in self.obstacles:
            if obstacle.segment_distance(start, end) + _EPS < required:
                _error("ROUTE_SEGMENT_CLEARANCE", index)

    def _segment_clearance(self, start, end):
        values = [
            min(
                _boundary_signed_distance(
                    start[0], start[1],
                    self.bounds["x_min"], self.bounds["y_min"],
                    self.bounds["x_max"], self.bounds["y_max"]),
                _boundary_signed_distance(
                    end[0], end[1],
                    self.bounds["x_min"], self.bounds["y_min"],
                    self.bounds["x_max"], self.bounds["y_max"]),
            ) - self.vehicle_radius_m
        ]
        values.extend(obstacle.segment_distance(start, end)
                      - self.vehicle_radius_m
                      for obstacle in self.obstacles)
        return min(values)

    def validate_route(self, points_xyz, origin_xyz, max_radius_m,
                       max_altitude_m):
        """Validate every point and continuous segment of a route.

        ``max_radius_m`` is a circular horizontal fence around ``origin_xyz``;
        both vehicle radius and horizontal margin are reserved inside it.
        ``max_altitude_m`` is the absolute ENU/map ceiling for the top of the
        vehicle plus the configured vertical margin.
        """

        if isinstance(points_xyz, (str, bytes)):
            _error("ROUTE_POINTS_INVALID")
        try:
            raw_points = list(points_xyz)
        except (TypeError, ValueError):
            _error("ROUTE_POINTS_INVALID")
        if not raw_points:
            _error("ROUTE_EMPTY")
        points = tuple(_finite_vector(point, "ROUTE_POINT", 3)
                       for point in raw_points)
        origin = _finite_vector(origin_xyz, "ORIGIN", 3)
        max_radius = _finite_number(max_radius_m, "MAX_RADIUS")
        max_altitude = _finite_number(max_altitude_m, "MAX_ALTITUDE")
        if max_radius < 0.0:
            _error("MAX_RADIUS_NEGATIVE")
        if max_altitude < self.ground_z:
            _error("MAX_ALTITUDE_BELOW_GROUND")

        for index, point in enumerate(points):
            self._validate_route_point(point, origin, max_radius,
                                        max_altitude, index)
        for index, (start, end) in enumerate(zip(points, points[1:])):
            self._validate_route_segment(start, end, index)

        minimum_clearance = min(self.clearance(point) for point in points)
        segment_clearances = [
            self._segment_clearance(start, end)
            for start, end in zip(points, points[1:])
        ]
        # A one-point route has no segments; keep its vertex clearance as the
        # result rather than calling min() with an empty expansion.
        if segment_clearances:
            minimum_clearance = min(minimum_clearance,
                                    min(segment_clearances))
        return {
            "valid": True,
            "minimum_clearance_m": float(minimum_clearance),
        }


class WorldLocalTransform:
    """Planar ENU transform between world and local coordinate frames.

    ``yaw_rad`` rotates local x/y axes into world x/y axes.  The two origin
    vectors are corresponding points, so z is translated but not rotated.
    """

    def __init__(self, world_origin_xyz, local_origin_xyz, yaw_rad=0.0):
        self.world_origin_xyz = _finite_vector(world_origin_xyz,
                                               "WORLD_ORIGIN", 3)
        self.local_origin_xyz = _finite_vector(local_origin_xyz,
                                               "LOCAL_ORIGIN", 3)
        self.yaw_rad = _finite_number(yaw_rad, "YAW")
        try:
            self._cos_yaw = math.cos(self.yaw_rad)
            self._sin_yaw = math.sin(self.yaw_rad)
        except (OverflowError, ValueError):
            _error("YAW_INVALID")
        norm = self._cos_yaw * self._cos_yaw + self._sin_yaw * self._sin_yaw
        if not math.isfinite(norm) or abs(norm - 1.0) > 1.0e-12:
            _error("ROTATION_NOT_ORTHOGONAL")

    @property
    def rotation_matrix(self):
        """The 2-D local-to-world rotation matrix, useful for diagnostics."""

        return ((self._cos_yaw, -self._sin_yaw),
                (self._sin_yaw, self._cos_yaw))

    def world_to_local(self, point):
        world = _finite_vector(point, "WORLD_POINT", 3)
        dx = world[0] - self.world_origin_xyz[0]
        dy = world[1] - self.world_origin_xyz[1]
        return (
            self._cos_yaw * dx + self._sin_yaw * dy + self.local_origin_xyz[0],
            -self._sin_yaw * dx + self._cos_yaw * dy + self.local_origin_xyz[1],
            world[2] - self.world_origin_xyz[2] + self.local_origin_xyz[2],
        )

    def local_to_world(self, point):
        local = _finite_vector(point, "LOCAL_POINT", 3)
        dx = local[0] - self.local_origin_xyz[0]
        dy = local[1] - self.local_origin_xyz[1]
        return (
            self.world_origin_xyz[0] + self._cos_yaw * dx - self._sin_yaw * dy,
            self.world_origin_xyz[1] + self._sin_yaw * dx + self._cos_yaw * dy,
            self.world_origin_xyz[2] + local[2] - self.local_origin_xyz[2],
        )

    def validate_pair(self, world, local, tolerance_m):
        """Return ``True`` when the finite pair agrees within tolerance.

        A finite but mismatched pair is a normal validation result and returns
        ``False``.  Malformed/non-finite arguments or a negative tolerance are
        input errors and raise ``SafetyError``.
        """

        tolerance = _finite_number(tolerance_m, "TRANSFORM_TOLERANCE")
        if tolerance < 0.0:
            _error("TRANSFORM_TOLERANCE_NEGATIVE")
        world_point = _finite_vector(world, "WORLD_POINT", 3)
        local_point = _finite_vector(local, "LOCAL_POINT", 3)
        expected_local = self.world_to_local(world_point)
        expected_world = self.local_to_world(local_point)
        return (_distance2(expected_local, local_point) <= tolerance + _EPS and
                _distance2(expected_world, world_point) <= tolerance + _EPS)

    def validate_planar_pair(self, world, local, tolerance_m):
        """Validate x/y alignment while allowing a live estimator z offset.

        PX4 local z can rebase during estimator convergence after takeoff.
        Horizontal world/local alignment remains fixed and is the contract
        needed by the 2-D planner; physical altitude is checked independently
        from Gazebo truth by the executor.
        """
        tolerance = _finite_number(tolerance_m, "TRANSFORM_TOLERANCE")
        if tolerance < 0.0:
            _error("TRANSFORM_TOLERANCE_NEGATIVE")
        world_point = _finite_vector(world, "WORLD_POINT", 3)
        local_point = _finite_vector(local, "LOCAL_POINT", 3)
        expected_local = self.world_to_local(world_point)
        return math.hypot(expected_local[0] - local_point[0],
                          expected_local[1] - local_point[1]) <= tolerance + _EPS


__all__ = [
    "SCHEMA",
    "SafetyError",
    "SafetyMap",
    "WorldLocalTransform",
]
