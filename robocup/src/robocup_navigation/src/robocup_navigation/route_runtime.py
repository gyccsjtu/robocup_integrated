"""Fail-closed runtime contract for the phase-1 static single-wall route.

This module deliberately has no ROS dependency.  It verifies the *current*
world/metadata/launch receipt, derives the route from metadata (never a saved
route JSON), and exposes the expected Gazebo model poses for the ROS executor
to check before arming and while flying.
"""
from __future__ import division

from dataclasses import dataclass
import hashlib
import json
import math
import os
import time
import xml.etree.ElementTree as ET
from numbers import Real

from robocup_navigation.astar import GridMap, MapError, load_metadata, metadata_endpoints, plan


RECEIPT_SCHEMA = "robocup_navigation/scene_launch/v1"

# Generator categories whose unit-preset metadata is compatible with the
# route-mode execution chain (GridMap.from_metadata, metadata_endpoints,
# verify_world_geometry).  goal_in_obstacle is deliberately excluded: it is
# rejected earlier by the outdoor-goal check, never reaches planning.
ROUTE_MODE_CATEGORIES = (
    "empty",
    "single_wall",
    "wall_with_gap",
    "u_shape",
    "narrow_corridor",
    "blocked",
    "no_path",
)

# Scene contract for Gazebo model-name lists.  ``static`` keeps the legacy
# fail-closed exact-match behaviour; ``perception`` tolerates extra (dynamic)
# obstacle models that the depth-camera stack is responsible for observing,
# while still requiring every verified scene model to be present at its
# checked pose.
SCENE_MODES = ("static", "perception")


def check_scene_model_list(expected_names, observed_names, scene_mode="static",
                           max_dynamic_models=8):
    """Validate a Gazebo model-name list against the verified scene contract.

    ``expected_names`` is the verified scene contract (metadata obstacle
    models plus the vehicle and ground plane); ``observed_names`` is any
    live model-name listing (service response or /gazebo/model_states keys).
    Returns the sorted tuple of extra dynamic model names — empty in static
    mode, which keeps the legacy exact-match behaviour:

      - any difference in static mode    -> GAZEBO_MODEL_LIST_MISMATCH
      - an expected model absent         -> GAZEBO_MODEL_LIST_MISSING:<names>
      - perception extras over the cap   -> GAZEBO_MODEL_LIST_OVERFLOW:<names>
    """
    if scene_mode not in SCENE_MODES:
        raise RouteError("SCENE_MODE_INVALID:" + str(scene_mode))
    if isinstance(max_dynamic_models, bool) or not isinstance(max_dynamic_models, int) \
            or max_dynamic_models < 0:
        raise RouteError("SCENE_MODE_DYNAMIC_LIMIT_INVALID")
    expected, observed = set(expected_names), set(observed_names)
    missing = sorted(expected - observed)
    extras = sorted(observed - expected)
    if scene_mode == "static":
        if missing or extras:
            raise RouteError("GAZEBO_MODEL_LIST_MISMATCH")
        return ()
    if missing:
        raise RouteError("GAZEBO_MODEL_LIST_MISSING:" + ",".join(missing))
    if len(extras) > max_dynamic_models:
        raise RouteError("GAZEBO_MODEL_LIST_OVERFLOW:" + ",".join(extras))
    return tuple(extras)


def perception_depth_age_s(last_depth_sim, sim, move_started_sim):
    """Observation age in simulated seconds for the perception watchdog.

    ``last_depth_sim`` is the stamp of the most recent depth frame (sim
    time); ``move_started_sim`` is when the MOVE phase first began and is
    used only while no frame has ever arrived (camera missing from launch).

    The age deliberately compares sim stamps against sim ``now``: a paused
    Gazebo freezes both ends at once, so a physics pause can never be
    misread as a perception outage.  For a live-but-delayed stream the age
    *is* the end-to-end observation latency, which caps acceptable sensor
    delay with the same constant.

    Returns None when the clock has not started yet (no frames and no MOVE
    begin), which the caller must treat as fresh.
    """
    if last_depth_sim is not None:
        return sim - last_depth_sim
    if move_started_sim is not None:
        return sim - move_started_sim
    return None


class RouteError(ValueError):
    """A scene, geometry, transform, or route is not safe to execute."""


def acceleration_limited_slew(current, target, velocity, dt, xy_speed, z_speed, xy_accel, z_accel):
    """One bounded-acceleration position-setpoint step, independently testable."""
    dt = max(0.0, min(0.1, _finite(dt, "SLEW_DT")))
    values = (_finite(xy_speed, "SLEW_XY_SPEED"), _finite(z_speed, "SLEW_Z_SPEED"),
              _finite(xy_accel, "SLEW_XY_ACCEL"), _finite(z_accel, "SLEW_Z_ACCEL"))
    if min(values) <= 0:
        raise RouteError("SLEW_LIMIT_NOT_POSITIVE")
    def vector(item, name):
        if not isinstance(item, (tuple, list)) or len(item) != 3:
            raise RouteError(name + "_INVALID")
        return tuple(_finite(value, name) for value in item)
    current, target, velocity = (vector(item, name) for item, name in
                                 ((current, "SLEW_CURRENT"), (target, "SLEW_TARGET"), (velocity, "SLEW_VELOCITY")))
    # Brake-speed target sqrt(2*a*d) ensures a zero-velocity waypoint is
    # reachable without an abrupt target change. The terminal snap is allowed
    # only when its remaining velocity is itself within one acceleration tick.
    dx, dy = target[0] - current[0], target[1] - current[1]
    distance = math.hypot(dx, dy)
    # Discrete stopping bound: a velocity v needs v²/(2a) + v*dt/2
    # metres before its sampled acceleration sequence reaches zero.
    brake_xy = max(0.0, (math.sqrt((values[2] * dt) ** 2 + 8.0 * values[2] * distance) - values[2] * dt) / 2.0 - values[2] * dt)
    wanted = min(values[0], brake_xy)
    desired_xy = (0.0, 0.0) if distance < 1e-9 else (wanted * dx / distance, wanted * dy / distance)
    delta = (desired_xy[0] - velocity[0], desired_xy[1] - velocity[1])
    delta_length = math.hypot(*delta)
    scale = min(1.0, values[2] * dt / delta_length) if delta_length else 0.0
    next_xy = (velocity[0] + delta[0] * scale, velocity[1] + delta[1] * scale)
    step_xy = (next_xy[0] * dt, next_xy[1] * dt)
    # At the last discrete braking quantum the conservative brake formula can
    # ask for zero speed one sub-step before the exact endpoint.  Finish that
    # residual only when both the residual velocity and current velocity fit
    # inside one acceleration tick; this keeps the sampled position derivative
    # within the same bound and avoids an asymptotic stop short of the target.
    if (distance <= values[2] * dt * dt + 1e-9 and
            math.hypot(velocity[0], velocity[1]) <= values[2] * dt + 1e-9):
        step_xy, next_xy = (dx, dy), (0.0, 0.0)
    elif math.hypot(*step_xy) > distance:
        # Terminal-crossing rule: crossing the target while still faster than
        # one acceleration tick is only tolerable when the crossing speed is
        # inside the vehicle's real flight envelope — that is the benign PX4
        # waypoint-overshoot signature, and the model may snap its reference
        # (the physical controller settles the last centimetres).  A crossing
        # FASTER than the vehicle can ever fly is a fabricated state, not an
        # overshoot: fail closed instead of masking a runaway.
        if math.hypot(*next_xy) > values[0] + 1e-9:
            raise RouteError("SLEW_BRAKE_INVARIANT_VIOLATED")
        step_xy, next_xy = (dx, dy), (0.0, 0.0)
    dz = target[2] - current[2]
    brake_z = max(0.0, (math.sqrt((values[3] * dt) ** 2 + 8.0 * values[3] * abs(dz)) - values[3] * dt) / 2.0 - values[3] * dt)
    desired_z = math.copysign(min(values[1], brake_z), dz) if dz else 0.0
    next_z = velocity[2] + max(-values[3] * dt, min(values[3] * dt, desired_z - velocity[2]))
    step_z = next_z * dt
    if abs(dz) <= values[3] * dt * dt + 1e-9 and abs(velocity[2]) <= values[3] * dt + 1e-9:
        step_z, next_z = dz, 0.0
    elif abs(step_z) > abs(dz):
        # Same envelope rule on the vertical axis: a crossing at or below the
        # configured vertical speed limit is a benign overshoot; anything
        # faster fails closed.
        if abs(next_z) > values[1] + 1e-9:
            raise RouteError("SLEW_BRAKE_INVARIANT_VIOLATED")
        step_z, next_z = dz, 0.0
    return (current[0] + step_xy[0], current[1] + step_xy[1], current[2] + step_z), next_xy + (next_z,)


@dataclass(frozen=True)
class ExpectedModel:
    name: str
    model_position: tuple
    yaw_rad: float
    size: tuple
    geometry_center: tuple


@dataclass(frozen=True)
class RoutePlan:
    run_scene: dict
    metadata_sha256: str
    world_sha256: str
    metadata: dict
    expected_models: tuple
    ground_model: ExpectedModel
    start_world: tuple
    goal_world: tuple
    route_world: tuple
    safety: object
    planning_elapsed_s: float


class GridSource:
    """Versioned producer of the planning grid behind a route.

    Two producers share one contract: ``snapshot()`` returns an immutable
    ``(grid, map_version)`` pair, where ``map_version`` is a monotonic
    integer that must change whenever the world observation underlying the
    grid changes enough to invalidate routes planned earlier (a perception
    window recentred, an obstacle update fused, a static map reloaded).
    Planners plan on the snapshot grid and re-check the version before
    committing; a moved version means the world raced the plan and the
    attempt must be retried on fresher data.

    This is the seam that lets the same replan chain run on the verified
    scene-metadata grid (static) and on a live LocalOccupancyGrid
    (perception) without knowing or caring which one backs it.
    """

    def __init__(self, grid=None, map_version=0, occupancy=None):
        if occupancy is not None:
            # Duck-typed LocalOccupancyGrid: held by reference, folded anew
            # on every snapshot so versions move with the perception window.
            if (not callable(getattr(occupancy, "to_grid_map", None)) or
                    not hasattr(occupancy, "map_version")):
                raise RouteError("GRID_SOURCE_OCCUPANCY_INVALID")
            self._occupancy = occupancy
            self._grid = None
            self._map_version = 0
            self._base_grid = None
            self._inflation_m = 0.0
            self._shadow_cells = 0
        else:
            self._occupancy = None
            self._base_grid = None
            self._inflation_m = 0.0
            self._shadow_cells = 0
            self._grid = self._validated_grid(grid)
            self._map_version = self._validated_version(map_version)

    @staticmethod
    def _validated_grid(grid):
        if grid is None or not callable(getattr(grid, "world_to_cell", None)) or \
                not callable(getattr(grid, "is_free", None)):
            raise RouteError("GRID_SOURCE_GRID_INVALID")
        return grid

    @staticmethod
    def _validated_version(map_version):
        if isinstance(map_version, bool) or not isinstance(map_version, int) or map_version < 0:
            raise RouteError("GRID_SOURCE_VERSION_INVALID")
        return map_version

    @classmethod
    def from_metadata(cls, metadata, map_version=0):
        """Static source: fold the verified scene metadata RLE grid once."""
        try:
            grid = GridMap.from_metadata(metadata)
        except (MapError, KeyError, TypeError, ValueError) as exc:
            raise RouteError("GRID_SOURCE_METADATA_INVALID:" + str(exc))
        return cls(grid, map_version)

    @classmethod
    def from_occupancy(cls, occupancy, base_grid=None, inflation_m=0.0,
                       shadow_cells=0):
        """Live source: any LocalOccupancyGrid-like object exposing
        ``to_grid_map()`` and a monotonic ``map_version``.

        ``base_grid`` merges the perception window over the verified static
        scene grid: a perceived OCCUPIED cell blocks, a perceived FREE cell
        says nothing beyond the base map, and an UNKNOWN cell defers to the
        base map — so goals behind an occluder stay plannable while newly
        spawned obstacles block.  ``inflation_m`` dilates the merged grid so
        A* keeps physical clearance against obstacles the continuous
        SafetyMap cannot see (dynamic geometry).  ``shadow_cells`` promotes
        UNKNOWN cells within that many steps of a perceived obstacle to
        occupied, bounding how far an obstacle can be assumed to continue
        into the unscanned FOV shadow (typically one clearance budget:
        ceil(inflation_m / resolution))."""
        source = cls(occupancy=occupancy)
        if isinstance(shadow_cells, bool) or not isinstance(shadow_cells, int) \
                or shadow_cells < 0:
            raise RouteError("GRID_SOURCE_SHADOW_CELLS_INVALID")
        source._shadow_cells = shadow_cells
        if base_grid is not None:
            if not (hasattr(base_grid, "cells") and hasattr(base_grid, "width")
                    and callable(getattr(base_grid, "world_to_cell", None))
                    and callable(getattr(base_grid, "cell_to_world", None))):
                raise RouteError("GRID_SOURCE_BASE_GRID_INVALID")
            source._base_grid = base_grid
        source._inflation_m = _finite(inflation_m, "GRID_SOURCE_INFLATION")
        if source._inflation_m < 0.0:
            raise RouteError("GRID_SOURCE_INFLATION_NEGATIVE")
        return source

    def _merged_grid(self, perceived):
        """OR the perceived occupied cells over a copy of the base map."""
        base = self._base_grid
        cells = list(base.cells)
        width, height, resolution = base.width, base.height, base.resolution
        origin_x, origin_y = base.origin[0], base.origin[1]
        for cy in range(perceived.height):
            row = cy * perceived.width
            for cx in range(perceived.width):
                if not perceived.cells[row + cx]:
                    continue
                wx = perceived.origin[0] + (cx + 0.5) * perceived.resolution
                wy = perceived.origin[1] + (cy + 0.5) * perceived.resolution
                cell = base.world_to_cell((wx, wy))
                if cell is not None:
                    cells[cell[1] * width + cell[0]] = 1
        return GridMap(width, height, resolution, tuple(base.origin), cells, base.frame_id)

    def snapshot(self):
        """Return the current ``(grid, map_version)`` pair.

        The returned grid is a freshly folded immutable view; callers must
        treat it as valid only for the returned version.
        """
        if self._occupancy is not None:
            try:
                # Two folds: PLAIN (no shadow) is inflated for clearance;
                # SHADOWED promotes UNKNOWN cells near OCCUPIED (FOV-edge
                # obstacle continuation) but is unioned WITHOUT re-inflation.
                # Inflating the shadowed fold stacked both reaches
                # (shadow_m + inflation_m) and sealed goals that merely sit
                # near a wall the camera had observed.
                if self._base_grid is not None:
                    plain = self._occupancy.to_grid_map(unknown_is_obstacle=False)
                    shadowed = self._occupancy.to_grid_map(
                        unknown_is_obstacle=False,
                        obstacle_shadow_cells=self._shadow_cells)
                else:
                    plain = self._occupancy.to_grid_map()
                    shadowed = self._occupancy.to_grid_map(
                        obstacle_shadow_cells=self._shadow_cells)
                version = self._occupancy.map_version
                revision = getattr(self._occupancy, "content_revision", None)
                if isinstance(revision, int) and not isinstance(revision, bool):
                    self._last_freshness = (version, revision)
                else:
                    self._last_freshness = (version,)
            except (TypeError, ValueError, AttributeError) as exc:
                raise RouteError("GRID_SOURCE_OCCUPANCY_FOLD_FAILED:" + str(exc))
            grid = self._validated_grid(plain)
            shadow = self._validated_grid(shadowed)
            version = self._validated_version(version)
            if self._base_grid is not None:
                grid = self._merged_grid(grid)
                shadow = self._merged_grid(shadow)
            if self._inflation_m > 0.0:
                grid = _inflate_grid(grid, self._inflation_m)
            if any(shadow.cells):
                merged = [1 if (p or s) else 0
                          for p, s in zip(grid.cells, shadow.cells)]
                grid = GridMap(grid.width, grid.height, grid.resolution,
                               tuple(grid.origin), merged, grid.frame_id)
            return grid, version
        return self._grid, self._map_version

    def freshness(self):
        """The staleness token captured at the LAST snapshot: map_version plus
        (when the occupancy exposes one) the content revision.  map_version
        alone only moves on window recenter, so content changes between the
        planning snapshot and the commit snapshot would otherwise pass a
        version-only comparison.  Corridor dedup keeps using map_version —
        content churn must never open a replan storm.  Metadata sources are
        immutable and always return the same token."""
        if self._occupancy is not None:
            return getattr(self, "_last_freshness", (self._map_version,))
        return (self._map_version,)


def _inflate_grid(grid, radius_m):
    """Return a copy of ``grid`` with every cell within ``radius_m`` of an
    occupied cell centre blocked (euclidean, cell space)."""
    if radius_m <= 0.0:
        return grid
    width, height, resolution = grid.width, grid.height, grid.resolution
    reach = int(math.floor(radius_m / resolution + 1e-9))
    if reach <= 0:
        return grid
    cells = list(grid.cells)
    occupied = [divmod(index, width) for index, value in enumerate(cells) if value == 1]
    if not occupied:
        return GridMap(width, height, resolution, tuple(grid.origin), cells, grid.frame_id)
    limit = (radius_m / resolution) ** 2
    for oy, ox in occupied:
        for dy in range(-reach, reach + 1):
            yy = oy + dy
            if yy < 0 or yy >= height:
                continue
            span = int(math.sqrt(max(0.0, limit - dy * dy)) + 1e-9)
            row = yy * width
            for xx in range(max(0, ox - span), min(width, ox + span + 1)):
                cells[row + xx] = 1
    return GridMap(width, height, resolution, tuple(grid.origin), cells, grid.frame_id)


def _finite(value, label):
    if isinstance(value, bool):
        raise RouteError(label + "_INVALID")
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise RouteError(label + "_INVALID")
    if not math.isfinite(value):
        raise RouteError(label + "_NONFINITE")
    return value


def _finite_world_xy(value, label):
    """Return a strict, finite world XY pair for an online replan request.

    Route configuration is loaded from YAML and consequently accepts values
    convertible to float.  A live Gazebo position / external goal is different:
    it is an in-memory numeric contract.  Rejecting strings and booleans here
    prevents an accidental log or message field from silently becoming a flight
    command.
    """
    if isinstance(value, (str, bytes)) or not isinstance(value, (tuple, list)) or len(value) != 2:
        raise RouteError(label + "_INVALID")
    parsed = []
    for component in value:
        if isinstance(component, bool) or not isinstance(component, Real):
            raise RouteError(label + "_INVALID")
        component = float(component)
        if not math.isfinite(component):
            raise RouteError(label + "_NONFINITE")
        parsed.append(component)
    return tuple(parsed)


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_json(path, label):
    try:
        with open(path, encoding="utf-8") as stream:
            return json.load(stream)
    except (OSError, ValueError) as exc:
        raise RouteError(label + "_INVALID:" + str(exc))


def _same_path(left, right):
    # Container paths are authoritative in a receipt; do not silently resolve
    # a host path to a different mounted source.
    return os.path.normpath(left) == os.path.normpath(right)


def load_scene_receipt(config, now_wall_s=None, expected_container_id=None):
    """Validate the fresh-launch proof and return its document.

    `created_utc` is deliberately Unix wall time, not ROS time: a stopped
    simulator must not make an old scene receipt appear fresh.
    """
    receipt_path = config.get("scene_receipt_file")
    if not isinstance(receipt_path, str) or not receipt_path:
        raise RouteError("SCENE_RECEIPT_FILE_MISSING")
    receipt = _read_json(receipt_path, "SCENE_RECEIPT")
    required = ("launch_id", "created_utc", "world_file", "metadata_file",
                "world_sha256", "metadata_sha256", "container_id", "container_started_at")
    if not isinstance(receipt, dict) or receipt.get("schema") != RECEIPT_SCHEMA:
        raise RouteError("SCENE_RECEIPT_SCHEMA_INVALID")
    if any(key not in receipt for key in required):
        raise RouteError("SCENE_RECEIPT_FIELDS_MISSING")
    if not isinstance(receipt["launch_id"], str) or not receipt["launch_id"]:
        raise RouteError("SCENE_RECEIPT_LAUNCH_ID_INVALID")
    created = _finite(receipt["created_utc"], "SCENE_RECEIPT_CREATED")
    now = time.time() if now_wall_s is None else _finite(now_wall_s, "NOW")
    ttl = _finite(config.get("scene_receipt_max_age_s", 300.0), "SCENE_RECEIPT_TTL")
    if ttl <= 0 or created > now + 5 or now - created > ttl:
        raise RouteError("SCENE_RECEIPT_STALE")
    for key in ("world_file", "metadata_file", "container_id"):
        if not isinstance(receipt[key], str) or not receipt[key]:
            raise RouteError("SCENE_RECEIPT_" + key.upper() + "_INVALID")
    if expected_container_id and not receipt["container_id"].startswith(str(expected_container_id)):
        raise RouteError("SCENE_RECEIPT_CONTAINER_MISMATCH")
    for key in ("world_sha256", "metadata_sha256"):
        if not isinstance(receipt[key], str) or len(receipt[key]) != 64:
            raise RouteError("SCENE_RECEIPT_" + key.upper() + "_INVALID")
    if not _same_path(receipt["world_file"], config["world_file"]):
        raise RouteError("SCENE_RECEIPT_WORLD_PATH_MISMATCH")
    if not _same_path(receipt["metadata_file"], config["metadata_file"]):
        raise RouteError("SCENE_RECEIPT_METADATA_PATH_MISMATCH")
    if _sha256(config["world_file"]) != receipt["world_sha256"]:
        raise RouteError("WORLD_SHA256_MISMATCH")
    if _sha256(config["metadata_file"]) != receipt["metadata_sha256"]:
        raise RouteError("METADATA_SHA256_MISMATCH")
    return receipt


def _pose(element, label):
    raw = (element.text or "").split()
    if len(raw) != 6:
        raise RouteError(label + "_POSE_INVALID")
    values = tuple(_finite(value, label + "_POSE") for value in raw)
    if abs(values[3]) > 1e-8 or abs(values[4]) > 1e-8:
        raise RouteError(label + "_ROLL_PITCH_UNSUPPORTED")
    return values


def parse_world_static_boxes(world_file):
    """Extract only the strict static, axis-aligned box form used in phase 1."""
    try:
        root = ET.parse(world_file).getroot()
    except (OSError, ET.ParseError) as exc:
        raise RouteError("WORLD_XML_INVALID:" + str(exc))
    if root.tag != "sdf" or len(root.findall("world")) != 1:
        raise RouteError("WORLD_ELEMENT_MISSING")
    world = root.find("world")
    if any(child.tag not in ("light", "model", "physics") for child in world):
        raise RouteError("WORLD_UNSUPPORTED_ELEMENT")
    models = {}
    for model in world.findall("model"):
        name = model.get("name")
        if not name:
            raise RouteError("WORLD_MODEL_NAME_MISSING")
        if name == "training_ground":
            _validate_ground(model)
            continue
        if any(child.tag not in ("static", "pose", "link") for child in model):
            raise RouteError("WORLD_MODEL_UNSUPPORTED_ELEMENT:" + name)
        if (model.findtext("static") or "").strip().lower() != "true":
            raise RouteError("WORLD_MODEL_NOT_STATIC:" + name)
        model_pose = _pose(model.find("pose"), "WORLD_MODEL_" + name) if model.find("pose") is not None else (0., 0., 0., 0., 0., 0.)
        links = model.findall("link")
        if len(links) != 1:
            raise RouteError("WORLD_MODEL_LINK_COUNT:" + name)
        link = links[0]
        if any(child.tag not in ("pose", "collision", "visual", "inertial") for child in link):
            raise RouteError("WORLD_LINK_UNSUPPORTED_ELEMENT:" + name)
        link_pose = _pose(link.find("pose"), "WORLD_LINK_" + name) if link.find("pose") is not None else (0., 0., 0., 0., 0., 0.)
        if any(abs(value) > 1e-8 for value in link_pose[:2] + link_pose[3:]):
            raise RouteError("WORLD_LINK_TRANSFORM_UNSUPPORTED:" + name)
        collisions = link.findall("collision")
        if len(collisions) != 1:
            raise RouteError("WORLD_COLLISION_COUNT:" + name)
        if any(child.tag not in ("pose", "geometry", "surface") for child in collisions[0]):
            raise RouteError("WORLD_COLLISION_UNSUPPORTED_ELEMENT:" + name)
        collision_pose = collisions[0].find("pose")
        if collision_pose is not None and any(abs(value) > 1e-8 for value in _pose(collision_pose, "WORLD_COLLISION_" + name)):
            raise RouteError("WORLD_COLLISION_TRANSFORM_UNSUPPORTED:" + name)
        size_text = collisions[0].findtext("geometry/box/size")
        geometry = collisions[0].find("geometry")
        if geometry is None or len(geometry) != 1 or geometry[0].tag != "box" or size_text is None:
            raise RouteError("WORLD_NON_BOX_COLLISION:" + name)
        values = tuple(_finite(value, "WORLD_BOX_SIZE") for value in size_text.split())
        if len(values) != 3 or min(values) <= 0:
            raise RouteError("WORLD_BOX_SIZE_INVALID:" + name)
        if name in models:
            raise RouteError("WORLD_MODEL_DUPLICATE:" + name)
        models[name] = ExpectedModel(name, model_pose[:3], model_pose[5], values,
                                     (model_pose[0], model_pose[1], model_pose[2] + link_pose[2]))
    return models


def _validate_ground(model):
    """The only non-box collision allowed by phase 1 is its named ground plane."""
    if (model.findtext("static") or "").strip().lower() != "true" or any(
            child.tag not in ("static", "pose", "link") for child in model):
        raise RouteError("WORLD_GROUND_INVALID")
    pose = model.find("pose")
    if pose is not None and any(abs(value) > 1e-8 for value in _pose(pose, "WORLD_GROUND")):
        raise RouteError("WORLD_GROUND_POSE_INVALID")
    links = model.findall("link")
    if len(links) != 1 or any(child.tag not in ("collision", "visual") for child in links[0]):
        raise RouteError("WORLD_GROUND_LINK_INVALID")
    collisions = links[0].findall("collision")
    if len(collisions) != 1 or collisions[0].find("pose") is not None:
        raise RouteError("WORLD_GROUND_COLLISION_INVALID")
    geometry = collisions[0].find("geometry")
    if geometry is None or len(geometry) != 1 or geometry[0].tag != "plane":
        raise RouteError("WORLD_GROUND_GEOMETRY_INVALID")
    normal = (geometry.findtext("plane/normal") or "").split()
    size = (geometry.findtext("plane/size") or "").split()
    if tuple(normal) != ("0", "0", "1") or len(size) != 2 or min(_finite(value, "WORLD_GROUND_SIZE") for value in size) < 100:
        raise RouteError("WORLD_GROUND_PLANE_INVALID")
    return ExpectedModel("training_ground", (0.0, 0.0, 0.0), 0.0,
                         tuple(_finite(value, "WORLD_GROUND_SIZE") for value in size) + (0.0,),
                         (0.0, 0.0, 0.0))


def parse_ground_model(world_file):
    try:
        root = ET.parse(world_file).getroot()
        world = root.find("world")
        grounds = [model for model in world.findall("model") if model.get("name") == "training_ground"]
    except (OSError, ET.ParseError, AttributeError) as exc:
        raise RouteError("WORLD_XML_INVALID:" + str(exc))
    if len(grounds) != 1:
        raise RouteError("WORLD_GROUND_MISSING")
    return _validate_ground(grounds[0])


def expected_models_from_metadata(metadata):
    obstacles = metadata.get("obstacles")
    if not isinstance(obstacles, list) or not obstacles:
        raise RouteError("METADATA_OBSTACLES_MISSING")
    expected = {}
    for obstacle in obstacles:
        if not isinstance(obstacle, dict) or obstacle.get("shape") != "box" or obstacle.get("blocking") is not True:
            raise RouteError("UNSUPPORTED_METADATA_GEOMETRY")
        name = obstacle.get("id")
        center, size = obstacle.get("center"), obstacle.get("size")
        if not isinstance(name, str) or not isinstance(center, list) or len(center) != 2 or not isinstance(size, list) or len(size) != 2:
            raise RouteError("METADATA_BOX_INVALID")
        yaw = _finite(obstacle.get("yaw"), "METADATA_YAW")
        z_min, z_max = _finite(obstacle.get("z_min"), "METADATA_Z_MIN"), _finite(obstacle.get("z_max"), "METADATA_Z_MAX")
        if abs(yaw) > 1e-8 or abs(z_min) > 1e-8 or z_max <= z_min:
            raise RouteError("UNSUPPORTED_METADATA_BOX_TRANSFORM:" + name)
        dimensions = (_finite(size[0], "METADATA_SIZE_X"), _finite(size[1], "METADATA_SIZE_Y"), z_max - z_min)
        if min(dimensions) <= 0 or name in expected:
            raise RouteError("METADATA_BOX_SIZE_OR_ID_INVALID")
        x, y = _finite(center[0], "METADATA_CENTER_X"), _finite(center[1], "METADATA_CENTER_Y")
        expected[name] = ExpectedModel(name, (x, y, z_min), yaw, dimensions,
                                       (x, y, z_min + dimensions[2] / 2.0))
    return expected


def _near(left, right, tolerance=1e-5):
    return abs(left - right) <= tolerance


def verify_world_geometry(metadata, world_file):
    expected = expected_models_from_metadata(metadata)
    parsed = parse_world_static_boxes(world_file)
    if set(expected) != set(parsed):
        raise RouteError("WORLD_METADATA_MODEL_SET_MISMATCH")
    for name, item in expected.items():
        actual = parsed[name]
        if (not all(_near(a, b) for a, b in zip(item.model_position, actual.model_position)) or
                not _near(item.yaw_rad, actual.yaw_rad) or
                not all(_near(a, b) for a, b in zip(item.size, actual.size)) or
                not all(_near(a, b) for a, b in zip(item.geometry_center, actual.geometry_center))):
            raise RouteError("WORLD_METADATA_GEOMETRY_MISMATCH:" + name)
    return tuple(expected[name] for name in sorted(expected))


def _remove_strict_collinear(points):
    result = []
    for point in points:
        if result and point == result[-1]:
            continue
        result.append(point)
        while len(result) >= 3:
            a, b, c = result[-3:]
            ab, bc = (b[0] - a[0], b[1] - a[1]), (c[0] - b[0], c[1] - b[1])
            if abs(ab[0] * bc[1] - ab[1] * bc[0]) > 1e-10 or ab[0] * bc[0] + ab[1] * bc[1] < 0:
                break
            result.pop(-2)
    return tuple(result)


def _route_safety(metadata, config):
    try:
        from robocup_navigation.route_safety import SafetyMap, SafetyError
    except ImportError as exc:
        raise RouteError("ROUTE_SAFETY_UNAVAILABLE:" + str(exc))
    try:
        return SafetyMap(metadata, vehicle_radius_m=config["vehicle_radius_m"],
                         horizontal_margin_m=config["horizontal_margin_m"],
                         vehicle_half_height_m=config["vehicle_half_height_m"],
                         vertical_margin_m=config["vertical_margin_m"]), SafetyError
    except (SafetyError, ValueError, KeyError, TypeError) as exc:
        raise RouteError("ROUTE_SAFETY_MAP_INVALID:" + str(exc))


def prepare_route(config, now_wall_s=None, expected_container_id=None):
    """Return a fresh checked plan.  It is safe to call from a bounded worker."""
    receipt = load_scene_receipt(config, now_wall_s, expected_container_id)
    begun = time.monotonic()
    try:
        metadata, metadata_sha = load_metadata(config["metadata_file"])
        grid = GridMap.from_metadata(metadata)
        start_xy, goal_xy, _goal_id = metadata_endpoints(metadata, config.get("route_goal_id"), True)
    except (MapError, OSError, KeyError, TypeError, ValueError) as exc:
        raise RouteError("METADATA_PLAN_INPUT_INVALID:" + str(exc))
    if metadata_sha != receipt["metadata_sha256"]:
        raise RouteError("METADATA_CHANGED_DURING_READ")
    _category = metadata.get("generator", {}).get("category")
    if _category not in ROUTE_MODE_CATEGORIES:
        raise RouteError("ROUTE_MODE_UNSUPPORTED_CATEGORY:" + str(_category))
    grid_inflation = _finite(metadata.get("grid", {}).get("inflation_m"), "GRID_INFLATION")
    planning_clearance = (_finite(config["vehicle_radius_m"], "VEHICLE_RADIUS") +
                          _finite(config["horizontal_margin_m"], "HORIZONTAL_MARGIN") +
                          _finite(config["planning_tracking_margin_m"], "PLANNING_TRACKING_MARGIN"))
    metadata_radius = _finite(metadata.get("safety", {}).get("robot_radius_m"), "METADATA_ROBOT_RADIUS")
    if metadata_radius + 1e-9 < config["vehicle_radius_m"]:
        raise RouteError("METADATA_ROBOT_RADIUS_TOO_SMALL")
    if grid_inflation + 1e-9 < planning_clearance:
        raise RouteError("GRID_INFLATION_TOO_SMALL_FOR_EXECUTION")
    altitude = _finite(config["route_altitude_m"], "ROUTE_ALTITUDE")
    ground = _finite(metadata.get("frame", {}).get("ground_z"), "GROUND_Z")
    if not _near(altitude, _finite(metadata.get("planning_altitude_m"), "METADATA_PLANNING_ALTITUDE")):
        raise RouteError("ROUTE_ALTITUDE_METADATA_MISMATCH")
    if tuple(start_xy) != tuple(config["route_start_world_xy"]) or tuple(goal_xy) != tuple(config["route_goal_world_xy"]):
        raise RouteError("ROUTE_ENDPOINT_METADATA_MISMATCH")
    expected = verify_world_geometry(metadata, config["world_file"])
    ground_model = parse_ground_model(config["world_file"])
    result = plan(grid, start_xy, goal_xy, connectivity=8, prevent_corner_cutting=True,
                  max_expansions=int(config["route_max_expansions"]))
    elapsed = time.monotonic() - begun
    if elapsed > _finite(config["route_planning_timeout_s"], "ROUTE_PLANNING_TIMEOUT"):
        raise RouteError("ROUTE_PLANNING_TIMEOUT")
    if not result.success:
        raise RouteError("ROUTE_PLAN_FAILED:" + result.reason)
    safety, safety_error = _route_safety(metadata, config)
    origin = (start_xy[0], start_xy[1], ground)
    try:
        raw_xy = (tuple(start_xy),) + tuple(result.points) + (tuple(goal_xy),)
        # A grid diagonal can clip the exact continuous centre offset.
        # Expand it through an A* corner-neighbour, selecting only a candidate
        # that SafetyMap proves safe; no smoothing is allowed afterwards.
        safe_xy = [raw_xy[0]]
        for current, following in zip(raw_xy, raw_xy[1:]):
            candidates = [()]
            if current[0] != following[0] and current[1] != following[1]:
                candidates = [(following[0], current[1]), (current[0], following[1])]
            selected = None
            for middle in candidates:
                trial = tuple((point[0], point[1], altitude) for point in
                              ((safe_xy[-1],) + ((middle,) if middle else ()) + (following,)))
                try:
                    safety.validate_route(trial, origin, config["max_radius_m"], config["max_altitude_m"])
                    selected = middle
                    break
                except safety_error:
                    pass
            if selected is None and candidates != [()]:
                raise RouteError("ROUTE_DIAGONAL_CLEARANCE_FAILED")
            if selected:
                safe_xy.append(selected)
            safe_xy.append(following)
        route_xy = _remove_strict_collinear(tuple(safe_xy))
        route_world = tuple((point[0], point[1], altitude) for point in route_xy)
        # Include the actual vertical takeoff corridor in continuous geometry
        # validation; it is not merely inferred from the A* cruise layer.
        report = safety.validate_route((origin,) + route_world, origin,
                                       config["max_radius_m"], config["max_altitude_m"])
        if not safety.landing_safe((start_xy[0], start_xy[1], ground)) or not safety.landing_safe((goal_xy[0], goal_xy[1], ground)):
            raise RouteError("LANDING_CORRIDOR_UNSAFE")
        if (not isinstance(report, dict) or
                _finite(report.get("minimum_clearance_m"), "ROUTE_CLEARANCE") + 1e-9 <
                config["minimum_clearance_m"] + config["planning_tracking_margin_m"]):
            raise RouteError("ROUTE_CLEARANCE_INSUFFICIENT")
    except safety_error as exc:
        raise RouteError("ROUTE_SAFETY_FAILED:" + str(exc))
    return RoutePlan(receipt, metadata_sha, receipt["world_sha256"], metadata, expected, ground_model,
                     (start_xy[0], start_xy[1], ground), (goal_xy[0], goal_xy[1], ground),
                     route_world, safety, elapsed)


def replan_route(route, current_world_xy, goal_world_xy, config, grid=None):
    """Compute an executable, immutable A* route from live world coordinates.

    ``route`` must be the already checked :class:`RoutePlan` returned by
    :func:`prepare_route`; this function intentionally does not read files,
    validate a scene receipt, or publish ROS messages.  By default it
    reconstructs the grid from that plan's verified metadata; callers with a
    live perception stack may pass an explicit ``grid`` (a GridMap folded by
    :class:`GridSource` from a LocalOccupancyGrid snapshot) and the plan is
    then proven against that observed world instead.  In both cases the
    existing continuous ``SafetyMap`` proves every exact connector and route
    segment.

    The returned tuple contains world-ENU ``(x, y, route_altitude_m)`` points.
    Its first and last elements are the exact supplied live position and goal,
    respectively (unless they are identical).  All failure strings are stable
    ``RouteError`` reasons suitable for an event log or controller state.
    """
    if not isinstance(route, RoutePlan):
        raise RouteError("REPLAN_ROUTE_INVALID")
    if not isinstance(config, dict):
        raise RouteError("REPLAN_CONFIG_INVALID")
    if route.safety is None or not hasattr(route.safety, "validate_route"):
        raise RouteError("REPLAN_SAFETY_UNAVAILABLE")

    current_xy = _finite_world_xy(current_world_xy, "REPLAN_START")
    goal_xy = _finite_world_xy(goal_world_xy, "REPLAN_GOAL")
    if grid is None:
        try:
            grid = GridMap.from_metadata(route.metadata)
        except (MapError, KeyError, TypeError, ValueError) as exc:
            raise RouteError("REPLAN_GRID_INVALID:" + str(exc))
    elif not (callable(getattr(grid, "world_to_cell", None)) and
              callable(getattr(grid, "is_free", None))):
        raise RouteError("REPLAN_GRID_SOURCE_INVALID")

    # Repeat the execution-margin agreement for every replan.  The original
    # RoutePlan proves the scene was valid when loaded; callers can still pass
    # a mismatched runtime config later, and that must fail closed.
    try:
        grid_inflation = _finite(route.metadata.get("grid", {}).get("inflation_m"), "REPLAN_GRID_INFLATION")
        metadata_radius = _finite(route.metadata.get("safety", {}).get("robot_radius_m"), "REPLAN_METADATA_ROBOT_RADIUS")
        vehicle_radius = _finite(config["vehicle_radius_m"], "REPLAN_VEHICLE_RADIUS")
        horizontal_margin = _finite(config["horizontal_margin_m"], "REPLAN_HORIZONTAL_MARGIN")
        vehicle_half_height = _finite(config["vehicle_half_height_m"], "REPLAN_VEHICLE_HALF_HEIGHT")
        vertical_margin = _finite(config["vertical_margin_m"], "REPLAN_VERTICAL_MARGIN")
        tracking_margin = _finite(config["planning_tracking_margin_m"], "REPLAN_TRACKING_MARGIN")
        altitude = _finite(config["route_altitude_m"], "REPLAN_ROUTE_ALTITUDE")
        max_radius = _finite(config["max_radius_m"], "REPLAN_MAX_RADIUS")
        max_altitude = _finite(config["max_altitude_m"], "REPLAN_MAX_ALTITUDE")
        minimum_clearance = _finite(config["minimum_clearance_m"], "REPLAN_MINIMUM_CLEARANCE")
        planning_timeout = _finite(config["route_planning_timeout_s"], "REPLAN_PLANNING_TIMEOUT")
        max_expansions = config["route_max_expansions"]
        if isinstance(max_expansions, bool) or not isinstance(max_expansions, int) or max_expansions <= 0:
            raise RouteError("REPLAN_MAX_EXPANSIONS_INVALID")
    except KeyError as exc:
        raise RouteError("REPLAN_CONFIG_MISSING:" + str(exc.args[0]))
    if vehicle_radius <= 0.0 or min(horizontal_margin, vehicle_half_height,
                                    vertical_margin, tracking_margin,
                                    minimum_clearance, max_radius) < 0.0 or planning_timeout <= 0.0:
        raise RouteError("REPLAN_CONFIG_NEGATIVE_LIMIT")
    if metadata_radius + 1e-9 < vehicle_radius:
        raise RouteError("REPLAN_METADATA_ROBOT_RADIUS_TOO_SMALL")
    if grid_inflation + 1e-9 < vehicle_radius + horizontal_margin + tracking_margin:
        raise RouteError("REPLAN_GRID_INFLATION_TOO_SMALL_FOR_EXECUTION")
    if not route.route_world or not _near(altitude, _finite(route.route_world[0][2], "REPLAN_PLAN_ALTITUDE")):
        raise RouteError("REPLAN_ROUTE_ALTITUDE_MISMATCH")
    if any(not _near(_finite(getattr(route.safety, name, None), "REPLAN_SAFETY_" + name.upper()), expected)
           for name, expected in (("vehicle_radius_m", vehicle_radius),
                                  ("horizontal_margin_m", horizontal_margin),
                                  ("vehicle_half_height_m", vehicle_half_height),
                                  ("vertical_margin_m", vertical_margin))):
        raise RouteError("REPLAN_SAFETY_CONFIG_MISMATCH")

    start_cell, goal_cell = grid.world_to_cell(current_xy), grid.world_to_cell(goal_xy)
    if start_cell is None:
        raise RouteError("REPLAN_START_OUT_OF_BOUNDS")
    if goal_cell is None:
        raise RouteError("REPLAN_GOAL_OUT_OF_BOUNDS")
    if not grid.is_free(start_cell):
        raise RouteError("REPLAN_START_OCCUPIED")
    if not grid.is_free(goal_cell):
        raise RouteError("REPLAN_GOAL_OCCUPIED")

    origin = tuple(route.start_world)
    if len(origin) != 3:
        raise RouteError("REPLAN_ORIGIN_INVALID")
    try:
        origin = tuple(_finite(value, "REPLAN_ORIGIN") for value in origin)
        ground = _finite(route.metadata.get("frame", {}).get("ground_z"), "REPLAN_GROUND_Z")
        # The external goal is exact, rather than a grid-cell centre.  Prove
        # that it fits inside the same reserved-radius envelope before A*
        # returns a misleading internal-diagonal error.  The reservation is
        # deliberate: SafetyMap keeps the vehicle body and horizontal margin
        # inside the configured flight fence.
        goal_radius = math.hypot(goal_xy[0] - origin[0], goal_xy[1] - origin[1])
        if goal_radius + route.safety.required_horizontal_clearance_m > max_radius + 1e-9:
            raise RouteError("REPLAN_GOAL_RADIUS_LIMIT")
        # A target may be clear at cruise altitude but impossible to descend
        # onto.  Check its separate vertical corridor before spending a search
        # budget or returning a route that a controller cannot finish.
        if not route.safety.landing_safe((goal_xy[0], goal_xy[1], ground)):
            raise RouteError("REPLAN_GOAL_LANDING_CORRIDOR_UNSAFE")
        begun = time.monotonic()
        result = plan(grid, current_xy, goal_xy, connectivity=8,
                      prevent_corner_cutting=True, max_expansions=max_expansions)
    except RouteError:
        raise
    except (MapError, ValueError, TypeError) as exc:
        raise RouteError("REPLAN_ASTAR_INPUT_INVALID:" + str(exc))
    if time.monotonic() - begun > planning_timeout:
        raise RouteError("REPLAN_PLANNING_TIMEOUT")
    if not result.success:
        raise RouteError("REPLAN_ASTAR_FAILED:" + result.reason)

    safety = route.safety
    safety_error = None
    try:
        # SafetyMap is supplied by the verified RoutePlan.  Its concrete error
        # type is retained only to translate geometry failures into stable
        # replan reasons without importing or rebuilding a second safety map.
        from robocup_navigation.route_safety import SafetyError
        safety_error = SafetyError
        # When current and goal are identical there is no connector to fly.
        # Do not manufacture a trip through the containing cell centre.
        raw_xy = ((current_xy,) if current_xy == goal_xy else
                  (current_xy,) + tuple(result.points) + (goal_xy,))
        safe_xy = [raw_xy[0]]
        for index, following in enumerate(raw_xy[1:]):
            current = safe_xy[-1]
            candidates = [()]
            if current[0] != following[0] and current[1] != following[1]:
                endpoint_connector = index == 0 or index == len(raw_xy) - 2
                # The live start and arbitrary goal are generally not cell
                # centres; their exact straight connector is checked as such.
                # An internal A* diagonal is accepted only when exact
                # continuous geometry proves it safe; otherwise try either
                # checked orthogonal turn.  Grid corner-cutting prevention is
                # still active in A* itself.
                if not endpoint_connector:
                    candidates = [(), (following[0], current[1]), (current[0], following[1])]
            selected = None
            for middle in candidates:
                points = (current,) + ((middle,) if middle else ()) + (following,)
                trial = tuple((point[0], point[1], altitude) for point in points)
                try:
                    safety.validate_route(trial, origin, max_radius, max_altitude)
                    selected = middle
                    break
                except SafetyError:
                    pass
            if selected is None:
                if index == 0:
                    raise RouteError("REPLAN_START_CONNECTION_UNSAFE")
                if index == len(raw_xy) - 2:
                    raise RouteError("REPLAN_GOAL_CONNECTION_UNSAFE")
                if candidates != [()]:
                    raise RouteError("REPLAN_DIAGONAL_CLEARANCE_FAILED")
                raise RouteError("REPLAN_SEGMENT_UNSAFE")
            if selected:
                safe_xy.append(selected)
            safe_xy.append(following)
        route_xy = _remove_strict_collinear(tuple(safe_xy))
        route_world = tuple((point[0], point[1], altitude) for point in route_xy)
        # This checks every endpoint and every continuous segment, including
        # altitude and radial fences.  Do it after the per-link pass so the
        # first/last connector errors above remain actionable to a controller.
        report = safety.validate_route(route_world, origin, max_radius, max_altitude)
        if (not isinstance(report, dict) or
                _finite(report.get("minimum_clearance_m"), "REPLAN_CLEARANCE") + 1e-9 <
                minimum_clearance + tracking_margin):
            raise RouteError("REPLAN_CLEARANCE_INSUFFICIENT")
    except RouteError:
        raise
    except safety_error as exc:
        raise RouteError("REPLAN_SAFETY_FAILED:" + str(exc))
    return tuple(route_world)


def replan_with_grid_source(route, current_world_xy, goal_world_xy, config,
                            grid_source, max_attempts=3):
    """Replan against a versioned :class:`GridSource` with a freshness check.

    A route is only committed when the ``map_version`` observed at planning
    time still matches a fresh snapshot taken afterwards.  When a live
    occupancy source moves underneath the planner (perception window
    recentred, obstacle update fused) the attempt is retried on the newer
    grid instead of committing a route proven against a stale world.  Static
    metadata sources never race and always return on the first attempt.

    Returns ``(route_points, map_version)``: the version stamped on the grid
    the returned route was actually proven against.  Callers must re-check
    that version before executing — a different version observed later marks
    the route expired no matter how recently it was planned.
    """
    if not isinstance(grid_source, GridSource):
        raise RouteError("REPLAN_GRID_SOURCE_INVALID")
    if isinstance(max_attempts, bool) or not isinstance(max_attempts, int) or max_attempts < 1:
        raise RouteError("REPLAN_MAX_ATTEMPTS_INVALID")
    for _ in range(max_attempts):
        grid, map_version = grid_source.snapshot()
        before = grid_source.freshness()
        import sys as _sys
        print("DBG attempt", _, "map_version", map_version, "before", before, file=_sys.stderr)
        points = replan_route(route, current_world_xy, goal_world_xy, config, grid=grid)
        _, _ = grid_source.snapshot()   # post-plan snapshot: captures the token
        after = grid_source.freshness()
        # Compare the FULL freshness token (map_version + content revision):
        # a recenter-stable grid whose marks changed mid-planning must not
        # pass a version-only check.  A mid-planning bump fails closed into
        # a retry, never into a commit on semi-stale content.
        if before == after:
            return points, map_version
    raise RouteError("REPLAN_GRID_SOURCE_RACING")


def verify_runtime_models(route, observed, position_tolerance_m=0.08, yaw_tolerance_rad=0.08):
    """Compare a fresh Gazebo observation: {model: ((x,y,z), yaw)}."""
    if not isinstance(observed, dict):
        raise RouteError("GAZEBO_MODEL_OBSERVATION_INVALID")
    expected_models = route.expected_models + ((route.ground_model,) if hasattr(route, "ground_model") else ())
    for expected in expected_models:
        item = observed.get(expected.name)
        if not isinstance(item, tuple) or len(item) != 2:
            raise RouteError("GAZEBO_MODEL_MISSING:" + expected.name)
        position, yaw = item
        if len(position) != 3 or math.dist(position, expected.model_position) > position_tolerance_m:
            raise RouteError("GAZEBO_MODEL_POSE_MISMATCH:" + expected.name)
        delta = math.atan2(math.sin(yaw - expected.yaw_rad), math.cos(yaw - expected.yaw_rad))
        if abs(delta) > yaw_tolerance_rad:
            raise RouteError("GAZEBO_MODEL_YAW_MISMATCH:" + expected.name)
    return True
