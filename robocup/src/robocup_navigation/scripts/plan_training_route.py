#!/usr/bin/env python3
"""Plan one offline A* route from training-world metadata."""

import argparse
import json
import os
import tempfile

import yaml

from robocup_navigation.astar import (GridMap, MapError, load_metadata,
                                      metadata_endpoints, plan, render_ascii)


def atomic_json(path, payload):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    handle, temporary = tempfile.mkstemp(prefix=".astar-", suffix=".tmp", dir=directory)
    try:
        with os.fdopen(handle, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description="Offline A* over training-city metadata v1")
    parser.add_argument("--metadata", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--goal-id", default=None)
    parser.add_argument("--start-x", type=float)
    parser.add_argument("--start-y", type=float)
    parser.add_argument("--goal-x", type=float)
    parser.add_argument("--goal-y", type=float)
    parser.add_argument("--output", default=None)
    parser.add_argument("--ascii-output", default=None)
    args = parser.parse_args(argv)
    output = args.output or os.path.splitext(args.metadata)[0] + ".astar.json"
    protected = {os.path.realpath(args.metadata), os.path.realpath(args.config)}
    if os.path.realpath(output) in protected or (args.ascii_output and
            os.path.realpath(args.ascii_output) in protected | {os.path.realpath(output)}):
        print("PLAN_INPUT_ERROR:OUTPUT_OVERLAPS_INPUT_OR_JSON")
        return 2

    try:
        metadata, digest = load_metadata(args.metadata)
        grid = GridMap.from_metadata(metadata)
        with open(args.config, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        if not isinstance(config, dict):
            raise MapError("PLANNER_CONFIG_INVALID")
        for key in ("require_outdoor_goal", "prevent_corner_cutting"):
            if not isinstance(config.get(key, True), bool):
                raise MapError("PLANNER_BOOLEAN_INVALID:%s" % key)
        if not config.get("prevent_corner_cutting", True):
            raise MapError("CORNER_CUTTING_NOT_SUPPORTED_BY_CLI")
        start, goal, goal_id = metadata_endpoints(
            metadata, args.goal_id,
            config.get("require_outdoor_goal", True) and args.goal_x is None)
        if (args.start_x is None) != (args.start_y is None):
            raise MapError("START_OVERRIDE_INCOMPLETE")
        if (args.goal_x is None) != (args.goal_y is None):
            raise MapError("GOAL_OVERRIDE_INCOMPLETE")
        if args.start_x is not None:
            start = (args.start_x, args.start_y)
        if args.goal_x is not None:
            goal = (args.goal_x, args.goal_y)
            goal_id = "coordinate_override"
        result = plan(grid, start, goal,
                      connectivity=config.get("connectivity", 8),
                      prevent_corner_cutting=config.get("prevent_corner_cutting", True),
                      max_expansions=config.get("max_expansions", 250000))
        drawing = render_ascii(grid, result) if args.ascii_output else None
    except (OSError, MapError, ValueError, yaml.YAMLError) as exc:
        print("PLAN_INPUT_ERROR:%s" % exc)
        try:
            atomic_json(output, {
                "schema": "robocup_navigation/astar_route/v1",
                "success": False, "reason": "PLAN_INPUT_ERROR:%s" % exc,
                "executable": False, "path_cells": [], "path_world": [],
            })
            if args.ascii_output:
                with open(args.ascii_output, "w", encoding="ascii") as stream:
                    stream.write("PLAN_INPUT_ERROR: no route\n")
        except OSError as write_error:
            print("PLAN_OUTPUT_ERROR:%s" % write_error)
        return 2

    payload = {
        "schema": "robocup_navigation/astar_route/v1",
        "planner": "astar_grid",
        "executable": False,
        "scope": "offline_2d_cell_centres; requires flight-frame and clearance validation",
        "planning_altitude_m": metadata.get("planning_altitude_m"),
        "grid_inflation_m": metadata["grid"].get("inflation_m"),
        "metadata": os.path.abspath(args.metadata),
        "metadata_sha256": digest,
        "frame_id": grid.frame_id,
        "resolution_m": grid.resolution,
        "connectivity": config.get("connectivity", 8),
        "prevent_corner_cutting": config.get("prevent_corner_cutting", True),
        "start_requested": list(start),
        "goal_requested": list(goal),
        "goal_id": goal_id,
        "success": result.success,
        "reason": result.reason,
        "expanded_nodes": result.expanded_nodes,
        "path_length_m": round(result.path_length_m, 6),
        "start_cell": list(result.start_cell) if result.start_cell else None,
        "goal_cell": list(result.goal_cell) if result.goal_cell else None,
        "path_cells": [list(cell) for cell in result.cells],
        "path_world": [[round(p[0], 6), round(p[1], 6)] for p in result.points],
    }
    try:
        if args.ascii_output:
            with open(args.ascii_output, "w", encoding="ascii", newline="\n") as stream:
                stream.write(drawing)
        atomic_json(output, payload)
    except OSError as exc:
        print("PLAN_OUTPUT_ERROR:%s" % exc)
        return 2
    print(json.dumps({key: payload[key] for key in
                      ("success", "reason", "goal_id", "path_length_m", "expanded_nodes")},
                     sort_keys=True))
    print(output)
    return 0 if result.success else 4


if __name__ == "__main__":
    raise SystemExit(main())
