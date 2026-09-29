#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate a deterministic Gazebo Classic training city (.world + .json).

Example (inside the container):

    python3 generate_training_city.py --preset unit --category single_wall \
        --seed 42 --output /tmp/unit.world --metadata /tmp/unit.json

The generator writes an SDF 1.5 world built from box/cylinder primitives only
(no Fuel, no model:// URIs, no vehicle, no plugins) and a machine-readable
metadata document. It never publishes anything to ROS.

Exit codes: 0 ok, 1 validation failed, 2 usage error.
"""

import argparse
import json
import os
import sys
from datetime import datetime, timezone

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import training_city_lib as lib  # noqa: E402


def build_parser():
    parser = argparse.ArgumentParser(
        description="Deterministic RoboCup training-city generator (Gazebo Classic).")
    parser.add_argument("--preset", default="unit", choices=sorted(lib.CITY_PRESETS),
                        help="unit (10x10 m), small (30x20 m) or full (200x100 m)")
    parser.add_argument("--category", default=None, choices=sorted(lib.UNIT_CATEGORIES),
                        help="unit-preset scenario class (ignored by small/full)")
    parser.add_argument("--seed", type=int, default=42,
                        help="randomisation seed; the same seed reproduces the same world")
    parser.add_argument("--config", default=None, help="path to training_city.yaml")
    parser.add_argument("--output", default=None, help="output .world path")
    parser.add_argument("--metadata", default=None, help="output .json metadata path")
    parser.add_argument("--spawn-env", default=None,
                        help="optional shell file receiving ROBOCUP_SPAWN_X / _Y / _Z / _YAW")
    parser.add_argument("--world-name", default=None, help="override the SDF world name")
    parser.add_argument("--no-grid", action="store_true",
                        help="omit the run-length occupancy grid from the metadata")
    parser.add_argument("--emit-timestamp", action="store_true",
                        help="add a UTC timestamp (breaks byte-for-byte determinism)")
    parser.add_argument("--allow-invalid", action="store_true",
                        help="write the world even when validation reports problems")
    parser.add_argument("--quiet", action="store_true", help="only print the two output paths")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    config = lib.load_config(args.config)
    spawn_z = float(config["generator"].get("simulation_spawn_z_m", 0.20))
    if not 0.05 <= spawn_z <= 1.0:
        parser.error("generator.simulation_spawn_z_m must be in [0.05, 1.0]")
    world_name = args.world_name or lib.make_world_name(config, args.preset, args.seed,
                                                         args.category)
    timestamp = (datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
                 if args.emit_timestamp else None)
    emit_grid = False if args.no_grid else None

    try:
        sdf, metadata = lib.build_world(args.preset, args.seed, config,
                                        category=args.category, world_name=world_name,
                                        emit_grid=emit_grid, timestamp=timestamp)
    except (ValueError, KeyError) as exc:
        sys.stderr.write("GENERATION_FAILED: %s\n" % exc)
        return 2

    problems = metadata["validation"]["problems"]
    if problems:
        sys.stderr.write("VALIDATION_PROBLEMS:\n")
        for problem in problems:
            sys.stderr.write("  - %s\n" % problem)
        if not args.allow_invalid:
            sys.stderr.write("refusing to write an invalid world "
                             "(use --allow-invalid to override for debugging)\n")
            return 1

    output = args.output or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "worlds", "generated",
        world_name + ".world")
    meta_path = args.metadata or os.path.splitext(output)[0] + ".json"
    output = os.path.abspath(output)
    meta_path = os.path.abspath(meta_path)
    for path in (output, meta_path):
        directory = os.path.dirname(path)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)

    with open(output, "w", encoding="utf-8", newline="\n") as stream:
        stream.write(sdf)
    with open(meta_path, "w", encoding="utf-8", newline="\n") as stream:
        json.dump(metadata, stream, indent=2, sort_keys=True, ensure_ascii=False)
        stream.write("\n")

    if args.spawn_env:
        spawn = metadata["spawn"]
        directory = os.path.dirname(os.path.abspath(args.spawn_env))
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        with open(args.spawn_env, "w", encoding="utf-8", newline="\n") as stream:
            stream.write("ROBOCUP_SPAWN_X=%s\n" % lib.fmt(spawn["center"][0]))
            stream.write("ROBOCUP_SPAWN_Y=%s\n" % lib.fmt(spawn["center"][1]))
            stream.write("ROBOCUP_SPAWN_Z=%s\n" % lib.fmt(spawn_z))
            stream.write("ROBOCUP_SPAWN_YAW=%s\n" % lib.fmt(spawn.get("yaw", 0.0)))
            stream.write("ROBOCUP_WORLD_NAME=%s\n" % metadata["world_name"])

    if not args.quiet:
        print("[generate_training_city] preset=%s category=%s seed=%d"
              % (args.preset, args.category or "-", args.seed))
        print("[generate_training_city] obstacles=%d spawn=%s goals=%d reachable=%s"
              % (len(metadata["obstacles"]),
                 metadata["spawn"]["center"],
                 len(metadata["goal_candidates"]),
                 metadata["connectivity"]["goals_reachable"]))
        print("[generate_training_city] free_cell_ratio=%s validation=%s"
              % (metadata["connectivity"]["free_cell_ratio"],
                 "OK" if metadata["validation"]["ok"] else "FAILED"))
    print(output)
    print(meta_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
