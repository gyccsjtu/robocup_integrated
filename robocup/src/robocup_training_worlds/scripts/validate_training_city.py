#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Validate a generated training world before Gazebo is allowed to start it.

Checks performed
----------------
SDF side:  well-formed XML, SDF 1.5, unique model names, collision + visual
           present for every model, no vehicle model (iris / px4 / mavros),
           no plugins, no online model URIs (Fuel, http).
Map side:  bounds, obstacle overlap, obstacle vertical coverage, spawn
           clearance, goal candidates outdoors, independent BFS connectivity.

Exit codes: 0 ok, 1 validation failed, 2 usage error.
"""

import argparse
import json
import os
import sys
import xml.etree.ElementTree as ElementTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import training_city_lib as lib  # noqa: E402

FORBIDDEN_NAME_TOKENS = ("iris", "px4", "mavros", "typhoon", "plane", "rover")


def _local(tag):
    return tag.rsplit("}", 1)[-1]


def validate_sdf(path, expected_world_name=None):
    """Return a list of SDF problems (empty when the world is usable)."""
    problems = []
    try:
        root = ElementTree.parse(path).getroot()
    except ElementTree.ParseError as exc:
        return ["SDF_PARSE_ERROR:%s" % exc]
    if _local(root.tag) != "sdf":
        problems.append("SDF_ROOT_NOT_SDF")
        return problems
    if root.get("version") != lib.SDF_VERSION:
        problems.append("SDF_VERSION_MISMATCH:%s" % root.get("version"))
    worlds = [child for child in root if _local(child.tag) == "world"]
    if len(worlds) != 1:
        problems.append("SDF_WORLD_COUNT:%d" % len(worlds))
        return problems
    world = worlds[0]
    name = world.get("name")
    if expected_world_name and name != expected_world_name:
        problems.append("WORLD_NAME_MISMATCH:%s!=%s" % (name, expected_world_name))
    if not name:
        problems.append("WORLD_NAME_MISSING")

    names = set()
    for model in world.findall("model"):
        model_name = model.get("name") or ""
        if not model_name:
            problems.append("MODEL_NAME_MISSING")
            continue
        if model_name in names:
            problems.append("DUPLICATE_MODEL_NAME:%s" % model_name)
        names.add(model_name)
        lowered = model_name.lower()
        for token in FORBIDDEN_NAME_TOKENS:
            if token in lowered:
                problems.append("VEHICLE_MODEL_PRESENT:%s" % model_name)
        links = model.findall("link")
        if not links:
            problems.append("MODEL_WITHOUT_LINK:%s" % model_name)
        for link in links:
            if link.find("collision") is None:
                problems.append("COLLISION_MISSING:%s" % model_name)
            if link.find("visual") is None:
                problems.append("VISUAL_MISSING:%s" % model_name)
    if not names:
        problems.append("NO_MODELS")

    for element in world.iter():
        tag = _local(element.tag)
        if tag == "plugin":
            problems.append("PLUGIN_PRESENT:%s" % (element.get("name") or "unnamed"))
        if tag == "uri":
            text = (element.text or "").strip()
            if text.startswith("http://") or text.startswith("https://") or "fuel" in text.lower():
                problems.append("ONLINE_MODEL_URI:%s" % text)
    for include in world.findall("include"):
        uri = include.find("uri")
        text = (uri.text if uri is not None else "") or ""
        if text.startswith("http://") or text.startswith("https://"):
            problems.append("ONLINE_MODEL_URI:%s" % text)
    return problems


def main(argv=None):
    parser = argparse.ArgumentParser(description="Validate a generated training world.")
    parser.add_argument("--world", required=True, help="path to the .world file")
    parser.add_argument("--metadata", default=None,
                        help="path to the .json metadata (defaults to the world path)")
    parser.add_argument("--expect-preset", default=None, choices=sorted(lib.CITY_PRESETS))
    parser.add_argument("--expect-seed", type=int, default=None)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    if not os.path.isfile(args.world):
        sys.stderr.write("WORLD_MISSING: %s\n" % args.world)
        return 2
    meta_path = args.metadata or os.path.splitext(args.world)[0] + ".json"
    if not os.path.isfile(meta_path):
        sys.stderr.write("METADATA_MISSING: %s\n" % meta_path)
        return 2

    try:
        with open(meta_path, "r", encoding="utf-8") as stream:
            metadata = json.load(stream)
    except (ValueError, OSError) as exc:
        sys.stderr.write("METADATA_UNREADABLE: %s\n" % exc)
        return 2

    problems = validate_sdf(args.world, metadata.get("world_name"))
    if args.expect_preset and metadata.get("generator", {}).get("preset") != args.expect_preset:
        problems.append("PRESET_MISMATCH:%s" % metadata.get("generator", {}).get("preset"))
    if args.expect_seed is not None and metadata.get("generator", {}).get("seed") != args.expect_seed:
        problems.append("SEED_MISMATCH:%s" % metadata.get("generator", {}).get("seed"))

    try:
        map_problems, checks = lib.validate_metadata(metadata)
    except (KeyError, ValueError, TypeError) as exc:
        sys.stderr.write("METADATA_SCHEMA_INVALID: %s\n" % exc)
        return 2
    problems += map_problems

    if problems:
        sys.stderr.write("WORLD_INVALID: %s\n" % args.world)
        for problem in problems:
            sys.stderr.write("  - %s\n" % problem)
        return 1
    if not args.quiet:
        print("[validate_training_city] OK preset=%s seed=%s obstacles=%d goals=%d/%d reachable"
              % (metadata["generator"]["preset"], metadata["generator"]["seed"],
                 len(metadata["obstacles"]), metadata["connectivity"]["goals_reachable"],
                 metadata["connectivity"]["goals_total"]))
        print("[validate_training_city] checks: %s" % json.dumps(checks, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
