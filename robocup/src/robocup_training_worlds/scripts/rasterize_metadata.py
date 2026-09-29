#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Turn generated metadata into a map_server-compatible occupancy map.

Output: a PGM image plus a YAML sidecar using the ROS ``nav_msgs/OccupancyGrid``
conventions (0 = free, 100 = occupied). Planners may consume either this pair
or the metadata grid directly; this module never publishes anything itself.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import training_city_lib as lib  # noqa: E402


def write_pgm(path, cells, width, height):
    occupied = bytearray()
    for value in cells:
        occupied.append(0 if value else 254)
    with open(path, "wb") as stream:
        stream.write(b"P5\n")
        stream.write(b"%d %d\n" % (width, height))
        stream.write(b"255\n")
        stream.write(bytes(occupied))


def main(argv=None):
    parser = argparse.ArgumentParser(description="Rasterize training-city metadata.")
    parser.add_argument("--metadata", required=True, help="path to the .json metadata")
    parser.add_argument("--pgm", default=None, help="output PGM path")
    parser.add_argument("--yaml", default=None, help="output YAML map sidecar path")
    args = parser.parse_args(argv)

    with open(args.metadata, "r", encoding="utf-8") as stream:
        metadata = json.load(stream)
    grid = metadata.get("grid") or {}
    if "data" not in grid:
        sys.stderr.write("GRID_MISSING: regenerate without --no-grid\n")
        return 2
    cells = lib.decode_rle(grid["data"], grid["width"], grid["height"])

    pgm_path = args.pgm or os.path.splitext(args.metadata)[0] + ".pgm"
    yaml_path = args.yaml or os.path.splitext(args.metadata)[0] + ".yaml"
    write_pgm(pgm_path, cells, grid["width"], grid["height"])
    origin = grid.get("origin", [0.0, 0.0])
    with open(yaml_path, "w", encoding="utf-8", newline="\n") as stream:
        stream.write("image: %s\n" % os.path.basename(pgm_path))
        stream.write("resolution: %s\n" % lib.fmt(grid["resolution_m"]))
        stream.write("origin: [%s, %s, 0.0]\n" % (lib.fmt(origin[0]), lib.fmt(origin[1])))
        stream.write("negate: 0\n")
        stream.write("occupied_thresh: 0.65\n")
        stream.write("free_thresh: 0.196\n")
        stream.write("mode: trinary\n")
    print(pgm_path)
    print(yaml_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
