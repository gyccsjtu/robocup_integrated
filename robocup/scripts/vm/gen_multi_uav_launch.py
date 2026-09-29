#!/usr/bin/env python3
"""Generate a multi-UAV PX4 + MAVROS + Gazebo launch file from fleet.yaml.

Why this exists instead of using PX4's stock multi_uav_mavros_sitl.launch:
  * the stock launch hardcodes 0-based namespaces (uav0..) and 3 vehicles;
  * it uses the *unpatched* MAVLink ports (14540 -> 14580), while this repo's
    PX4 carries an XTDrone patch that moves the offboard ports to
    24540+instance (remote) / 34580+instance (local). Stock therefore never
    connects: MAVROS dials 14580 while PX4 listens on 34580.
  * the coordination layer addresses UAVs as uav_1..uav_6 (see
    docs/coordination_interface_v0.md), which is what fleet.yaml encodes.

The generated file is a **development harness** (interface v0 §5): it is not
the official competition launch. Every naming/port decision comes from
config/multi_uav/fleet.yaml so that swapping in the official launch later is a
one-file change.

Usage:
  gen_multi_uav_launch.py --out config/multi_uav/multi_uav_dev.launch
  gen_multi_uav_launch.py --out /tmp/x.launch --limit 3
"""
import argparse
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mu_fleet  # noqa: E402

HEADER = """<?xml version="1.0"?>
<!-- GENERATED FILE. DO NOT EDIT BY HAND.
     Source of truth: config/multi_uav/fleet.yaml
     Generator:       scripts/vm/gen_multi_uav_launch.py
     Status:          DEV HARNESS (interface v0 section 5), NOT the official
                      competition launch. Replace this file once the official
                      multi-UAV launch is frozen.
     Layout: one <group ns="uav_N"> per UAV, each containing
               * single_vehicle_spawn.launch (PX4 SITL instance + Gazebo spawn)
               * mavros/px4.launch           (MAVROS on fcu_url from fleet.yaml)
             MAVLink ports follow this repo's PX4 patch:
               fcu_url = udp://:24540+instance@localhost:34580+instance
-->
<launch>
"""

FOOTER = "</launch>\n"


def build(doc, limit=None):
    rows = mu_fleet.uavs(doc)
    if limit:
        rows = rows[:limit]

    root = ET.Element("launch")

    def arg(parent, name, value):
        ET.SubElement(parent, "arg", {"name": name, "value": value})

    def default(parent, name, value):
        # Top-level args must be `default`, otherwise roslaunch rejects a
        # command-line override (gui:=true -> "Arg xml is <arg ... value=...>").
        ET.SubElement(parent, "arg", {"name": name, "default": value})

    # --- global args ------------------------------------------------------
    default(root, "gui", "false")
    default(root, "world", "$(find mavlink_sitl_gazebo)/worlds/empty.world")
    default(root, "paused", "false")
    default(root, "verbose", "false")

    gaz = ET.SubElement(root, "include",
                        {"file": "$(find gazebo_ros)/launch/empty_world.launch"})
    arg(gaz, "gui", "$(arg gui)")
    arg(gaz, "world_name", "$(arg world)")
    arg(gaz, "paused", "$(arg paused)")
    arg(gaz, "verbose", "$(arg verbose)")

    for r in rows:
        grp = ET.SubElement(root, "group", {"ns": r["ros_ns"].lstrip("/")})
        arg(grp, "ID", str(r["px4_instance"]))
        arg(grp, "fcu_url", r["fcu_url"])

        spawn = ET.SubElement(grp, "include",
                              {"file": "$(find px4)/launch/single_vehicle_spawn.launch"})
        arg(spawn, "x", str(r["spawn_x"]))
        arg(spawn, "y", str(r["spawn_y"]))
        arg(spawn, "z", str(r["spawn_z"]))
        arg(spawn, "R", "0")
        arg(spawn, "P", "0")
        arg(spawn, "Y", "0")
        arg(spawn, "vehicle", r["vehicle"])
        arg(spawn, "ID", str(r["px4_instance"]))
        arg(spawn, "mavlink_udp_port", str(r["mavlink_udp_port"]))
        arg(spawn, "mavlink_tcp_port", str(r["mavlink_tcp_port"]))
        arg(spawn, "gst_udp_port", str(r["gst_udp_port"]))
        arg(spawn, "video_uri", str(r["gst_udp_port"]))
        arg(spawn, "mavlink_cam_udp_port", str(r["mavlink_cam_udp_port"]))

        mav = ET.SubElement(grp, "include",
                            {"file": "$(find mavros)/launch/px4.launch"})
        arg(mav, "fcu_url", "$(arg fcu_url)")
        arg(mav, "gcs_url", "")
        arg(mav, "tgt_system", str(r["mavlink_target_system"]))
        arg(mav, "tgt_component", "1")

    # Hand-rolled indent: xml.etree.ElementTree.indent is Python 3.9+, and the
    # VM baseline (Ubuntu 20.04) ships 3.8 — this script must run there too.
    lines = []

    def walk(el, depth):
        pad = "    " * depth
        attrs = "".join(' %s="%s"' % (k, v) for k, v in el.attrib.items())
        if len(el) == 0:
            lines.append("%s<%s%s/>" % (pad, el.tag, attrs))
            return
        lines.append("%s<%s%s>" % (pad, el.tag, attrs))
        for child in el:
            walk(child, depth + 1)
        lines.append("%s</%s>" % (pad, el.tag))

    # Emit each child of <launch> as a self-contained element.
    for child in root:
        walk(child, 1)
    body = "\n".join(lines) + "\n"
    return HEADER + body + FOOTER, rows


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", required=True, help="output .launch path")
    ap.add_argument("--limit", type=int, default=None, help="only the first N UAVs")
    args = ap.parse_args()

    doc = mu_fleet.load()
    text, rows = build(doc, args.limit)
    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)

    print("wrote %s (%d UAVs)" % (out, len(rows)))
    for r in rows:
        print("  uav_%-2d ns=%-7s model=%-6s inst=%d fcu=%s spawn=(%s,%s,%s)"
              % (r["id"], r["ros_ns"], r["model_name"], r["px4_instance"],
                 r["fcu_url"], r["spawn_x"], r["spawn_y"], r["spawn_z"]))


if __name__ == "__main__":
    main()
