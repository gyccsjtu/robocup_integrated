#!/usr/bin/env python3
"""Fleet mapping reader for the multi-UAV 接入层.

Derives per-UAV ROS namespaces, Gazebo model names, PX4 instance numbers,
spawn poses and every MAVLink port from config/multi_uav/fleet.yaml
(the single source of truth).

Usage:
  mu_fleet.py size     -> fleet_size
  mu_fleet.py rows     -> TSV: id ros_ns mavros_ns model udp local tcp
  mu_fleet.py json     -> {fleet_size, uavs:[...]} (all derived fields)

Scope: naming/plumbing only. No clearance, clearance-timeout, route-version or
release rules live here (those belong to the Codex core; see AGENTS.md).
"""
import json
import os
import sys

import yaml

WS = os.environ.get("ROBOCUP_WORKSPACE", os.path.expanduser("~/robocup/robocup_ws"))
FLEET = os.environ.get("MU_FLEET_YAML", os.path.join(WS, "config/multi_uav/fleet.yaml"))

# Keep this order: scripts/vm/start_multi_uav.sh reads rows positionally.
ROW_KEYS = ("id", "ros_ns", "mavros_ns", "model_name",
            "mavlink_udp_port", "mavlink_udp_local", "mavlink_tcp_port")


def load():
    with open(FLEET) as fh:
        return yaml.safe_load(fh)


def uavs(doc):
    """Return the derived per-UAV rows (1-based logical ids uav_1..uav_N)."""
    sdf = doc["model_sdf_port_base"]
    spawn = doc["spawn"]
    base = int(doc["px4_instance_base"])
    out = []
    for n in range(1, int(doc["fleet_size"]) + 1):
        ros_ns = doc["ros_ns_template"].format(n=n)
        inst = base + (n - 1)
        # Both ends of the MAVLink link, straight from the PX4 port patch.
        mavros_local = int(doc["mavros_local_base"]) + inst
        px4_local = int(doc["px4_mavlink_local_base"]) + inst
        out.append({
            "id": n,
            "ros_ns": ros_ns,
            "mavros_ns": ros_ns.rstrip("/") + "/" + doc["mavros_sub"],
            "model_name": doc["model_name_template"].format(n=n),
            "vehicle": doc.get("vehicle", "iris"),
            "px4_instance": inst,
            "mavros_local_port": mavros_local,
            "px4_mavlink_port": px4_local,
            # MAVROS dials PX4; PX4 answers to the packet source, i.e. our local port.
            "fcu_url": "udp://:%d@localhost:%d" % (mavros_local, px4_local),
            "mavlink_target_system": 1 + inst,   # MAVLink sysid = 1 + px4 instance
            "spawn_x": round(float(spawn["start_xy"][0]) + (n - 1) * float(spawn["x_spacing_m"]), 3),
            "spawn_y": round(float(spawn["start_xy"][1]) + (n - 1) * float(spawn["y_spacing_m"]), 3),
            "spawn_z": round(float(spawn["z_m"]), 3),
            # Ports baked into the vehicle SDF (sim link + camera/gimbal MAVLink
            # bridges). These MUST be base + px4_instance: PX4 starts its sim
            # link as `px4-simulator ... -c $((4560+px4_instance))`, and the
            # Gazebo mavlink_interface plugin has to listen on that same port.
            # Off-by-one here either cross-wires two vehicles (PX4 attaches to
            # the neighbour's plugin) or hangs rcS outright.
            "mavlink_udp_port": int(sdf["mavlink_udp_port"]) + inst,
            "mavlink_tcp_port": int(sdf["mavlink_tcp_port"]) + inst,
            "gst_udp_port": int(sdf["gst_udp_port"]) + inst,
            "mavlink_cam_udp_port": int(sdf["mavlink_cam_udp_port"]) + inst,
        })
        # Back-compat alias: the row column named *_local has always meant the
        # MAVROS-side fcu_url local port.
        out[-1]["mavlink_udp_local"] = mavros_local
    return out


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "rows"
    doc = load()
    rows = uavs(doc)
    if cmd == "size":
        print(int(doc["fleet_size"]))
    elif cmd == "json":
        print(json.dumps({"fleet_size": int(doc["fleet_size"]), "uavs": rows}, indent=2))
    elif cmd == "rows":
        for r in rows:
            print("\t".join(str(r[k]) for k in ROW_KEYS))
    else:
        print("usage: mu_fleet.py [size|rows|json]", file=sys.stderr)
        sys.exit(2)


if __name__ == "__main__":
    main()
