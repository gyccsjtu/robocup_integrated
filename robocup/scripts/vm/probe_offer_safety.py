#!/usr/bin/env python3
"""Offline diagnosis: why does the core not grant?

Captures the live VEHICLE_STATE / ROUTE_OFFER messages of a running fleet and
replays them into the Codex core, printing every condition of `_offer_safe`
plus `_conflict` per UAV. Read-only: it never modifies the core.

Usage (on the VM, while the coordination stack is running):
    python3 probe_offer_safety.py --fleet 6 --out /tmp/probe.json
"""
import argparse
import json
import math
import os
import sys

import rospy
from std_msgs.msg import String

WS = os.environ.get("ROBOCUP_WORKSPACE", os.path.expanduser("~/robocup/robocup_ws"))
sys.path.insert(0, os.path.join(WS, "src", "robocup_navigation", "src"))
from robocup_navigation.coordination import Coordinator  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--fleet", type=int, default=6)
    ap.add_argument("--config", default=os.path.join(
        WS, "config", "coordination", "six_uav_gazebo.json"))
    ap.add_argument("--out", default="/tmp/probe.json")
    args = ap.parse_args()

    cfg = json.load(open(args.config))
    fleet = ["uav_%d" % i for i in range(1, args.fleet + 1)]

    rospy.init_node("probe_offer_safety", anonymous=True)
    states, offers = {}, {}
    for u in fleet:
        try:
            m = rospy.wait_for_message("/%s/coordination/vehicle_state" % u, String, timeout=15)
            states[u] = json.loads(m.data)
        except Exception as exc:  # noqa: BLE001
            print("MISSING vehicle_state for %s: %s" % (u, exc))
    for _ in range(args.fleet):
        try:
            m = rospy.wait_for_message("/coordination/route_offer", String, timeout=12)
            d = json.loads(m.data)
            offers[d["uav_id"]] = d
        except Exception:  # noqa: BLE001
            break

    json.dump({"states": states, "offers": offers}, open(args.out, "w"), indent=2)

    tasks = {}
    for u, o in offers.items():
        tid = o["task_id"]
        if tid not in tasks:
            tasks[tid] = {"target_id": o.get("target_id", tid), "xyz": list(o["points"][-1])}
    if not tasks:
        print("no offers captured; nothing to evaluate")
        return

    roles = {u: ["VEHICLE_STATE", "COMMAND_ACK"] for u in fleet}
    roles.update(planner=["ROUTE_OFFER"], clock=["TICK"],
                 verifier=["RESOURCE_CLEAR"], perception=["TARGET_REPORT"])
    c = Coordinator("probe", fleet, tasks, cfg["limits"], roles, "probe")

    seq = {}
    for u, s in states.items():
        seq[u] = seq.get(u, 0) + 1
        c.receive(dict(schema_version=1, run_id="probe", source_id=u, seq=seq[u],
                       sim_s=0.0, kind="VEHICLE_STATE", data=s), 0.0)
    pseq = 0
    for u, o in offers.items():
        pseq += 1
        c.receive(dict(schema_version=1, run_id="probe", source_id="planner", seq=pseq,
                       sim_s=0.0, kind="ROUTE_OFFER", data=o), 0.0)

    l = cfg["limits"]
    print("fleet=%s  captured states=%d offers=%d" % (fleet, len(states), len(offers)))
    print("all fresh+OK: %s" % all(c._fresh(u) and c.vehicles[u]["health"] == "OK"
                                   for u in fleet))
    for u in fleet:
        if u not in offers or (u, list(tasks)[0]) not in c.offers:
            print("  %s: no offer keyed for it" % u)
            continue
        tid = offers[u]["task_id"]
        o = c.offers[(u, tid)]
        s = c.vehicles[u]
        print("  %s mode=%s health=%s vel=%.3f stopped=%s | offer_safe=%s conflict=%s"
              % (u, s["mode"], s["health"], math.dist(s["velocity_xyz"], (0, 0, 0)),
                 c._stopped(u), c._offer_safe(o), c._conflict(o["points"], u)))
        d0 = math.dist(o["points"][0][:3], s["xyz"])
        print("      first-pt dist %.2f (tol %.2f)  clearance %.1f  map_rev %s/%s"
              % (d0, l["position_tolerance_m"], o["clearance_m"],
                 o["map_revision"], s["map_revision"]))
    print("wrote %s" % args.out)


if __name__ == "__main__":
    main()
