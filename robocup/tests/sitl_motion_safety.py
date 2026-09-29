#!/usr/bin/env python3
"""Real single-iris SITL fault exercise. Never sends MAVROS control commands."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, String

parser = argparse.ArgumentParser()
parser.add_argument("case", choices=["cancel", "pose_stale", "duplicate"])
args = parser.parse_args()
root = Path("/workspace")
run = root / "logs" / "flight" / ("safety_" + args.case + "_" + time.strftime("%Y%m%dT%H%M%S"))
run.mkdir(parents=True)
rospy.init_node("motion_safety_test", anonymous=True)
events = []
relay_enabled = True
pose_pub = rospy.Publisher("/motion_validation/pose", PoseStamped, queue_size=1)
def relay(msg):
    if relay_enabled:
        pose_pub.publish(msg)
rospy.Subscriber("/mavros/local_position/pose", PoseStamped, relay, queue_size=1)
rospy.Subscriber("/uav_navigation/status", String, lambda msg: events.append(json.loads(msg.data)), queue_size=100)
cancel_pub = rospy.Publisher("/uav_navigation/cancel", Empty, queue_size=1)
command = ["python3", str(root / "src/robocup_navigation/scripts/uav_navigation_node.py"),
           "_config_file:=/workspace/src/robocup_navigation/config/single_uav_waypoints.yaml",
           "_log_dir:=" + str(run)]
if args.case == "pose_stale":
    command.append("/mavros/local_position/pose:=/motion_validation/pose")
start = time.monotonic()
trigger = None
duplicate_code = None
with open(run / "console.log", "w") as console:
    proc = subprocess.Popen(command, stdout=console, stderr=subprocess.STDOUT)
    while proc.poll() is None and time.monotonic() - start < 180:
        latest = events[-1] if events else None
        if trigger is None and latest and latest["phase"] == "HOVER" and latest["armed"]:
            trigger = time.monotonic()
            if args.case == "pose_stale":
                relay_enabled = False
            else:
                if args.case == "duplicate":
                    duplicate = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10)
                    duplicate_code = duplicate.returncode
                    print(duplicate.stdout.decode(), flush=True)
                    assert duplicate_code == 3, "duplicate controller was not refused"
                    assert proc.poll() is None, "duplicate launch evicted active controller"
                cancel_pub.publish(Empty())
        if trigger and args.case == "pose_stale" and time.monotonic() - trigger > 4:
            relay_enabled = True
        time.sleep(0.1)
    if proc.poll() is None:
        relay_enabled = True
        cancel_pub.publish(Empty())
        proc.wait(timeout=125)
        raise AssertionError("fault test timed out; requested controlled landing")
code = proc.returncode
rows = [json.loads(line) for line in (run / "events.jsonl").read_text().splitlines()]
reason = "POSE_STALE" if args.case == "pose_stale" else "CANCELLED"
assert trigger is not None, "never reached airborne hover"
assert code == (1 if args.case == "pose_stale" else 2), "unexpected controller exit code"
landed = [r for r in rows if r["event"] == "LANDED"]
assert landed and landed[-1]["armed"] is False and landed[-1]["landed_state"] == 1
assert rows[-1]["reason"] == reason
assert not any(r["event"] == "ARRIVED" and r.get("name") in ("A", "B") for r in rows)
print(json.dumps(dict(passed=True, case=args.case, log=str(run / "events.jsonl"),
                     exit_code=code, reason=reason, duplicate_exit_code=duplicate_code,
                     wall_s=time.monotonic() - start), indent=2))
