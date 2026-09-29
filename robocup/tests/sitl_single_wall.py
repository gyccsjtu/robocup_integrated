#!/usr/bin/env python3
"""Independent Gazebo truth observer; test output is cancel or a navigation goal."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import subprocess
import threading
import time

import rospy
import yaml
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, String

from robocup_navigation.route_safety import SafetyMap


def draw_routes(path, metadata, planned, actual):
    """Offline vector evidence, axes are Gazebo world metres."""
    bounds = metadata["bounds"]
    x0, y0, x1, y1 = (bounds[k] for k in ("x_min", "y_min", "x_max", "y_max"))
    scale = 65.0
    def xy(point):
        return (55 + (point[0] - x0) * scale, 45 + (y1 - point[1]) * scale)
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="780" height="780" viewBox="0 0 780 780">',
             '<rect width="780" height="780" fill="white"/>',
             '<text x="55" y="25" font-size="18">single_wall: planned vs Gazebo actual (world ENU, m)</text>']
    for x in range(math.ceil(x0), math.floor(x1) + 1):
        sx, _ = xy((x, y0))
        parts.append('<path d="M %g 45 V 695" stroke="#eee"/>' % sx)
        parts.append('<text x="%g" y="715" font-size="12">%d</text>' % (sx, x))
    for y in range(math.ceil(y0), math.floor(y1) + 1):
        _, sy = xy((x0, y))
        parts.append('<path d="M 55 %g H 705" stroke="#eee"/>' % sy)
        parts.append('<text x="25" y="%g" font-size="12">%d</text>' % (sy, y))
    for obstacle in metadata["obstacles"]:
        a, b, c, d = obstacle["bbox"]
        sx, sy = xy((a, d))
        parts.append('<rect x="%g" y="%g" width="%g" height="%g" fill="#59636b"/>' %
                     (sx, sy, (c-a)*scale, (d-b)*scale))
    for points, color, width in ((planned, "#186cc4", 5), (actual, "#e06816", 2)):
        coords = " ".join("%g,%g" % xy(p) for p in points)
        parts.append('<polyline points="%s" fill="none" stroke="%s" stroke-width="%d"/>' % (coords, color, width))
    parts.append('<text x="55" y="750" fill="#186cc4" font-size="16">Blue: planned</text>')
    parts.append('<text x="250" y="750" fill="#e06816" font-size="16">Orange: actual centre trajectory</text></svg>')
    path.write_text("\n".join(parts))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", choices=("normal", "cancel", "replan"), default="normal")
    parser.add_argument("--config", default="/workspace/src/robocup_navigation/config/single_wall_route.yaml")
    args = parser.parse_args()
    config = yaml.safe_load(Path(args.config).read_text())
    run = Path("/workspace/logs/single_wall") / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ") + "_" + args.case)
    run.mkdir(parents=True)
    rospy.init_node("single_wall_validation", anonymous=True)
    cancel = rospy.Publisher("/uav_navigation/cancel", Empty, queue_size=1)
    goal_pub = rospy.Publisher("/uav_navigation/goal", PoseStamped, queue_size=1)
    mutex = threading.Lock()
    samples, events = [], []
    last = [0.0]
    with (run / "ground_truth.jsonl").open("x", buffering=1) as truth:
        def observe(msg):
            now = time.monotonic()
            if now - last[0] < 0.045 or "iris" not in msg.name:
                return
            last[0] = now
            i = msg.name.index("iris")
            p, v = msg.pose[i].position, msg.twist[i].linear
            row = dict(wall_s=now, ros_s=rospy.Time.now().to_sec(),
                       position=[p.x, p.y, p.z], velocity=[v.x, v.y, v.z])
            with mutex:
                samples.append(row)
                truth.write(json.dumps(row, allow_nan=False) + "\n")

        def event(msg):
            with mutex:
                events.append(json.loads(msg.data))

        sub = rospy.Subscriber("/gazebo/model_states", ModelStates, observe, queue_size=10)
        status_sub = rospy.Subscriber("/uav_navigation/status", String, event, queue_size=200)
        ready_until = time.monotonic() + 10
        while not samples and time.monotonic() < ready_until:
            time.sleep(0.05)
        if not samples:
            raise RuntimeError("NO_GAZEBO_GROUND_TRUTH_BEFORE_FLIGHT")
        command = ["python3", "/workspace/src/robocup_navigation/scripts/uav_navigation_node.py",
                   "_config_file:=" + args.config, "_log_dir:=" + str(run)]
        start, trigger, timed_out = time.monotonic(), None, False
        with (run / "console.log").open("x") as console:
            proc = subprocess.Popen(command, stdout=console, stderr=subprocess.STDOUT)
            try:
                while proc.poll() is None:
                    now = time.monotonic()
                    with mutex:
                        latest = events[-1] if events else {}
                        current = samples[-1]
                    # Trigger after leaving takeoff and making horizontal progress.
                    if args.case == "cancel" and trigger is None and latest.get("armed") and current["position"][0] > -2.0:
                        cancel.publish(Empty())
                        trigger = now
                        print("CANCEL_REQUESTED", flush=True)
                    if (args.case == "replan" and trigger is None and latest.get("armed") and
                            current["position"][0] > -2.2 and goal_pub.get_num_connections()):
                        goal = PoseStamped()
                        goal.header.stamp = rospy.Time.now()
                        goal.header.frame_id = config["external_goal_frame_id"]
                        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = 2.5, -2.0, 0.0
                        goal.pose.orientation.w = 1.0
                        goal_pub.publish(goal)
                        trigger = now
                        print("REPLAN_GOAL_REQUESTED", flush=True)
                    if now - start > float(config.get("mission_timeout_s", 480)) + 90:
                        timed_out = True
                        cancel.publish(Empty())
                        proc.send_signal(signal.SIGINT)
                        proc.wait(timeout=float(config.get("landing_timeout_s", 120)) + 30)
                        break
                    time.sleep(0.05)
            except BaseException:
                cancel.publish(Empty())
                if proc.poll() is None:
                    proc.send_signal(signal.SIGINT)
                    try:
                        proc.wait(timeout=float(config.get("landing_timeout_s", 120)) + 30)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=5)
                raise
        sub.unregister()
        status_sub.unregister()
    rows = [json.loads(line) for line in (run / "events.jsonl").read_text().splitlines()]
    # Use the exact source metadata recorded by the controller configuration.
    metadata_file = config.get("metadata_file", "/workspace/src/robocup_training_worlds/worlds/generated/training_city_unit_single_wall_s42.json")
    metadata = json.loads(Path(metadata_file).read_text())
    safety = SafetyMap(metadata, vehicle_radius_m=config.get("vehicle_radius_m", 0.4),
                       horizontal_margin_m=config.get("horizontal_margin_m", 0.2),
                       vehicle_half_height_m=config.get("vehicle_half_height_m", 0.15),
                       vertical_margin_m=config.get("vertical_margin_m", 0.1))
    clearance = min(safety.clearance(row["position"]) for row in samples)
    max_gap = max((b["ros_s"] - a["ros_s"] for a, b in zip(samples, samples[1:])), default=0)
    goal = [2.5, -2.0] if args.case == "replan" else metadata["goal_candidates"][0]["center"]
    arrived = [r for r in rows if r["event"] == "ARRIVED" and r.get("name") != "TAKEOFF"]
    landed = [r for r in rows if r["event"] == "LANDED"]
    expected = 2 if args.case == "cancel" else 0
    checks = dict(controller_exit=proc.returncode == expected,
                  result=rows[-1].get("code") == expected,
                  landed=bool(landed and landed[-1].get("armed") is False and landed[-1].get("landed_state") == 1),
                  no_timeout=not timed_out,
                  required_body_clearance=clearance + 1e-9 >= config["minimum_clearance_m"],
                  horizontal_fence=max(math.hypot(r["position"][0] - samples[0]["position"][0],
                                                   r["position"][1] - samples[0]["position"][1]) for r in samples) <= config["max_radius_m"],
                  within_altitude=(max(r["position"][2] for r in samples) +
                                   config["vehicle_half_height_m"] + config["vertical_margin_m"]
                                   <= config["max_altitude_m"] + 1e-9),
                  observations_continuous=max_gap < 0.25,
                  took_off=max(r["position"][2] for r in samples) > 2.0)
    if args.case in ("normal", "replan"):
        # Evaluate goal proximity while airborne, not just drift after touching down.
        final_error = min((math.hypot(r["position"][0] - goal[0], r["position"][1] - goal[1])
                           for r in samples if r["position"][2] > 2.0), default=float("inf"))
        checks["actual_goal_reached"] = final_error <= config["arrival_tolerance_m"] + 0.05
        if not math.isfinite(final_error):
            final_error = None
        checks["route_arrivals"] = bool(arrived)
        expected_arrivals = [r.get("name") for r in rows if r["event"] == "TARGET"]
        actual_arrivals = [r.get("name") for r in rows if r["event"] == "ARRIVED"]
        if args.case == "normal":
            checks["all_targets_arrived_in_order"] = expected_arrivals == actual_arrivals
        else:
            accepted = [r for r in rows if r["event"] == "GOAL_ACCEPTED"]
            external_prefix = "external_%s_" % accepted[-1]["goal_seq"] if accepted else "external_missing_"
            external_targets = [name for name in expected_arrivals if name and name.startswith(external_prefix)]
            checks["replan_triggered"] = trigger is not None
            checks["replan_accepted"] = bool(accepted)
            checks["external_targets_arrived_in_order"] = external_targets == [
                name for name in actual_arrivals if name and name.startswith(external_prefix)]
        checks["arrival_tolerance_and_speed"] = all(r["distance_m"] <= config["arrival_tolerance_m"] and
                                                    r["speed_mps"] <= config["arrival_speed_mps"] for r in arrived)
        completed_hovers = [r.get("name") for r in rows if r["event"] == "HOVER_DONE"]
        checks["all_hovers_completed"] = (expected_arrivals == completed_hovers if args.case == "normal" else
                                           bool(completed_hovers and completed_hovers[-1].startswith(external_prefix)))
        checks["second_xy_pair_verified"] = any(r["event"] == "ROUTE_TRANSFORM_SECOND_PAIR_CONFIRMED" for r in rows)
        checks["no_failure_events"] = not any(r["event"] in ("FAILURE", "LANDING_UNCONFIRMED") for r in rows)
    else:
        final_error = None
        checks["cancel_triggered"] = trigger is not None
        checks["cancel_reason"] = rows[-1].get("reason") == "CANCELLED"
        checks["goal_not_reached"] = max(r["position"][0] for r in samples) < goal[0] - 0.5
    result = dict(passed=all(checks.values()), case=args.case, checks=checks,
                  log_dir=str(run), controller_exit_code=proc.returncode, wall_s=time.monotonic() - start,
                  samples=len(samples), max_ros_sample_gap_s=max_gap, minimum_body_clearance_m=clearance,
                  maximum_world_altitude_m=max(r["position"][2] for r in samples), goal_error_m=final_error,
                  maximum_horizontal_speed_mps=max(math.hypot(*r["velocity"][:2]) for r in samples),
                  scope="20 Hz sampled Gazebo truth; conservative body envelope, not a continuous-contact sensor proof")
    ready = next((r for r in rows if r["event"] == "ROUTE_READY"), {})
    accepted = [r for r in rows if r["event"] == "GOAL_ACCEPTED" and r.get("route_world")]
    planned = accepted[-1]["route_world"] if accepted else ready.get("route_world", [])
    draw_routes(run / "route_overlay.svg", metadata, planned, [r["position"] for r in samples])
    (run / "validation.json").write_text(json.dumps(result, indent=2, allow_nan=False))
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
