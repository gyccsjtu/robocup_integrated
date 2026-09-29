#!/usr/bin/env python3
"""Read-only flight guard and fresh-scene receipt. Never publishes controls."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import time
import uuid

import rospy
from mavros_msgs.msg import ExtendedState, State
from gazebo_msgs.srv import GetWorldProperties


def guarded_ground():
    rospy.init_node("single_wall_ground_guard", anonymous=True)
    if not rospy.get_param("/use_sim_time", False):
        raise RuntimeError("SIMULATION_REQUIRED")
    with open("/tmp/robocup_single_uav_controller.lock", "a") as lease:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        code, _, graph = rospy.get_master().getSystemState()
        if code != 1:
            raise RuntimeError("ROS_GRAPH_UNAVAILABLE")
        for topic, nodes in graph[0]:
            control = topic.startswith(("/mavros/setpoint_position/", "/mavros/setpoint_velocity/",
                                        "/mavros/setpoint_attitude/"))
            control = control or topic in ("/mavros/setpoint_raw/local", "/mavros/setpoint_raw/global",
                                           "/mavros/setpoint_raw/attitude", "/mavros/actuator_control")
            if control and nodes:
                raise RuntimeError("CONTROL_WRITER_ACTIVE:" + topic)
        all_nodes = {n for group in graph for _, nodes in group for n in nodes}
        if "/uav_navigation" in all_nodes:
            raise RuntimeError("CONTROLLER_STILL_RUNNING")
        state = rospy.wait_for_message("/mavros/state", State, timeout=8)
        landed = rospy.wait_for_message("/mavros/extended_state", ExtendedState, timeout=8)
        now = rospy.Time.now().to_sec()
        for msg in (state, landed):
            stamp = msg.header.stamp.to_sec()
            if stamp <= 0 or not 0 <= now - stamp <= 5:
                raise RuntimeError("STALE_GROUND_CONFIRMATION")
        if not state.connected or state.armed or landed.landed_state != ExtendedState.LANDED_STATE_ON_GROUND:
            raise RuntimeError("REFUSE_RESET_NOT_CONNECTED_DISARMED_LANDED")
        print("GROUND_GUARD_OK connected=true armed=false landed_state=1", flush=True)


def receipt(args):
    guarded_ground()
    root = Path("/workspace")
    state_dir = root / "build/training_city"
    world = Path((state_dir / "current_world.txt").read_text().strip())
    metadata = Path((state_dir / "current_metadata.txt").read_text().strip())
    data = json.loads(metadata.read_text())
    generator = data["generator"]
    if (generator["preset"], generator["category"], generator["seed"]) != ("unit", "single_wall", 42):
        raise RuntimeError("NOT_SINGLE_WALL_SEED_42")
    rospy.wait_for_service("/gazebo/get_world_properties", timeout=8)
    response = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)()
    expected = {"training_ground", "iris"} | {o["id"] for o in data["obstacles"]}
    if not response.success or set(response.model_names) != expected:
        raise RuntimeError("LOADED_SCENE_MODEL_MISMATCH:" + str(response.model_names))
    payload = dict(schema="robocup_navigation/scene_launch/v1", launch_id=str(uuid.UUID(args.launch_id)),
                   created_utc=time.time(), world_file=str(world), metadata_file=str(metadata),
                   world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
                   metadata_sha256=hashlib.sha256(metadata.read_bytes()).hexdigest(),
                   container_id=args.container_id, container_started_at=args.started_at,
                   verified_model_names=sorted(response.model_names),
                   provenance="host force-recreated sim, then checked ground and loaded models")
    path = state_dir / "active_launch.json"
    temporary = path.with_suffix(".json." + args.launch_id + ".tmp")
    with temporary.open("x") as stream:
        json.dump(payload, stream, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)
    print(json.dumps(payload, sort_keys=True))


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("guard")
    command = sub.add_parser("receipt")
    command.add_argument("--launch-id", required=True)
    command.add_argument("--container-id", required=True)
    command.add_argument("--started-at", required=True)
    args = parser.parse_args()
    if args.command == "guard":
        guarded_ground()
    else:
        receipt(args)


if __name__ == "__main__":
    main()
