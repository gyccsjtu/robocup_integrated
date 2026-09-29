#!/usr/bin/env bash
set -Eeuo pipefail

source /workspace/scripts/container/single_uav_env.sh >/dev/null

fail() {
  printf '[health] ERROR: %s\n' "$*" >&2
  exit 1
}

[[ "$(rostopic type /clock)" == "rosgraph_msgs/Clock" ]] || fail "/clock type is missing or wrong"
[[ "$(rostopic type /mavros/state)" == "mavros_msgs/State" ]] || fail "/mavros/state type is missing or wrong"
[[ "$(rostopic type /mavros/local_position/pose)" == "geometry_msgs/PoseStamped" ]] || fail "local pose type is missing or wrong"

world="$(rosservice call /gazebo/get_world_properties '{}')"
grep -q -- '- iris' <<<"${world}" || fail "Gazebo iris model is missing"

state="$(timeout 10s rostopic echo -n 1 /mavros/state)"
grep -q 'connected: True' <<<"${state}" || fail "MAVROS is not connected to PX4"
timeout 10s rostopic echo -n 1 /clock >/dev/null || fail "no simulated clock sample"

pose="$(timeout 10s rostopic echo -n 1 /mavros/local_position/pose)"
grep -q 'frame_id: "map"' <<<"${pose}" || fail "no local pose in map frame"

printf '[health] ROS master: OK\n'
printf '[health] Gazebo model iris: OK\n'
printf '[health] /clock rosgraph_msgs/Clock: OK\n'
printf '[health] MAVROS connected: OK\n'
printf '[health] local pose geometry_msgs/PoseStamped: OK\n'
