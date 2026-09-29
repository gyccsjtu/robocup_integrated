#!/usr/bin/env bash
# Offline generator tests: no Gazebo, no GUI, no ROS master.
set -Eeuo pipefail

workspace="${ROBOCUP_WORKSPACE:-/workspace}"
pkg="${workspace}/src/robocup_training_worlds"

cd "${pkg}"
exec python3 -m unittest discover -s tests -p 'test_*.py' -v
