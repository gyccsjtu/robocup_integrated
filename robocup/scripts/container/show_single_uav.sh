#!/usr/bin/env bash
# Display only: this process never sends MAVROS flight commands.
set -eo pipefail
source /workspace/scripts/container/single_uav_env.sh
export XDG_RUNTIME_DIR=/tmp/robocup-gui-runtime
mkdir -p "${XDG_RUNTIME_DIR}"
chmod 700 "${XDG_RUNTIME_DIR}"
gzclient --verbose &
client_pid=$!
trap 'kill -TERM "${client_pid}" 2>/dev/null || true' EXIT
trap 'exit 0' INT TERM
ready=false
deadline=$((SECONDS + 55))
while (( SECONDS < deadline )); do
  if ! kill -0 "${client_pid}" 2>/dev/null; then
    wait "${client_pid}" || true
    echo '[viewer] gzclient exited before its camera became available' >&2
    exit 1
  fi
  if timeout 3s gz camera -l 2>/dev/null | grep -qx gzclient_camera; then
    if timeout 5s gz camera -c gzclient_camera -f iris; then
      touch /tmp/robocup-gui-ready
      echo '[viewer] READY: Gazebo camera connected and following iris'
      ready=true
      break
    fi
  fi
  sleep 1
done
if [[ "${ready}" != true ]]; then
  echo '[viewer] Timed out waiting for the Gazebo camera' >&2
  exit 1
fi
wait "${client_pid}"
