#!/usr/bin/env bash
# MULTI-UAV START (接入层 only).
#
# What this does:
#   1. derive per-UAV namespaces/model names/ports from config/multi_uav/fleet.yaml
#   2. GENERATE and START a multi-UAV PX4 + MAVROS + Gazebo launch
#      (config/multi_uav/multi_uav_dev.launch, generated -- see
#       scripts/vm/gen_multi_uav_launch.py; it is a DEV HARNESS per
#       interface v0 section 5, not the official competition launch)
#   3. generate per-UAV node configs (namespaced) from a base scenario config
#   4. launch N namespaced uav_navigation_node processes, one log dir per UAV
#
# What it deliberately does NOT do:
#   * no clearance / safety-distance / timeout-action / route-version rules
#     (those belong to the Codex core -- see AGENTS.md)
#   * no claim of flight-worthiness: starting processes is not evidence of a
#     working swarm. Acceptance still requires the Codex verdict on real runs.
#
# Environment knobs:
#   ROBOCUP_MU_SIM=0        skip step 2 (sim already running / external launch)
#   ROBOCUP_MU_NODES=0      skip step 4 (bring up the sim only)
#   ROBOCUP_MU_LIMIT=N      only bring up the first N UAVs
#   ROBOCUP_MU_WORLD=<path> override the Gazebo world
#   ROBOCUP_MU_WAIT=1       wait until all /uav_N/mavros/state report connected
#   ROBOCUP_GAZEBO_GUI=true enable the Gazebo GUI (needs a display)
set -Eeuo pipefail
_mu_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "${_mu_here}/multi_uav_common.sh"

GUI="${ROBOCUP_GAZEBO_GUI:-false}"
WITH_SIM="${ROBOCUP_MU_SIM:-1}"
WITH_NODES="${ROBOCUP_MU_NODES:-1}"
LIMIT="${ROBOCUP_MU_LIMIT:-}"
WORLD="${ROBOCUP_MU_WORLD:-}"
BASE_CFG="${MU_BASE_CFG:-${MU_WS}/build/training_city/scenario_dynamic_block.yaml}"
CFG_DIR="${MU_WS}/build/training_city/multi_uav"
LOG_ROOT="${MU_LOG_ROOT:-${MU_WS}/logs/multi_uav}"
RUN_TS="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="${LOG_ROOT}/${RUN_TS}"

mu_log "fleet size=$(mu_size) run=${RUN_TS} gui=${GUI} sim=${WITH_SIM} nodes=${WITH_NODES} limit=${LIMIT:-all}"
if [[ "${WITH_NODES}" == "1" ]]; then
    [[ -f "${BASE_CFG}" ]] || mu_fail "base config not found: ${BASE_CFG} (prepare a scenario first)"
fi
mkdir -p "${RUN_DIR}"

# --- step 1: generate the multi-UAV launch -------------------------------
LAUNCH_FILE="${CFG_DIR}/multi_uav_dev.launch"
gen_args=(--out "${LAUNCH_FILE}")
[[ -n "${LIMIT}" ]] && gen_args+=(--limit "${LIMIT}")
"${MU_PY}" "${_mu_here}/gen_multi_uav_launch.py" "${gen_args[@]}" | sed 's/^/[sim] /'

# --- step 2: simulation stack (PX4 SITL + MAVROS + Gazebo) ---------------
if [[ "${WITH_SIM}" == "1" ]]; then
    # ROS/colcon setup scripts dereference unset vars, so relax -u while sourcing.
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
    # shellcheck disable=SC1091
    source "${MU_WS}/scripts/container/single_uav_env.sh"
    set -u

    sim_args=("${LAUNCH_FILE}" "gui:=${GUI}")
    [[ -n "${WORLD}" ]] && sim_args+=("world:=${WORLD}")

    mu_log "starting sim: roslaunch ${sim_args[*]}"
    setsid roslaunch "${sim_args[@]}" >"${RUN_DIR}/sim.log" 2>&1 &
    echo "$!" > "${RUN_DIR}/sim.pid"
    mu_log "sim pid=$(cat "${RUN_DIR}/sim.pid") log=${RUN_DIR}/sim.log"
else
    mu_log "ROBOCUP_MU_SIM=0 -> assuming an external simulation stack"
fi

# --- step 3: wait for MAVROS (optional) ----------------------------------
if [[ "${ROBOCUP_MU_WAIT:-0}" == "1" ]]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
    set -u
    mu_log "waiting for ROS master..."
    for _ in $(seq 1 60); do rostopic list >/dev/null 2>&1 && break; sleep 2; done
    while read -r id ros_ns _rest; do
        [[ -n "${LIMIT}" && "${id}" -gt "${LIMIT}" ]] && continue
        mu_log "waiting for ${ros_ns}/mavros/state ..."
        for _ in $(seq 1 90); do
            if timeout 5 rostopic echo -n1 "${ros_ns}/mavros/state" 2>/dev/null | grep -q "connected: True"; then
                mu_log "  ${ros_ns} connected"
                break
            fi
            sleep 2
        done
    done < <(mu_rows)
fi

# --- step 4: per-UAV navigation nodes (namespaced) -----------------------
if [[ "${WITH_NODES}" != "1" ]]; then
    mu_log "ROBOCUP_MU_NODES=0 -> sim only; run dir ${RUN_DIR}"
    exit 0
fi

while read -r id ros_ns mavros_ns model _udp _local _tcp; do
    [[ -n "${LIMIT}" && "${id}" -gt "${LIMIT}" ]] && continue
    logd="${RUN_DIR}/uav_${id}"
    mkdir -p "${logd}"
    mu_log "uav ${id}: ns=${ros_ns} mavros=${mavros_ns} model=${model} logs=${logd}"
    ROS_NAMESPACE="${ros_ns}" nohup rosrun robocup_navigation uav_navigation_node.py \
        _config_file:="${CFG_DIR}/scenario_uav_${id}.yaml" \
        _log_dir:="${logd}" > "${logd}/node.log" 2>&1 &
    echo "$!" > "${logd}/node.pid"
done < <(mu_rows)

mu_log "launched namespaced navigation nodes; run dir ${RUN_DIR}"
