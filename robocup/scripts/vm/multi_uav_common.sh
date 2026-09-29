#!/usr/bin/env bash
# Shared helpers for the six-UAV fleet scripts. Source this file; do not exec it.
#
# IMPORTANT — pkill safety:
#   All patterns used to kill simulation processes are defined HERE, inside a
#   file. They are therefore never part of the invoking shell's own command
#   line, so `pkill -f <pattern>` cannot match (and kill) the launcher itself.
#   (An inline `pkill -9 -f run_headon` in an ad-hoc ssh command DOES self-match
#   and kills the remote shell — see project memory.)
set -uo pipefail

_mu_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROBOCUP_WORKSPACE="${ROBOCUP_WORKSPACE:-$(cd "${_mu_dir}/../.." && pwd)}"
MU_WS="${ROBOCUP_WORKSPACE}"
export MU_FLEET_YAML="${MU_FLEET_YAML:-${MU_WS}/config/multi_uav/fleet.yaml}"
MU_PY="${MU_PY:-python3}"
MU_FLEET_PY="${_mu_dir}/mu_fleet.py"

mu_log()  { printf '[multi_uav] %s\n' "$*"; }
mu_fail() { printf '[multi_uav] ERROR: %s\n' "$*" >&2; exit 1; }

mu_size() { "${MU_PY}" "${MU_FLEET_PY}" size; }
mu_rows() { "${MU_PY}" "${MU_FLEET_PY}" rows; }

# Kill every simulation/navigation process owned by this fleet.
mu_kill_all() {
    pkill -9 -f "uav_navigation_nod[e]" 2>/dev/null || true
    pkill -9 -f "fly_headon_[b]"        2>/dev/null || true
    pkill -9 -f "dynamic_block_spawne[r]" 2>/dev/null || true
    pkill -9 -f "mavros_nod[e]"         2>/dev/null || true
    pkill -9 -f "spawn_mode[l]"         2>/dev/null || true
    pkill -9 gzserver 2>/dev/null || true
    pkill -9 gzclient 2>/dev/null || true
    pkill -9 px4      2>/dev/null || true
    pkill -9 rosmaster 2>/dev/null || true
    # The two multi-UAV roslaunch entry points (stock + this repo's dev harness).
    # Bracket the last char so the pattern cannot match this file's own text.
    pkill -9 -f "multi_uav_de[v]"              2>/dev/null || true
    pkill -9 -f "multi_uav_mavros_sit[l]"      2>/dev/null || true
}

# True if any fleet process is still alive.
mu_any_alive() {
    pgrep -af "uav_navigation_nod[e]|mavros_nod[e]|gzserve[r]|px[4]" >/dev/null 2>&1
}
