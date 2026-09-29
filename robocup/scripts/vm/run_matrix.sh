#!/usr/bin/env bash
# Generic scenario-matrix runner (批量测试编排 + 日志采集).
#
# Runs every scenario listed in a spec file sequentially, records each step's
# OUTCOME lines into one summary, then builds a run manifest via collect_runs.py.
#
# Spec format (one scenario per line; blank lines and '#' comments ignored):
#     <label>|<launcher>|<space-separated args>
# Example:
#     headon#1|scripts/vm/run_headon.sh|42 headon 0.30
#     dynamic_block|scripts/vm/run_dynamic_block.sh|42
#
# Boundary (AGENTS.md): this script only orchestrates and collects logs. It does
# NOT decide PASS/FAIL -- acceptance verdicts wait for the Codex-frozen event
# schema v1 (collect_runs.py reports PENDING_SCHEMA until then).
set -uo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/multi_uav_common.sh"

SPEC="${1:?usage: run_matrix.sh <spec-file>}"
[[ -f "${SPEC}" ]] || mu_fail "spec not found: ${SPEC}"

SUMMARY="${MU_WS}/logs/matrix_summary.txt"
MARKER="${MU_WS}/logs/matrix_done.marker"
mkdir -p "${MU_WS}/logs"
: > "${SUMMARY}"
rm -f "${MARKER}"

echo "MATRIX_START $(date -u +%Y%m%dT%H%M%SZ) spec=${SPEC}" | tee -a "${SUMMARY}"

# Each launcher self-cleans, but clear leftovers first for a deterministic start.
mu_kill_all
sleep 2

while IFS='|' read -r label launcher args; do
    label="$(printf '%s' "${label}" | xargs)"
    launcher="$(printf '%s' "${launcher}" | xargs)"
    args="$(printf '%s' "${args:-}" | xargs)"
    [[ -z "${label}" || "${label:0:1}" == "#" ]] && continue
    echo "" | tee -a "${SUMMARY}"
    echo "########## ${label} ##########" | tee -a "${SUMMARY}"
    # Intentional word-splitting of ${args}: the spec passes launcher CLI args.
    # shellcheck disable=SC2086
    bash "${MU_WS}/${launcher}" ${args} 2>&1 | tee -a "${SUMMARY}"
    echo "########## ${label} DONE ##########" | tee -a "${SUMMARY}"
done < "${SPEC}"

echo "" | tee -a "${SUMMARY}"
echo "MATRIX_DONE $(date -u +%Y%m%dT%H%M%SZ)" | tee -a "${SUMMARY}"

"${MU_PY}" "${MU_WS}/scripts/vm/collect_runs.py" \
    --logs-root "${MU_WS}/logs" --glob "scenario_matrix/*/" | tee -a "${SUMMARY}"

echo "MATRIX_DONE_RC=0" > "${MARKER}"
mu_log "matrix complete -> ${SUMMARY}"
