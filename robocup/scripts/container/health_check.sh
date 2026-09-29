#!/usr/bin/env bash
set -Eeuo pipefail

status=0
ok() { printf '[PASS] %s\n' "$*"; }
warn() { printf '[WARN] %s\n' "$*"; }
fail() { printf '[FAIL] %s\n' "$*" >&2; status=1; }

if [[ -f "${ROBOCUP_ROS_SETUP:-}" ]]; then
  source "$ROBOCUP_ROS_SETUP"
  ok "ROS setup: $ROBOCUP_ROS_SETUP"
else
  fail "ROS setup missing: ${ROBOCUP_ROS_SETUP:-unset}"
fi

if [[ -d "${ROBOCUP_OFFICIAL_ASSETS_DIR:-}" ]] && [[ -n "$(find "$ROBOCUP_OFFICIAL_ASSETS_DIR" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  ok "official assets mounted read-only at $ROBOCUP_OFFICIAL_ASSETS_DIR"
else
  fail "official assets are absent or empty"
fi

if command -v nvidia-smi >/dev/null 2>&1; then
  ok "NVIDIA utility visible: $(nvidia-smi --query-gpu=name --format=csv,noheader | head -n1)"
elif [[ "${ROBOCUP_GPU_REQUIRED:-0}" == "1" ]]; then
  fail "GPU was required but nvidia-smi is not visible in container"
else
  warn "GPU is not visible in container (not required by current configuration)"
fi

if command -v rosnode >/dev/null 2>&1 && rosnode list >/tmp/robocup_rosnode.out 2>/tmp/robocup_rosnode.err; then
  ok "ROS master responds"
else
  fail "ROS master is not reachable"
fi

if command -v rostopic >/dev/null 2>&1; then
  topic_file=/workspace/config/expected_topics.txt
  if [[ -s "$topic_file" ]]; then
    while IFS= read -r topic; do
      [[ -z "$topic" || "$topic" == \#* ]] && continue
      rostopic info "$topic" >/dev/null 2>&1 && ok "expected topic visible: $topic" || fail "expected topic absent: $topic"
    done < "$topic_file"
  else
    warn "no official expected topics configured; no topic names were invented"
  fi
  rostopic info /clock >/dev/null 2>&1 && ok "ROS clock topic visible" || warn "ROS clock topic not visible"
else
  fail "rostopic is unavailable"
fi

exit "$status"
