#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(CDPATH= cd -- "$script_dir/.." && pwd -P)"
env_file="$workspace_root/.env"
compose_file="$workspace_root/docker-compose.yml"

log() { printf '[robocup] %s\n' "$*" >&2; }
die() { log "ERROR: $*"; exit 1; }

require_docker() {
  command -v docker >/dev/null 2>&1 || die 'Docker CLI is unavailable; install/configure it outside this project first.'
  [[ -f "$env_file" ]] || die 'Missing .env. Copy .env.example and fill organizer-confirmed values only.'
}

env_value() {
  local key="$1"
  awk -v prefix="${key}=" 'index($0, prefix) == 1 { print substr($0, length(prefix) + 1); exit }' "$env_file"
}

assert_official_config() {
  require_docker
  local key value
  for key in ROBOCUP_OFFICIAL_IMAGE ROBOCUP_OFFICIAL_ASSETS_DIR ROBOCUP_ROS_SETUP ROBOCUP_OFFICIAL_START_COMMAND; do
    value="$(env_value "$key")"
    [[ -n "$value" && "$value" != PENDING_* ]] || die "Official configuration is incomplete: $key. Do not guess it."
  done
  local asset_dir
  asset_dir="$(env_value ROBOCUP_OFFICIAL_ASSETS_DIR)"
  [[ "$asset_dir" = /* ]] || asset_dir="$workspace_root/$asset_dir"
  [[ -d "$asset_dir" ]] || die "Official assets directory is missing: $asset_dir"
}

compose() {
  require_docker
  docker compose --env-file "$env_file" --project-directory "$workspace_root" -f "$compose_file" "$@"
}
