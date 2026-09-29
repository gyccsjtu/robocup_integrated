#!/usr/bin/env bash
set -Eeuo pipefail
source "$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)/common.sh"

"$script_dir/stop.sh"
for name in build devel install logs; do
  target="$workspace_root/$name"
  [[ "$target" == "$workspace_root/"* ]] || die "Unsafe reset target: $target"
  if [[ -e "$target" ]]; then
    log "removing generated runtime directory: $target"
    rm -rf -- "$target"
  fi
done
rm -f -- "$workspace_root/.catkin_workspace"
if [[ -L "$workspace_root/src/CMakeLists.txt" ]]; then
  log "removing generated catkin symlink: $workspace_root/src/CMakeLists.txt"
  rm -f -- "$workspace_root/src/CMakeLists.txt"
fi
mkdir -p "$workspace_root/logs"
log 'reset complete; source, configuration, and official assets were preserved.'
