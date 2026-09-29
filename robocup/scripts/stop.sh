#!/usr/bin/env bash
set -Eeuo pipefail
source "$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)/common.sh"

if [[ -f "$env_file" ]]; then
  log 'stopping containers without deleting images or official assets'
  compose down --remove-orphans
else
  log 'No .env exists; nothing to stop.'
fi
