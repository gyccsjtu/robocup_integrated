#!/usr/bin/env bash
set -Eeuo pipefail
source "$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)/common.sh"

assert_official_config
log 'running container health checks'
compose exec --no-TTY sim bash /workspace/scripts/container/health_check.sh
