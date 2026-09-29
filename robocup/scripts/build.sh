#!/usr/bin/env bash
set -Eeuo pipefail
source "$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)/common.sh"

assert_official_config
log 'validating compose configuration and local official image'
compose config --quiet
docker image inspect "$(env_value ROBOCUP_OFFICIAL_IMAGE)" >/dev/null || die 'Organizer image is not present locally. Import its supplied tarball before an offline run.'
log 'running clean catkin build in the organizer image'
compose run --rm --no-deps --entrypoint bash sim /workspace/scripts/container/init_workspace.sh
