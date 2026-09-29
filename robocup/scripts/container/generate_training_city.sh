#!/usr/bin/env bash
# Generate and validate one training world inside the container.
#
# Writes the world + metadata under
#   /workspace/src/robocup_training_worlds/worlds/generated/
# (bind-mounted, so the files are visible on the host) and records the paths
# of the artefacts that start_training_city.sh is allowed to launch.
set -Eeuo pipefail

workspace="${ROBOCUP_WORKSPACE:-/workspace}"
pkg="${workspace}/src/robocup_training_worlds"
out_dir="${pkg}/worlds/generated"
state="${workspace}/build/training_city"

preset="unit"
seed="42"
category=""
world=""

usage() {
    cat >&2 <<'USAGE'
usage: generate_training_city.sh [--preset unit|small|full] [--seed N]
                                 [--category NAME] [--output PATH]

  --preset     unit (10x10 m), small (30x20 m) or full (200x100 m)
  --seed       randomisation seed (the same seed reproduces the same layout)
  --category   unit-preset scenario class (empty, single_wall, wall_with_gap,
               u_shape, narrow_corridor, blocked, no_path, goal_in_obstacle)
  --output     explicit .world path (default: worlds/generated/<name>.world)
USAGE
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --preset) preset="$2"; shift 2 ;;
        --seed) seed="$2"; shift 2 ;;
        --category) category="$2"; shift 2 ;;
        --output) world="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) printf '[generate_training_city] unknown argument: %s\n' "$1" >&2; usage; exit 2 ;;
    esac
done

case "$preset" in
    unit|small|full) ;;
    *) printf '[generate_training_city] invalid preset: %s\n' "$preset" >&2; usage; exit 2 ;;
esac

mkdir -p "$out_dir" "$state"

if [[ -z "$world" ]]; then
    suffix=""
    [[ -n "$category" ]] && suffix="_${category}"
    world="${out_dir}/training_city_${preset}${suffix}_s${seed}.world"
fi

args=(--config "${pkg}/config/training_city.yaml"
      --preset "$preset"
      --seed "$seed"
      --output "$world"
      --spawn-env "${state}/spawn.env")
if [[ -n "$category" ]]; then
    args+=(--category "$category")
fi

printf '[generate_training_city] preset=%s category=%s seed=%s\n' \
    "$preset" "${category:-default}" "$seed"

python3 "${pkg}/scripts/generate_training_city.py" "${args[@]}" >/dev/null

# Independent re-validation of what is actually on disk. A failure here must
# stop the pipeline: start_training_city.sh refuses to fly on a stale map.
python3 "${pkg}/scripts/validate_training_city.py" \
    --world "$world" --metadata "${world%.world}.json" >/dev/null

printf '%s\n' "$world" > "${state}/current_world.txt"
printf '%s\n' "${world%.world}.json" > "${state}/current_metadata.txt"

printf '[generate_training_city] ready: %s\n' "$world"
printf '%s\n' "$world"
printf '%s\n' "${world%.world}.json"
