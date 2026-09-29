# robocup_training_worlds

Deterministic Gazebo Classic training-city generator for the 2026 "multi-rotor
swarm cooperative search simulation" event. It produces a self-contained SDF
world plus a machine-readable metadata document for every seed.

The package generates **environments only**. It never publishes MAVROS
setpoints, never spawns a vehicle, never starts a second MAVROS and never
starts a second controller: the iris, PX4 SITL, MAVROS and the single control
node keep coming from the existing baseline in `robocup_navigation`.

## What is official and what is a training assumption

Official rules (sections 2.4 / 2.5):

- competition area 200 m x 100 m, urban environment;
- buildings, lamp posts and similar objects are present;
- house / lamp positions are randomised by a script before each attempt;
- 6 UAVs, flight altitude <= 6 m;
- 6 targets, outdoors, GNSS available.

Everything below is a **team training assumption** and must never be presented
as an official parameter: road width, block sizes, building counts, sizes and
heights, lamp post size and spacing, the spawn and goal candidate placement,
and the `full` coordinate convention `x in [-100, 100]`, `y in [-50, 50]`
(frame `map`, ENU, metres, ground at `z = 0`). The official world, its
randomisation script and the judge interface have not been published.

## Layout

```text
robocup_training_worlds/
├── config/training_city.yaml               # presets, safety margins, categories
├── scripts/
│   ├── training_city_lib.py                # geometry, grid/BFS, SDF writer, validator
│   ├── generate_training_city.py           # CLI: world + metadata
│   ├── validate_training_city.py           # CLI: pre-launch gate (no GUI)
│   └── rasterize_metadata.py               # CLI: metadata -> PGM + YAML occupancy map
├── launch/training_city_single_uav.launch  # injects the world into px4 mavros_posix_sitl.launch
├── worlds/generated/                       # run-time output (one layout per seed)
└── tests/test_training_city_generator.py   # offline tests
```

## Presets

| preset | size | mode | purpose |
|---|---|---|---|
| `unit` | 10 m x 10 m | fixed categories | A* unit scenarios; sized for the current 5 m radius controller |
| `small` | 30 m x 20 m | random city | single-UAV detour, replanning and coverage rehearsal |
| `full` | 200 m x 100 m | random city | rule-scale coverage and stress testing; **not** for the current 5 m radius controller |

## Generator CLI

```bash
python3 scripts/generate_training_city.py \
  --preset unit --category single_wall --seed 42 \
  --output /tmp/unit_single_wall.world \
  --metadata /tmp/unit_single_wall.json
```

Useful flags: `--category` (unit only), `--no-grid` (omit the occupancy grid),
`--spawn-env FILE` (write `ROBOCUP_SPAWN_X/Y/Z/YAW` for the launch file),
`--allow-invalid` (debug only), `--emit-timestamp` (breaks byte determinism).

Exit codes: `0` ok, `1` validation failed (nothing is written), `2` usage error.

## unit categories

`empty`, `single_wall`, `wall_with_gap`, `u_shape`, `narrow_corridor`,
`blocked` / `no_path` (negative: no path exists), `goal_in_obstacle`
(negative: the goal lies inside a solid block).

Categories fix the **topology**, not the coordinates: wall positions, gap
widths, corridor widths and opening sizes change with the seed.

## Metadata

One JSON document per world: seed, preset, bounds, safety margins, planning
altitude, every obstacle (id, type, shape, centre, size / radius, height,
z range, bbox, footprint), the spawn zone, the outdoor goal candidates, a
run-length encoded inflated occupancy grid, the independent BFS connectivity
result, and the validation report. The same seed reproduces byte-identical
`.world` and `.json` output.

Planners may consume `grid` directly, or the PGM + YAML pair produced by
`rasterize_metadata.py`:

```bash
python3 scripts/rasterize_metadata.py --metadata /tmp/unit_single_wall.json
```

## Safety design

- Every blocking obstacle is inflated by `safety.safety_margin_m` before the
  occupancy grid is built, so a corridor narrower than twice the margin is
  reported as blocked on purpose.
- Connectivity is checked with a plain 4-connected flood fill on that grid -
  never with the A* under test - so diagonal corner gaps cannot be used.
- Walls that must seal a passage stop 5 cm short of the neighbouring wall.
  The gap is impassable once inflated, but the two solids never touch, so the
  "no overlapping obstacles" check stays meaningful.
- Obstacles must span `[0, planning_altitude + vertical_margin]`, so a
  fixed-altitude 2-D planner cannot fly over them.

## Tests

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
```

Covered: determinism (byte-identical worlds and metadata), seed sensitivity,
bounds, no overlap, spawn clearance, goal candidates outdoors, obstacle
altitude coverage, connectivity plus the intentional no-path negative cases,
SDF well-formedness and version, unique model names, collision/visual presence
and agreement, no vehicle model, no plugins, no online model URIs, grid
round-trip, and the validator's ability to reject tampered worlds.
