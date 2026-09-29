#!/usr/bin/env python3
"""Offline 100-seed engineering threshold (寻路专项开发计划 初始门槛):

  - 4 reachable categories x 25 seeds  -> prepare_route MUST produce a
    collision-free route (code 0 path, no exception).
  - 3 unsolvable categories x 10 seeds -> bounded exit with a RouteError code.
  - empty x 5 -> sanity planning.

No ROS, no flight: pure route_runtime on freshly generated worlds.
Writes CSV to /tmp/offline_batch_results.csv and prints a summary with
planning-time P50/P95 per category.
"""
import csv
import hashlib
import json
import os
import statistics
import subprocess
import sys
import time
import uuid

WS = os.environ.get("ROBOCUP_WORKSPACE", os.path.expanduser("~/robocup/robocup_ws"))
SRC = WS + "/src/robocup_navigation/src"
sys.path.insert(0, SRC)
import yaml  # noqa: E402
from robocup_navigation import route_runtime  # noqa: E402

BATCH = (
    [("single_wall", s) for s in range(1, 26)] +
    [("wall_with_gap", s) for s in range(1, 26)] +
    [("u_shape", s) for s in range(1, 26)] +
    [("narrow_corridor", s) for s in range(1, 26)] +
    [("empty", s) for s in range(1, 6)] +
    [("blocked", s) for s in range(1, 11)] +
    [("no_path", s) for s in range(1, 11)] +
    [("goal_in_obstacle", s) for s in range(1, 11)]
)

GEN = WS + "/scripts/container/generate_training_city.sh"
TOOLS = WS + "/scripts/vm/vm_scenario_tools.py"
BASE_CFG = WS + "/src/robocup_navigation/config/single_wall_route.yaml"
CSV_OUT = "/tmp/offline_batch_results.csv"

rows = []
ENV = dict(os.environ, ROBOCUP_WORKSPACE=WS)  # generator defaults to /workspace without it
for index, (cat, seed) in enumerate(BATCH, 1):
    t0 = time.time()
    gen = subprocess.run(["bash", GEN, "--preset", "unit", "--category", cat,
                          "--seed", str(seed)],
                         capture_output=True, text=True, env=ENV)
    if gen.returncode != 0:
        rows.append([cat, seed, "GENERATE_FAILED", gen.stderr.strip()[-120:], 0.0, time.time() - t0])
        print("[%3d/%d] %s s%d GENERATE_FAILED" % (index, len(BATCH), cat, seed))
        continue

    prep = subprocess.run(["python3", TOOLS, "prepare", cat, str(seed), WS],
                          capture_output=True, text=True, env=ENV)
    if prep.returncode != 0:
        rows.append([cat, seed, "PREPARE_FAILED", prep.stderr.strip()[-120:], 0.0, time.time() - t0])
        print("[%3d/%d] %s s%d PREPARE_FAILED" % (index, len(BATCH), cat, seed))
        continue

    # prepare's yaml string-substitution only matches when seed==42, so do
    # NOT trust scenario_<cat>.yaml.  Use its printed START/GOAL (derived
    # from this seed's metadata) and build the config explicitly.
    import re
    m_start = re.search(r"START=(-?[\d.]+),(-?[\d.]+)", prep.stdout)
    m_goal = re.search(r"GOAL=(-?[\d.]+),(-?[\d.]+)", prep.stdout)
    if not (m_start and m_goal):
        rows.append([cat, seed, "ENDPOINT_PARSE_FAILED", prep.stdout.strip()[-100:], 0.0, time.time() - t0])
        print("[%3d/%d] %s s%d ENDPOINT_PARSE_FAILED" % (index, len(BATCH), cat, seed))
        continue

    world = "%s/src/robocup_training_worlds/worlds/generated/training_city_unit_%s_s%d.world" % (WS, cat, seed)
    metadata_file = world[:-len(".world")] + ".json"
    cfg = yaml.safe_load(open(BASE_CFG))
    cfg["world_file"] = world
    cfg["metadata_file"] = metadata_file
    cfg["route_start_world_xy"] = [float(m_start.group(1)), float(m_start.group(2))]
    cfg["route_goal_world_xy"] = [float(m_goal.group(1)), float(m_goal.group(2))]

    receipt = {
        "schema": route_runtime.RECEIPT_SCHEMA,
        "launch_id": str(uuid.uuid4()),
        "created_utc": time.time(),
        "world_file": world,
        "metadata_file": metadata_file,
        "world_sha256": hashlib.sha256(open(world, "rb").read()).hexdigest(),
        "metadata_sha256": hashlib.sha256(open(metadata_file, "rb").read()).hexdigest(),
        "container_id": "offline-batch",
        "container_started_at": time.time() - 1,
    }
    receipt_path = "/tmp/offline_receipt_%s_%d.json" % (cat, seed)
    json.dump(receipt, open(receipt_path, "w"))
    cfg["scene_receipt_file"] = receipt_path

    try:
        p0 = time.time()
        plan = route_runtime.prepare_route(cfg)
        elapsed = time.time() - p0
        rows.append([cat, seed, "PLAN_OK",
                     "route_points=%d" % len(plan.route_world), elapsed,
                     time.time() - t0])
        print("[%3d/%d] %s s%d PLAN_OK %.3fs" % (index, len(BATCH), cat, seed, elapsed))
    except route_runtime.RouteError as exc:
        elapsed = time.time() - p0
        code = str(exc).split(":")[0]
        rows.append([cat, seed, "BOUNDED_EXIT", code, elapsed, time.time() - t0])
        print("[%3d/%d] %s s%d BOUNDED_EXIT %s" % (index, len(BATCH), cat, seed, code))
    except Exception as exc:  # noqa: BLE001 - anything else is a harness bug
        rows.append([cat, seed, "UNEXPECTED", type(exc).__name__ + ":" + str(exc)[:100],
                     0.0, time.time() - t0])
        print("[%3d/%d] %s s%d UNEXPECTED %s" % (index, len(BATCH), cat, seed, exc))

with open(CSV_OUT, "w", newline="") as fh:
    writer = csv.writer(fh)
    writer.writerow(["category", "seed", "outcome", "detail", "plan_s", "total_s"])
    writer.writerows(rows)

print("\n===== SUMMARY =====")
by_cat = {}
for cat, seed, outcome, detail, plan_s, total_s in rows:
    by_cat.setdefault(cat, []).append((outcome, plan_s))

REACHABLE = {"single_wall", "wall_with_gap", "u_shape", "narrow_corridor", "empty"}
UNSOLVABLE = {"blocked", "no_path", "goal_in_obstacle"}
all_ok = True
for cat in sorted(by_cat):
    entries = by_cat[cat]
    outcomes = [o for o, _ in entries]
    times = [t for _, t in entries if t > 0]
    p50 = statistics.median(times) if times else 0.0
    p95 = statistics.quantiles(times, n=20)[18] if len(times) >= 20 else (max(times) if times else 0.0)
    if cat in REACHABLE:
        ok = outcomes.count("PLAN_OK")
        status = "PASS" if ok == len(outcomes) else "FAIL"
        if status == "FAIL":
            all_ok = False
        print("%-16s %s %d/%d PLAN_OK  plan P50=%.3fs P95=%.3fs"
              % (cat, status, ok, len(outcomes), p50, p95))
    else:
        codes = {}
        for o, _ in entries:
            codes[o] = codes.get(o, 0) + 1
        bounded = sum(v for k, v in codes.items() if k == "BOUNDED_EXIT")
        status = "PASS" if bounded == len(entries) else "FAIL"
        if status == "FAIL":
            all_ok = False
        detail_codes = {}
        for cat2, seed2, outcome, detail, plan_s, total_s in rows:
            if cat2 == cat and outcome == "BOUNDED_EXIT":
                detail_codes[detail] = detail_codes.get(detail, 0) + 1
        print("%-16s %s %d/%d bounded  codes=%s  P50=%.3fs"
              % (cat, status, bounded, len(entries), detail_codes, p50))

print("THRESHOLD_%s" % ("PASS" if all_ok else "FAIL"))
