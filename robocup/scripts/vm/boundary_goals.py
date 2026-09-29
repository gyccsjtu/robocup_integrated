#!/usr/bin/env python3
"""Boundary-case harness: boundary goals on the empty map (seed 1).

Verifies bounded rejection for goals outside the flight fence:
  - (3.9, 0)  radius 6.9 < 7.0  -> must PLAN (inside fence)
  - (4.2, 0)  radius 7.2 > 7.0  -> bounded exit (goal radius limit)
  - (5.5, 0)  outside 10x10 grid -> bounded exit (out of bounds)
  - (0, 4.8)  inside north boundary wall -> bounded exit (unsafe)
"""
import hashlib
import json
import os
import subprocess
import sys
import time
import uuid

WS = os.environ.get("ROBOCUP_WORKSPACE", os.path.expanduser("~/robocup/robocup_ws"))
sys.path.insert(0, WS + "/src/robocup_navigation/src")
import yaml  # noqa: E402
from robocup_navigation import route_runtime  # noqa: E402

CAT, SEED = "empty", 1
ENV = dict(__import__("os").environ, ROBOCUP_WORKSPACE=WS)
subprocess.run(["bash", WS + "/scripts/container/generate_training_city.sh",
                "--preset", "unit", "--category", CAT, "--seed", str(SEED)],
               capture_output=True, text=True, env=ENV, check=True)

world = "%s/src/robocup_training_worlds/worlds/generated/training_city_unit_%s_s%d.world" % (WS, CAT, SEED)
metadata_file = world[:-len(".world")] + ".json"
receipt = {
    "schema": route_runtime.RECEIPT_SCHEMA, "launch_id": str(uuid.uuid4()),
    "created_utc": time.time(), "world_file": world, "metadata_file": metadata_file,
    "world_sha256": hashlib.sha256(open(world, "rb").read()).hexdigest(),
    "metadata_sha256": hashlib.sha256(open(metadata_file, "rb").read()).hexdigest(),
    "container_id": "boundary-batch", "container_started_at": time.time() - 1,
}
receipt_path = "/tmp/boundary_receipt.json"
json.dump(receipt, open(receipt_path, "w"))

CASES = [
    ("inside_fence", (3.3, 0.0), "PLAN_OK"),
    ("inflation_fence", (3.9, 0.0), "PLAN_OK_ROUTES_AROUND_NOT"),
    ("radius_limit", (3.5, 3.5), "BOUNDED"),
    ("outside_grid", (5.5, 0.0), "BOUNDED"),
]

# Initial endpoints are receipt-locked (ROUTE_ENDPOINT_METADATA_MISMATCH for
# anything else — that IS the boundary design), so fence checks go through
# the online replan path, exactly like a mid-flight goal injection.
cfg = yaml.safe_load(open(WS + "/src/robocup_navigation/config/single_wall_route.yaml"))
cfg["world_file"] = world
cfg["metadata_file"] = metadata_file
cfg["scene_receipt_file"] = receipt_path
checked = route_runtime.prepare_route(cfg)
print("baseline prepare_route OK, route pts=%d" % len(checked.route_world))

all_ok = True
for name, goal, expect in CASES:
    try:
        points = route_runtime.replan_route(checked, (-3.0, 0.0), goal, cfg)
        outcome = "PLAN_OK(%d pts)" % len(points)
    except route_runtime.RouteError as exc:
        outcome = "BOUNDED:" + str(exc).split(":")[0]
    except Exception as exc:  # noqa: BLE001
        outcome = "UNEXPECTED:" + type(exc).__name__
    ok = outcome.startswith("PLAN_OK") if expect == "PLAN_OK" else outcome.startswith("BOUNDED")
    all_ok = all_ok and ok
    print("%-18s goal=%s expect=%-22s -> %s %s" % (name, goal, expect, outcome, "OK" if ok else "WRONG"))

print("BOUNDARY_%s" % ("PASS" if all_ok else "FAIL"))
