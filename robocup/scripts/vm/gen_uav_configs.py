#!/usr/bin/env python3
"""Generate per-UAV node configs from a base scenario config + fleet mapping.

For each UAV it copies the base scenario YAML and overrides exactly two keys:
    mavros_namespace:   <ros_ns>/mavros      (from fleet.yaml)
    vehicle_model_name: <model_name>         (Gazebo model of that UAV)

Every other key — including all clearance / safety / route values — is left
untouched. Those values are owned by the Codex core; this generator must never
tune them.

Usage:
  gen_uav_configs.py --base build/training_city/scenario_dynamic_block.yaml \
                     --out-dir build/training_city/multi_uav
"""
import argparse
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mu_fleet  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--fleet", default=os.environ.get("MU_FLEET_YAML"))
    args = ap.parse_args()

    if args.fleet:
        os.environ["MU_FLEET_YAML"] = args.fleet
        mu_fleet.FLEET = args.fleet

    with open(args.base) as fh:
        base = yaml.safe_load(fh)
    os.makedirs(args.out_dir, exist_ok=True)

    rows = mu_fleet.uavs(mu_fleet.load())
    for r in rows:
        cfg = dict(base)
        cfg["mavros_namespace"] = r["mavros_ns"]
        cfg["vehicle_model_name"] = r["model_name"]
        path = os.path.join(args.out_dir, "scenario_uav_%d.yaml" % r["id"])
        with open(path, "w") as fh:
            yaml.safe_dump(cfg, fh, allow_unicode=True, sort_keys=False)
        print("UAV %d -> %s (mavros_ns=%s model=%s)"
              % (r["id"], path, r["mavros_ns"], r["model_name"]))
    print("WROTE=%d" % len(rows))


if __name__ == "__main__":
    main()
