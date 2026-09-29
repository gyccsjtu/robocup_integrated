#!/usr/bin/env python3
"""Bring up the six-UAV coordination stack on top of a running Gazebo sim.

Starts, per run:
    * one coordination_node.py (the Codex core, wrapped by the WB adapter)
    * one coordination_executor.py per UAV (actuator + state/ACK reporter)
    * one coordination_targetsim.py (DEV perception substitute)

Then waits `run.duration_s`, stops everything and prints where the logs are.

This is a DEV HARNESS (interface v0 section 5). It is not the competition
pipeline and it proves nothing on its own -- evidence comes from
coord_events.jsonl plus the Codex verdict afterwards.

Usage:
    python3 start_coordination_sim.py --out <run_dir> [--duration 150]
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import mu_fleet  # noqa: E402

WS = os.environ.get("ROBOCUP_WORKSPACE", os.path.abspath(os.path.join(_HERE, "..", "..")))
DEFAULT_CFG = os.path.join(WS, "config", "coordination", "six_uav_gazebo.json")


def _spawn(cmd, log_path, env=None):
    env = dict(os.environ if env is None else env)
    src = os.path.join(WS, "src", "robocup_navigation", "src")
    env["PYTHONPATH"] = src + os.pathsep + env.get("PYTHONPATH", "")
    fh = open(log_path, "w")
    return subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, env=env,
                            preexec_fn=os.setsid)


def _stop(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except Exception:
        pass
    for _ in range(30):
        if proc.poll() is not None:
            return
        time.sleep(0.1)
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except Exception:
        proc.kill()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=DEFAULT_CFG)
    ap.add_argument("--out", required=True, help="run directory (created)")
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--limit", type=int, default=None, help="only the first N UAVs")
    args = ap.parse_args()

    cfg = json.load(open(args.config))
    fleet = mu_fleet.uavs(mu_fleet.load())
    if args.limit:
        fleet = fleet[:args.limit]
    duration = args.duration or cfg["run"]["duration_s"]

    out = os.path.abspath(args.out)
    cfgd = os.path.join(out, "cfg")
    logd = os.path.join(out, "logs")
    os.makedirs(cfgd, exist_ok=True)
    os.makedirs(logd, exist_ok=True)

    # --- per-UAV executor configs -----------------------------------------
    targets = cfg["targets"]
    obs_alt = float(cfg.get("observation_altitude_m",
                            cfg["executor"]["takeoff_altitude_m"]))
    for i, u in enumerate(fleet):
        t = targets[i % len(targets)]
        uav_cfg = dict(cfg["executor"])
        uav_cfg.update(dict(uav_id=u["id"] and "uav_%d" % u["id"],
                            mavros_ns=u["mavros_ns"],
                            spawn_xyz=[u["spawn_x"], u["spawn_y"], u["spawn_z"]],
                            task_id="task_%d" % u["id"],
                            target_id=t["target_id"],
                            target_xyz=[t["xyz"][0], t["xyz"][1], obs_alt],
                            log_dir=os.path.join(out, "uav_%d" % u["id"])))
        path = os.path.join(cfgd, "uav_%d.json" % u["id"])
        json.dump(uav_cfg, open(path, "w"), indent=2)

    # --- target sim config -------------------------------------------------
    ts_cfg = dict(targets=[targets[i % len(targets)] for i in range(len(fleet))],
                  rate_hz=cfg["run"]["targetsim_rate_hz"])
    json.dump(ts_cfg, open(os.path.join(cfgd, "targetsim.json"), "w"), indent=2)

    # --- tasks for the coordinator ----------------------------------------
    tasks = {"task_%d" % u["id"]:
             {"target_id": targets[i % len(targets)]["target_id"],
              "xyz": [targets[i % len(targets)]["xyz"][0],
                      targets[i % len(targets)]["xyz"][1], obs_alt]}
             for i, u in enumerate(fleet)}

    procs = []
    run_id = os.path.basename(out.rstrip("/"))
    # Run the scripts directly instead of `rosrun`: the catkin devel space
    # does not pick up newly added scripts until the package is rebuilt.
    pkg_scripts = os.path.join(WS, "src", "robocup_navigation", "scripts")
    wb_scripts = os.path.join(WS, "scripts")

    # executors FIRST: the monitor has to cover the whole run, so it must be able
    # to see every vehicle before the coordinator declares RUN_STARTED.
    for u in fleet:
        env = dict(os.environ)
        env["ROS_NAMESPACE"] = u["ros_ns"]
        cmd = ["python3", "-u", os.path.join(pkg_scripts, "coordination_executor.py"),
               "_config_file:=" + os.path.join(cfgd, "uav_%d.json" % u["id"])]
        procs.append(_spawn(cmd, os.path.join(logd, "uav_%d.log" % u["id"]), env))
        print("[coord_sim] uav_%d pid=%d ns=%s" % (u["id"], procs[-1].pid, u["ros_ns"]))
        time.sleep(0.4)

    # wait until every executor is publishing VEHICLE_STATE, then start the
    # coordinator; otherwise coverage_start_sim_s lands after RUN_STARTED and the
    # verdict can only ABSTAIN (MONITOR_COVERAGE_INCOMPLETE).
    def _topics():
        try:
            r = subprocess.run(["rostopic", "list"], stdout=subprocess.PIPE,
                               stderr=subprocess.DEVNULL, timeout=10)
            return r.stdout.decode("utf-8", "replace")
        except Exception:  # noqa: BLE001
            return ""

    # A/B switch: with ROBOCUP_COORD_FIRST=1 the coordinator is started before the
    # executors (the ordering used by the runs that did grant routes), instead of
    # after the fleet reports (which the coverage check wants).
    coord_first = os.environ.get("ROBOCUP_COORD_FIRST", "0") == "1"
    if coord_first:
        coord_cmd = ["python3", "-u", os.path.join(pkg_scripts, "coordination_node.py"),
                     "_run_id:=" + run_id,
                     "_fleet_ids:=" + json.dumps(["uav_%d" % u["id"] for u in fleet]),
                     "_limits:=" + json.dumps(cfg["limits"]),
                     "_tasks:=" + json.dumps(tasks),
                     "_tick_hz:=%s" % cfg["run"]["tick_hz"],
                     "_log_dir:=" + os.path.join(out, "coord"),
                     "_debug_core:=" + ("true" if os.environ.get("ROBOCUP_DEBUG_CORE") == "1" else "false")]
        procs.append(_spawn(coord_cmd, os.path.join(logd, "coordinator.log")))
        print("[coord_sim] coordinator pid=%d (ROBOCUP_COORD_FIRST=1)" % procs[-1].pid)

    need = ["/uav_%d/coordination/vehicle_state" % u["id"] for u in fleet]
    for _ in range(60):
        listing = _topics()
        if all(t in listing for t in need):
            break
        time.sleep(2)
    else:
        print("[coord_sim] WARNING: not all vehicle_state topics appeared")
    time.sleep(3)   # let a few samples land before the run is declared started

    coord_cmd = ["python3", "-u", os.path.join(pkg_scripts, "coordination_node.py"),
                 "_run_id:=" + run_id,
                 "_fleet_ids:=" + json.dumps(["uav_%d" % u["id"] for u in fleet]),
                 "_limits:=" + json.dumps(cfg["limits"]),
                 "_tasks:=" + json.dumps(tasks),
                 "_tick_hz:=%s" % cfg["run"]["tick_hz"],
                 "_log_dir:=" + os.path.join(out, "coord"),
                 "_debug_core:=" + ("true" if os.environ.get("ROBOCUP_DEBUG_CORE") == "1" else "false")]
    if not coord_first:
        procs.append(_spawn(coord_cmd, os.path.join(logd, "coordinator.log")))
        print("[coord_sim] coordinator pid=%d (after fleet reported)" % procs[-1].pid)

    # dev perception substitute
    ts_cmd = ["python3", "-u", os.path.join(wb_scripts, "coordination_targetsim.py"),
              "_targets:=" + json.dumps(ts_cfg["targets"]),
              "_rate_hz:=%s" % ts_cfg["rate_hz"]]
    procs.append(_spawn(ts_cmd, os.path.join(logd, "targetsim.log")))
    print("[coord_sim] targetsim pid=%d" % procs[-1].pid)

    print("[coord_sim] running %.0fs -> %s" % (duration, out))
    try:
        time.sleep(duration)
    except KeyboardInterrupt:
        print("[coord_sim] interrupted")
    finally:
        for p in procs:
            _stop(p)
        print("[coord_sim] stopped %d processes" % len(procs))

    # --- quick evidence summary -------------------------------------------
    ev = os.path.join(out, "coord", "coord_events.jsonl")
    print("[coord_sim] events: %s (%s)" % (ev, "present" if os.path.exists(ev) else "MISSING"))
    if os.path.exists(ev):
        from collections import Counter
        c = Counter()
        for line in open(ev):
            try:
                c[json.loads(line).get("data", {}).get("event") or
                  json.loads(line).get("event")] += 1
            except ValueError:
                c["<malformed>"] += 1
        for k, v in sorted(c.items(), key=lambda kv: -kv[1]):
            print("   %-34s %d" % (k, v))
    print("[coord_sim] done. run dir: %s" % out)


if __name__ == "__main__":
    main()
