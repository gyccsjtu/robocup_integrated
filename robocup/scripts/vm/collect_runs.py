#!/usr/bin/env python3
"""Batch run collection + summarization for the RoboCup scenario matrix.

Collects every run directory under a logs root (default ``logs/scenario_matrix``),
reads each ``events.jsonl`` when present, and emits:

  * a machine-readable manifest (JSON) with one record per run
  * a human summary: durations, terminal reason codes, and missing-evidence count

VERDICT BOUNDARY (see AGENTS.md):
    PASS/FAIL is NOT decided here.  Event-schema parsing and the acceptance
    verdict wait for the Codex-frozen interface (schema v1).  This tool
    therefore reports ``verdict: PENDING_SCHEMA`` for every run unless a verdict
    module is supplied via ``--verdict-module`` -- a Python file exposing
    ``verdict(run: dict) -> (bool | None, str)``.  A verdict module MUST NOT
    report PASS on missing evidence; return ``None`` to abstain.

Stdlib only, so it runs with the VM's system python3.
"""
import argparse
import glob
import json
import os
import sys
from collections import Counter

DEFAULT_ROOT = os.environ.get(
    "ROBOCUP_WORKSPACE", os.path.expanduser("~/robocup/robocup_ws")
) + "/logs"

# Keys that commonly carry a terminal verdict in the current (pre-freeze) logs.
_REASON_KEYS = ("reason", "code", "result", "outcome")
_SENTINEL_EVENTS = ("RESULT", "FAILURE", "ABORTED", "LANDED")


def read_events(path):
    events, bad = [], 0
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    events.append(json.loads(line))
                except ValueError:
                    bad += 1
    except OSError as exc:
        return [], 0, str(exc)
    return events, bad, None


def summarize_run(run_dir):
    events_path = os.path.join(run_dir, "events.jsonl")
    rec = {
        "run_dir": run_dir,
        "name": os.path.basename(run_dir.rstrip("/\\")),
        "has_events": os.path.exists(events_path),
        "n_events": 0,
        "malformed_lines": 0,
        "started_ts": None,
        "last_ts": None,
        "duration_s": None,
        "terminal": None,
        "reasons": Counter(),
        "read_error": None,
        "verdict": "PENDING_SCHEMA",
        "verdict_detail": "verdict waits for frozen schema v1",
    }
    if not rec["has_events"]:
        rec["verdict_detail"] = "no events.jsonl (missing evidence)"
        return rec

    events, bad, err = read_events(events_path)
    rec["n_events"] = len(events)
    rec["malformed_lines"] = bad
    rec["read_error"] = err
    if not events:
        rec["verdict_detail"] = "events.jsonl has no parsable records"
        return rec

    def ts_of(e):
        for k in ("stamp", "sim", "elapsed_s", "time", "t"):
            if isinstance(e.get(k), (int, float)):
                return e[k]
        return None

    first, last = events[0], events[-1]
    rec["started_ts"] = ts_of(first)
    rec["last_ts"] = ts_of(last)
    a, b = rec["started_ts"], rec["last_ts"]
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) and b >= a:
        rec["duration_s"] = round(float(b - a), 2)

    for e in events:
        for k in _REASON_KEYS:
            if k in e and e[k] not in (None, ""):
                rec["reasons"]["%s=%s" % (k, e[k])] += 1
    for e in reversed(events):
        if str(e.get("event", "")).upper() in _SENTINEL_EVENTS or any(
            k in e for k in ("reason", "code")
        ):
            rec["terminal"] = {k: e[k] for k in e if k in ("event",) + _REASON_KEYS}
            break
    return rec


def summarize_coord(run_dir):
    """Summarize ``coord_events.jsonl`` (schema v1) WITHOUT judging PASS/FAIL.

    The acceptance verdict belongs to the Codex core
    (``scripts/coordination_verdict.py``).  This only reports what the
    coordination event stream contains, so a pure-coordination run is no longer
    mis-reported as ``missing evidence`` by the legacy events summary.
    """
    path = os.path.join(run_dir, "coord_events.jsonl")
    rec = {"has_events": os.path.exists(path), "n_events": 0, "malformed_lines": 0,
           "run_started": False, "run_finished": False, "outcome": None,
           "failures": None, "kinds": {}}
    if not rec["has_events"]:
        return rec
    events, bad, _ = read_events(path)
    rec["n_events"], rec["malformed_lines"] = len(events), bad
    kinds = Counter()
    for e in events:
        data = e.get("data") if isinstance(e.get("data"), dict) else {}
        ev = data.get("event")
        if ev:
            kinds[ev] += 1
        if ev == "RUN_STARTED":
            rec["run_started"] = True
        elif ev == "RUN_FINISHED":
            rec["run_finished"] = True
            rec["outcome"] = data.get("outcome")
            rec["failures"] = data.get("failures")
    rec["kinds"] = dict(kinds)
    return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--logs-root", default=DEFAULT_ROOT)
    ap.add_argument("--glob", default="*/", help="run dirs under logs-root (default: all subdirs)")
    ap.add_argument("--out", default=None, help="manifest JSON path (default: <logs-root>/run_manifest.json)")
    ap.add_argument("--verdict-module", default=None,
                    help="optional python file exposing verdict(run)->(bool|None,str)")
    args = ap.parse_args()

    roots = sorted(d for d in glob.glob(os.path.join(args.logs_root, args.glob)) if os.path.isdir(d))
    if not roots:
        print("no run directories under %s" % args.logs_root, file=sys.stderr)
        return 1

    verdict_fn = None
    if args.verdict_module:
        ns = {}
        with open(args.verdict_module) as fh:
            exec(compile(fh.read(), args.verdict_module, "exec"), ns)
        verdict_fn = ns.get("verdict")
        if not callable(verdict_fn):
            print("verdict module has no callable 'verdict'", file=sys.stderr)
            return 2

    records = []
    for d in roots:
        rec = summarize_run(d)
        rec["coord"] = summarize_coord(d)
        if verdict_fn:
            try:
                ok, detail = verdict_fn(rec)
                rec["verdict"] = {True: "PASS", False: "FAIL", None: "ABSTAIN"}[ok] if ok in (True, False, None) else str(ok)
                rec["verdict_detail"] = detail
            except Exception as exc:  # noqa: BLE001
                rec["verdict"] = "VERDICT_ERROR"
                rec["verdict_detail"] = "%s: %s" % (type(exc).__name__, exc)
        records.append(rec)

    out = args.out or os.path.join(args.logs_root, "run_manifest.json")
    with open(out, "w", encoding="utf-8") as fh:
        json.dump({"records": records}, fh, indent=2, ensure_ascii=False)

    # --- human summary -----------------------------------------------------
    print("runs=%d  manifest=%s" % (len(records), out))
    print("%-30s %6s %6s %8s  %-22s %s" % ("run", "events", "coord", "dur_s", "terminal", "verdict"))
    for r in records:
        term = ""
        if r["terminal"]:
            term = " ".join("%s=%s" % (k, v) for k, v in r["terminal"].items())
        print("%-30s %6d %6d %8s  %-22s %s" % (r["name"][:30], r["n_events"], r["coord"]["n_events"],
                                               r["duration_s"] if r["duration_s"] is not None else "-",
                                               term[:22], r["verdict"]))
    missing = sum(1 for r in records if not r["has_events"] and not r["coord"]["has_events"])
    malformed = sum(r["malformed_lines"] + r["coord"]["malformed_lines"] for r in records)
    print("missing_evidence=%d (neither events.jsonl nor coord_events.jsonl) malformed_lines=%d"
          % (missing, malformed))
    coord_out = Counter(r["coord"]["outcome"] for r in records if r["coord"]["outcome"])
    if coord_out:
        print("coord outcomes: " + ", ".join("%s=%d" % (k, v) for k, v in sorted(coord_out.items())))
    if not verdict_fn:
        print("NOTE: verdict column is PENDING_SCHEMA by design -- acceptance PASS/FAIL "
              "waits for the Codex-frozen event schema v1 (AGENTS.md).")
    else:
        printed = Counter(r["verdict"] for r in records)
        print("verdicts: " + ", ".join("%s=%d" % (k, v) for k, v in sorted(printed.items())))
    return 0


if __name__ == "__main__":
    sys.exit(main())
