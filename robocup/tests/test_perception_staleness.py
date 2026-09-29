#!/usr/bin/env python3
"""Perception watchdog age helper (route_runtime.perception_depth_age_s).

Covers the four observation regimes the executor can be in:
  - live stream, stamps tracking sim time      -> ~0 age
  - live-but-delayed stream                    -> age == end-to-end latency
  - dead stream after a healthy start          -> age grows without bound
  - camera missing from the very launch        -> age counts from MOVE begin
  - pre-MOVE with no frames                    -> None (never stale)
plus the paused-Gazebo invariant: sim now and the last stamp freeze
together, so the age must not grow while physics is paused.
"""
import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/robocup_navigation/src"))

from robocup_navigation.route_runtime import perception_depth_age_s  # noqa: E402


def approx(a, b, tol=1e-9):
    return abs(a - b) <= tol


def test_live_stream_age_is_zero():
    assert approx(perception_depth_age_s(100.0, 100.0, 90.0), 0.0)
    assert approx(perception_depth_age_s(100.0, 100.05, 90.0), 0.05)


def test_delayed_stream_age_is_latency():
    # frames keep arriving but carry stamps 1.5 s behind sim now
    assert approx(perception_depth_age_s(98.5, 100.0, 90.0), 1.5)


def test_dead_stream_age_grows():
    assert approx(perception_depth_age_s(90.0, 100.0, 85.0), 10.0)
    assert approx(perception_depth_age_s(90.0, 200.0, 85.0), 110.0)


def test_never_fused_counts_from_move_begin():
    assert approx(perception_depth_age_s(None, 100.0, 95.0), 5.0)
    # MOVE not started yet (e.g. still PRESTREAM): clock not armed
    assert perception_depth_age_s(None, 100.0, None) is None


def test_pre_move_never_stale():
    assert perception_depth_age_s(None, 100.0, None) is None


def test_paused_gazebo_freezes_age():
    # pause at sim 100.0 with the last frame stamped 99.7; sim stays frozen
    for sim in (100.0, 100.0, 100.0):
        assert approx(perception_depth_age_s(99.7, sim, 95.0), 0.3)


def test_negative_age_is_passthrough():
    # a stamp slightly ahead of sim now (publisher jitter) stays negative;
    # the caller compares against a positive max_age so it reads as fresh
    assert approx(perception_depth_age_s(100.01, 100.0, 95.0), -0.01)


if __name__ == "__main__":
    test_live_stream_age_is_zero()
    test_delayed_stream_age_is_latency()
    test_dead_stream_age_grows()
    test_never_fused_counts_from_move_begin()
    test_pre_move_never_stale()
    test_paused_gazebo_freezes_age()
    test_negative_age_is_passthrough()
    print("test_perception_staleness: ALL PASS")
