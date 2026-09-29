#!/usr/bin/env python3
"""DEV-ONLY perception substitute: publishes target positions as a topic.

Why this exists
---------------
The vision/target-detection pipeline belongs to a teammate and is not part of
this work package. To validate the six-UAV *coordination* algorithm on its own,
perception is stubbed out: this node broadcasts where the targets are, exactly
the way a detector eventually will, on the same interface.

Rules (do not break these)
--------------------------
* This is a **development harness** (interface v0 section 5). It reads no
  Gazebo ground truth and is not part of the competition pipeline. On the real
  system the same topic is filled by the detector; the coordination layer does
  not change.
* It never reports a position it was not configured with, and it stamps every
  report so stale data can be rejected downstream.
* Motion is configuration, not invention: targets only move along the paths
  given in `~targets`.

Usage:
    rosrun robocup_navigation coordination_targetsim.py \
        _targets:="[{target_id: target_1, xyz: [8.0, -6.0, 0.0]}]"
"""
import json
import math
import time

import rospy
from std_msgs.msg import String


def _position(spec, t):
    """Position of a target at time t, per its configured motion."""
    xyz = [float(v) for v in spec["xyz"]]
    motion = spec.get("motion") or {}
    kind = motion.get("kind", "static")
    if kind == "linear":
        span = float(motion.get("span_m", 4.0))
        period = float(motion.get("period_s", 20.0))
        axis = int(motion.get("axis", 1))
        phase = 2.0 * math.pi * (t % period) / period
        xyz[axis] += span * 0.5 * math.sin(phase)
    return xyz


class TargetSim(object):
    def __init__(self):
        self.targets = rospy.get_param("~targets", [])
        self.hz = float(rospy.get_param("~rate_hz", 2.0))
        self.frame = rospy.get_param("~frame_id", "world_enu")
        self.conf = float(rospy.get_param("~confidence", 1.0))
        self.seq = 0
        self.pub = rospy.Publisher("/coordination/target_report", String, queue_size=20)
        if not self.targets:
            rospy.logwarn("[targetsim] ~targets is empty; publishing nothing")

    def spin(self):
        rate = rospy.Rate(self.hz)
        t0 = time.monotonic()
        while not rospy.is_shutdown():
            now = time.monotonic()
            for spec in self.targets:
                self.seq += 1
                # Topic payload is the bare data object: the coordinator node
                # hands the whole parsed payload to the core as `data`.
                payload = dict(target_id=spec["target_id"], frame_id=self.frame,
                               xyz=_position(spec, now - t0),
                               confidence=self.conf,
                               observation_id="obs-%s-%d" % (spec["target_id"], self.seq))
                self.pub.publish(String(json.dumps(payload, sort_keys=True)))
            rate.sleep()


def main():
    rospy.init_node("coordination_targetsim")
    TargetSim().spin()


if __name__ == "__main__":
    main()
