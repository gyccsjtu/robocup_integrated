#!/usr/bin/env python3
"""Clock-pause watcher: waits for cruise, pauses Gazebo physics (freezing
/clock), holds it PAUSE_S of wall time, then unpauses.  Expected node
behaviour: CLOCK_STALLED within clock_timeout_s of the pause -> fail-safe
landing; after unpause the landing completes and RESULT=CLOCK_STALLED."""
import subprocess
import sys
import time

import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

PAUSE_S = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0

rospy.init_node("clock_pause_watcher", disable_signals=True)
state = {"armed": False, "z": 0.0, "sim": None}


def on_state(msg):
    state["armed"] = msg.armed


def on_odom(msg):
    state["z"] = msg.pose.pose.position.z


def on_clock(msg):
    state["sim"] = msg.clock.to_sec()


rospy.Subscriber("/mavros/state", State, on_state, queue_size=1)
rospy.Subscriber("/mavros/global_position/local", Odometry, on_odom, queue_size=1)
rospy.Subscriber("/clock", Clock, on_clock, queue_size=1)

deadline = time.time() + 240
while time.time() < deadline and not rospy.is_shutdown():
    if state["armed"] and state["z"] >= 2.0 and state["sim"] is not None:
        break
    time.sleep(0.5)
else:
    print("CLOCK_PAUSE_ERROR=NEVER_CRUISE")
    sys.exit(1)

time.sleep(5.0)
print("PAUSE_AT sim=%.2f" % state["sim"], flush=True)
subprocess.call(["rosservice", "call", "/gazebo/pause_physics"])
t0 = time.monotonic()
time.sleep(PAUSE_S)
print("UNPAUSE_AT held=%.1fs" % (time.monotonic() - t0), flush=True)
subprocess.call(["rosservice", "call", "/gazebo/unpause_physics"])
print("CLOCK_PAUSE_DONE")
