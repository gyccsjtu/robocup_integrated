#!/usr/bin/env python3
"""Boundary-case harness: goal preemption mid-flight.

Waits until the iris is cruising on its planned route, then publishes a NEW
goal (gazebo_world frame, fresh sim stamp).  Expected event chain:
REPLAN_STARTED -> GOAL_ACCEPTED -> arrival at the NEW goal -> MISSION_COMPLETE.
"""
import math
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

NEW_GOAL = (2.5, 2.0)


def main():
    rospy.init_node("goal_preempt_injector", anonymous=True, disable_signals=True)
    state = {"armed": False, "xy": (0.0, 0.0), "z": 0.0, "sim": None}

    def on_state(msg):
        state["armed"] = msg.armed

    def on_odom(msg):
        p = msg.pose.pose.position
        state["xy"] = (p.x, p.y)
        state["z"] = p.z

    def on_clock(msg):
        # /clock is authoritative under use_sim_time; rospy.Time.now() is
        # NOT — this injector may init before roslaunch sets the parameter.
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
        print("GOAL_PREEMPT_ERROR=NEVER_CRUISE")
        return

    # Cruise reached; let the drone advance along the route before preempting.
    time.sleep(6.0)
    msg = PoseStamped()
    msg.header.stamp = rospy.Time(secs=int(state["sim"]), nsecs=int((state["sim"] % 1.0) * 1e9))
    msg.header.frame_id = "gazebo_world"
    msg.header.seq = 1
    msg.pose.position.x = NEW_GOAL[0]
    msg.pose.position.y = NEW_GOAL[1]
    msg.pose.position.z = 2.4
    msg.pose.orientation.w = 1.0
    pub = rospy.Publisher("/uav_navigation/goal", PoseStamped, queue_size=1)
    time.sleep(0.5)
    pub.publish(msg)
    print("GOAL_PREEMPT_INJECTED x=%.1f y=%.1f sim_s=%.2f" % (NEW_GOAL[0], NEW_GOAL[1], state["sim"]))


if __name__ == "__main__":
    main()
