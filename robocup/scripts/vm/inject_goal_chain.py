#!/usr/bin/env python3
"""Boundary-case harness: CHAINED goal preemption mid-flight.

Injects a chain of NEW goals while the iris is chasing the previous one.
Each next goal is injected once the drone is within APPROACH_M of the current
goal (so every leg is genuinely flown and preempted before arrival).
Expected event chain, repeated per leg:
  REPLAN_STARTED -> GOAL_PREEMPTED -> GOAL_ACCEPTED
and after the FINAL goal: arrival -> MISSION_COMPLETE.

Frame note: goals are published in gazebo_world (the nav node transforms
them), but /mavros/global_position/local odom has its origin at the SPAWN,
NOT at the gazebo world origin.  The proximity check therefore converts each
gazebo goal into odom coordinates:  odom_goal = goal_gazebo + (odom_0 - spawn),
where spawn comes from the scenario yaml (route_start_world_xy) and odom_0 is
the first odom fix captured on the ground.

All goals are inside max_radius_m=7.0 and free space on the empty map.
"""
import math
import os
import time

import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

# leg 1: original goal (3,0) -> here; leg 2 -> here; leg 3 = final target
GOAL_CHAIN = [(-1.0, 2.5), (2.0, -2.5), (-2.0, -2.0)]
APPROACH_M = 2.0
LEG_TIMEOUT_S = 120.0
CRUISE_ADVANCE_S = 6.0
_WS = os.environ.get("ROBOCUP_WORKSPACE", os.path.expanduser("~/robocup/robocup_ws"))
SCENARIO_YAML = os.path.join(_WS, "build/training_city/scenario_empty.yaml")


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def load_spawn_xy():
    """Poll the scenario yaml for route_start_world_xy (gazebo frame)."""
    deadline = time.time() + 90
    while time.time() < deadline and not rospy.is_shutdown():
        try:
            with open(SCENARIO_YAML) as f:
                cfg = yaml.safe_load(f)
            xy = cfg.get("route_start_world_xy")
            if xy and len(xy) == 2:
                return (float(xy[0]), float(xy[1]))
        except (OSError, yaml.YAMLError, ValueError, TypeError):
            pass
        time.sleep(1.0)
    return None


def main():
    rospy.init_node("goal_preempt_chain_injector", anonymous=True,
                    disable_signals=True)
    state = {"armed": False, "xy": (0.0, 0.0), "z": 0.0, "sim": None,
             "odom0": None}

    def on_state(msg):
        state["armed"] = msg.armed

    def on_odom(msg):
        p = msg.pose.pose.position
        state["xy"] = (p.x, p.y)
        state["z"] = p.z
        if state["odom0"] is None:
            state["odom0"] = (p.x, p.y)  # ground fix at the spawn

    def on_clock(msg):
        # /clock is authoritative under use_sim_time; rospy.Time.now() is
        # NOT — this injector may init before roslaunch sets the parameter.
        state["sim"] = msg.clock.to_sec()

    rospy.Subscriber("/mavros/state", State, on_state, queue_size=1)
    rospy.Subscriber("/mavros/global_position/local", Odometry, on_odom,
                     queue_size=1)
    rospy.Subscriber("/clock", Clock, on_clock, queue_size=1)

    spawn = load_spawn_xy()
    if spawn is None:
        print("GOAL_CHAIN_ERROR=NO_SPAWN_IN_YAML")
        return

    deadline = time.time() + 240
    while time.time() < deadline and not rospy.is_shutdown():
        if (state["armed"] and state["z"] >= 2.0
                and state["sim"] is not None and state["odom0"] is not None):
            break
        time.sleep(0.5)
    else:
        print("GOAL_CHAIN_ERROR=NEVER_CRUISE")
        return

    # gazebo -> odom offset captured while the drone sat on the ground
    offset = (state["odom0"][0] - spawn[0], state["odom0"][1] - spawn[1])
    print("GOAL_CHAIN_OFFSET odom0=(%.2f,%.2f) spawn=(%.2f,%.2f) "
          "offset=(%.2f,%.2f)"
          % (state["odom0"][0], state["odom0"][1], spawn[0], spawn[1],
             offset[0], offset[1]), flush=True)

    # Cruise reached; let the drone advance on the ORIGINAL goal first.
    time.sleep(CRUISE_ADVANCE_S)
    pub = rospy.Publisher("/uav_navigation/goal", PoseStamped, queue_size=1)
    time.sleep(0.5)

    seq = 0
    for i, goal in enumerate(GOAL_CHAIN):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time(secs=int(state["sim"]),
                                      nsecs=int((state["sim"] % 1.0) * 1e9))
        msg.header.frame_id = "gazebo_world"
        msg.header.seq = seq + 1
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.position.z = 2.4
        msg.pose.orientation.w = 1.0
        pub.publish(msg)
        seq += 1
        print("GOAL_CHAIN_INJECTED seq=%d x=%.1f y=%.1f sim_s=%.2f"
              % (seq, goal[0], goal[1], state["sim"]), flush=True)

        if i == len(GOAL_CHAIN) - 1:
            break  # final goal: let the mission complete on it

        # wait until the drone closes in on THIS goal, then preempt again
        goal_odom = (goal[0] + offset[0], goal[1] + offset[1])
        leg_deadline = time.time() + LEG_TIMEOUT_S
        preempted = False
        while time.time() < leg_deadline and not rospy.is_shutdown():
            if dist(state["xy"], goal_odom) < APPROACH_M:
                time.sleep(0.5)  # let it commit a bit closer
                preempted = True
                break
            time.sleep(0.2)
        if not preempted:
            print("GOAL_CHAIN_ERROR=LEG_TIMEOUT leg=%d goal=%.1f,%.1f "
                  "pos=%.2f,%.2f" % (i + 1, goal[0], goal[1],
                                     state["xy"][0], state["xy"][1]),
                  flush=True)
            return

    print("GOAL_CHAIN_INJECTED_ALL seq=%d" % seq, flush=True)


if __name__ == "__main__":
    main()
