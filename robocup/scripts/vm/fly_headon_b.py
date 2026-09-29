#!/usr/bin/env python3
"""Head-on intruder drone B for the dual-UAV corridor case.

B is a STATIC gazebo box driven kinematically (no PX4, no MAVROS): from the
navigation stack's perspective an intruder vehicle is exactly a moving
obstacle the depth camera must see and avoid.  B spawns at the goal side of
the empty-map flight line once drone A is cruising, then flies west along
y=0 at INTRUDER_SPEED m/s — head-on into A's route.

Trajectory (sim-stamped, gazebo frame) is logged for post-flight min-distance
analysis against A's telemetry.
"""
import math
import os
import sys
import time

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SpawnModel
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="uav_b">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>0.4 0.4 0.4</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.4 0.4 0.4</size></box></geometry>
        <material><ambient>0.1 0.1 0.9 1</ambient><diffuse>0.1 0.1 0.9 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""

SPAWN_X = 3.5
SPAWN_Z = 2.4
TRIGGER_X = -1.5      # A starts moving east; B launches when A passes this


def main():
    argv = sys.argv[1:]
    start_x = float(argv[0]) if argv else -3.0
    mode = argv[1] if len(argv) > 1 else "headon"
    speed = float(argv[2]) if len(argv) > 2 else 0.30
    log_path = argv[3] if len(argv) > 3 else os.path.expanduser("~/headon_b_traj.log")
    # intruder trajectory in gazebo world frame
    if mode == "headon":
        spawn, vel, stop = (3.5, 0.0), (-speed, 0.0), (-4.5, 0.0)
    elif mode == "cross_north":          # north -> south across the route
        spawn, vel, stop = (2.0, 2.5), (0.0, -speed), (2.0, -4.5)
    elif mode == "cross_south":          # south -> north across the route
        spawn, vel, stop = (2.0, -2.5), (0.0, speed), (2.0, 4.5)
    else:
        raise SystemExit("bad mode " + mode)
    bx, by = spawn
    vx, vy = vel
    sx, sy = stop
    rospy.init_node("headon_intruder_b", disable_signals=True)

    state = {"armed": False, "x": start_x, "z": 0.0, "sim": None}

    def on_state(msg):
        state["armed"] = msg.armed

    def on_odom(msg):
        state["x"] = msg.pose.pose.position.x
        state["z"] = msg.pose.pose.position.z

    def on_clock(msg):
        state["sim"] = msg.clock.to_sec()

    rospy.Subscriber("/mavros/state", State, on_state, queue_size=1)
    rospy.Subscriber("/mavros/global_position/local", Odometry, on_odom,
                     queue_size=1)
    rospy.Subscriber("/clock", Clock, on_clock, queue_size=1)

    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=60)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=60)
    spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    # wait for A to cruise, then until it has advanced to the trigger line
    deadline = time.time() + 240
    while time.time() < deadline and not rospy.is_shutdown():
        if state["armed"] and state["z"] >= 2.0 and state["sim"] is not None:
            break
        time.sleep(0.5)
    else:
        print("HEADON_B_ERROR=NEVER_CRUISE")
        return
    while time.time() < deadline and not rospy.is_shutdown():
        if state["x"] >= TRIGGER_X:
            break
        time.sleep(0.2)
    else:
        print("HEADON_B_ERROR=A_NEVER_REACHED_TRIGGER")
        return

    # spawn B, static box, then drive it along its trajectory kinematically
    from geometry_msgs.msg import Pose, Point, Quaternion
    pose = Pose(position=Point(bx, by, SPAWN_Z),
                orientation=Quaternion(0, 0, 0, 1))
    resp = None
    for attempt in range(3):
        resp = spawn("uav_b", SDF, "", pose, "world")
        if resp.success:
            break
        print("HEADON_B_SPAWN_RETRY attempt=%d: %s" % (attempt + 1,
                                                       resp.status_message),
              flush=True)
        time.sleep(3)
    if resp is None or not resp.success:
        print("HEADON_B_ERROR=SPAWN_FAILED:" +
              (resp.status_message if resp else "no response"))
        return
    print("HEADON_B_SPAWNED sim=%.2f a_x=%.2f mode=%s" % (state["sim"], state["x"], mode),
          flush=True)
    log = open(log_path, "a", buffering=1)
    log.write("# run start sim=%.2f mode=%s speed=%.2f\n"
              % (state["sim"], mode, speed))

    rate = rospy.Rate(20)
    last_sim = state["sim"]
    while not rospy.is_shutdown():
        # stop once every MOVING axis has crossed its end coordinate in the
        # direction of travel; a stationary axis never contributes
        reached_x = True if vx == 0 else (bx <= sx if vx < 0 else bx >= sx)
        reached_y = True if vy == 0 else (by <= sy if vy < 0 else by >= sy)
        if reached_x and reached_y:
            break
        if state["sim"] is not None and state["sim"] > last_sim:
            dt = min(state["sim"] - last_sim, 0.2)  # wall/sim hiccup guard
            bx += vx * dt
            by += vy * dt
            last_sim = state["sim"]
        msg = ModelState()
        msg.model_name = "uav_b"
        msg.pose.position.x = bx
        msg.pose.position.y = by
        msg.pose.position.z = SPAWN_Z
        msg.pose.orientation.w = 1.0
        try:
            set_state(msg)
        except rospy.ServiceException:
            pass
        log.write("%.2f %.3f %.3f %.3f\n"
                  % (state["sim"], bx, by, SPAWN_Z))
        rate.sleep()
    log.close()
    print("HEADON_B_DONE pos=(%.2f,%.2f) sim=%.2f" % (bx, by, state["sim"]),
          flush=True)


if __name__ == "__main__":
    main()
