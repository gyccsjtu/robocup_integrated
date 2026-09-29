#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
swarm_offboard_server  ——  三机统一 Offboard 控制底座
====================================================

为什么要这一层
--------------
1) PX4 在 OFFBOARD 模式下必须持续收到 setpoint（>2Hz）。一旦停止发送，
   会判定「OFFBOARD 丢失」触发失效保护并转 AUTO.RTL，飞机当场掉下去。
   所以必须有一个**常驻进程**负责心跳，而不能由算法脚本兼职。

2) 三架 PX4 SITL 各自独立运行，每架都把**自己的出生点**当作本地坐标系原点，
   因此 mavros 的 local_position 全都接近 (0,0)。直接拿来算协同会全盘错位。
   本节点在启动时读取 Gazebo 真值，记下各机出生点，自动完成
   「世界/地图系 -> 该机本地系」的平移换算。

3) SITL 没有遥控器，PX4 会因「RC 丢失」持续触发失效保护导致无法解锁。
   本节点自动放宽相关参数，并在设置后回读校验。

上层想写协同算法，只需要
------------------------
    # 发布目标（世界/地图系，单位米，ENU）
    rostopic pub /swarm/cmd/uav_1 geometry_msgs/PoseStamped "
    header: {frame_id: world}
    pose: {position: {x: -10.0, y: 5.0, z: 3.0}}"

读取统一状态：/swarm/status (robocup_swarm/UavStatus 数组语义，此处用话题集合)

用法:  python3 swarm_offboard_server.py            # 起飞到默认 3 米并悬停
       python3 swarm_offboard_server.py 4.0        # 指定悬停高度
"""

import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, ParamSet, ParamGet

UAVS = ["uav_1", "uav_2", "uav_3", "uav_4", "uav_5", "uav_6"]
GAZEBO_MODELS = ["iris_1", "iris_2", "iris_3", "iris_4", "iris_5", "iris_6"]   # Gazebo 里的模型名

# 三机的出生点（世界/地图系 x, y）。
# 必须显式配置，不能靠运行时推断：控制底座若中途重启，此时飞机往往已经
# 飞离出生点，用「当前位置」当原点会让整套坐标换算全盘错位。
# 下面这组是实测得到的 spawn 位置，可用 ROS 参数 ~origin/<model> 覆盖。
DEFAULT_ORIGINS = {
    "iris_1": (-4.75, 7.72),
    "iris_2": ( 0.00, 7.97),
    "iris_3": ( 4.98, 7.99),
}
# 只有当地面漂移量小于此值才接受「自动探测」到的出生点
SPAWN_Z_TOL = 0.5

# SITL 无遥控器 / 无失控保护时必须放宽的参数，否则无法解锁或会掉机。
# 每一项都会设置后回读校验 —— 直接设而回读出现过「静默失败」。
FAILSAFE_PARAMS = [
    ("NAV_RCL_ACT", 0),      # 0=Disabled，RC 丢失不再自动 RTL（SITL 无遥控器）
    ("COM_RCL_EXCEPT", 4),   # 4=允许无 RC 时切换 OFFBOARD 等模式
    ("COM_OBL_ACT", 1),      # 1=Hold，OFFBOARD 丢失时悬停而不是掉下去
]

HEARTBEAT_HZ = 20
HOLD_ALT = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0


class Agent(object):
    """单架机的接管与坐标换算"""

    def __init__(self, ns, model_name):
        self.ns = ns
        self.model_name = model_name
        self.state = State()
        self.world_pose = None          # 当前世界坐标 (x, y, z)
        self.origin = None              # 出生点世界坐标，换算基准
        self.target_world = None        # 上层下发的目标（世界系）
        self.has_cmd = False

        rospy.Subscriber("/%s/mavros/state" % ns, State, self._state_cb)
        rospy.Subscriber("/swarm/cmd/%s" % ns, PoseStamped, self._cmd_cb)
        self.pub = rospy.Publisher(
            "/%s/mavros/setpoint_position/local" % ns, PoseStamped, queue_size=10)

        rospy.wait_for_service("/%s/mavros/cmd/arming" % ns, timeout=30)
        rospy.wait_for_service("/%s/mavros/set_mode" % ns, timeout=30)
        rospy.wait_for_service("/%s/mavros/param/set" % ns, timeout=30)
        rospy.wait_for_service("/%s/mavros/param/get" % ns, timeout=30)
        self.arm_srv = rospy.ServiceProxy("/%s/mavros/cmd/arming" % ns, CommandBool)
        self.mode_srv = rospy.ServiceProxy("/%s/mavros/set_mode" % ns, SetMode)
        self.set_param = rospy.ServiceProxy("/%s/mavros/param/set" % ns, ParamSet)
        self.get_param = rospy.ServiceProxy("/%s/mavros/param/get" % ns, ParamGet)

    def _state_cb(self, msg):
        self.state = msg

    def _cmd_cb(self, msg):
        p = msg.pose.position
        self.target_world = (p.x, p.y, p.z)
        self.has_cmd = True
        rospy.logdebug("[%s] 收到目标 world=(%.1f, %.1f, %.1f)" % (self.ns, p.x, p.y, p.z))

    # ---------- 参数：设置后必须回读校验，否则可能静默失败 ----------
    def relax_failsafe(self, retries=3):
        for attempt in range(retries):
            all_ok = True
            for pid, want in FAILSAFE_PARAMS:
                try:
                    self.set_param(param_id=pid, value={"integer": want, "real": 0.0})
                    got = self.get_param(param_id=pid).value.integer
                except Exception as e:
                    rospy.logwarn("[%s] %s 设置异常: %s" % (self.ns, pid, e))
                    got = None
                if got != want:
                    all_ok = False
                    rospy.logwarn("[%s] %s 期望 %d 实际 %s，重试 %d/%d"
                                  % (self.ns, pid, want, got, attempt + 1, retries))
            if all_ok:
                rospy.loginfo("[%s] 失效保护参数已放宽" % self.ns)
                return True
            time.sleep(1.0)
        rospy.logerr("[%s] 失效保护参数放宽失败，该机可能无法解锁" % self.ns)
        return False

    # ---------- 世界系 -> 该机本地系 ----------
    def to_local(self, world_xyz):
        if self.origin is None:
            return None
        wx, wy, wz = world_xyz
        ox, oy, oz = self.origin
        return (wx - ox, wy - oy, wz - oz)

    def local_target(self):
        if self.has_cmd and self.target_world is not None:
            loc = self.to_local(self.target_world)
            if loc is not None:
                return loc
        if self.origin is None:
            return (0.0, 0.0, HOLD_ALT)
        # 默认：在出生点正上方悬停
        return (0.0, 0.0, HOLD_ALT)

    def publish_setpoint(self):
        loc = self.local_target()
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = loc
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)

    def try_takeover(self):
        if not self.state.connected or self.origin is None:
            return
        if self.state.mode != "OFFBOARD":
            try:
                self.mode_srv(custom_mode="OFFBOARD")
            except Exception:
                pass
        elif not self.state.armed:
            try:
                r = self.arm_srv(True)
                if not r.success:
                    rospy.logwarn_throttle(5, "[%s] 解锁被拒，可能仍在自检" % self.ns)
            except Exception:
                pass


def main():
    rospy.init_node("swarm_offboard_server", anonymous=True)
    agents = {ns: Agent(ns, GAZEBO_MODELS[i]) for i, ns in enumerate(UAVS)}

    def models_cb(msg):
        for ns, ag in agents.items():
            if ag.model_name not in msg.name or ag.origin is not None:
                continue
            i = msg.name.index(ag.model_name)
            p = msg.pose[i].position
            ag.world_pose = (p.x, p.y, p.z)
            # 仅当仍在地面（未起飞）时，才相信这是出生点；否则一律用配置值
            if p.z < SPAWN_Z_TOL:
                ag.origin = (p.x, p.y, 0.0)
                rospy.loginfo("[%s] 出生点(实测) world=(%.2f, %.2f)" % (ns, p.x, p.y))
            elif ag.model_name in DEFAULT_ORIGINS:
                ox, oy = DEFAULT_ORIGINS[ag.model_name]
                ag.origin = (ox, oy, 0.0)
                rospy.logwarn("[%s] 已在空中(z=%.2f)，出生点改用配置值 (%.2f, %.2f)"
                              % (ns, p.z, ox, oy))

    rospy.Subscriber("/gazebo/model_states", ModelStates, models_cb)

    # 等待 Gazebo 上齐真值
    deadline = time.time() + 30
    while time.time() < deadline and not rospy.is_shutdown():
        if all(a.origin is not None for a in agents.values()):
            break
        rospy.sleep(0.5)

    # 兜底：仍拿不到就用配置值补齐
    for ns, ag in agents.items():
        if ag.origin is None and ag.model_name in DEFAULT_ORIGINS:
            ox, oy = DEFAULT_ORIGINS[ag.model_name]
            ag.origin = (ox, oy, 0.0)
            rospy.logwarn("[%s] 未收到真值，出生点使用配置值 (%.2f, %.2f)" % (ns, ox, oy))

    missing = [ns for ns, a in agents.items() if a.origin is None]
    if missing:
        rospy.logerr("未能确定出生点: %s —— 该机无法参与坐标换算" % missing)
    else:
        for ns, ag in agents.items():
            rospy.loginfo("[%s] 坐标系原点 = (%.2f, %.2f)" % (ns, ag.origin[0], ag.origin[1]))

    for a in agents.values():
        a.relax_failsafe()

    # ---- 心跳线程：维持 OFFBOARD 的命脉，绝不能停 ----
    # 接管必须「长期持续」：飞机一旦因故掉到地面/脱离 OFFBOARD，
    # 若只在启动后短时间内重试，之后再也拉不起来。
    def heartbeat():
        rate = rospy.Rate(HEARTBEAT_HZ)
        last_takeover = 0.0
        while not rospy.is_shutdown():
            now = time.time()
            do_takeover = (now - last_takeover) >= 2.0
            for ns, ag in agents.items():
                ag.publish_setpoint()
                if do_takeover:
                    ag.try_takeover()
            if do_takeover:
                last_takeover = now
            rate.sleep()

    threading.Thread(target=heartbeat, daemon=True).start()
    rospy.loginfo("心跳已启动 (%d Hz)，默认悬停高度 %.1f m" % (HEARTBEAT_HZ, HOLD_ALT))

    rospy.spin()


if __name__ == "__main__":
    main()
