#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单机避障导航：A* 规划（障碍物膨胀 + Catmull-Rom 平滑），offboard 限速跟踪。

三处核心修复（按优先级）：
1. 障碍物膨胀：A* 前把障碍物向外膨胀 = 无人机半径 + 安全裕度，
   否则 A* 贴墙规划、模型有体积直接撞墙。
2. 路径平滑：Catmull-Rom 把折线拐点变成连续可跟踪轨迹，避免跟踪抖动撞墙。
3. 限速连续跟踪：setpoint 沿路径以恒定速度推进、跨航点不停顿，
   不再把目标点直接怼给 PX4（否则全速冲过头、切角撞墙）。

地图几何（single_wall_s42）：
- 中间竖直墙 unit_wall_0000：x∈[0.023,0.423]、y∈[-4.55,-0.416]，高 4m
- 演示绕墙：起点/终点分居墙两侧且落在墙的 y 高度带内
    _start_x:=-3 _start_y:=-2.5 _goal_x:=3 _goal_y:=-2.5
"""
import math
import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import State, ParamValue
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
from robocup_navigation.astar import load_metadata, GridMap, plan


def inflate_grid(grid, inflation_m):
    """对障碍物向外膨胀 inflation_m，返回新 GridMap（A* 不再贴墙规划）。

    inflation_m = 无人机半径(含旋翼) + 安全裕度。膨胀值不能太大，
    否则狭窄通道被判不可通行。
    """
    r = int(math.ceil(inflation_m / grid.resolution))
    if r <= 0:
        return grid
    w, h = grid.width, grid.height
    occ = [(ix, iy) for iy in range(h) for ix in range(w)
           if not grid.is_free((ix, iy))]
    cells = bytearray(grid.cells)
    r2 = r * r
    for cx, cy in occ:
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r2:
                    continue
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < w and 0 <= ny < h:
                    cells[ny * w + nx] = 1
    return GridMap(w, h, grid.resolution, grid.origin, bytes(cells), grid.frame_id)


def smooth_path(points, samples_per_segment=6):
    """Catmull-Rom 样条平滑：折线拐点 -> 连续可跟踪轨迹。"""
    pts = list(points)
    if len(pts) < 3:
        return pts
    pts = [pts[0]] + pts + [pts[-1]]
    out = []
    for i in range(len(pts) - 3):
        p0, p1, p2, p3 = pts[i], pts[i + 1], pts[i + 2], pts[i + 3]
        for j in range(samples_per_segment):
            t = j / float(samples_per_segment)
            t2, t3 = t * t, t * t * t
            x = 0.5 * ((2 * p1[0]) + (-p0[0] + p2[0]) * t +
                       (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2 +
                       (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3)
            y = 0.5 * ((2 * p1[1]) + (-p0[1] + p2[1]) * t +
                       (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2 +
                       (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3)
            out.append((x, y))
    out.append(pts[-2])
    return out


class SingleUavAvoidance:
    def __init__(self):
        self.metadata = rospy.get_param("~metadata")
        self.altitude = rospy.get_param("~altitude_m", 2.4)
        self.tol = rospy.get_param("~arrival_tolerance_m", 0.3)
        self.inflation_m = rospy.get_param("~inflation_m", 0.5)
        self.max_speed = rospy.get_param("~max_speed_m_s", 0.4)

        # ===== 载入地图 =====
        md, _ = load_metadata(self.metadata)
        self.grid = GridMap.from_metadata(md)
        goal_cfg = md["goal_candidates"][0]["center"]

        # ===== A* 起点/终点（世界坐标=map 坐标）=====
        self.start = (rospy.get_param("~start_x", None),
                      rospy.get_param("~start_y", None))
        self.goal = (rospy.get_param("~goal_x", goal_cfg[0]),
                     rospy.get_param("~goal_y", goal_cfg[1]))

        # ===== MAVROS 控制接口 =====
        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        rospy.Subscriber("/mavros/state", State, self._state_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._pose_cb)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._models_cb)
        self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.param_srv = rospy.ServiceProxy("/mavros/param/set", ParamSet)
        self.state = State()
        self.local = None
        self.model_states = None
        self.offset = None
        self.rate = rospy.Rate(20)

    def _state_cb(self, msg):
        self.state = msg

    def _pose_cb(self, msg):
        p = msg.pose.position
        self.local = (p.x, p.y, p.z)

    def _models_cb(self, msg):
        self.model_states = msg

    def _iris_world(self):
        if self.model_states is None:
            return None
        for i, name in enumerate(self.model_states.name):
            if "iris" in name:
                p = self.model_states.pose[i].position
                return (p.x, p.y, p.z)
        return None

    def _send(self, x, y, z):
        sp = PoseStamped()
        sp.pose.position.x = x
        sp.pose.position.y = y
        sp.pose.position.z = z
        self.setpoint_pub.publish(sp)

    def _follow_path(self, path):
        """限速连续跟踪：虚拟目标点沿路径以 max_speed 推进，跨航点不停顿。

        path 为世界坐标，先转 local 坐标。虚拟点从当前位置出发，每周期
        前进 step=max_speed*dt，可一次跨多个航点，最终悬停到终点收敛。
        """
        lp = [(x - self.offset[0], y - self.offset[1]) for x, y in path]
        dt = 1.0 / 20.0
        step = self.max_speed * dt
        vx, vy = self.local[0], self.local[1]
        i = 0
        n = len(lp)
        while not rospy.is_shutdown() and i < n:
            tx, ty = lp[i]
            dx, dy = tx - vx, ty - vy
            d = math.hypot(dx, dy)
            while d <= step and i < n - 1:
                vx, vy = tx, ty
                i += 1
                tx, ty = lp[i]
                dx, dy = tx - vx, ty - vy
                d = math.hypot(dx, dy)
            if d <= step:
                vx, vy = tx, ty
                i += 1
            else:
                vx += dx / d * step
                vy += dy / d * step
            self._send(vx, vy, self.altitude)
            self.rate.sleep()
        # 到达终点：悬停直到实际位置收敛
        gx, gy = lp[-1]
        while not rospy.is_shutdown():
            if math.hypot(gx - self.local[0], gy - self.local[1]) < self.tol:
                break
            self._send(gx, gy, self.altitude)
            self.rate.sleep()

    def _set_param(self, param_id, integer_value):
        try:
            resp = self.param_srv(param_id, ParamValue(integer=integer_value, real=0.0))
            if resp.success:
                rospy.loginfo("参数 %s=%d 已生效", param_id, integer_value)
                return True
            rospy.logwarn("设置 %s=%d 失败", param_id, integer_value)
            return False
        except Exception as exc:
            rospy.logwarn("设置 %s 异常: %s", param_id, exc)
            return False

    def _configure_fcu_params(self):
        """SITL 无遥控器会触发 RC 失联 failsafe，导致拒绝解锁/进入 OFFBOARD。"""
        self._set_param("NAV_RCL_ACT", 0)
        self._set_param("COM_RCL_EXCEPT", 4)
        # 注意：绝不能设 SYS_HAS_MAG=0！那会让 EKF2 拿不到磁力计数据，
        # 静止时 GPS 定不了航向，EKF2 不输出位置，local_position 断流。
        # 磁力计必须保持 SYS_HAS_MAG=1，让 EKF2 正常工作。

    def run(self):
        while not rospy.is_shutdown() and not self.state.connected:
            self.rate.sleep()
        rospy.loginfo("MAVROS 已连接")
        while self.local is None and not rospy.is_shutdown():
            self.rate.sleep()
        while self._iris_world() is None and not rospy.is_shutdown():
            self.rate.sleep()

        # ===== 坐标标定 =====
        iris = self._iris_world()
        self.offset = (iris[0] - self.local[0], iris[1] - self.local[1])
        spawn = (iris[0], iris[1])
        rospy.loginfo("实际出生点(map)=%.2f,%.2f | local=%.2f,%.2f | offset=%.2f,%.2f",
                      spawn[0], spawn[1], self.local[0], self.local[1],
                      self.offset[0], self.offset[1])

        # ===== A* 起点 =====
        sx = self.start[0] if self.start[0] is not None else spawn[0]
        sy = self.start[1] if self.start[1] is not None else spawn[1]
        self.start = (sx, sy)

        # ===== 障碍物膨胀 + A* 规划 =====
        inflated = inflate_grid(self.grid, self.inflation_m)
        route = plan(inflated, self.start, self.goal, connectivity=8)
        if not route.success:
            rospy.logerr("A* 规划失败: %s", route.reason)
            raise SystemExit(1)
        raw = list(route.points)
        rospy.loginfo("A* 成功: %d 个航点, 长度 %.2fm（膨胀 %.2fm）",
                      len(raw), route.path_length_m, self.inflation_m)

        # ===== 路径平滑 =====
        self.path = smooth_path(raw, samples_per_segment=6)
        rospy.loginfo("平滑后 %d 个点", len(self.path))

        # ===== 配置 FCU 参数 =====
        self._configure_fcu_params()

        # ===== 预热 setpoint（原地）=====
        for _ in range(60):
            self._send(self.local[0], self.local[1], self.altitude)
            self.rate.sleep()

        # ===== 切 OFFBOARD =====
        if not self.mode_srv(0, "OFFBOARD").mode_sent:
            rospy.logerr("切换 OFFBOARD 失败，退出")
            raise SystemExit(1)
        for _ in range(50):
            if self.state.mode == "OFFBOARD":
                break
            self._send(self.local[0], self.local[1], self.altitude)
            self.rate.sleep()
        if self.state.mode != "OFFBOARD":
            rospy.logerr("OFFBOARD 未生效，当前模式: %s", self.state.mode)
            raise SystemExit(1)
        rospy.loginfo("已进入 OFFBOARD")

        # ===== 解锁 =====
        if not self.arm_srv(True).success:
            rospy.logerr("解锁失败，退出")
            raise SystemExit(1)
        for _ in range(50):
            if self.state.armed:
                break
            self.rate.sleep()
        if not self.state.armed:
            rospy.logerr("解锁未生效，退出")
            raise SystemExit(1)
        rospy.loginfo("已解锁")

        # ===== 起飞（原地爬升）=====
        for _ in range(120):
            self._send(self.local[0], self.local[1], self.altitude)
            self.rate.sleep()
        rospy.loginfo("已起飞到 %.1fm", self.altitude)

        # ===== 沿平滑路径限速飞行（避障核心）=====
        self._follow_path(self.path)

        # ===== 终点悬停 + 降落 =====
        glx = self.goal[0] - self.offset[0]
        gly = self.goal[1] - self.offset[1]
        for _ in range(60):
            self._send(glx, gly, self.altitude)
            self.rate.sleep()
        rospy.loginfo("到达终点 %.2f,%.2f，AUTO.LAND 降落", self.goal[0], self.goal[1])
        self.mode_srv(0, "AUTO.LAND")


if __name__ == "__main__":
    rospy.init_node("single_uav_avoidance")
    SingleUavAvoidance().run()
