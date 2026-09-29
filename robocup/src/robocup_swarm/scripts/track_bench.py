#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SITL 追踪能力实测：真实 PX4 能否做到离线实验假设的 6 m/s 追踪。

背景
----
离线实验（scripts/vm/strategy_compare.py）得出：恒速 6 m/s 直追是最优策略，
理想躲藏下也有 94% 达成率。但那个结论建立在「无人机可以 6 m/s 瞬间转向」
的假设上。真实 PX4 有：

  * MPC_ACC_HOR_MAX = 5 m/s²  加速到 6 m/s 需 1.2 s
  * MPC_XY_CRUISE  = 5 m/s    位置控制模式的默认巡航速度
  * MPC_XY_VEL_MAX = 12 m/s   硬上限（6 够用）

本节点把离线验证过的 chase 策略接到真实 PX4 上做闭环，实测：

  1. 实际能达到的水平速度（含转弯时的掉速）
  2. 位置跟踪误差与速度超调（会不会冲过头）
  3. LOS 连续性：真实轨迹下能否累积满 15 s
  4. 与离线模型的偏差（实际速度 vs 期望速度）

用法：
    ROS_NAMESPACE=uav_1 rosrun robocup_swarm track_bench.py \
        _uav_id:=uav_1 _model_name:=iris1 _duration:=60
"""

import json
import math
import os
import sys

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import CommandBool, ParamSet, SetMode

WS = os.environ.get("ROBOCUP_WS", os.path.expanduser("~/team_ws/robocup"))
sys.path.insert(0, os.path.join(WS, "scripts", "vm"))

try:
    import strategy_compare as SC
except ImportError:
    SC = None

# ---- 测试参数 ----
ALTITUDE = 6.0            # 飞行高度（规则限高 6 m）
SPEED_MAX = 6.0           # 目标速度（规则上限）
SPEED_CMD = 6.0           # 本测试下发的速度
CTRL_HZ = 20.0
SAMPLE_HZ = 10.0
CONFIRM_S = 15.0


class TrackBench(object):
    def __init__(self, uav_id, model_name):
        self.uav_id = uav_id
        self.model_name = model_name
        self.duration = float(rospy.get_param("~duration", 60.0))
        self.policy = rospy.get_param("~speed_policy", "const")   # const | adaptive

        # 状态
        self.state = State()
        self.local_xy = None
        self.local_z = None
        self.world_xy = None
        self.offset = None
        self.prev_xy = None
        self.prev_t = None

        # 记录
        self.samples = []      # (t, cmd_speed, act_speed, err_x, err_y, los)
        self.streak = 0.0
        self.max_streak = 0.0
        self.los_breaks = 0
        self.seen_last = False

        # 目标（虚拟，按躲藏模型运动）
        self.tgt = None
        self.tgt_vel = (0.0, 0.0)
        self.escape_dir = (0.0, 0.0)
        self.blocked = 0
        self.g = SC.MapGrid(SC.METADATA) if SC else None
        self.hide = SC.HIDE_IDEAL if SC else 0

        # ROS 接口
        self.arm_srv = rospy.ServiceProxy("/%s/mavros/cmd/arming" % uav_id, CommandBool)
        self.mode_srv = rospy.ServiceProxy("/%s/mavros/set_mode" % uav_id, SetMode)
        self.param_srv = rospy.ServiceProxy("/%s/mavros/param/set" % uav_id, ParamSet)
        rospy.Subscriber("/%s/mavros/state" % uav_id, State, self._on_state)
        rospy.Subscriber("/%s/mavros/local_position/pose" % uav_id, PoseStamped, self._on_pose)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_models)
        self.vel_pub = rospy.Publisher("/%s/mavros/setpoint_velocity/cmd_vel" % uav_id,
                                       TwistStamped, queue_size=10)
        self.rate = rospy.Rate(CTRL_HZ)

    # -------- 回调 --------
    def _on_state(self, m):
        self.state = m

    def _on_pose(self, m):
        self.local_z = m.pose.position.z
        self.local_xy = (m.pose.position.x, m.pose.position.y)

    def _on_models(self, m):
        try:
            i = m.name.index(self.model_name)
        except ValueError:
            return
        p = m.pose[i].position
        self.world_xy = (p.x, p.y)

    # -------- 起飞 --------
    def _send(self, vx, vy, vz=0.0):
        c = TwistStamped()
        c.twist.linear.x = vx
        c.twist.linear.y = vy
        c.twist.linear.z = vz
        self.vel_pub.publish(c)

    def _configure(self):
        """放宽 PX4 限制以支持 6 m/s 追踪。

        MPC_XY_CRUISE 默认 5 m/s，是**位置控制**模式的巡航速度；
        MPC_XY_VEL_MAX 默认 12 m/s 够用；MPC_ACC_HOR_MAX 默认 5 m/s²，
        加速到 6 m/s 需 1.2 s，这里提到 8 以加快响应。
        """
        for pid, ival, rval in (("NAV_RCL_ACT", 0, 0.0),
                                ("COM_RCL_EXCEPT", 7, 0.0),
                                ("MPC_XY_VEL_MAX", 12, 12.0),
                                ("MPC_XY_CRUISE", 6, 6.0),
                                ("MPC_ACC_HOR_MAX", 8, 8.0)):
            try:
                self.param_srv(pid, ParamValue(integer=ival, real=rval))
            except Exception as exc:
                rospy.logwarn("[bench] 设置 %s 失败: %s", pid, exc)

    def _arm_takeoff(self):
        # 等 EKF
        while not rospy.is_shutdown() and (self.local_z is None or abs(self.local_z) > 1.5):
            self._send(0, 0)
            self.rate.sleep()
        for _ in range(120):
            self._send(0, 0)
            self.rate.sleep()
        # OFFBOARD（永不放弃）
        n = 0
        while not rospy.is_shutdown() and self.state.mode != "OFFBOARD":
            n += 1
            self.mode_srv(0, "OFFBOARD")
            for _ in range(40):
                if self.state.mode == "OFFBOARD":
                    break
                self._send(0, 0)
                self.rate.sleep()
        rospy.loginfo("[bench] OFFBOARD（重试 %d 次）", n)
        while not rospy.is_shutdown() and not self.state.armed:
            self.arm_srv(True)
            for _ in range(40):
                if self.state.armed:
                    break
                self._send(0, 0)
                self.rate.sleep()
        rospy.loginfo("[bench] 已解锁")
        # 爬升
        rospy.loginfo("[bench] 爬升到 %.0f m", ALTITUDE)
        while not rospy.is_shutdown():
            if self.local_z is not None and abs(self.local_z - ALTITUDE) < 0.4:
                break
            vz = 0.6 * (ALTITUDE - (self.local_z or 0.0))
            self._send(0, 0, max(-1.5, min(1.5, vz)))
            self.rate.sleep()
        rospy.loginfo("[bench] 到达 %.1f m", self.local_z)

    # -------- 目标模型 --------
    def _init_target(self):
        """在无人机附近 20 m 处放一个目标（保证初始可见）。"""
        if self.g is None or self.world_xy is None:
            return False
        import random
        for _ in range(400):
            ang = random.uniform(0, 2 * math.pi)
            d = random.uniform(18.0, 20.0)
            tx = self.world_xy[0] + d * math.cos(ang)
            ty = self.world_xy[1] + d * math.sin(ang)
            if abs(tx) > 90 or abs(ty) > 42:
                continue
            if not self.g.free(tx, ty):
                continue
            if self.g.open_space_score(tx, ty) < 0.5:
                continue
            if not self.g.visible(self.world_xy[0], self.world_xy[1], tx, ty):
                continue
            self.tgt = [tx, ty]
            rospy.loginfo("[bench] 目标初始位置 (%.1f, %.1f)，距无人机 %.1f m",
                          tx, ty, d)
            return True
        return False

    def _step_target(self, dt):
        if self.tgt is None or self.world_xy is None:
            return
        d = math.hypot(self.tgt[0] - self.world_xy[0], self.tgt[1] - self.world_xy[1])
        if d >= SC.SENSE_R:
            return
        away = (self.tgt[0] - self.world_xy[0], self.tgt[1] - self.world_xy[1])
        n = math.hypot(away[0], away[1]) or 1.0
        if self.blocked > 0:
            fdx, fdy = self.escape_dir
        else:
            fd = SC.pick_flee_dir(self.g, self.tgt, self.world_xy, self.hide, self.escape_dir)
            fdx, fdy = fd
            self.escape_dir = fd
        nx, ny, hit = self.g.speed_toward(self.tgt[0], self.tgt[1],
                                          self.tgt[0] + fdx * 10, self.tgt[1] + fdy * 10,
                                          SC.TARGET_FLEE)
        if hit:
            self.blocked += 1
            if self.blocked >= 3:
                self.blocked = 0
        else:
            self.blocked = 0
        self.tgt[0], self.tgt[1] = nx, ny

    # -------- 自适应调速 --------
    def _speed(self, dist, dx, dy, act_vx, act_vy, los_ok=None):
        """按态势给出速度。

        离线实验里自适应没收益，是因为模型假设「目标永远朝前跑」（6 m/s 追
        2 m/s 时目标总在前方），过冲判据从不触发。

        SITL 实测揭穿了这一点：真实飞行有加速度限制和位置超调，实测最小间距
        只有 0.30 m —— 无人机确实冲到了目标身上。冲过去之后目标按规则掉头
        （远离无人机），无人机再冲 → 来回震荡 → LOS 反复中断（实测 19 次）。

        所以本版针对**真实存在的震荡**调速，判据来自实测量而非模型假设：
          * 距离过近（<3 m）             → 减速，避免撞上去
          * 实测速度方向与「指向目标」方向背离 → 正在过冲/掉头，减速
          * 其余                          → 全速
        """
        if self.policy != "adaptive":
            return SPEED_CMD

        # 视线断了：全速去补（30 秒预算紧张，减速只会更糟）
        if los_ok is False:
            return SPEED_CMD

        # 太近：目标就在 3 m 内，继续全速会直接冲过去
        if dist < 3.0:
            return max(0.8, SPEED_CMD * 0.35)
        if dist < 5.0:
            return max(1.2, SPEED_CMD * 0.6)

        # 过冲判据：实测速度（我实际在往哪走）与「指向目标」方向背离
        a_speed = math.hypot(act_vx, act_vy)
        if a_speed > 0.5 and dist > 1e-3:
            to_tgt_x, to_tgt_y = dx / dist, dy / dist
            cosang = (act_vx * to_tgt_x + act_vy * to_tgt_y) / a_speed
            if cosang < 0.3:          # 实际运动方向明显偏离目标 → 正在掉头
                return max(1.0, SPEED_CMD * 0.5)

        return SPEED_CMD

    # -------- 主循环 --------
    def run(self):
        while not rospy.is_shutdown() and not self.state.connected:
            self.rate.sleep()
        rospy.loginfo("[bench] MAVROS 已连接")
        while not rospy.is_shutdown() and (self.world_xy is None or self.local_xy is None):
            self.rate.sleep()
        self.offset = (self.world_xy[0] - self.local_xy[0],
                       self.world_xy[1] - self.local_xy[1])
        self._configure()
        self._arm_takeoff()

        if not self._init_target():
            rospy.logerr("[bench] 目标初始化失败")
            return
        rospy.loginfo("[bench] 开始追踪测试，时长 %.0f s，下发速度 %.1f m/s",
                      self.duration, SPEED_CMD)

        t0 = rospy.Time.now().to_sec()
        last = t0
        last_sample = t0
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            dt = max(1e-3, min(now - last, 0.3))
            last = now
            if now - t0 > self.duration:
                break

            self._step_target(dt)

            # ---- 追踪策略：chase + 可选自适应调速 ----
            gx, gy = self.tgt[0], self.tgt[1]
            dx = gx - self.world_xy[0]
            dy = gy - self.world_xy[1]
            dist = math.hypot(dx, dy)

            # 实测速度（用于过冲判据：真实位置差分，不是下发的期望速度）
            act_vx = act_vy = 0.0
            if self.prev_xy is not None and self.prev_t is not None:
                dtp = now - self.prev_t
                if dtp > 1e-3:
                    act_vx = (self.world_xy[0] - self.prev_xy[0]) / dtp
                    act_vy = (self.world_xy[1] - self.prev_xy[1]) / dtp

            spd = self._speed(dist, dx, dy, act_vx, act_vy)

            if dist > 1e-3:
                vx = dx / dist * spd
                vy = dy / dist * spd
            else:
                vx = vy = 0.0
            vz = 0.6 * (ALTITUDE - (self.local_z or ALTITUDE))
            self._send(vx, vy, max(-1.5, min(1.5, vz)))

            # ---- LOS 与确认计时 ----
            los = self.g.visible(self.world_xy[0], self.world_xy[1], gx, gy) and dist < SC.SENSE_R
            if los:
                self.streak += dt
                self.max_streak = max(self.max_streak, self.streak)
            else:
                if self.seen_last and self.streak > 0:
                    self.los_breaks += 1
                self.streak = 0.0
            self.seen_last = los

            # ---- 采样 ----
            if now - last_sample >= 1.0 / SAMPLE_HZ:
                last_sample = now
                act = 0.0
                if self.prev_xy is not None and self.prev_t is not None:
                    dtp = now - self.prev_t
                    if dtp > 1e-3:
                        act = math.hypot(self.world_xy[0] - self.prev_xy[0],
                                         self.world_xy[1] - self.prev_xy[1]) / dtp
                self.prev_xy = self.world_xy
                self.prev_t = now
                self.samples.append(dict(t=now - t0, cmd=spd,
                                         act=act, dist=dist, los=los,
                                         streak=self.streak))
            self.rate.sleep()

        self._report()

    def _report(self):
        print("\n" + "=" * 72)
        print("SITL 追踪能力实测报告")
        print("=" * 72)
        print("机型 %s | 高度 %.1f m | 速度策略 %s | 目标 %.1f m/s | 躲藏=%s"
              % (self.model_name, ALTITUDE, self.policy, SC.TARGET_FLEE if SC else 2.0,
                 "理想" if self.hide == SC.HIDE_IDEAL else "无"))
        n = len(self.samples)
        if n < 5:
            print("采样太少（%d），测试可能未正常完成" % n)
            return
        acts = [s["act"] for s in self.samples]
        cmds = [s["cmd"] for s in self.samples]
        dists = [s["dist"] for s in self.samples]
        los_n = sum(1 for s in self.samples if s["los"])
        avg_cmd = sum(cmds) / n
        print("\n采样 %d 个（%.1f Hz）" % (n, SAMPLE_HZ))
        print("  下发速度:     平均 %.2f m/s（最小 %.2f 最大 %.2f）"
              % (avg_cmd, min(cmds), max(cmds)))
        print("  实际水平速度: 平均 %.2f m/s  最大 %.2f m/s"
              % (sum(acts) / n, max(acts)))
        print("  速度达成率:   %.1f%%  (实际均值 / 下发均值)"
              % (100.0 * (sum(acts) / n) / max(avg_cmd, 1e-6)))
        print("  与目标间距:   平均 %.2f m  最小 %.2f m" % (sum(dists) / n, min(dists)))
        print("  LOS 占比:     %.1f%% (%d/%d 采样)" % (100.0 * los_n / n, los_n, n))
        print("  最长连续确认: %.2f s (需 %.0f s)" % (self.max_streak, CONFIRM_S))
        print("  LOS 中断次数: %d" % self.los_breaks)
        if min(dists) < 1.0:
            print("  ⚠ 最小间距仅 %.2f m —— 存在冲撞目标的风险" % min(dists))
        ok = self.max_streak >= CONFIRM_S
        print("\n结论: %s" % ("达成 15 s 连续确认 ✓" if ok else "未达成 15 s ✗"))
        if acts:
            avg = sum(acts) / n
            if avg < SPEED_CMD * 0.85:
                print("  ⚠ 实际速度仅为满速的 %.0f%% —— 转弯掉速明显"
                      % (100.0 * avg / SPEED_CMD))
            else:
                print("  ✓ 实际速度接近满速")
        print("=" * 72 + "\n")


if __name__ == "__main__":
    rospy.init_node("track_bench")
    uid = rospy.get_param("~uav_id", "uav_1")
    model = rospy.get_param("~model_name", "iris_1")
    TrackBench(uid, model).run()
