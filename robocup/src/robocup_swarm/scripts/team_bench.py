#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SITL 验证：3 架全部做 tracker，对抗理想躲藏目标 —— **多轮统计**。

为什么是 3 tracker，不是 2T+1B
------------------------------
离线几何仿真（tracker_blocker.py，n=200，同一套地图与规则）实测：

    3 trackers         67.5% (135/200)   同时观测 2.09
    2 tracker + blocker 60.5% (121/200)  同时观测 1.41
    tracker + blocker   41.5%            同时观测 0.70

且消融显示「让 Blocker 抢最近机」与「Tracker 站观测侧」两个"改进"都是负收益
（60.5% → 62.0% / 35.0%）。Blocker 这条路线**放弃**，3 架都做 tracker。

为什么必须多轮
--------------
真实飞行有转弯掉速、位置超调、通信延迟，单轮达成与否含很大随机性。上一版
单轮跑出 13.10 s（需 15 s）就下结论，是无效的 —— 离线同配置本来就只有约
60~67% 达成率。**必须多轮统计达成率**才算验证。

每轮协议（与离线初始条件对齐，保证可比）
---------------------------------------
  1. 选一个可通行且开阔的目标点（靠近三机质心，缩短转场）
  2. 三机飞到半径 REGROUP_R 的均布站位（120° 间隔），等全部就位
  3. 放下目标，开始计时；15 s 连续确认 → 达成；30 s 未消除 → 瞬移（失败）

用法（SITL 已起 3 机）
    ROS_NAMESPACE= rosrun robocup_swarm team_bench.py \
        _rounds:=10 _round_time:=45 _hide:=ideal
"""

import math
import os
import random
import sys

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ParamValue, State
from mavros_msgs.srv import CommandBool, ParamSet, SetMode

WS = os.environ.get("ROBOCUP_WS", os.path.expanduser("~/team_ws/robocup"))
sys.path.insert(0, os.path.join(WS, "scripts", "vm"))
sys.path.insert(0, os.path.join(WS, "src", "robocup_swarm", "scripts"))

from strategy_compare import (MapGrid, METADATA, pick_flee_dir,  # noqa: E402
                              HIDE_IDEAL, HIDE_LOCAL, HIDE_NONE,
                              TARGET_FLEE, SENSE_R)
from cooperative_tracker import CooperativeTracker  # noqa: E402

# 转场用项目里已验证的 A*（同一份 metadata 栅格，见 docs/astar_validation_2026-09-05.md）。
# 实测 20 条真实转场航段 20/20 成功、平均 22 ms —— 每轮只规划一次，开销可忽略。
try:
    from robocup_navigation import astar as _astar
except ImportError:                       # 未装该包时降级为纯局部避障
    _astar = None

ALTITUDE = 6.0
SPEED_MAX = 6.0
CTRL_HZ = 20.0
TRACK_STANDOFF = 8.0     # Tracker 与目标保持的距离
TRACK_SPLIT_DEG = 40.0   # 多架 tracker 绕目标的角向分离（防叠在一起）
REGROUP_R = 14.0         # 每轮开始前三机的均布站位半径
REGROUP_TOL = 16.0       # 视为"到位"的容差

CONFIRM_S = 15.0
EVADE_S = 30.0


class UAV(object):
    """单架无人机的 MAVROS 接口封装。"""

    def __init__(self, uav_id, model_name):
        self.uav_id = uav_id
        self.model_name = model_name
        self.state = State()
        self.local_xy = None
        self.local_z = None
        self.world_xy = None
        self.offset = None
        self.role = "tracker"
        # 卡死检测：同一位置徘徊超过 STUCK_S 且高度掉到 STUCK_Z 以下 → 判定卡死
        self._last_xy = None
        self._last_move_t = None
        self.stuck = False
        self.arm_srv = rospy.ServiceProxy("/%s/mavros/cmd/arming" % uav_id, CommandBool)
        self.mode_srv = rospy.ServiceProxy("/%s/mavros/set_mode" % uav_id, SetMode)
        self.param_srv = rospy.ServiceProxy("/%s/mavros/param/set" % uav_id, ParamSet)
        rospy.Subscriber("/%s/mavros/state" % uav_id, State, self._on_state)
        rospy.Subscriber("/%s/mavros/local_position/pose" % uav_id,
                         PoseStamped, self._on_pose)
        self.vel_pub = rospy.Publisher("/%s/mavros/setpoint_velocity/cmd_vel" % uav_id,
                                       TwistStamped, queue_size=10)

    def _on_state(self, m):
        self.state = m

    def _on_pose(self, m):
        self.local_xy = (m.pose.position.x, m.pose.position.y)
        self.local_z = m.pose.position.z

    def send(self, vx, vy, vz=0.0):
        c = TwistStamped()
        c.twist.linear.x = vx
        c.twist.linear.y = vy
        c.twist.linear.z = vz
        self.vel_pub.publish(c)

    # ---------------- 卡死检测 ----------------
    def check_stuck(self, now, min_z=1.5, stuck_s=8.0, move_m=1.0):
        """是否卡死（撞楼后趴地不动）—— 返回 True 表示**已卡死**。

        实测教训：uav_1 在转场时啃上建筑，此后永久卡在 armed+OFFBOARD 趴地
        状态，而 harness 完全没有察觉 —— 后面 18 轮全部因"转场凑不齐"作废，
        白跑 25 分钟。宁可早报错重跑，也不要静默浪费一整轮统计。

        判据：位置连续 stuck_s 秒位移不足 move_m，**且**高度低于 min_z。
        只看"没动"会误伤 —— 悬停等指令、被 EKF 拖住的机器都可能短暂静止；
        加上高度判据后，只有真正落地趴窝才会命中。
        """
        if self.world_xy is None:
            return False
        xy = self.world_xy
        if self._last_xy is None:
            self._last_xy, self._last_move_t = xy, now
            return False
        if math.hypot(xy[0] - self._last_xy[0], xy[1] - self._last_xy[1]) > move_m:
            self._last_xy, self._last_move_t = xy, now
            self.stuck = False
            return False
        moving_long = (now - self._last_move_t) > stuck_s
        low = (self.local_z is not None and self.local_z < min_z)
        if moving_long and low:
            self.stuck = True
        return self.stuck


class RoundLog(object):
    """单轮结果。"""

    def __init__(self, idx):
        self.idx = idx
        self.achieved = False
        self.ended_by = "timeout"
        self.elapsed = 0.0
        self.streak = 0.0            # 本轮最长连续覆盖（s）
        self.live = []               # 同时观测架数采样
        self.los = {}                # uav_id -> [命中, 总数]


class TeamBench(object):
    def __init__(self):
        self.rounds = int(rospy.get_param("~rounds", 10))
        self.round_time = float(rospy.get_param("~round_time", 45.0))
        self.regroup_timeout = float(rospy.get_param("~regroup_timeout", 60.0))
        hide = rospy.get_param("~hide", "ideal")
        self.hide = {"none": HIDE_NONE, "local": HIDE_LOCAL,
                     "ideal": HIDE_IDEAL}[hide]

        self.g = MapGrid(METADATA)
        self.uavs = {}
        for i, uid in enumerate(["uav_1", "uav_2", "uav_3"]):
            self.uavs[uid] = UAV(uid, "iris_%d" % (i + 1))
        self.trackers = list(self.uavs.keys())

        self.tgt = None
        self.tr = CooperativeTracker()

        rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_models)
        self.rate = rospy.Rate(CTRL_HZ)

        self.logs = []

    def _on_models(self, m):
        for uid, u in self.uavs.items():
            try:
                i = m.name.index(u.model_name)
            except ValueError:
                continue
            p = m.pose[i].position
            u.world_xy = (p.x, p.y)

    # ---------------- 起飞 ----------------
    def _configure(self, u):
        for pid, ival, rval in (("NAV_RCL_ACT", 0, 0.0),
                                ("COM_RCL_EXCEPT", 7, 0.0),
                                ("MPC_XY_VEL_MAX", 12, 12.0),
                                ("MPC_XY_CRUISE", 6, 6.0),
                                ("MPC_ACC_HOR_MAX", 8, 8.0)):
            try:
                u.param_srv(pid, ParamValue(integer=ival, real=rval))
            except Exception:
                pass

    def _takeoff_all(self):
        """并行推进三机起飞（预热→OFFBOARD→解锁→爬升）。

        坑 1：PX4 有 "auto preflight disarming" —— 解锁后若数秒内没有实际起飞
        （位置/速度没变化），commander 会自动上锁。必须先逐架解锁完再统一爬升的
        写法会让第一架被自动上锁，出现 OFFBOARD 但 armed=False、停在地面。
        因此每轮循环同时推进三机各自的状态机，任一机就绪立刻给它爬升速度。

        坑 2：arm_srv 返回 success=True 只代表 PX4 接受了请求，不代表真解锁 ——
        EKF 未完全就绪时会静默拒绝。必须持续重试并**以 state.armed 为准**。
        """
        state = {uid: "wait_ekf" for uid in self.uavs}
        attempts = {uid: 0 for uid in self.uavs}
        t_start = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            for uid, u in self.uavs.items():
                st = state[uid]
                vz = 0.6 * (ALTITUDE - (u.local_z or 0.0))
                vz = max(-1.5, min(1.5, vz))

                if st == "wait_ekf":
                    if u.local_z is not None and abs(u.local_z) < 1.5:
                        state[uid] = "offboard"
                    u.send(0, 0, 0.0)

                elif st == "offboard":
                    if u.state.mode == "OFFBOARD":
                        state[uid] = "arm"
                    elif attempts[uid] % 20 == 0:
                        u.mode_srv(0, "OFFBOARD")
                    attempts[uid] += 1
                    u.send(0, 0, 0.0)

                elif st == "arm":
                    if u.state.armed:
                        state[uid] = "climb"
                        rospy.loginfo("[team] %s 已解锁，开始爬升", uid)
                    elif attempts[uid] % 20 == 0:
                        u.arm_srv(True)
                    attempts[uid] += 1
                    u.send(0, 0, 0.0)

                else:  # climb
                    u.send(0, 0, vz)
                    if u.local_z is not None and abs(u.local_z - ALTITUDE) < 0.5:
                        state[uid] = "ready"

            if all(state[uid] == "ready" for uid in self.uavs):
                break
            if rospy.Time.now().to_sec() - t_start > 90.0:
                bad = [uid for uid in self.uavs if state[uid] != "ready"]
                rospy.logerr("[team] 起飞超时，未就位的机: %s（状态 %s）",
                             bad, {b: state[b] for b in bad})
                break
            self.rate.sleep()

        rospy.loginfo("[team] 三机就位，高度 %.1f m",
                      max(u.local_z or 0 for u in self.uavs.values()))

    # ---------------- 每轮：选点 + 转场 ----------------
    def _pick_target_xy(self):
        """选一个可通行、开阔、且离三机质心不远的目标点（缩短转场）。"""
        pts = [u.world_xy for u in self.uavs.values() if u.world_xy]
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        for _ in range(500):
            ang = random.uniform(0, 2 * math.pi)
            d = random.uniform(15.0, 40.0)
            tx = cx + d * math.cos(ang)
            ty = cy + d * math.sin(ang)
            if abs(tx) > 88 or abs(ty) > 40:
                continue
            if not self.g.free(tx, ty) or self.g.open_space_score(tx, ty) < 0.5:
                continue
            # 站位环要求**开阔**，不只是"可通行"。
            # 实测教训：uav_1 卡死在 (-35.4,-7.8) —— 元数据地图里该点 free=False，
            # 是建筑内部一条 1 格宽的缝。`free()` 只看单格，1 格宽的缝也算通；
            # 但 6 m/s 的机进不去。必须用 open_space_score 过滤这种"纸面可通行"。
            ring_ok = True
            base = random.uniform(0, 2 * math.pi)
            for i in range(len(self.trackers)):
                a = base + 2 * math.pi * i / len(self.trackers)
                sx = tx + REGROUP_R * math.cos(a)
                sy = ty + REGROUP_R * math.sin(a)
                if (not self.g.free(sx, sy)
                        or self.g.open_space_score(sx, sy) < 0.4):
                    ring_ok = False
                    break
            if ring_ok:
                return tx, ty, base
        return None

    def _regroup(self, tx, ty, base, idx=None):
        """三机飞到目标周围的均布站位，等全部就位。

        这是为了与离线仿真的初始条件对齐（离线是三机都在目标 12~20 m 内开局）。
        不做转场的话，轮次之间无人机散在场上，达成率会被人为压低、也无法与
        离线数字对比。

        超时/卡死时打印每机的位置与到目标的距离、以及"哪个方向被挡" ——
        实测这里失败过 3 轮（三机卡在 x≈-95 一带），没有诊断信息就只能靠猜。
        """
        n = len(self.trackers)
        goals = {}
        for i, uid in enumerate(self.trackers):
            a = base + 2 * math.pi * i / n
            goals[uid] = (tx + REGROUP_R * math.cos(a), ty + REGROUP_R * math.sin(a))

        # ---- 先用 A* 规划每机的转场航线（每轮只做一次）----
        #
        # 为什么必须用全局规划而不是"朝站位 + 局部避障"：纯局部贪心会掉进凹形
        # 建筑的局部最小值。实测 uav_1 连续多轮冻在 (-71.5, 20.3)，十六个方向
        # 全被挡 → 每帧输出 0 速度，模拟 200 步零位移，离站位 39 m 永远到不了。
        # 这不是调参能解决的拓扑问题，必须全局搜索。
        #
        # A* 用的是同一份 metadata 栅格（与 self.g 同源），实测 20/20 成功、
        # 平均 22 ms，且路径本身保证避开建筑。
        routes = {uid: None for uid in self.uavs}
        if _astar is not None:
            try:
                md, _ = _astar.load_metadata(METADATA)
                grid = _astar.GridMap.from_metadata(md)
                for uid, u in self.uavs.items():
                    if u.world_xy is None:
                        continue
                    r = _astar.plan(grid, u.world_xy, goals[uid],
                                    connectivity=8, prevent_corner_cutting=True,
                                    max_expansions=250000)
                    if r.success and r.points:
                        # 抽稀：每隔约 2 m 取一个航点，避免几百点的路径抖动
                        pts = list(r.points)
                        kept = [pts[0]]
                        for p in pts[1:]:
                            if math.hypot(p[0] - kept[-1][0],
                                          p[1] - kept[-1][1]) >= 2.0:
                                kept.append(p)
                        if kept[-1] != pts[-1]:
                            kept.append(pts[-1])
                        routes[uid] = kept
                    else:
                        rospy.logwarn("[team] %s 转场 A* 失败（%s），回退局部避障",
                                      uid, r.reason)
            except Exception as exc:                       # noqa: BLE001
                rospy.logwarn("[team] A* 不可用（%s），回退局部避障", exc)

        wp = {uid: 0 for uid in self.uavs}      # 每机当前航点索引

        t0 = rospy.Time.now().to_sec()
        self._regroup_best = None      # 本轮"三机距离之和"的历史最优
        self._regroup_best_t = t0
        while not rospy.is_shutdown():
            # 把**友机**也算作障碍：`_toward` 只查建筑，三机互不避让就会叠成
            # 一团谁也过不去（实测第 8 轮：uav_2/uav_3 相距 1 m，双双卡死，
            # 三机全挤在 (-40.x,-0.9)）。给每机单独构造一个"含友机"的判通函数。
            free_of = {}
            for uid, u in self.uavs.items():
                if u.world_xy is None:
                    continue
                others = [v.world_xy for k, v in self.uavs.items()
                          if k != uid and v.world_xy is not None]

                def mk(others):
                    def fn(px, py):
                        if not self.g.free(px, py):
                            return False
                        for ox, oy in others:
                            if math.hypot(px - ox, py - oy) < 2.5:
                                return False
                        return True
                    return fn
                free_of[uid] = mk(others)

            ready = 0
            for uid, u in self.uavs.items():
                if u.world_xy is None or uid not in free_of:
                    continue
                gx, gy = goals[uid]
                ux, uy = u.world_xy

                # 沿 A* 航线推进航点：到达当前航点(2 m 内)就切下一个。
                # 只为**避开建筑**服务，所以友机不作为航点切换的判据 ——
                # 否则被友机挡住时会永远卡在同一个航点。
                route = routes.get(uid)
                look = (gx, gy)
                if route:
                    i = wp[uid]
                    while i < len(route) - 1 and math.hypot(
                            route[i][0] - ux, route[i][1] - uy) < 2.0:
                        i += 1
                    wp[uid] = i
                    look = route[i]

                fn = free_of[uid]
                # 目标点本身若被友机"占住"，也允许靠近（否则永远到不了位）
                vx, vy = _toward((ux, uy), look, SPEED_MAX, fn, stop=1.0)
                u.send(vx, vy, self._vz(u))
                if math.hypot(ux - gx, uy - gy) < 3.0:
                    ready += 1
            if ready == len(self.uavs):
                return True
            # 卡死检测：任一机趴地不动超过 8 s → 立刻失败返回，由上层判断是否中止
            now = rospy.Time.now().to_sec()
            for uid, u in self.uavs.items():
                if not u.check_stuck(now):
                    continue
                rospy.logerr("[team] %s 已卡死（world=(%.1f,%.1f) z=%.2f）—— 转场放弃",
                             uid, u.world_xy[0], u.world_xy[1], u.local_z or 0.0)
                self._dump_stuck(idx, goals)
                return False
            if rospy.Time.now().to_sec() - t0 > self.regroup_timeout:
                self._dump_stuck(idx, goals)
                return False
            # 无进展提前失败：转场失败时每轮要空等满 regroup_timeout(60 s)，
            # 连续几轮就是好几分钟白烧。若最近 20 s 内**三机距离之和**没有
            # 任何改善，说明已经卡住（撞墙推不动 / 被建筑困住），不必等满。
            #
            # 为什么用"和"而不是"最小值"：取 min 时，只要有一架到位就饱和在
            # ~0.2 m，另两架再怎么挣扎都"没有改善"，每轮必然误判成无进展 ——
            # 实测第 3/6/7 轮全部因此被误杀。
            best = sum(math.hypot(u.world_xy[0] - goals[uid][0],
                                  u.world_xy[1] - goals[uid][1])
                       for uid, u in self.uavs.items() if u.world_xy)
            if self._regroup_best is None or best < self._regroup_best - 0.5:
                self._regroup_best = best
                self._regroup_best_t = now
            elif now - self._regroup_best_t > 20.0:
                rospy.logwarn("[team] 第 %s 轮：20 s 内转场无进展（三机距离和仍 %.1f m），"
                              "提前放弃", idx, best)
                self._dump_stuck(idx, goals)
                return False
            self.rate.sleep()
        return False

    def _dump_stuck(self, idx, goals):
        """转场失败时打印诊断：卡在哪、哪个方向被挡。"""
        rospy.logwarn("[team] 第 %s 轮转场诊断（目标站位 / 实测位置 / 距离 / 方向可通行性）:",
                      idx)
        for uid, u in self.uavs.items():
            if u.world_xy is None:
                rospy.logwarn("    %s: 无世界坐标", uid)
                continue
            gx, gy = goals[uid]
            ux, uy = u.world_xy
            dx, dy = gx - ux, gy - uy
            d = math.hypot(dx, dy)
            blocked = []
            if d > 1e-6:
                for probe in (1.0, 2.0, 4.0, 8.0):
                    px, py = ux + dx / d * probe, uy + dy / d * probe
                    if not self.g.free(px, py):
                        blocked.append("%.0fm处被挡" % probe)
                        break
            rospy.logwarn("    %s: 站位(%.1f,%.1f) 实际(%.1f,%.1f) 距离 %.1f m %s",
                          uid, gx, gy, ux, uy, d,
                          " ".join(blocked) if blocked else "直线可通行(可能在震荡)")

    def _vz(self, u):
        vz = 0.6 * (ALTITUDE - (u.local_z or ALTITUDE))
        return max(-1.5, min(1.5, vz))

    # ---------------- 目标运动 ----------------
    def _move_target(self, dt):
        """规则 3：以 2 m/s 远离**最近的无人机**。

        注意：`pick_flee_dir` 只在"远离"方向的 ±90° 内选向（并且不把无人机
        当成障碍），所以目标不会掉头 —— 这是**本仿真目标模型**的设定，公开
        规则只说"以 2 m/s 逃跑"，并未禁止转向。故这里测出的是最坏情况下界。
        """
        if self.tgt is None:
            return
        pos = [u.world_xy for u in self.uavs.values() if u.world_xy]
        if not pos:
            return
        nx, ny = min(pos, key=lambda p: math.hypot(p[0] - self.tgt[0],
                                                   p[1] - self.tgt[1]))
        fd = pick_flee_dir(self.g, self.tgt, (nx, ny), self.hide)
        tx, ty, _ = self.g.speed_toward(self.tgt[0], self.tgt[1],
                                        self.tgt[0] + fd[0] * 10,
                                        self.tgt[1] + fd[1] * 10, TARGET_FLEE)
        self.tgt[0], self.tgt[1] = tx, ty

    # ---------------- 单轮 ----------------
    def _round(self, idx):
        log = RoundLog(idx)
        pick = self._pick_target_xy()
        if pick is None:
            rospy.logerr("[team] 第 %d 轮：找不到合适目标点，跳过", idx)
            return None
        tx, ty, base = pick
        rospy.loginfo("[team] 第 %d 轮：目标 (%.1f, %.1f)，回合站位半径 %.0f m",
                      idx, tx, ty, REGROUP_R)

        # 转场失败就**作废本轮**，不要勉强开始。
        # 冒烟测试的教训：转场超时时三机散在城里（就位 1/3），我却照常开始计时，
        # 结果整轮 2 架机 LOS 0.0% —— 那测的是"转场失败"不是"追踪能力"，
        # 计数进去会把达成率系统性拉低。
        if not self._regroup(tx, ty, base, idx):
            rospy.logwarn("[team] 第 %d 轮：转场未完成，作废本轮（不计数）", idx)
            return None

        self.tgt = [tx, ty]

        # 目标就位后才登记到协同确认器：tracked_since 必须从此刻起算，
        # 否则 (now - tracked_since) 一上来就超过规则4 的 30 s 阈值。
        self.tr = CooperativeTracker()
        self.tr.add_target("t0", now=rospy.Time.now().to_sec())
        self.tr.assign_observers("t0", self.trackers)

        log.los = {u: [0, 0] for u in self.trackers}
        t0 = rospy.Time.now().to_sec()
        last = t0
        cur_streak = 0.0

        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            dt = max(1e-3, min(now - last, 0.3))
            last = now
            log.elapsed = now - t0
            if log.elapsed > self.round_time:
                log.ended_by = "timeout"
                break

            self._move_target(dt)

            # ---- 观测 ----
            for u in self.trackers:
                U = self.uavs[u]
                if U.world_xy is None:
                    continue
                d = math.hypot(self.tgt[0] - U.world_xy[0], self.tgt[1] - U.world_xy[1])
                vis = d < SENSE_R and self.g.visible(U.world_xy[0], U.world_xy[1],
                                                     self.tgt[0], self.tgt[1])
                self.tr.report(u, "t0", now, self.tgt[0], self.tgt[1], los=vis)
                log.los[u][1] += 1
                if vis:
                    log.los[u][0] += 1

            live = len(self.tr.targets["t0"].live_observers(now))
            log.live.append(live)
            cur_streak = cur_streak + dt if live > 0 else 0.0
            log.streak = max(log.streak, cur_streak)

            for _tid, ev in self.tr.update(now):
                if ev == "confirmed":
                    log.achieved = True
                    log.ended_by = "confirmed"
                    self._fly_trackers()
                    return log
                if ev == "evade":
                    log.ended_by = "evade"
                    self._fly_trackers()
                    return log

            self._fly_trackers()
            self.rate.sleep()

        return log

    def _fly_trackers(self):
        """Tracker 控制：**追尾 standoff**（离线实测最优）。

        为什么"追尾"而不是"站到逃跑反方向"：离线 n=200 消融里，观测侧站位把
        达成率从 60.5% 砸到 35.0% —— 因为三架会从"沿轨迹一字排开"塌缩成
        "挤在同一侧"，同时观测从 1.41 掉到 1.58→更差且更易同框被同一栋楼挡住。
        所以维持原版：径向保持 TRACK_STANDOFF，再加一点角向分离防叠机。
        """
        k = len(self.trackers)
        for i, uid in enumerate(self.trackers):
            U = self.uavs[uid]
            if U.world_xy is None:
                continue
            ux, uy = U.world_xy
            d = math.hypot(self.tgt[0] - ux, self.tgt[1] - uy)
            if d > TRACK_STANDOFF + 1.0:
                gx, gy = self.tgt[0], self.tgt[1]        # 太远 → 直追
            elif d < TRACK_STANDOFF - 1.0 and d > 1e-6:
                gx = ux + (ux - self.tgt[0]) / d * (TRACK_STANDOFF - d)
                gy = uy + (uy - self.tgt[1]) / d * (TRACK_STANDOFF - d)
            else:
                gx, gy = ux, uy                          # 保持

            # 角向分离：绕目标旋转 aim 点，径向距离不变
            if k >= 2:
                off = math.radians(TRACK_SPLIT_DEG) * (2.0 * i - (k - 1))
                dx, dy = gx - self.tgt[0], gy - self.tgt[1]
                ca, sa = math.cos(off), math.sin(off)
                gx = self.tgt[0] + dx * ca - dy * sa
                gy = self.tgt[1] + dx * sa + dy * ca
            # aim 点落在障碍里时，不能简单回退到"原地不动" —— 那正是 uav_1
            # 永久卡死的机制：它挤进建筑格后，每帧都判定 aim 不可通行 → 回退到
            # 当前位置 → 输出 0 速度 → 再也出不来。
            # 改为：把 aim 点沿"目标→自身"方向**拉回可通行处**，保证总有出路。
            if not self.g.free(gx, gy):
                for back in (1.0, 2.0, 3.0, 5.0, 8.0, 12.0):
                    ddx, ddy = ux - gx, uy - gy
                    dn = math.hypot(ddx, ddy)
                    if dn < 1e-6:
                        break
                    px = gx + ddx / dn * back
                    py = gy + ddy / dn * back
                    if self.g.free(px, py):
                        gx, gy = px, py
                        break
                else:
                    gx, gy = ux, uy

            vx, vy = _toward((ux, uy), (gx, gy), SPEED_MAX, self.g.free, stop=0.5)
            # 机间分离（斥力，避免相撞 / 叠成一团）。
            # 力随距离线性增大：4 m 处刚生效，1 m 处给满 6 m/s —— 原来恒定的
            # 2.0 m/s 太弱，三机在追踪时贴到 1 m 也推不开（实测第 8 轮转场时
            # uav_2/uav_3 相距 1 m 双双卡死）。
            for other, V in self.uavs.items():
                if other == uid or V.world_xy is None:
                    continue
                dd = math.hypot(ux - V.world_xy[0], uy - V.world_xy[1])
                if 1e-3 < dd < 4.0:
                    push = 6.0 * (4.0 - dd) / 3.0
                    vx += (ux - V.world_xy[0]) / dd * push
                    vy += (uy - V.world_xy[1]) / dd * push
            sp = math.hypot(vx, vy)
            if sp > SPEED_MAX:
                vx *= SPEED_MAX / sp
                vy *= SPEED_MAX / sp
            U.send(vx, vy, self._vz(U))

    # ---------------- 主流程 ----------------
    def run(self):
        rospy.loginfo("[team] 等待 MAVROS 连接...")
        for u in self.uavs.values():
            while not rospy.is_shutdown() and not u.state.connected:
                self.rate.sleep()
        # 等位姿话题。**必须有超时**：订阅没建好（例如 __init__ 被改坏）会让
        # 这里永远等下去 —— 实测静默挂了 36 分钟，只留下一行"等待 MAVROS 连接"，
        # 排查时间远超写这个超时的成本。宁可报错退出，也不要无声空转。
        t_wait = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and any(
                u.world_xy is None or u.local_xy is None for u in self.uavs.values()):
            if rospy.Time.now().to_sec() - t_wait > 30.0:
                missing = [uid for uid, u in self.uavs.items()
                           if u.world_xy is None or u.local_xy is None]
                rospy.logerr("[team] 等待 30 s 仍无位姿：%s —— 退出。"
                             "（检查 /uav_N/mavros/local_position/pose 是否发布、"
                             "以及 UAV.__init__ 里的 Subscriber 是否都建好了）", missing)
                raise RuntimeError("位姿话题超时")
            self.rate.sleep()

        for u in self.uavs.values():
            u.offset = (u.world_xy[0] - u.local_xy[0], u.world_xy[1] - u.local_xy[1])
            self._configure(u)
        rospy.loginfo("[team] 全部连接，开始起飞")

        self._takeoff_all()

        rospy.loginfo("[team] 角色: 3 tracker | 目标 %.1f m/s | 躲藏=%s | %d 轮 × 最长 %.0f s",
                      TARGET_FLEE, rospy.get_param("~hide", "ideal"),
                      self.rounds, self.round_time)

        consec_fail = 0
        for r in range(1, self.rounds + 1):
            if rospy.is_shutdown():
                break
            log = self._round(r)
            if log is None:
                consec_fail += 1
                # 有无人机卡死 → 后面所有轮次都会失败，立刻中止。
                # 实测教训：uav_1 卡死后 harness 毫无察觉，白跑 18 轮 25 分钟。
                dead = [uid for uid, u in self.uavs.items() if u.stuck]
                if dead:
                    rospy.logerr("[team] %s 已卡死，中止后续轮次（避免白跑）。"
                                 " 请重启 SITL 后重跑。", dead)
                    break
                # 连续失败（非卡死）也要停：转场失败通常是环境已坏（EKF 漂移、
                # 某机被推挤），继续跑只会复制同样的失败，白烧时间。
                if consec_fail >= 3:
                    rospy.logerr("[team] 连续 %d 轮转场失败，判定环境已坏，中止。"
                                 " 请重启 SITL 后重跑。", consec_fail)
                    break
                continue
            consec_fail = 0
            self.logs.append(log)
            rospy.loginfo("[team] 第 %d 轮：%s，耗时 %.1f s，最长连续 %.2f s",
                          r, "达成" if log.achieved else "未达成（%s）" % log.ended_by,
                          log.elapsed, log.streak)
            self._print_running()

        self._report()

    def _print_running(self):
        n = len(self.logs)
        ok = sum(1 for l in self.logs if l.achieved)
        print("   累计: %d/%d 达成 (%.0f%%)" % (ok, n, 100.0 * ok / max(n, 1)))

    def _report(self):
        n = len(self.logs)
        print("\n" + "=" * 72)
        print("SITL 3-Tracker 编组多轮统计")
        print("=" * 72)
        if n == 0:
            print("没有完成的轮次。")
            print("=" * 72 + "\n")
            return
        ok = [l for l in self.logs if l.achieved]
        print("配置:       3 tracker | 目标 %.1f m/s | 躲藏=%s"
              % (TARGET_FLEE, rospy.get_param("~hide", "ideal")))
        print("轮次:       %d" % n)
        print("达成率:     %.1f%% (%d/%d)   [离线同配置基准 67.5%%]"
              % (100.0 * len(ok) / n, len(ok), n))
        if ok:
            print("达成耗时:   平均 %.1f s" % (sum(l.elapsed for l in ok) / len(ok)))
        print("最长连续:   平均 %.2f s | 最好 %.2f s | 需 %.0f s"
              % (sum(l.streak for l in self.logs) / n,
                 max(l.streak for l in self.logs), CONFIRM_S))
        print("同时观测均值: %.2f 架" % (sum(sum(l.live) / max(len(l.live), 1)
                                             for l in self.logs) / n))
        print("结束原因:   %s"
              % {r: sum(1 for l in self.logs if l.ended_by == r)
                 for r in sorted(set(l.ended_by for l in self.logs))})
        print("各机 LOS 占比:")
        for u in self.trackers:
            hit = sum(l.los.get(u, [0, 0])[0] for l in self.logs)
            tot = sum(l.los.get(u, [0, 0])[1] for l in self.logs)
            print("  %s: %.1f%%" % (u, 100.0 * hit / max(tot, 1)))
        print("")
        print("判读：达成率与离线基准（67.5%%）的差距 = 真实飞行代价")
        print("      （转弯掉速、位置超调、控制延迟）。若显著低于基准，")
        print("      瓶颈在飞控跟随精度而非协同策略。")
        print("=" * 72 + "\n")


def _toward(cur, goal, speed, free_fn, stop=0.5):
    """朝 goal 的指令速度，**带贴墙滑行规避 + 避障减速**。

    为什么必须有规避：最早这里是纯直线定速，城里一碰到楼就永远推墙推不动 ——
    冒烟测试直接卡住三机里两机（转场超时 1/3、整轮 LOS 0.0%）。离线仿真的
    MapGrid.speed_toward 本来就带 ±20~120° 的贴墙滑行回退，SITL 这边漏了，
    两边因此完全不可比。

    为什么必须**减速**（实测教训）：只加规避仍然撞楼 —— uav_1 在第 2 轮追踪时
    啃上建筑后永久卡死（armed+OFFBOARD 却趴在地面），把后面 18 轮全部拖成
    "转场失败作废"。原因是 6 m/s、20 Hz 下每帧飞 0.3 m，而实机转弯掉速 +
    位置超调都算在控制周期之外，2 m 的前瞻根本来不及反应。
    所以：前瞻距离随速度放大，且越需要偏转就压得越慢。
    """
    dx, dy = goal[0] - cur[0], goal[1] - cur[1]
    d = math.hypot(dx, dy)
    if d <= stop or d < 1e-6:
        return 0.0, 0.0
    ux, uy = dx / d, dy / d

    # --- 脱困：自身已落在障碍格里时，先朝最近自由格走 ---
    # 实测教训：uav_1 挤进建筑后，十六个方向的探测**全部从非法起点出发**，
    # 于是每帧都返回 (0,0)，永久卡死（模拟 200 步零位移）。必须显式脱困 ——
    # 找最近的可通行点，径直飞出去，这一步优先于任何目标导向。
    if not free_fn(cur[0], cur[1]):
        for rad in (0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0):
            for k in range(16):
                a = math.radians(k * 22.5)
                ex = cur[0] + rad * math.cos(a)
                ey = cur[1] + rad * math.sin(a)
                if free_fn(ex, ey):
                    vx, vy = ex - cur[0], ey - cur[1]
                    vn = math.hypot(vx, vy)
                    if vn < 1e-6:
                        continue
                    sp = min(3.0, abs(speed))       # 脱困用中速，别猛冲
                    return vx / vn * sp, vy / vn * sp
        return 0.0, 0.0
    # 前瞻距离：必须覆盖**刹车距离**。6 m/s 下 MPC_ACC_HOR_MAX=8 m/s² 的刹车
    # 距离约 2.25 m，再加 MAVROS→PX4→Gazebo 约 0.3 s 指令延迟（1.8 m），
    # 实际需要 ~4 m 才来得及。取 speed*0.6 并下限 1.5 m。
    probe = max(1.5, abs(speed) * 0.60)

    def clear(at_ang_rad):
        """沿该方向从近到远**逐点**采样，任一采样点被挡即判定不可通行。

        为什么要多点而不是只探最远端：uav_1 实测卡死在 (-35.4,-7.8)，那里是
        建筑内部一条 1 格宽的缝 —— 只探 2.4 m 一个点会正好落进缝里判"可通行"，
        于是径直撞进楼。逐点采样才能看见近处的墙。
        """
        vx = ux * math.cos(at_ang_rad) - uy * math.sin(at_ang_rad)
        vy = ux * math.sin(at_ang_rad) + uy * math.cos(at_ang_rad)
        steps = max(2, int(probe / 0.4))
        for s in range(1, steps + 1):
            f = probe * s / steps
            if not free_fn(cur[0] + vx * f, cur[1] + vy * f):
                return None
        return vx, vy

    head = clear(0.0)
    if head is not None:
        return head[0] * speed, head[1] * speed

    for ang in (20, -20, 40, -40, 60, -60, 80, -80, 100, -100, 120, -120,
                140, -140, 160, -160):
        r = math.radians(ang)
        got = clear(r)
        if got is not None:
            # 偏得越多越慢（障碍区自然降速），但不低于 1.5 m/s 以免磨蹭
            sp = max(1.5, abs(speed) * math.cos(r))
            return got[0] * sp, got[1] * sp
    return 0.0, 0.0        # 四面皆堵 → 停住（不发旧速度，避免漂移）


if __name__ == "__main__":
    rospy.init_node("team_bench")
    TeamBench().run()
