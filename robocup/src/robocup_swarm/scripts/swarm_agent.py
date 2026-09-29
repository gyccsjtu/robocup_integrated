#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""集群搜索单机节点（每架无人机跑一个实例，自建实现）。

职责（步骤 1：两机共享状态）：
  1. 从 /gazebo/model_states 读自身模型的世界坐标（地图系，与生成器 metadata 一致），
     从 /uav_N/mavros/local_position/pose 读高度与连接状态；
  2. 发布 /swarm/uav_status（UavStatus 自建消息）给集中式管理器；
  3. 订阅 /swarm/assignment（SearchAssignment），过滤出指派给自己的搜索格，
     用 ENU 速度控制飞向格中心。

坐标系说明（关键，避免多机坐标系踩坑）：
  世界/地图系与 gazebo 世界都是 ENU（x 东 y 北 z 上），各机本地 ENU 系只是
  原点不同、轴向平行。因此「世界坐标差」算出的速度向量可直接作为
  setpoint_velocity/cmd_vel 下发，无需逐机做 TF 变换。

只依赖标准库 + rospy + 标准消息，无 ROS 自定义依赖之外的第三方库。
"""

import os
import re
import rospy
import math
import traceback
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
from std_msgs.msg import String, Float32

from robocup_swarm.msg import UavStatus, SearchAssignment, TargetState, TargetDetection
from robocup_navigation.astar import load_metadata, GridMap, plan
from swarm_task import LineOfSight, DETECT_RADIUS, CoverageGrid, GRID_SIZE_M

# 覆盖栅格参数（与 manager 一致）
MAP_X_MIN, MAP_X_MAX = -100.0, 100.0
MAP_Y_MIN, MAP_Y_MAX = -50.0, 50.0

# ============================ 参数 ============================
# 工作空间根目录：可用环境变量 ROBOCUP_WS 覆盖（云端部署/换用户时无需改代码）
WS_ROOT = os.environ.get("ROBOCUP_WS", "/home/ros/team_ws/robocup")
SEARCH_ALTITUDE = 4.5     # 搜索高度 m（最高限制）

# === 官方裁判合规护栏（score_cal.py: z > 6.0 -> score=0 并立即终止任务）===
# 2026-09-29 P1-1：默认 4.2 低于 ALT_TARGET_CAP(4.5) 与分层顶层(4.6)，
#   会造成「爬升->被压回->再爬」的极限环（uav_4）。此前 batch 脚本每轮显式传 5.2
#   掩盖了它，只有不传 env 的默认路径会踩。这里把默认值统一到已验证的 5.2。
#   实测（8+ 轮）：zmax 4.81~4.87，尖峰 0，over6=0，距 6m 红线余量 1.13m。
ALT_HARD_CEIL     = float(os.environ.get('ALT_HARD_CEIL', '5.2'))
# 2026-09-27 事故：提高巡航/避让增益后 iris_3 冲到 6.061m，官方 score=0。
# 原来 4.6 的「1.4m 余量」在大机动超调下根本不够，收到 4.2 并加装最后防线。
ALT_PANIC         = float(os.environ.get('ALT_PANIC', '5.0'))       # 强制快降阈值 m
ALT_PANIC_DESCENT = float(os.environ.get('ALT_PANIC_DESCENT', '-1.6'))  # 快降速度 m/s
ALT_PANIC_HSCALE  = float(os.environ.get('ALT_PANIC_HSCALE', '0.3'))    # 此时水平速度系数
MAX_ACC           = float(os.environ.get('MAX_ACC', '2.5'))         # 水平加速度限幅 m/s^2
# 高度硬上限。原来是 5.5，仍实测到 iris_6 冲到 6.356m（>6m 官方直接判 0 分并终止任务），
# 说明 5.5 的超调余量不够；收到 4.6 后距 6m 有 1.4m 缓冲。
ALT_HARD_DESCENT  = -1.0   # 强制下降速度 m/s（原 -0.6 下降太慢）
# === 6m 红线二次保险（2026-09-28）===
# NBV 提速后 nbv_d 冲到 5.63m。PANIC 4.6m 那道兜不住：PX4 默认 MPC_Z_VEL_MAX_DN=1.0
# 会把下发的 -2.5 裁成 -1.0。这里再加 5.0m 一道，并在 PX4 侧放开下降上限配合。
# 水平机动是超调的源头，紧急时直接停掉水平速度，只留竖直快降。
ALT_EMERG_CEIL     = float(os.environ.get('ALT_EMERG_CEIL', '5.0'))      # 二次保险阈值 m
ALT_EMERG_DESCENT  = float(os.environ.get('ALT_EMERG_DESCENT', '-3.0'))  # 强制快降 m/s
ALT_EMERG_HSCALE   = float(os.environ.get('ALT_EMERG_HSCALE', '0.35'))   # 水平速度系数
#   注意：不能设 0.0！实测急停会让机身拉平、多余升力转垂直，反而抬头上冲 1.3m
#        （prb30：t=52.7 两秒尖峰 3.7->5.06）。0.35 平滑减速，无抬头效应。
ALT_TARGET_CAP     = float(os.environ.get('ALT_TARGET_CAP', '4.5'))      # 目标高度上限（巡航尖峰的源头）

# 2026-09-29：护栏常量一致性自检。历史上 ALT_HARD_CEIL(4.2) < ALT_TARGET_CAP(4.5)
# 静默存在了很久，只在极限环出现后才被反推出来。这里启动即告警。
def _check_alt_consistency():
    if ALT_HARD_CEIL < ALT_TARGET_CAP:
        rospy.logerr('[ALT_CFG] 配置漂移：ALT_HARD_CEIL(%.2f) < ALT_TARGET_CAP(%.2f) '
                     '-> 目标高于硬顶，将出现爬升/压回极限环！'
                     % (ALT_HARD_CEIL, ALT_TARGET_CAP))
    if ALT_EMERG_CEIL <= ALT_HARD_CEIL:
        rospy.logwarn('[ALT_CFG] ALT_EMERG_CEIL(%.2f) <= ALT_HARD_CEIL(%.2f)：'
                      '二次保险先于硬顶触发，等价于把硬顶提到 %.2f'
                      % (ALT_EMERG_CEIL, ALT_HARD_CEIL, ALT_EMERG_CEIL))
    rospy.loginfo('[ALT_CFG] HARD_CEIL=%.2f EMERG_CEIL=%.2f TARGET_CAP=%.2f PANIC=%.2f'
                  % (ALT_HARD_CEIL, ALT_EMERG_CEIL, ALT_TARGET_CAP, ALT_PANIC))


# === 追踪移动目标时的 A* 重规划节流 ===
TRACK_REPLAN_MOVE = 3.0    # 目标移动超过此距离才重规划 (m)
TRACK_REPLAN_SEC  = 2.0    # 距上次规划超过此时间才重规划 (s)
PLAN_FALLBACK = int(os.environ.get("PLAN_FALLBACK", "1"))  # 1=A* 失败时退化为直飞（关掉 = 复现停摆 bug）
MAX_SPEED       = 5.0     # 巡航速度上限 m/s（比赛无限制，实测可跑 10+m/s）
POS_KP          = 0.8     # 位置 P 控制增益
ARRIVE_TOL      = 0.8     # 到达格中心判定半径 m（< 此值视为已到，开始原地搜索）
PUB_RATE        = 10.0    # 状态发布频率 Hz
CTRL_RATE       = 20.0    # 控制频率 Hz
DETECT_RATE     = 5.0     # 目标检测发布频率 Hz（规则3 几何判定）

# ---- 自适应高度 ----
ALT_OPEN_SPACE   = 5.5     # 开阔区域高度 m
ALT_BUILDING     = 4.5      # 建筑附近高度 m
ALT_TRACKING    = 4.0      # 追踪目标高度 m
ALT_TAKEOFF     = 2.0      # 起飞/降落高度 m
ALT_BASE        = float(os.environ.get('ALT_BASE', '2.8'))    # 最低巡航层
ALT_STEP        = float(os.environ.get('ALT_STEP', '0.6'))    # 层间距
ALT_NLAYER      = int(os.environ.get('ALT_NLAYER', '4'))     # 层数
ALT_CEILING     = float(os.environ.get('ALT_CEILING', '4.6')) # 硬顶：官方对 >6m 重罚，这里留 1.4m 余量
VERT_SEP        = float(os.environ.get('VERT_SEP', '0.8'))   # 判定「不在同一层」的竖直间隔
MIN_CRUISE_ALT  = 2.0                                        # 追踪降高后的下限
CLIMB_NO_AVOID  = float(os.environ.get('CLIMB_NO_AVOID', '1.0'))
SAFE_3D         = float(os.environ.get('SAFE_3D', '4.0'))
# 2026-09-27 复标：3.0 时实测最近 0.70~0.85m（一次碰撞 −30 分）。
# 控制周期 20Hz、友机位置来自 10Hz 广播，5m/s 下 0.2s 滞后就是 1m ——
# 3m 门限根本来不及反应，抬到 4.0 才有足够的提前量。
SLOWDOWN_DIST   = float(os.environ.get('SLOWDOWN_DIST', '14.0'))

# === 2026-09-29 P1-2：三处高度护栏原来静默覆写 cmd.twist.linear.z ===
# 出事只能靠抓包反推。这里统一打点：同 tag 首次必打，之后每 5s 一条并带累计次数。
_ALT_GUARD_STATE = {}


def _alt_guard_hit(tag, z, ceil, vz, vx, vy):
    import time as _time
    _now = _time.time()
    _n = _ALT_GUARD_STATE.get(tag + '_n', 0) + 1
    _ALT_GUARD_STATE[tag + '_n'] = _n
    if _now - _ALT_GUARD_STATE.get(tag, 0.0) >= 5.0 or _n == 1:
        _ALT_GUARD_STATE[tag] = _now
        try:
            rospy.logwarn('[ALT_GUARD] %s z=%.2f > ceil=%.2f -> vz=%.2f '
                          '(vx=%.2f vy=%.2f) hits=%d',
                          tag, z, ceil, vz, vx, vy, _n)
        except Exception:
            pass
# === 2026-09-29 P0-1/P0-2/P1-3 新增 ===
# ORCA 可解性硬顶：sep_vel 必须严格小于 MAX_SPEED，否则半平面 v·n >= sep_vel
# 在 |v| <= MAX_SPEED 下数学上无解（v·n <= |v|），退化成不避让。
# 0.8 -> sep_vel <= 4.0，全向采样相邻夹角 22.5°，最坏 cos(11.25°)=0.98，
# |v|=5.0 时 v·n 可达 4.9 > 4.0，恒有解。
SEP_CAP_FRAC    = float(os.environ.get('SEP_CAP_FRAC', '0.8'))
# 远场（d3 > SAFE_3D）分离要求的除数。原来是硬编码 2.0 -> sep_vel 仅 0~3.0，
# d3=8m 时只有 1.0 m/s，表现为「只降速不避让」（P0-2 真空带）。
# 1.0 -> 远场 sep_vel = 10-d3（d3=8 时 2.0，翻倍）。设 2.0 可完全回退旧行为做 A/B 对照。
SEP_FAR_DIV     = float(os.environ.get('SEP_FAR_DIV', '1.0'))
# 目标高度与硬顶的安全间隔：目标必须低于硬顶，否则反复触发硬顶下降形成极限环
ALT_HARD_MARGIN = float(os.environ.get('ALT_HARD_MARGIN', '0.2'))
# 坠地检测（P1-3）：不依赖新订阅，用「高度持续低于目标 + 几乎不动」判定
CRASH_DETECT    = int(os.environ.get('CRASH_DETECT', '1'))
CRASH_DROP_M    = float(os.environ.get('CRASH_DROP_M', '1.5'))   # 低于目标多少米算掉高
CRASH_V_EPS     = float(os.environ.get('CRASH_V_EPS', '0.3'))    # 水平速度低于此值算不动
CRASH_HOLD_S    = float(os.environ.get('CRASH_HOLD_S', '3.0'))   # 持续多久才判定
# 2026-09-27：原来是 FRIEND_SAFE_DIST*3 = 30m。6 架同场几乎恒有友机落在
# 30m 内 → 全程只有 3.0 m/s，搜索吞吐被腰斩。收到 14m，其余时间跑满。
SLOW_SPEED      = float(os.environ.get('SLOW_SPEED', '3.0'))
SEP_GAIN        = float(os.environ.get('SEP_GAIN', '1.0'))     # sep_vel 除数，越小排斥越强
AVOID_ANGLES    = int(os.environ.get('AVOID_ANGLES', '16'))    # ORCA 候选方向数（全向）
DANGER_3D       = float(os.environ.get('DANGER_3D', '3.0'))    # 低于此间距直接纯排斥
# 2026-09-29：2.5 -> 3.0。sep_vel 被可解性削顶后近距排斥变弱，
# 纯排斥兜底线相应上移，覆盖原来「ORCA 无解 + 兜底未触发」的死亡带。
# 三维安全距离。垂直分离本来就该算进「够不够远」里，所以不再二值判断同层与否。
# 起飞爬升期豁免水平避让的高度余量：z < ALT_TAKEOFF + CLIMB_NO_AVOID 时不避让。
# 不豁免的话，起飞区 6 架间距只有 8m，全部落进 ORCA 互斥范围 → 谁也爬不起来。
ALT_NARROW      = 3.5       # 窄通道高度 m
BUILDING_DIST    = 5.0      # 建筑判定距离 m（小于此值视为建筑附近）

# ---- 目标盘旋确认 ----
ORBIT_RADIUS    = 5.0     # 盘旋半径 m（收紧：离目标越近，2D 视线穿过建筑的概率越低）
                            # 仍 << DETECT_RADIUS=20m，且留足建筑避障余量
# 线速度 = ORBIT_SPEED * ORBIT_RADIUS，必须 < 1.0 m/s。
# 官方 control_actor.py:211 —— 飞机以 >1.0 m/s 在 actor 20m 内连续待 2s，
# actor 就进入逃跑态（速度 1.0 -> 2.0 且主动远离），坐标误差随之翻倍，
# 官方 err_threshold=1m 就会频繁判定断链、15s 重来。0.18*5=0.9 m/s 安全。
ORBIT_SPEED     = 0.18     # 盘旋角速度 rad/s（r=5m 时线速度 0.9 m/s）
# 逃跑判定要「连续 2s」满足才触发，所以不必一进 20m 就压速：
# 以 MAX_SPEED=5 m/s 从 20m 冲到 15m 只要 1s，tracking_flag 攒不满 20 次。
# 压速点越靠内，全速段越长，接近越快。
SPOOK_DIST      = 15.0    # 进入此距离才压速（官方逃跑判定边界是 20m）
SPOOK_SPEED     = 0.9     # 必须 < 1.0 m/s，否则触发 control_actor 的 catching_flag
CONFIRM_TIME    = 15.0     # 连续确认时间才消除（规则5）

# ---- 盘旋放弃 / 防扎堆（2026-09-27：修「飞机被已消除目标占死 571s」）----
TARGET_STALE    = float(os.environ.get("TARGET_STALE", "2.0"))    # 目标多久没位置更新就判定已删除 s
ORBIT_GIVEUP    = float(os.environ.get("ORBIT_GIVEUP", "30.0"))   # 满 15s 后官方这么久还没消除 → 放弃 s
GIVEUP_COOLDOWN = float(os.environ.get("GIVEUP_COOLDOWN", "45.0"))# 放弃后这段时间内不再自动盘旋该目标 s
CLAIM_ENABLE    = int(os.environ.get("CLAIM_ENABLE", "1"))        # 防扎堆：别机正在确认的目标不再抢
CLAIM_FRESH     = float(os.environ.get("CLAIM_FRESH", "3.0"))     # 认领消息的新鲜期 s

# ---- 确认失败退避（2026-09-28：修「单机被抖动目标锁死 600s」）----
BACKOFF_ENABLE    = int(os.environ.get("BACKOFF_ENABLE", "1"))          # 0=关闭（A/B 对照）
CONFIRM_RESET_MAX = int(os.environ.get("CONFIRM_RESET_MAX", "3"))       # 官方重置这么多次就退避
BACKOFF_COOLDOWN  = float(os.environ.get("BACKOFF_COOLDOWN", "60.0"))   # 退避时长 s
RESET_DECAY       = float(os.environ.get("RESET_DECAY", "120.0"))       # 距上次重置这么久就清零计数 s

# ---- 友机避碰 ----
FRIEND_SAFE_DIST = 10.0  # 增加到10m    # 友机安全距离 m（小于此值开始排斥）
FRIEND_K        = 2.0  # 增加排斥增益      # 排斥增益

# ---- A* 避障飞行 ----
METADATA_PATH   = os.environ.get(
    "ROBOCUP_METADATA",
    os.path.join(WS_ROOT, "src/robocup_training_worlds/worlds/generated/training_city_full_s7.json"))
INFLATE_M       = 0.5     # A* 障碍膨胀半径 m（避免贴墙规划，覆盖旋翼半径+裕度）
LOOKAHEAD       = 3.0     # 路径跟踪前瞻距离 m（> 刹停距离 v²/2a=2.25m）
STABLE_NEEDED   = 40      # EKF 稳定判定：连续多少次 20Hz 采样高度达标（40 = 2s）


def inflate_grid(grid, inflation_m):
    """对障碍物向外膨胀 inflation_m（圆盘结构元素），返回新 GridMap。

    复用团队单机避障的同名工具逻辑：A* 若直接在原始栅格规划会贴墙，
    膨胀后路径与墙面保持安全距离。
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


class SwarmAgent(object):
    def __init__(self, uav_id, model_name):
        self.uav_id = uav_id
        self.model_name = model_name

        # ---- 高度层分配（基于 ID，6m 以内）----
        # uav_1/2 -> 5.0m, uav_3/4 -> 5.5m, uav_5/6 -> 6.0m（层间距 0.5m）
        uav_num = int(uav_id.split('_')[1]) if '_' in uav_id else 1
        self.altitude_layer = ALT_BASE + ((uav_num - 1) % ALT_NLAYER) * ALT_STEP
        # 原来是 3 档 0.5m 循环，6 架里 4 架同高；改成 4 档 0.6m，配合下面 0.8m 的同层阈值
        # 保证「同层的才会互相水平避让，不同层的不会误跳过」。

        # ---- 状态 ----
        self.local_xy = None        # (x, y) MAVROS 局部坐标（轻量、高频）
        self.offset = None          # 局部->世界 的恒定平移 (dx, dy)，启动时标定一次
        self.local_z = None         # 高度（来自 local_position）
        self.state = State()
        self.assignment = None      # SearchAssignment 当前任务

        # ---- A* 避障 ----
        self.md, _ = load_metadata(METADATA_PATH)
        self.grid = inflate_grid(GridMap.from_metadata(self.md), INFLATE_M)
        self.path = []              # 当前全局路径（世界坐标航点列表）
        self.path_target = None     # 当前路径终点（格中心），用于判断是否需重规划
        self._last_plan_t = 0.0

        # ---- 覆盖栅格（与 manager 一致，10m 格）----
        self.cov_grid = CoverageGrid(MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX, GRID_SIZE_M)

        # ---- 目标检测（规则3 几何判定：距离 + 视线遮挡）----
        # 用**未膨胀**的原始栅格做 LOS 判定（膨胀是给飞行留裕度的，
        # 判定遮挡要用真实建筑轮廓，否则会把建筑边缘 0.5m 内误判为遮挡）。
        raw_grid = GridMap.from_metadata(self.md)
        self.los = LineOfSight(
            lambda ix, iy: (not raw_grid.in_bounds((ix, iy)))
                           or (not raw_grid.is_free((ix, iy))),
            cell_size=raw_grid.resolution, origin=raw_grid.origin)
        self.targets = {}           # target_id -> (x, y, vx, vy)，来自 /swarm/target_states
        self._last_detect_t = 0.0

        # ---- 目标盘旋确认 ----
        self._orbit_target = None    # 当前盘旋目标 ID
        self._last_track_goal = None # 追踪时上次 A* 的目标点（用于节流）
        self._last_track_plan_t = 0.0  # 追踪时上次 A* 规划时刻
        self._plan_fallback_n = 0  # A* 失败退化直飞的次数
        self._orbit_center = None    # 盘旋中心 (x, y)
        self._confirm_start = 0.0   # 连续确认开始时间
        self._last_confirm_t = 0.0   # 上次确认时间
        self._target_to_orbit = None  # 待盘旋目标位置 (x, y)
        self._t_seen = {}          # tid -> 最后一次收到位置的时刻（判定目标是否已消失）
        self._giveup_until = {}    # tid -> 该时刻前不再自动盘旋（放弃过 / 已消除）
        self._claims = {}          # tid -> (uav_id, 时刻) 别机正在确认的目标
        self._left_seen = False    # 是否已收到过有效的 /left_actors
        self._last_claim_t = 0.0   # 认领广播节流
        self._find_t = {}           # actor 下标 -> 官方 /find_actor_N 最近的时间戳
        self._reset_n = {}          # actor 下标 -> 已观测到的官方重置次数
        self._reset_t = {}          # actor 下标 -> 最近一次重置的时刻
        for _i in range(6):
            rospy.Subscriber('/find_actor_%d' % _i, Float32,
                             self._find_cb, callback_args=_i, queue_size=5)

        # ---- 友机避碰 ----
        self._friend_positions = {}  # 其他无人机位置 {uav_id: (x, y, z)}

        # ---- MAVROS 服务 ----
        self.arm_srv = rospy.ServiceProxy("/%s/mavros/cmd/arming" % uav_id, CommandBool)
        self.mode_srv = rospy.ServiceProxy("/%s/mavros/set_mode" % uav_id, SetMode)
        self.param_srv = rospy.ServiceProxy("/%s/mavros/param/set" % uav_id, ParamSet)

        # ---- 订阅 ----
        # 注意：**不订阅 /gazebo/model_states**。它是 250Hz × 687 模型 × 6.4MB/s 的
        # 巨型消息，每个 agent 都要用 Python 反序列化，实测每机吃掉 ~37% CPU
        # （比 PX4 还高），6 机时是灾难。改为只订阅轻量的 MAVROS local_position，
        # 启动时用一次 model_states 标定「局部->世界」的恒定平移即可。
        rospy.Subscriber("/%s/mavros/state" % uav_id, State, self._state_cb)
        # 官方剩余 actor 清单：权威的「谁还在场上」，用来剔掉已被删除的目标
        rospy.Subscriber("/left_actors", String, self._left_actors_cb, queue_size=5)
        # 别机认领广播：防止多架机扎堆确认同一个目标
        rospy.Subscriber("/swarm/orbit_claim", String, self._claim_cb, queue_size=20)
        self._claim_pub = rospy.Publisher("/swarm/orbit_claim", String, queue_size=10)
        rospy.Subscriber("/%s/mavros/local_position/pose" % uav_id, PoseStamped, self._local_cb)
        rospy.Subscriber("/swarm/assignment", SearchAssignment, self._assign_cb)
        rospy.Subscriber("/swarm/target_states", TargetState, self._target_cb)
        rospy.Subscriber("/swarm/finish", String, self._finish_cb)
        rospy.Subscriber("/swarm/uav_status", UavStatus, self._friend_status_cb)

        # ---- 发布 ----
        self.status_pub = rospy.Publisher("/swarm/uav_status", UavStatus, queue_size=5)
        self.detect_pub = rospy.Publisher("/swarm/detection", TargetDetection, queue_size=10)
        self.vel_pub = rospy.Publisher("/%s/mavros/setpoint_velocity/cmd_vel" % uav_id,
                                       TwistStamped, queue_size=5)

        self.ctrl_rate = rospy.Rate(CTRL_RATE)
        self.pub_rate = rospy.Rate(PUB_RATE)
        self._last_status_t = rospy.Time.now()
        self._mission_finished = False  # 任务完成标志
        self._landing = False  # 降落标志

        # 自适应速度状态
        self._last_heading = None  # 上次朝向
        self._last_speed = 0.0    # 上次速度大小

    @property
    def world_xy(self):
        """世界坐标 = 局部坐标 + 标定偏移（偏移未标定好前返回 None）。"""
        if self.local_xy is None or self.offset is None:
            return None
        return (self.local_xy[0] + self.offset[0], self.local_xy[1] + self.offset[1])

    def _calibrate_offset(self):
        """用一次 /gazebo/model_states 标定局部->世界的恒定平移，随后不再订阅。

        世界系（地图/metadata）与各机 MAVROS 局部系轴向平行、仅原点不同，
        故偏移是常量（实测两机分别恒定，误差 <0.01m），标定一次即可。
        """
        try:
            msg = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=20.0)
        except rospy.ROSException as exc:
            rospy.logerr("[%s] 标定失败：取不到 model_states（%s）", self.uav_id, exc)
            return False
        try:
            i = msg.name.index(self.model_name)
        except ValueError:
            rospy.logerr("[%s] 标定失败：model_states 里没有 %s", self.uav_id, self.model_name)
            return False
        # 等 local_xy 就绪
        for _ in range(200):
            if self.local_xy is not None:
                break
            self.ctrl_rate.sleep()
        if self.local_xy is None:
            rospy.logerr("[%s] 标定失败：local_position 无数据", self.uav_id)
            return False
        p = msg.pose[i].position
        self.offset = (p.x - self.local_xy[0], p.y - self.local_xy[1])
        rospy.loginfo("[%s] 坐标系标定完成：offset=(%.2f, %.2f)", self.uav_id,
                      self.offset[0], self.offset[1])
        return True

    # ---------------- 回调 ----------------
    def _state_cb(self, msg):
        self.state = msg

    def _local_cb(self, msg):
        self.local_z = msg.pose.position.z
        self.local_xy = (msg.pose.position.x, msg.pose.position.y)

    def _assign_cb(self, msg):
        if msg.uav_id != self.uav_id:
            return
        self.assignment = msg
        # 处理特殊任务类型
        if msg.task_type == 1:  # 目标确认/追踪
            rospy.loginfo("[%s] 收到目标追踪任务 %s @ (%.1f, %.1f)",
                          self.uav_id, msg.target_id if hasattr(msg, 'target_id') else '?',
                          msg.target_x, msg.target_y)
            # 直接设置盘旋目标（让 agent 飞过去盘旋确认）
            # 这里暂时用 target_x/y 作为目标位置
            self._orbit_target = None  # 让 _control 发现目标后自动盘旋
            # 或者直接设目标让它飞过去
            self._target_to_orbit = (msg.target_x, msg.target_y)
        elif msg.task_type == 2:  # RTL 返航
            rospy.loginfo("[%s] 收到 RTL 返航指令", self.uav_id)
            self.mode_srv.call(0, "RTL")  # 切换到 RTL 模式
        elif msg.task_type == 3:  # 降落
            rospy.loginfo("[%s] 收到降落指令", self.uav_id)
            self._landing = True  # 进入降落模式

    def _target_cb(self, msg):
        """缓存恐怖分子真值位置（几何判定用；真机上是视觉/裁判给出的观测）。"""
        if msg.eliminated:
            self.targets.pop(msg.target_id, None)
            # BUGFIX: 目标消除后必须清除盘旋状态，否则飞机会卡在盘旋不动
            if self._orbit_target == msg.target_id:
                rospy.loginfo("[%s] 目标 %s 已消除，清除盘旋状态", self.uav_id, msg.target_id)
                self._orbit_target = None
                self._orbit_center = None
                self._confirm_start = 0.0
            return
        self.targets[msg.target_id] = (msg.x, msg.y, msg.vx, msg.vy)
        self._t_seen[msg.target_id] = rospy.Time.now().to_sec()

    def _friend_status_cb(self, msg):
        """接收友机位置和高度，用于避碰"""
        if msg.uav_id != self.uav_id:
            self._friend_positions[msg.uav_id] = (msg.x, msg.y, msg.z)

    def _finish_cb(self, msg):
        """收到任务完成广播后退出搜索循环。"""
        if msg.data == "MISSION_FINISHED":
            rospy.loginfo("[%s] 收到任务完成广播", self.uav_id)
            self._mission_finished = True
            # 兜底：如果还没进入降落模式，则进入
            if not self._landing and self.assignment is not None and getattr(self.assignment, 'task_type', 0) == 3:
                self._landing = True

    # ---------------- 目标检测（规则3：几何判定） ----------------
    def _detect_targets(self):
        """对每个已知目标做「距离 + 视线遮挡」判定，命中则发布 TargetDetection。

        规则3 的几何判定：水平距离 < DETECT_RADIUS **且** 中间无建筑遮挡。
        注意只用水平距离 —— 无人机在 6m 高度、目标在地面，垂直差恒定，
        水平距才是决定「能否看到」的量（与比赛判定的平面几何一致）。
        """
        wx = self.world_xy
        if wx is None:
            return
        now = rospy.Time.now().to_sec()
        if now - self._last_detect_t < 1.0 / DETECT_RATE:
            return
        self._last_detect_t = now

        for tid, (tx, ty, _vx, _vy) in list(self.targets.items()):
            d = math.hypot(tx - wx[0], ty - wx[1])
            if d > DETECT_RADIUS:
                continue
            if not self.los.visible(wx[0], wx[1], tx, ty):
                continue    # 隔着建筑，不算看到
            m = TargetDetection()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = "map"
            m.uav_id = self.uav_id
            m.target_id = tid
            m.x, m.y = tx, ty
            m.confidence = 1.0
            m.source = 0     # 0=几何判定
            self.detect_pub.publish(m)

    # ---------------- 参数与模式 ----------------
    def _set_param(self, param_id, value):
        from mavros_msgs.msg import ParamValue
        try:
            return self.param_srv(param_id, ParamValue(integer=value, real=0.0)).success
        except Exception as exc:
            rospy.logwarn_throttle(10, "[%s] 设置 %s 异常: %s", self.uav_id, param_id, exc)
            return False

    def _configure_fcu(self):
        """SITL 无遥控器会触发 RC 失联 failsafe，拒绝解锁/进 OFFBOARD。"""
        self._set_param("NAV_RCL_ACT", 0)
        self._set_param("COM_RCL_EXCEPT", 4)

    def _arm_and_offboard(self):
        """预热 setpoint → 切 OFFBOARD → 解锁。与单机避障已验证的时序一致。

        多机 SITL 注意：第二架及以后的机 EKF 收敛更慢，local_position 可能
        出现 -12m 之类的瞬时坏值，直接切 OFFBOARD 会失败。故先等 EKF 收敛
        （local_z 就绪且 |z| 合理），OFFBOARD 切换失败则重试而非直接放弃。

        关键：预热阶段必须发「纯零速」(vz=0)，而非 _send_vel 的竖直爬升速度。
        单机脚本 dwa_avoidance 预热发 _send_vel(0,0,0)，若此处带 vz=0.5*(6-z)
        的爬升速度，EKF 未收敛时 PX4 会拒绝 OFFBOARD。
        """
        # 等 EKF 大致稳定：要求「连续 STABLE_NEEDED 次采样」高度在出生点附近。
        # 单次采样不够 —— 多机 SITL 下 EKF 位置先收敛、速度/姿态后稳定，瞬时达标
        # 就切 OFFBOARD 会被 PX4 拒。阈值放宽到 1.5m（地面上有噪声抖动），真正
        # 是否就绪交给 PX4 preflight 判定，靠后续多次重试兜底。

        # 重启协同层时飞机已在空中：跳过等待直接进 OFFBOARD
        if self.state.armed and self.state.mode == "OFFBOARD" and self.local_z is not None and self.local_z > 1.0:
            rospy.loginfo("[%s] 已在空中 OFFBOARD（z=%.2f）→ 跳过起飞等待",
                          self.uav_id, self.local_z)
            return True

        stable = 0
        while not rospy.is_shutdown():
            if self.local_z is not None and abs(self.local_z) < 1.5:
                stable += 1
                if stable >= STABLE_NEEDED:
                    break
            else:
                if stable > 0:
                    rospy.logwarn_throttle(5, "[%s] EKF 抖动（z=%.2f），重新计数",
                                           self.uav_id,
                                           self.local_z if self.local_z is not None else -999)
                stable = 0
            self._send_vel(0.0, 0.0, vz=0.0)
            self.ctrl_rate.sleep()
        rospy.loginfo("[%s] EKF 已稳定（z=%.2f，连续 %d 次采样达标）", self.uav_id,
                      self.local_z if self.local_z is not None else -999, stable)

        # 预热：持续发纯零速，让 OFFBOARD setpoint 生效（vz=0 而非爬升）
        for _ in range(120):
            self._send_vel(0.0, 0.0, vz=0.0)
            self.ctrl_rate.sleep()

        # 切 OFFBOARD：**持续重试直到成功**（多机 SITL 下 EKF 速度/姿态估计就绪
        # 时间不定，固定次数会误判放弃，导致必须重启整个 SITL。永不放弃 + 每次
        # 重试间隔继续发零速预热，EKF 稳了自然就进得去。）
        attempt = 0
        while not rospy.is_shutdown() and self.state.mode != "OFFBOARD":
            attempt += 1
            if not self.mode_srv(0, "OFFBOARD").mode_sent:
                rospy.logwarn_throttle(10, "[%s] 切换 OFFBOARD 请求失败（第 %d 次）",
                                       self.uav_id, attempt)
            for _ in range(50):
                if self.state.mode == "OFFBOARD":
                    break
                self._send_vel(0.0, 0.0, vz=0.0)
                self.ctrl_rate.sleep()
            if self.state.mode == "OFFBOARD":
                break
            rospy.logwarn_throttle(10, "[%s] OFFBOARD 未生效（第 %d 次，当前 %s），等待 EKF 稳定后重试",
                                   self.uav_id, attempt, self.state.mode)
            for _ in range(40):
                if self.state.mode == "OFFBOARD":
                    break
                self._send_vel(0.0, 0.0, vz=0.0)
                self.ctrl_rate.sleep()
        if rospy.is_shutdown():
            return False
        rospy.loginfo("[%s] 已进入 OFFBOARD（重试 %d 次）", self.uav_id, attempt)

        # 解锁：同样**持续重试直到成功**。PX4 preflight 会因 EKF 速度估计未稳 /
        # Roll failure 拒解锁，等 EKF 稳了自然会成功。
        attempt = 0
        while not rospy.is_shutdown() and not self.state.armed:
            attempt += 1
            if not self.arm_srv(True).success:
                rospy.logwarn_throttle(10, "[%s] 解锁请求失败（第 %d 次），等待 EKF 稳定后重试",
                                       self.uav_id, attempt)
            for _ in range(50):
                if self.state.armed:
                    break
                self._send_vel(0.0, 0.0, vz=0.0)
                self.ctrl_rate.sleep()
        if rospy.is_shutdown():
            return False
        rospy.loginfo("[%s] OFFBOARD + 解锁完成（解锁重试 %d 次）", self.uav_id, attempt)
        return True

    # ---------------- 控制 ----------------
    def _compute_desired_altitude(self):
        """根据当前状态自适应计算目标高度的偏移量。

        基础高度由 altitude_layer 决定（5.0/5.5/6.0m），
        这里返回相对于基础高度的偏移量（不超过 6m 上限）。
        策略：
        - 追踪目标时：-1.0m（更好地观察）
        - 建筑附近：-0.5m（保持距离）
        - 开阔区域：0m（常规搜索）
        """
        wx, wy = self.world_xy
        if wx is None:
            return 0.0

        # 检查是否在追踪目标
        if self._orbit_target is not None:
            return -1.0

        # 检查是否在建筑附近（使用未膨胀的原始栅格）
        if hasattr(self, 'los') and self.los:
            # 简单判断：当前位置是否靠近障碍
            # 用 A* 膨胀后的栅格判断
            cell = self.grid.world_to_cell((wx, wy))
            if cell and not self.grid.is_free(cell):
                # 已经在障碍附近，降低高度
                return -0.5

        # 检查与最近建筑的距离
        # 简化：用 coverage grid 中心判断
        if hasattr(self, 'cov_grid') and self.cov_grid:
            cov_cell = self.cov_grid.world_to_cell(wx, wy)
            if cov_cell:
                cx = self.cov_grid.x_min + (cov_cell[0] + 0.5) * self.cov_grid.cell_m
                cy = self.cov_grid.y_min + (cov_cell[1] + 0.5) * self.cov_grid.cell_m
                # 检查这个位置是否在建筑附近
                # 简化：使用 A* 栅格判断
                astar_cell = self.grid.world_to_cell((wx, wy))
                if astar_cell and not self.grid.is_free(astar_cell):
                    return -0.5

        # 开阔区域
        return 0.0

    def _send_vel(self, vx, vy, vz=None):
        """下发 ENU 水平速度 + 自适应高度。

        vz=None 时按当前高度竖直 P 控制爬升到自适应高度；预热/悬停阶段显式传
        vz=0.0 发纯零速，避免 EKF 未收敛时带爬升速度导致 OFFBOARD 被拒。
        """
        # === 2026-09-27：水平加速度限幅 ===
        # 避让增益提高后，ORCA 输出可能在相邻帧跳到近乎反向（22:29 轮事故：
        # iris_3 因此把高度超调从 0.2m 放大到 1.5m，冲过官方 6m 红线，score=0）。
        # 这里把「指令加速度」钉死，PX4 位置环才有能力跟踪，机身不再大幅倾斜。
        if not hasattr(self, "_last_cmd_v"):
            self._last_cmd_v = None
        if self._last_cmd_v is not None:
            _lvx, _lvy = self._last_cmd_v
            _maxdv = MAX_ACC / CTRL_RATE
            _dvx, _dvy = vx - _lvx, vy - _lvy
            _dvn = math.hypot(_dvx, _dvy)
            if _dvn > _maxdv and _dvn > 1e-9:
                vx = _lvx + _dvx * _maxdv / _dvn
                vy = _lvy + _dvy * _maxdv / _dvn
        self._last_cmd_v = (vx, vy)

        # === 高度最后防线（加速度限幅之后再裁，避免被限幅抵消）===
        _zn = self.local_z
        if _zn is not None and _zn > ALT_PANIC:
            vx = vx * ALT_PANIC_HSCALE
            vy = vy * ALT_PANIC_HSCALE

        # 计算目标高度 = ID对应基础高度 + 场景偏移（不超过 6m 上限）
        base_alt = self.altitude_layer
        alt_offset = self._compute_desired_altitude()  # 返回偏移量
        # === 2026-09-29 P1-1：目标高度必须低于硬顶，否则极限环 ===
        # ALT_HARD_CEIL 默认 4.2，而 ALT_TARGET_CAP=4.5 / 顶层目标 4.6，
        # 目标高于硬顶 -> 爬上去被压回 -> 再爬 -> uav_4 高度极限环。
        _alt_cap = min(ALT_TARGET_CAP, ALT_HARD_CEIL - ALT_HARD_MARGIN)
        if _alt_cap < MIN_CRUISE_ALT:
            _alt_cap = MIN_CRUISE_ALT
        target_alt = max(MIN_CRUISE_ALT, min(_alt_cap, base_alt + alt_offset))
        # 上限 4.5（远低于官方 6m），下限 MIN_CRUISE_ALT 防止追踪降高过低

        cmd = TwistStamped()
        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        # 纯速度模式：vz 语义是「保持当前高度」，这里给竖直 P 控制爬升到自适应高度
        if vz is not None:
            cmd.twist.linear.z = vz
        elif self.local_z is not None:
            # 自适应高度控制
            alt_error = target_alt - self.local_z
            cmd.twist.linear.z = 0.5 * alt_error  # P 控制
            # 限制升降速度
            if cmd.twist.linear.z > 1.0:
                cmd.twist.linear.z = 1.0
            if cmd.twist.linear.z < -1.0:
                cmd.twist.linear.z = -1.0
        # === 高度硬护栏 ===
        # 官方 score_cal.py 判定 z > 6.0 直接 score=0 并 _finish() 终止任务，
        # 是一票否决。不能只依赖 P 控制不超调，这里无条件兜底强制下降。
        if self.local_z is not None and self.local_z > ALT_HARD_CEIL:
            cmd.twist.linear.z = ALT_HARD_DESCENT
            _alt_guard_hit('HARD', self.local_z, ALT_HARD_CEIL, ALT_HARD_DESCENT,
                           cmd.twist.linear.x, cmd.twist.linear.y)
        if self.local_z is not None and self.local_z > ALT_PANIC:
            cmd.twist.linear.z = ALT_PANIC_DESCENT
            _alt_guard_hit('PANIC', self.local_z, ALT_PANIC, ALT_PANIC_DESCENT,
                           cmd.twist.linear.x, cmd.twist.linear.y)
        # === 6m 红线二次保险：无条件覆盖，放在 publish 前最后一步 ===
        if self.local_z is not None and self.local_z > ALT_EMERG_CEIL:
            cmd.twist.linear.z = ALT_EMERG_DESCENT
            _alt_guard_hit('EMERG', self.local_z, ALT_EMERG_CEIL, ALT_EMERG_DESCENT,
                           cmd.twist.linear.x, cmd.twist.linear.y)
            cmd.twist.linear.x = cmd.twist.linear.x * ALT_EMERG_HSCALE
            cmd.twist.linear.y = cmd.twist.linear.y * ALT_EMERG_HSCALE
        # === 2026-09-29 P1-3：坠地检测 ===
        # 原来没有任何姿态/坠机检测，坠机后 landed_state 仍报 IN_AIR，系统完全不知情。
        # 这里不新增订阅，用「实际高度持续远低于目标 + 几乎无水平运动」判定。
        if CRASH_DETECT and self.local_z is not None:
            try:
                _drop = target_alt - self.local_z
            except Exception:
                _drop = 0.0
            if _drop > CRASH_DROP_M and math.hypot(vx, vy) < CRASH_V_EPS:
                self._crash_cnt = getattr(self, '_crash_cnt', 0) + 1
            else:
                self._crash_cnt = 0
            if (self._crash_cnt >= int(CRASH_HOLD_S * CTRL_RATE)
                    and not getattr(self, '_crash_flagged', False)):
                self._crash_flagged = True
                try:
                    rospy.logerr('[CRASH] %s 疑似坠地 z=%.2f target=%.2f v=%.2f '
                                 '(landed_state 仍可能报 IN_AIR)',
                                 getattr(self, 'uav_id', getattr(self, 'ns', '?')),
                                 self.local_z, target_alt, math.hypot(vx, vy))
                except Exception:
                    pass
        self.vel_pub.publish(cmd)

    def _need_replan_track(self, goal):
        """追踪移动目标时的 A* 重规划节流。

        原实现每个控制循环都重跑全图 A*（目标一移动就整条 replan），实测把控制
        频率从 20Hz 拖到 5.5Hz，且日志刷屏（单架曾累计 10020 次规划）。
        这里改成：无路径 / 目标移动超阈值 / 距上次规划超时，三者满足其一才重规划。
        """
        if not self.path:
            return True
        if self._last_track_goal is None:
            return True
        moved = math.hypot(goal[0] - self._last_track_goal[0],
                           goal[1] - self._last_track_goal[1])
        if moved >= TRACK_REPLAN_MOVE:
            return True
        if (rospy.Time.now().to_sec() - self._last_track_plan_t) >= TRACK_REPLAN_SEC:
            return True
        return False

    def _plan_path(self, goal_xy):
        """从当前世界坐标 A* 规划到 goal_xy，绕开膨胀后的建筑障碍。"""
        if self.world_xy is None:
            return False
        route = plan(self.grid, self.world_xy, goal_xy, connectivity=8)
        if not route.success and route.reason == "START_OCCUPIED":
            # 起点落在膨胀障碍内（刚起飞/贴墙）→ 找最近自由栅格重试
            nearest = self._nearest_free_cell(goal_xy)
            if nearest is not None:
                route = plan(self.grid, nearest, goal_xy, connectivity=8)
        if not route.success and PLAN_FALLBACK:
            # 2026-09-28：START_OUT_OF_BOUNDS 会让飞机永久停摆。实测 uav_1 被推到
            # y≈69（A* 栅格 y_max=65 之外）后每 2s 规划失败一次、速度指令全程 0，
            # 整轮只飞 10m；6 架里 4 架这样趴着 → 全机均速 0.47 m/s、扫描率只有
            # 理论值的 1/8。越界/无解也要动起来：先把起点拉回最近自由栅格重试。
            nearest = self._nearest_free_cell(goal_xy)
            if nearest is not None:
                route = plan(self.grid, nearest, goal_xy, connectivity=8)
        if not route.success and PLAN_FALLBACK and self.world_xy is not None:
            # 还不行就退化成直飞航点串。可能穿楼，但原地不动是 100% 无收益，
            # 而且穿楼前还过一道建筑避障 + 高度护栏。
            x0, y0 = self.world_xy
            dx, dy = goal_xy[0] - x0, goal_xy[1] - y0
            d = math.hypot(dx, dy)
            n = max(2, int(d / 3.0))
            self.path = [(x0 + dx * (i + 1) / float(n),
                          y0 + dy * (i + 1) / float(n)) for i in range(n)]
            self.path_target = goal_xy
            self._plan_fallback_n += 1
            rospy.logwarn_throttle(10, "[%s] A* 失败(%s) → 直飞 (%.1f,%.1f)，累计 %d 次",
                                   self.uav_id, route.reason, goal_xy[0], goal_xy[1],
                                   self._plan_fallback_n)
            return True
        if not route.success:
            rospy.logwarn("[%s] A* 规划到 (%.1f,%.1f) 失败: %s",
                          self.uav_id, goal_xy[0], goal_xy[1], route.reason)
            return False
        self.path = smooth_path(route.points, samples_per_segment=6)
        self.path_target = goal_xy
        rospy.loginfo("[%s] A* 规划：%d 航点 -> %d 平滑点",
                      self.uav_id, len(route.points), len(self.path))
        return True

    def _nearest_free_cell(self, goal_xy):
        """起点被占时，找当前坐标附近最近自由栅格（局部搜索，避免全地图 O(w*h) 扫描）。"""
        if self.world_xy is None:
            return None
        cx, cy = self.world_xy
        # 以当前坐标为圆心，半径 5m 内找自由栅格（膨胀后建筑间距 > 5m）
        radius_cells = int(math.ceil(5.0 / self.grid.resolution))
        center_cell = self.grid.world_to_cell((cx, cy))
        if center_cell is None:
            return None
        best, best_d = None, float("inf")
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                cell = (center_cell[0] + dx, center_cell[1] + dy)
                if not self.grid.is_free(cell):
                    continue
                wx, wy = self.grid.cell_to_world(cell)
                d = math.hypot(wx - cx, wy - cy)
                if d < best_d:
                    best_d = d
                    best = (wx, wy)
        return best

    def _pick_local_goal(self):
        """沿全局路径从最近点往前取 LOOKAHEAD 距离的引导目标。"""
        if not self.path or self.world_xy is None:
            return None
        cx, cy = self.world_xy
        best = 0
        best_dist = float("inf")
        for i, (px, py) in enumerate(self.path):
            d = math.hypot(px - cx, py - cy)
            if d < best_dist:
                best_dist = d
                best = i
        acc = 0.0
        for i in range(best, len(self.path) - 1):
            x0, y0 = self.path[i]
            x1, y1 = self.path[i + 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            if acc + seg >= LOOKAHEAD:
                frac = (LOOKAHEAD - acc) / max(seg, 1e-6)
                return (x0 + (x1 - x0) * frac, y0 + (y1 - y0) * frac)
            acc += seg
        return self.path[-1]

    def _control(self):
        """有任务 → A* 绕障飞向格中心；无任务 → 原地悬停。"""
        # === 降落模式 ===
        if self._landing:
            if self.local_z is not None and self.local_z < 0.3:
                # 触地，解锁
                rospy.loginfo("[%s] 已触地，解锁", self.uav_id)
                try:
                    self.arm_srv(False)
                except Exception as e:
                    rospy.logwarn("[%s] 解锁失败: %s", self.uav_id, e)
                self._landing = False
                return
            else:
                # 持续发 vz = -1.0 下降，不依赖模式切换
                self._send_vel(0.0, 0.0, vz=-1.0)
                return

        if self.world_xy is None:
            self._send_vel(0.0, 0.0)
            return

        if self.assignment is None:
            self.path = []
            self.path_target = None
            self._orbit_target = None
            self._target_to_orbit = None
            self._send_vel(0.0, 0.0)
            return

        # === 任务类型处理 ===
        task_type = getattr(self.assignment, 'task_type', 0)

        # 目标追踪任务（task_type=1）：飞向目标位置
        if task_type == 1 and self._target_to_orbit is not None:
            tx, ty = self._target_to_orbit
            dist = math.hypot(tx - self.world_xy[0], ty - self.world_xy[1])
            if dist < ORBIT_RADIUS:
                # 到达目标附近，开始盘旋
                self._orbit_center = (tx, ty)
                # 存真实 target_id
                target_id = getattr(self.assignment, 'target_id', None)
                self._orbit_target = target_id if target_id else "tracking"
                self._confirm_start = rospy.Time.now().to_sec()
                self._last_confirm_t = self._confirm_start
                # 不要清除 _target_to_orbit，_fly_orbit 需要用
                self._fly_orbit()
                return
            else:
                # 飞向目标（移动目标：A* 重规划节流）
                if self._need_replan_track((tx, ty)):
                    if not self._plan_path((tx, ty)):
                        self._send_vel(0.0, 0.0)
                        return
                    self._last_track_goal = (tx, ty)
                    self._last_track_plan_t = rospy.Time.now().to_sec()
                local_goal = self._pick_local_goal()
                if local_goal is None:
                    self._send_vel(0.0, 0.0)
                    return
                err_x = local_goal[0] - self.world_xy[0]
                err_y = local_goal[1] - self.world_xy[1]
                vx = POS_KP * err_x
                vy = POS_KP * err_y
                spd = math.hypot(vx, vy)
                # 接近 actor 时压速，避免触发官方逃跑机制（详见 SPOOK_SPEED 注释）
                cap = SPOOK_SPEED if dist < SPOOK_DIST else MAX_SPEED
                if spd > cap:
                    vx *= cap / spd
                    vy *= cap / spd
                vx, vy = self._apply_friend_avoidance(vx, vy)
                self._send_vel(vx, vy)
                return

        # === 搜索任务（task_type=0）===
        # 检查是否在盘旋，以及是否能看到目标
        self._update_orbit()
        if self._orbit_target is not None:
            # 正在盘旋确认，执行盘旋飞行
            self._fly_orbit()
            return

        goal = (self.assignment.target_x, self.assignment.target_y)
        dist = math.hypot(goal[0] - self.world_xy[0], goal[1] - self.world_xy[1])

        if dist < ARRIVE_TOL:
            # 已到达格中心，检查是否有可盘旋的目标
            self.path = []
            self._check_start_orbit()
            if self._orbit_target is not None:
                self._fly_orbit()
            else:
                self._send_vel(0.0, 0.0)
            return

        # 目标变了或还没有路径 → 重新 A* 规划（失败时 2s 内不重复尝试，避免刷屏）
        if self.path_target is None or self.path_target != goal or not self.path:
            now = rospy.Time.now().to_sec()
            if now - self._last_plan_t >= 2.0:
                self._last_plan_t = now
                if not self._plan_path(goal):
                    # 规划失败（无路可达）→ 原地悬停，等下一次分配
                    self._send_vel(0.0, 0.0)
                    return
            else:
                # 刚失败过，还在冷却期，悬停等待
                self._send_vel(0.0, 0.0)
                return

        # 沿路径跟踪：取 lookahead 引导点，P 控制 + 限速
        local_goal = self._pick_local_goal()
        if local_goal is None:
            self._send_vel(0.0, 0.0)
            return
        err_x = local_goal[0] - self.world_xy[0]
        err_y = local_goal[1] - self.world_xy[1]
        vx = POS_KP * err_x
        vy = POS_KP * err_y
        spd = math.hypot(vx, vy)
        if spd > MAX_SPEED:
            vx *= MAX_SPEED / spd
            vy *= MAX_SPEED / spd

        # === 友机避碰 ===
        vx, vy = self._apply_friend_avoidance(vx, vy)

        self._send_vel(vx, vy)

    # ---- 已消除目标清理 / 盘旋放弃 / 防扎堆（2026-09-27 新增）----
    def _left_actors_cb(self, msg):
        """官方 /left_actors：不在清单里的 actor = 已被官方确认并删除。

        这是「飞机被已消除目标占死」的权威兜底。official_target_bridge 只转发
        model_states，actor 被 del_model 后它就不再发该目标，而它发的
        TargetState.eliminated 恒为 False —— 于是 agent 的 self.targets 永不清理，
        _update_orbit 的「目标已消失」分支永远不触发，飞机对着一个已经不存在的
        actor 盘旋到超时（实测 uav_5/uav_6 对 t1 盘旋 571.3s，4/6 架机全程空转）。
        """
        ids = set(int(x) for x in re.findall(r'-?\d+', str(msg.data)))
        if not ids:
            return                      # 空清单不采信（可能只是还没发布 / 任务已结束）
        self._left_seen = True
        for tid in list(self.targets.keys()):
            m = re.match(r'^t(\d+)$', str(tid))
            if m is None:
                continue
            if int(m.group(1)) not in ids:
                self.targets.pop(tid, None)
                self._t_seen.pop(tid, None)
                self._giveup_until[tid] = float('inf')   # 已消除，永不追
                if self._orbit_target == tid:
                    self._abort_orbit('%s 已被官方消除，回归搜索' % tid)

    def _abort_orbit(self, reason):
        """无条件退出盘旋并回到搜索分支。

        关键：必须清空 _target_to_orbit，否则 _control 里
        `if task_type == 1 and self._target_to_orbit is not None` 仍然成立，
        飞机又会飞回那个已经不存在的目标点。
        """
        tid = self._orbit_target
        self._orbit_target = None
        self._orbit_center = None
        self._confirm_start = 0.0
        self._target_to_orbit = None
        self.path = []
        self.path_target = None
        rospy.logwarn('[%s] 放弃盘旋 %s：%s', self.uav_id, tid, reason)

    def _claim_cb(self, msg):
        """别机广播「我正在确认 tid」。"""
        try:
            uid, tid = str(msg.data).split(':', 1)
        except ValueError:
            return
        if uid != self.uav_id:
            self._claims[tid] = (uid, rospy.Time.now().to_sec())

    def _find_cb(self, msg, actor_idx):
        """官方 /find_actor_N：时间戳每变一次 = 官方刚刚重置了一次 15s 确认。

        这是团队侧唯一能观测到「确认又断了」的权威信号。官方 _reset_detection()
        把 count_flag 清零、find_time 归零，下一次有效上报又会
        count_flag False->True 并重新 publish 一个新的时间戳。
        """
        t = float(msg.data)
        old = self._find_t.get(actor_idx)
        if old is not None and abs(t - old) < 1e-6:
            return
        self._find_t[actor_idx] = t
        if old is None:
            return                          # 首次发现，不算重置
        self._reset_n[actor_idx] = self._reset_n.get(actor_idx, 0) + 1
        self._reset_t[actor_idx] = rospy.Time.now().to_sec()
        tid = 't%d' % actor_idx
        rospy.logwarn('[%s] 官方重置了 %s 的确认（第 %d 次）',
                      self.uav_id, tid, self._reset_n[actor_idx])
        if self._orbit_target != tid:
            return
        # 镜像官方窗口：把自己的确认计时也归零，ORBIT_GIVEUP 的语义才成立
        self._confirm_start = rospy.Time.now().to_sec()
        self._last_confirm_t = self._confirm_start
        if BACKOFF_ENABLE and self._reset_n[actor_idx] >= CONFIRM_RESET_MAX:
            self._giveup_until[tid] = rospy.Time.now().to_sec() + BACKOFF_COOLDOWN
            self._abort_orbit('官方已重置 %d 次确认，退避 %.0fs 去搜别处'
                              % (self._reset_n[actor_idx], BACKOFF_COOLDOWN))

    def _publish_claim(self):
        """每 0.5s 广播一次自己正在确认的目标，供别机避让（防多机扎堆）。"""
        if not CLAIM_ENABLE or self._orbit_target is None:
            return
        now = rospy.Time.now().to_sec()
        if now - self._last_claim_t < 0.5:
            return
        self._last_claim_t = now
        try:
            self._claim_pub.publish('%s:%s' % (self.uav_id, self._orbit_target))
        except Exception:
            pass

    def _claimed_by_other(self, tid):
        """返回正在确认 tid 的别机 id（无人认领或已过期则返回 None）。"""
        if not CLAIM_ENABLE:
            return None
        ent = self._claims.get(tid)
        if ent is None:
            return None
        uid, t = ent
        if rospy.Time.now().to_sec() - t > CLAIM_FRESH:
            return None
        return uid

    def _orbit_stale(self):
        """当前盘旋目标是否已经没有位置更新（多半已被官方删除）。"""
        tid = self._orbit_target
        if tid is None:
            return False
        t_seen = self._t_seen.get(tid, 0.0)
        if not t_seen:
            return False
        return (rospy.Time.now().to_sec() - t_seen) > TARGET_STALE

    def _update_orbit(self):
        """更新盘旋状态：检查目标是否可见，更新确认时间"""
        if self._orbit_target is None:
            return

        wx = self.world_xy
        if wx is None:
            return

        tid = self._orbit_target
        if tid not in self.targets:
            # 目标已消失，停止盘旋
            self._orbit_target = None
            self._orbit_center = None
            return

        tx, ty, _, _ = self.targets[tid]
        dist = math.hypot(tx - wx[0], ty - wx[1])

        now = rospy.Time.now().to_sec()
        # 检查是否能看到目标（距离 + LOS）
        if dist < DETECT_RADIUS and self.los.visible(wx[0], wx[1], tx, ty):
            if self._confirm_start == 0.0:
                self._confirm_start = now
            self._last_confirm_t = now
        else:
            # 看不到目标，重置确认
            self._confirm_start = 0.0

    def _check_start_orbit(self):
        """检查是否需要开始盘旋（发现可确认的目标）"""
        wx = self.world_xy
        if wx is None:
            return

        for tid, (tx, ty, _, _) in self.targets.items():
            dist = math.hypot(tx - wx[0], ty - wx[1])
            # 在检测范围内且 LOS 可见
            if dist >= DETECT_RADIUS or not self.los.visible(wx[0], wx[1], tx, ty):
                continue
            # 放弃冷却期内不再自动追（刚放弃过的目标，别立刻又贴回去）
            if rospy.Time.now().to_sec() < self._giveup_until.get(tid, 0.0):
                continue
            # 距上次官方重置已久 → 清零计数（误差是时变的，回来值得再试一次）
            _m = re.match(r'^t(\d+)$', str(tid))
            if _m is not None:
                _i = int(_m.group(1))
                if rospy.Time.now().to_sec() - self._reset_t.get(_i, 0.0) > RESET_DECAY:
                    self._reset_n[_i] = 0
            # 别机已经在确认这个目标 → 不扎堆，继续搜自己的格
            _other = self._claimed_by_other(tid)
            if _other is not None:
                continue
            # 开始盘旋确认
            self._orbit_target = tid
            self._orbit_center = (tx, ty)
            self._confirm_start = rospy.Time.now().to_sec()
            self._last_confirm_t = self._confirm_start
            rospy.loginfo("[%s] 开始盘旋确认目标 %s", self.uav_id, tid)
            break

    def _fly_orbit(self):
        """执行盘旋飞行：绕目标做圆周运动"""
        if self._orbit_target is None:
            self._send_vel(0.0, 0.0)
            return

        wx, wy = self.world_xy
        if wx is None:
            return

        # 目标长时间没有位置更新 = 已被官方删除（bridge 只转发还存在的 actor）。
        # 不判这一条，飞机就会对着一个不存在的 actor 一直盘旋到任务超时。
        if self._orbit_stale():
            tid0 = self._orbit_target
            gap = rospy.Time.now().to_sec() - self._t_seen.get(tid0, 0.0)
            self.targets.pop(tid0, None)
            self._giveup_until[tid0] = float('inf')
            self._abort_orbit('已 %.1fs 无位置更新，判定已消除' % gap)
            return

        # 用最新目标位置更新盘旋中心
        if self._orbit_target == "tracking" and self._target_to_orbit is not None:
            tx, ty = self._target_to_orbit
        elif self._orbit_target in self.targets:
            # 从已知目标获取最新位置
            tx, ty, _, _ = self.targets[self._orbit_target]
        else:
            # 无目标位置，原地悬停
            self._send_vel(0.0, 0.0)
            return

        self._orbit_center = (tx, ty)
        now = rospy.Time.now().to_sec()

        # 计算当前相对于目标的角度
        angle = math.atan2(wy - ty, wx - tx)

        # 更新角度（顺时针盘旋）
        dt = 1.0 / CTRL_RATE
        angle += ORBIT_SPEED * dt
        if angle > math.pi:
            angle -= 2 * math.pi

        # 目标位置
        target_x = tx + ORBIT_RADIUS * math.cos(angle)
        target_y = ty + ORBIT_RADIUS * math.sin(angle)

        # P 控制飞向盘旋点
        err_x = target_x - wx
        err_y = target_y - wy
        vx = POS_KP * err_x
        vy = POS_KP * err_y
        spd = math.hypot(vx, vy)
        if spd > MAX_SPEED:
            vx *= MAX_SPEED / spd
            vy *= MAX_SPEED / spd

        # 友机避碰
        vx, vy = self._apply_friend_avoidance(vx, vy)

        self._send_vel(vx, vy)

        # 检查确认时间
        confirm_duration = now - self._confirm_start
        self._publish_claim()

        if confirm_duration >= CONFIRM_TIME:
            rospy.loginfo("[%s] 目标 %s 已连续确认 %.1fs，可消除", self.uav_id, self._orbit_target, confirm_duration)
            # 团队侧已满足 15s，官方却迟迟没消除 = 官方链路断了（坐标过期 / 遮挡 / 它在逃跑）。
            # 继续死等只会把飞机锁死（实测 571s），不如放弃、让 manager 重新派活。
            if confirm_duration >= ORBIT_GIVEUP:
                self._giveup_until[self._orbit_target] = (
                    rospy.Time.now().to_sec() + GIVEUP_COOLDOWN)
                self._abort_orbit('确认 %.0fs 官方仍未消除，放弃（%.0fs 内不再追）'
                                  % (confirm_duration, GIVEUP_COOLDOWN))

    def _adaptive_speed(self, vx, vy):
        """根据场景自适应速度：搜索快、确认慢、避障稳"""
        # 基础速度
        base_speed = MAX_SPEED

        # 1. 追踪目标时降速（确认需要稳定）
        if self._orbit_target is not None:
            base_speed = 1.5
        # 2. ORCA 激活时（友机近）降速
        elif self._friend_positions:
            # 检查是否有近距友机
            wx, wy = self.world_xy
            for fid, (fx, fy, fz) in self._friend_positions.items():
                dist = math.hypot(fx - wx, fy - wy)
                if dist < SLOWDOWN_DIST:
                    base_speed = SLOW_SPEED
                    break
        # 3. 大幅转向时降速（防止超调）
        if vx != 0 or vy != 0:
            current_heading = math.atan2(vy, vx)
            if self._last_heading is not None:
                heading_diff = abs(current_heading - self._last_heading)
                # 处理角度跳变（-pi 到 pi）
                if heading_diff > math.pi:
                    heading_diff = 2 * math.pi - heading_diff
                if heading_diff > 0.5:  # > 30度
                    base_speed = min(base_speed, 3.0)
            self._last_heading = current_heading

        # 应用自适应速度
        current_speed = math.hypot(vx, vy)
        if current_speed > base_speed:
            scale = base_speed / current_speed if current_speed > 0 else 1
            vx *= scale
            vy *= scale

        return vx, vy

    def _apply_friend_avoidance(self, vx, vy):
        """ORCA 避碰：修复无解兜底、对称死锁、速度衰减问题"""
        wx, wy = self.world_xy
        if wx is None:
            return vx, vy

        # 先应用自适应速度
        vx, vy = self._adaptive_speed(vx, vy)

        # 起飞/爬升阶段豁免水平避让：起飞区 6 架间距只有 8m，全部落在 ORCA
        # 互斥范围内会直接死锁（实测 ORCA 无解 110 次、6 架全程趴地起不来）。
        if self.local_z is not None and self.local_z < ALT_TAKEOFF + CLIMB_NO_AVOID:
            return vx, vy

        # ====== BUG 1: 无解时没兜底 - 改为强制使用排斥 ======
        # 先检查是否有近距友机需要避让
        has_conflict = False
        for fid, (fx, fy, fz) in self._friend_positions.items():
            dist = math.hypot(fx - wx, fy - wy)
            if 0.5 < dist < FRIEND_SAFE_DIST:
                has_conflict = True
                break

        if not has_conflict:
            return vx, vy  # 无冲突直接返回

        # 当前速度
        v_current = math.hypot(vx, vy)
        # ====== BUG 2: 速度衰减后无法恢复 - 始终保持最小速度 ======
        MIN_SPEED = MAX_SPEED * 0.3  # 最小巡航速度 0.9 m/s
        if v_current < MIN_SPEED:
            v_current = MIN_SPEED

        # ORCA 半平面集合
        orca_halfplanes = []

        for fid, (fx, fy, fz) in self._friend_positions.items():
            dist = math.hypot(fx - wx, fy - wy)
            # ====== 高度分层：用真实高度判断 ======
            # 本机真实高度
            my_alt = self.local_z if self.local_z is not None else self.altitude_layer
            # 友机高度（从状态消息获取真实高度）
            friend_alt = fz if fz is not None else self.altitude_layer
            # 用三维距离判断够不够远：垂直 separation 与水平距离同等计数。
            # 旧写法只用二值的「是否同层」，层间距 0.5m 时名义分层却仍能让两机
            # 水平贴到 0.22m（实测 iris_2×iris_6）；反过来像 VERT_SEP=0.8 那样
            # 让相邻层也避让，又会避让密度爆炸、起飞区直接互斥死锁。
            dz = abs(my_alt - friend_alt)
            d3 = math.hypot(dist, dz)
            if d3 > SAFE_3D:
                continue

            # ====== BUG 3: 对称死锁 - 用 ID 做 tie-break ======
            # 奇数 ID 往左，偶数 ID 往右
            uav_num = int(self.uav_id.split('_')[1]) if '_' in self.uav_id else 0
            bias_dir = 1 if uav_num % 2 == 1 else -1  # 1=右, -1=左

            if dist < FRIEND_SAFE_DIST * 3:
                px, py = fx - wx, fy - wy
                if dist > 0.01:
                    nx, ny = px / dist, py / dist
                    # 加 bias 让对称情况不反复横跳
                    nx += bias_dir * 0.3
                    ny += bias_dir * 0.3
                    # 归一化
                    n_len = math.hypot(nx, ny)
                    if n_len > 0.01:
                        nx, ny = nx / n_len, ny / n_len

                    # 2026-09-27：用三维距离（分层高度也算进去）+ 近距加大增益。
                    # 原来的水平距离 + 固定 /2.0，同层贴近时才够强，来不及分离。
                    # === 2026-09-29 P0-1/P0-2 ===
                    # 原实现：(a) d3 跨过 SAFE_3D 时分母 2.0->1.0，sep_vel 由 3.0 突跳到 6.0；
                    #        sep_vel=6.0 > MAX_SPEED=5.0 时半平面 v·n>=6.0 无解 -> 不避让（坠机起因）
                    #        (b) d3 in (4,10] 时 sep_vel 仅 0~3.0，太弱 -> 只降速不转向（真空带）
                    # 修：分母在 [DANGER_3D, SAFE_3D] 上连续过渡到远场值 SEP_FAR_DIV，
                    #     再对 sep_vel 做可解性削顶。SEP_FAR_DIV=2.0 即回退旧远场行为。
                    _frac = max(0.0, min(1.0,
                               (d3 - DANGER_3D) / max(1e-6, SAFE_3D - DANGER_3D)))
                    _div = SEP_GAIN + (SEP_FAR_DIV - SEP_GAIN) * _frac
                    sep_vel = max(0.0, (FRIEND_SAFE_DIST - d3)) / _div
                    # 可解性硬顶：绝不越过 MAX_SPEED * SEP_CAP_FRAC
                    sep_vel = min(sep_vel, MAX_SPEED * SEP_CAP_FRAC)
                    orca_halfplanes.append((nx, ny, sep_vel))

        # 最后防线：已经贴到 DANGER_3D 以内，任何 ORCA 解都来不及 → 纯排斥
        _min_d3 = None
        for fid, (fx, fy, fz) in self._friend_positions.items():
            _dz = abs((self.local_z if self.local_z is not None else self.altitude_layer)
                      - (fz if fz is not None else self.altitude_layer))
            _d3 = math.hypot(math.hypot(fx - wx, fy - wy), _dz)
            if _min_d3 is None or _d3 < _min_d3:
                _min_d3 = _d3
        if _min_d3 is not None and _min_d3 < DANGER_3D:
            _rx, _ry = 0.0, 0.0
            for fid, (fx, fy, fz) in self._friend_positions.items():
                _dd = math.hypot(fx - wx, fy - wy)
                if 0.01 < _dd < DANGER_3D * 2.0:
                    _wg = (DANGER_3D * 2.0 - _dd) / _dd
                    _rx += (wx - fx) * _wg
                    _ry += (wy - fy) * _wg
            _rl = math.hypot(_rx, _ry)
            if _rl > 1e-6:
                _sp = MAX_SPEED * 0.6
                return _rx / _rl * _sp, _ry / _rl * _sp

        # 搜索安全速度（全向采样）
        if orca_halfplanes:
            v_angle = math.atan2(vy, vx)
            best_vx, best_vy = vx, vy
            best_score = -float('inf')

            # 2026-09-27：候选改为全向。原来只在 [v_angle±1.5rad] 采样，
            # 相向接近时必须大转向才能满足半平面 → 无解 → 退化到强制排斥，
            # 实测贴到 0.70m。全向采样保证只要几何上存在安全方向就找得到。
            _nang = max(8, AVOID_ANGLES)
            _angles = [v_angle]   # 先放当前朝向，评分相同时保持惯性
            for _k in range(_nang):
                _angles.append(v_angle + 2.0 * math.pi * _k / _nang)
            for angle in _angles:
                for speed in [MIN_SPEED, MAX_SPEED * 0.5, MAX_SPEED * 0.7, MAX_SPEED]:
                    cand_vx = speed * math.cos(angle)
                    cand_vy = speed * math.sin(angle)
                    valid = True
                    for nx, ny, min_vel in orca_halfplanes:
                        vel_along_n = cand_vx * nx + cand_vy * ny
                        if vel_along_n < min_vel - 0.1:
                            valid = False
                            break
                    if valid:
                        score = -abs(speed - v_current)  # 优先保持原速度
                        if score > best_score:
                            best_score = score
                            best_vx, best_vy = cand_vx, cand_vy

            if best_score == -float('inf'):
                # ====== BUG 1 修复: 无解时强制排斥 ======
                rospy.logwarn("[%s] ORCA 无解，强制排斥", self.uav_id)
                for fid, (fx, fy, fz) in self._friend_positions.items():
                    dist = math.hypot(fx - wx, fy - wy)
                    if 0.5 < dist < FRIEND_SAFE_DIST:
                        force = FRIEND_K * (FRIEND_SAFE_DIST - dist) / dist
                        best_vx += (wx - fx) * force
                        best_vy += (wy - fy) * force
            vx, vy = best_vx, best_vy

        # 限幅到 MAX_SPEED（防止 ORCA 排斥力过大导致撞墙）
        spd = math.hypot(vx, vy)
        if spd > MAX_SPEED:
            vx = vx * MAX_SPEED / spd
            vy = vy * MAX_SPEED / spd

        return vx, vy

    # ---------------- 状态上报 ----------------
    def _publish_status(self):
        if self.world_xy is None:
            return

        st = UavStatus()
        st.header.stamp = rospy.Time.now()
        st.uav_id = self.uav_id
        st.x = self.world_xy[0]
        st.y = self.world_xy[1]
        st.z = self.local_z if self.local_z is not None else 0.0
        st.connected = self.state.connected

        # 用实际位置计算所在格子（而非被分配的格子）- 使用 10m 覆盖栅格
        # 覆盖判定由 manager 端用感知半径批量处理，agent 只需上报位置
        actual_cell = self.cov_grid.world_to_cell(self.world_xy[0], self.world_xy[1])
        if actual_cell is not None:
            st.cell_ix = actual_cell[0]
            st.cell_iy = actual_cell[1]
            # 只要有有效位置就报 confidence=1.0，manager 会用 20m 半径批量覆盖
            st.confidence = 1.0
        else:
            st.cell_ix = -1
            st.cell_iy = -1
            st.confidence = 0.0

        # 仍然记录被分配的格子（用于调试对比）
        if self.assignment is not None:
            st.assigned_cell_ix = self.assignment.cell_ix
            st.assigned_cell_iy = self.assignment.cell_iy
            st.assigned_target_x = self.assignment.target_x
            st.assigned_target_y = self.assignment.target_y
        else:
            st.assigned_cell_ix = -1
            st.assigned_cell_iy = -1
            st.assigned_target_x = 0.0
            st.assigned_target_y = 0.0

        self.status_pub.publish(st)

    # ---------------- 主循环 ----------------
    def run(self):
        # 等待连接
        while not rospy.is_shutdown() and not self.state.connected:
            self.ctrl_rate.sleep()
        rospy.loginfo("[%s] MAVROS 已连接", self.uav_id)
        # 等局部位置就绪
        while not rospy.is_shutdown() and self.local_xy is None:
            self.ctrl_rate.sleep()
        # 标定「局部->世界」的恒定平移（只取一次 model_states，之后不再订阅）
        if not self._calibrate_offset():
            raise SystemExit(1)

        self._configure_fcu()
        if not self._arm_and_offboard():
            raise SystemExit(1)

        rospy.loginfo("[%s] 开始协同搜索（高度层 %.1f m），等待管理器分配任务", self.uav_id, self.altitude_layer)
        while not rospy.is_shutdown():
            # 收到任务完成广播后退出搜索循环（但如果有降落任务，先执行降落）
            if self._mission_finished and not self._landing:
                rospy.loginfo("[%s] 任务已完成，退出搜索循环", self.uav_id)
                break
            try:
                self._control()
                self._detect_targets()      # 规则3：几何判定，命中即上报管理器
                if (rospy.Time.now() - self._last_status_t).to_sec() >= 1.0 / PUB_RATE:
                    self._publish_status()
                    self._last_status_t = rospy.Time.now()
            except Exception as e:
                rospy.logerr("[%s] 主循环异常: %s\n%s", self.uav_id, e, traceback.format_exc())
            self.ctrl_rate.sleep()


if __name__ == "__main__":
    rospy.init_node("swarm_agent", anonymous=True)
    uav_id = rospy.get_param("~uav_id", "uav_1")
    model_name = rospy.get_param("~model_name", "iris_1")
    rospy.loginfo("swarm_agent 启动: uav_id=%s model=%s", uav_id, model_name)
    SwarmAgent(uav_id, model_name).run()
