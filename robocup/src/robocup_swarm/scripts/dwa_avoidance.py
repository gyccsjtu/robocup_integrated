#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RoboCup 单机前置测试：A* 全局规划 + DWA 局部避障（平滑速度控制）。

分层架构（在已验证 A* 单机避障基础上叠加 DWA，不改动 single_uav_avoidance.py）：
  A* 全局层  -> 占据栅格 + 障碍物膨胀(0.4m) -> 全局航点 -> Catmull-Rom 平滑 -> 绿色路径
  DWA 局部层 -> 跟踪最近航点，在加速度窗口内采样 (vx,vy) -> 选代价最小 -> 发布平滑速度
  高度控制  -> 竖直速度 P 控制器，保持 2.4m 高度（纯速度模式必须自己做高度闭环）

速度指令走 mavros setpoint_velocity（TwistStamped，ENU 局部系），不是位置点，避免速度突变。
障碍物来源：/scan 激光（实时避障，按航向旋转到 ENU）+ A* 膨胀栅格（静态墙，已规划绕行）。
不依赖 move_base，符合比赛约束。

运行示例（先确保仿真已起、MAVROS 已连接）：
  source /opt/ros/noetic/setup.bash
  source ~/team_ws/robocup/devel/setup.bash
  python3 dwa_avoidance.py \
    _metadata:=/home/ros/team_ws/robocup/src/robocup_training_worlds/worlds/generated/training_city_unit_single_wall_s42.json \
    _start_x:=-3 _start_y:=-2.5 _goal_x:=3 _goal_y:=-2.5

可覆盖参数（_xx:=值）：_max_speed_m_s、_max_accel_m_s2、_inflation_m、_safety_dist_m、
_altitude_m、_vel_topic、_scan_topic、_ekf_acc_limit_m。
"""
import math
from collections import deque
import rospy
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from mavros_msgs.msg import State, ParamValue
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from robocup_navigation.astar import load_metadata, GridMap, plan
from local_esdf import LocalEsdf, UNKNOWN  # 阶段A：局部 ESDF 距离场（原创，同目录）

# ============================ 参数（统一放这里，可被 ~ 参数覆盖） ============================
MAX_SPEED      = 3.0    # 水平最大速度 m/s（比赛标准地图 200m×100m：3m/s 保守提速，刹停2.25m<前瞻3m<通道5m）
MAX_ACCEL      = 2.0    # 最大加速度 m/s^2（刹停距离 v²/2a=1.0m）
INFLATION_M    = 0.0    # A* 额外膨胀半径 m。
                        # ⚠️ 生成器 metadata 的 grid 已按官方 safety_margin_m=0.8 预膨胀
                        # （0.4 机体半径 + 0.2 静态净空 + 0.2 跟踪余量）。A* 若再二次膨胀
                        # 会双重膨胀，把贴墙的 goal（如 u_shape 里距墙 1.15m 的 goal）挤进
                        # 膨胀区导致 GOAL_OCCUPIED。故此处归零，直接用已膨胀的 grid 规划。
ROBOT_RADIUS   = 0.45   # 无人机旋翼半径（裸机体 footprint，不含任何裕度）
OBS_RADIUS     = 0.45   # 动态柱子水平半径（0.6×0.6 盒半对角线 0.42 + 裕度）。
                        # ⚠️ 柱子是实体盒子，不能当点！碰撞距离必须 = 无人机半径 + 柱子半径
SAFETY_MARGIN  = 0.15   # 安全裕度 m（激光噪声 + 定位/控制跟踪误差）
BRAKING_MARGIN = 0.20   # 刹车裕度 m（高速接近时预留的制动距离，保守固定值）
                        # 有效安全距离 SAFE_DIST = ROBOT_RADIUS + SAFETY_MARGIN + BRAKING_MARGIN
                        # 预测轨迹进入此范围 → 直接淘汰（这是撞柱/撞墙的根治：footprint 膨胀）
SAFE_DIST      = ROBOT_RADIUS + SAFETY_MARGIN + BRAKING_MARGIN  # 0.80m 有效碰撞半径
SAFETY_DIST    = 2.5    # 静态障碍物(墙)软避障距离 m（> 刹停2.25m；比赛通道5~11m够宽）
DYN_SAFETY_DIST = 0.8   # 动态障碍物(柱子)软避障距离 m（无人机中心到柱子中心，已含柱子半径）
HARD_BRAKE_DIST = 1.8   # 静态障碍物硬刹车距离 m（> 机体半径+裕度，< 软避障2.5m）
ESCAPE_DIST    = 0.6    # 逃逸触发距离 m（到柱子表面）：低速逼近时用这个阈值
ESCAPE_DIST_FAST = 1.0  # 逃逸触发距离 m（到柱子表面）：相对接近速度 > ESCAPE_REL_SPEED 时放大到这个，提前逃逸
ESCAPE_REL_SPEED = 1.0  # 判定「高速逼近」的相对接近速度阈值 m/s
ESCAPE_PREDICT = 0.8    # 逃逸碰撞预测时长 s：按当前速度预测未来 0.8s 内会碰撞才触发
ARRIVE_TOL     = 0.3    # 到达目标判定半径 m
ALTITUDE       = 6.0    # 飞行高度 m（比赛 planning_altitude_m=6.0，灯杆7.5m/建筑9~17m仍在避障范围）
CTRL_DT        = 0.05   # 控制周期 s（20Hz）
PREDICT_TIME   = 1.0    # DWA 轨迹预测时长 s（3m/s×1.0s=3.0m 前瞻；对移动柱子做匀速预测）
                        # 必须满足：> 刹停2.25m（否则刹不住）、< 到墙/柱最小间距（否则全样本淘汰被困）
LOOKAHEAD      = 1.8    # 前瞻距离 m（3m/s 下略放大，太远会抖、太近会贴柱）
VX_SAMPLES     = 11     # vx 采样数
VY_SAMPLES     = 11     # vy 采样数（横向绕行的关键：vy 必须全范围采样，不能只 cur_vy±dv）
W_HEADING      = 0.3    # 目标方向代价权重（削弱，让障碍物代价能压过它）
W_DIST         = 0.5    # 目标距离代价权重
W_OBSTACLE     = 3.0    # 障碍物代价权重（障碍物在警戒距离内连续施压，越近越陡）
W_PROGRESS     = 0.5    # 目标进展代价权重：速度在目标方向投影为负（远离目标）时惩罚
WARN_DIST      = 4.0    # 障碍物警戒距离 m：进入此范围就开始产生避障代价（3m/s 高速需更早转向）
W_ACCEL        = 0.8    # 加速度软约束权重（替代硬裁窗口，允许横向大速度绕行又抑制突变）
W_SPEED        = 0.1    # 速度（更快）代价权重
W_SMOOTH       = 0.3    # 速度平滑代价权重
W_AVOID_DIR    = 0.9    # 方向锁偏置权重：锁定绕行方向后，惩罚横向速度与锁定方向相反的候选
# ===== 目标接近放宽（goal 距墙 < SAFETY_DIST 的口袋场景，如 U 形死角）=====
# goal 本身距墙只有 1.15m < SAFETY_DIST(1.5)/HARD_BRAKE_DIST(1.2)，软障碍代价会阻止
# 无人机完成最后接近 → 入口悬停 → TRAP → RETREAT → 重进 → 死循环。距最终目标 < 此值时，
# 软障碍权重/脱困/紧急逃逸随接近线性放宽，让无人机逼近并停在 goal；硬淘汰 safe_radius
# 仍防真实碰撞（goal 距墙 1.15m > safe_radius 0.6m，物理安全）。
GOAL_RELAX_DIST = 2.0   # 距最终目标 < 此值(m)时开始放宽
GOAL_RELAX_MIN_W = 0.1  # 目标处软障碍权重保留比例（0=完全关闭软代价，1=不放松）
TERMINAL_DIST   = 3.5   # 距最终目标 < 此值(m)时切入终末接近控制器（绕过 DWA 采样/状态机脱困）
                        # ⚠️ 必须 > DWA 前瞻(PREDICT_TIME×max_speed=3.0m)：否则无人机在袋口外
                        # 就被 DWA 的「前进预测撞 goal 身后墙」卡死横跳，d_goal 降不到该值，
                        # 终末控制器永远不触发，形成 ESCAPE↔RETREAT 死循环。
ALT_KP         = 0.8    # 高度 P 控制器比例增益
MAX_VZ         = 0.5    # 竖直速度上限 m/s
EKF_ACC_LIMIT  = 2.0    # EKF 水平位置标准差阈值，超限判定定位漂移
STUCK_TIME     = 5.0    # 停滞判定时长 s（超时触发重规划）
STUCK_PROGRESS = 0.1    # 停滞判定最小前进量 m

# ===== 状态机 + 方向迟滞 + 脱困（解决左右振荡/局部极小值）=====
# 方向锁：DWA 选定绕行方向后，锁定一段时间，避免左右障碍轻微变化就翻转
DIR_LOCK_TIME  = 1.5    # 方向锁定最短时间 s（期间不重选左右方向）
DIR_SWITCH_MARGIN = 0.5 # 只有另一侧自由空间明显更大(>此裕度m)才允许切换方向
# 卡死/局部极小值检测（时间窗口统计，不只看瞬时速度）
TRAP_WINDOW    = 3.0    # 统计窗口 s
TRAP_MIN_DISP  = 0.35   # 窗口内位移 < 此值判定「几乎没动」 m
TRAP_MIN_PROG  = 0.20   # 窗口内到目标距离下降 < 此值判定「无目标进展」 m
# 脱困模式
ESCAPE_LOCK_TIME = 1.5  # 脱困方向最短保持时间 s（期间禁止切换左右）
ESCAPE_MIN_TIME   = 2.0 # 脱困最短持续时间 s（没达到不算脱困成功）
ESCAPE_MAX_TIME   = 6.0 # 脱困最大持续时间 s（超时强制退出，防死锁）
RECOVER_PROG   = 0.5    # 脱困后目标距离下降/横向位移超过此值 -> 认为已脱困，进入 RECOVER
ESCAPE_SPEED   = 1.2    # 脱困速度 m/s（小场地不宜过快，避免横移撞墙）
RECOVER_TIME   = 1.5    # RECOVER 最短持续时间 s（脱困后保持方向锁、继续 DWA 恢复巡航）
ESCAPE_COOLDOWN = 1.0   # 脱困冷却 s（退出 ESCAPE 后此时间内禁止再次触发，防 ESCAPE↔RECOVER 抖振）
# ===== 出口识别（第六优先级：出现安全通道时快速穿出，不原地试探）=====
EXIT_CLEARANCE   = 1.2  # 出口净空阈值 m：预测轨迹最近障碍距离 > 此值 且 有目标进展 → 出口可用
EXIT_CLEAR_CYCLES = 6   # 连续净空周期数（20Hz → 0.3s）判定出口稳定可用
EXIT_SPEED_BOOST = 1.0  # 出口可用时前进速度倾向系数（提高合理前进速度）
# ===== RETREAT（后退脱困：左右都堵时禁止 LEFT/RIGHT 无限互换）=====
HARD_BLOCK       = 0.80  # 硬阻塞阈值 m：当前方向净空 < 此值判定「被堵死」
RETREAT_SPEED    = 0.8   # 后退速度 m/s（低到中速，方向锁定）
RETREAT_MIN_TIME = 1.5   # 后退最短持续时间 s（期间不重选方向）
RETREAT_MIN_DIST = 0.8   # 后退累计最小位移 m（没退够不退出）
RETREAT_CLEAR    = 1.5   # 退出条件：最近障碍距离 > 此值 且 满足 min_time+min_dist
RETREAT_MAX_TIME = 5.0   # 后退最大持续时间 s（超时强制退出重规划，防死胡同死锁）
RETREAT_CREEP_SPEED = 0.3  # 贴脸时低速爬离速度 m/s（方向=远离障碍，安全；0 会永久卡死）
RETREAT_SWITCH_MARGIN = 0.3  # 后退方向重选裕度 m：候选方向净空必须比当前方向大此值才切换
RETREAT_RELOCK_TIME = 0.5    # 后退方向重选评估的最小间隔 s（限流，防每帧重算振荡）
# ===== emergency collision constraint（问题6：激光近距硬约束）=====
EMERGENCY_FRONT = 0.20  # front < 此值：任何朝前(vx 前向分量>0)轨迹直接判 collision
EMERGENCY_SIDE  = 0.20  # left/right < 此值：向该侧运动的轨迹直接判 collision
# ===== 速度相关制动裕度（第三优先级）=====
# 对每个候选速度 (vx,vy)：braking_distance = |v|^2 / (2*max_accel)
# 动态安全半径 safe_radius(v) = ROBOT_RADIUS + SAFETY_MARGIN + braking_distance
# 预测轨迹最近距离 < safe_radius(v) → 该候选直接淘汰（碰撞轨迹不评分）
BRAKING_DECEL   = 2.0   # 制动减速度 m/s^2（默认 = MAX_ACCEL）

# ===== 阶段A：局部 ESDF 距离场（替换点到线段距离，最小侵入）=====
ESDF_RES        = 0.25  # ESDF 栅格分辨率 m
ESDF_SIDE       = 20.0  # ESDF 滚动窗口边长 m（80×80 格）
ESDF_INFLATE    = 0.25  # ESDF 建场时的障碍膨胀半径 m（填补激光束间隙，非安全裕度）
ESDF_TRAJ_STEP  = 0.1   # 轨迹沿 ESDF 采样点间距 m（越小越密越准，越大越快）
USE_DYNAMIC_OBS = False # 本阶段去排动态柱预测（静态未知环境）；保留代码，未来可恢复


def inflate_grid(grid, inflation_m):
    """对障碍物向外膨胀 inflation_m（圆盘结构元素），返回新 GridMap。"""
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


class DwaAvoidance:
    def __init__(self):
        # ---- 参数读取（~ 前缀，可命令行覆盖）----
        self.metadata = rospy.get_param("~metadata")
        self.altitude = rospy.get_param("~altitude_m", ALTITUDE)
        self.max_speed = rospy.get_param("~max_speed_m_s", MAX_SPEED)
        self.max_accel = rospy.get_param("~max_accel_m_s2", MAX_ACCEL)
        self.inflation_m = rospy.get_param("~inflation_m", INFLATION_M)
        self.safety_dist = rospy.get_param("~safety_dist_m", SAFETY_DIST)
        self.hard_brake_dist = rospy.get_param("~hard_brake_dist_m", HARD_BRAKE_DIST)
        self.warn_dist = rospy.get_param("~warn_dist_m", WARN_DIST)
        self.dyn_safety_dist = rospy.get_param("~dyn_safety_dist_m", DYN_SAFETY_DIST)
        self.escape_dist = rospy.get_param("~escape_dist_m", ESCAPE_DIST)
        self.escape_dist_fast = rospy.get_param("~escape_dist_fast_m", ESCAPE_DIST_FAST)
        self.escape_rel_speed = rospy.get_param("~escape_rel_speed_m_s", ESCAPE_REL_SPEED)
        self.escape_predict = rospy.get_param("~escape_predict_s", ESCAPE_PREDICT)
        self.obs_radius = rospy.get_param("~obs_radius_m", OBS_RADIUS)
        self.arrive_tol = rospy.get_param("~arrival_tolerance_m", ARRIVE_TOL)
        self.alt_kp = rospy.get_param("~alt_kp", ALT_KP)
        self.max_vz = rospy.get_param("~max_vz_m_s", MAX_VZ)
        self.ekf_acc_limit = rospy.get_param("~ekf_acc_limit_m", EKF_ACC_LIMIT)
        self.vel_topic = rospy.get_param("~vel_topic", "/mavros/setpoint_velocity/cmd_vel")
        # 注意：PX4 SITL 里 iris_2d_lidar 的激光话题带模型名前缀 /iris_2d_lidar_0/scan，
        # 不是裸 /scan。用命令行 _scan_topic:=xxx 可覆盖。
        self.scan_topic = rospy.get_param("~scan_topic", "/iris_2d_lidar_0/scan")

        # ---- 阶段A：局部 ESDF 距离场 ----
        self.esdf_res = rospy.get_param("~esdf_res_m", ESDF_RES)
        self.esdf_side = rospy.get_param("~esdf_side_m", ESDF_SIDE)
        self.esdf_inflate = rospy.get_param("~esdf_inflate_m", ESDF_INFLATE)
        self.esdf_traj_step = rospy.get_param("~esdf_traj_step_m", ESDF_TRAJ_STEP)
        self.use_dynamic_obs = rospy.get_param("~use_dynamic_obs", USE_DYNAMIC_OBS)
        self.esdf = LocalEsdf(self.esdf_res, self.esdf_side, self.esdf_inflate)

        # ---- 载入地图，A* 终点（世界坐标 = map 坐标）----
        md, _ = load_metadata(self.metadata)
        self.grid = GridMap.from_metadata(md)
        goal_cfg = md["goal_candidates"][0]["center"]
        self.start = (rospy.get_param("~start_x", None),
                      rospy.get_param("~start_y", None))
        self.goal = (rospy.get_param("~goal_x", goal_cfg[0]),
                     rospy.get_param("~goal_y", goal_cfg[1]))
        self.goal_updated = False

        # ---- 发布：速度指令 + RViz 可视化 ----
        self.vel_pub = rospy.Publisher(self.vel_topic, TwistStamped, queue_size=10)
        self.path_pub = rospy.Publisher("~global_path", Marker, queue_size=1)
        self.inflated_pub = rospy.Publisher("~inflated_obstacles", Marker, queue_size=1)
        self.window_pub = rospy.Publisher("~dwa_window", Marker, queue_size=1)
        self.scan_pub = rospy.Publisher("~laser_scan_viz", Marker, queue_size=1)
        self.pos_pub = rospy.Publisher("~current_pos", Marker, queue_size=1)

        # ---- 订阅 ----
        rospy.Subscriber("/mavros/state", State, self._state_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._pose_cb)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self._ekf_cb)
        rospy.Subscriber(self.scan_topic, LaserScan, self._scan_cb)
        rospy.Subscriber("/goal", PoseStamped, self._goal_cb)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._models_cb)

        # ---- 服务 ----
        self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.param_srv = rospy.ServiceProxy("/mavros/param/set", ParamSet)

        # ---- 运行状态 ----
        self.state = State()
        self.local = None          # ENU 局部坐标 (x,y,z)
        self.yaw = 0.0             # 航向角 rad（ENU 系）
        self.model_states = None
        self.offset = None         # gazebo世界 -> mavros局部 偏移
        self.scan = None
        self.ekf_bad = False
        self.ekf_std = 0.0
        self.cur_vx = 0.0
        self.cur_vy = 0.0
        self.escaping = False          # 紧急逃逸迟滞状态（避免边界来回横跳）
        self.escape_dir = (0.0, 0.0)   # 逃逸方向低通缓存（避免逐帧翻转）
        self.stuck_start = None        # 物理卡死检测：指令有速度但位置几乎不动的时间戳
        self.stuck_ref_pos = (0.0, 0.0)

        # ===== 状态机 + 方向迟滞 + 脱困 =====
        # 状态：NORMAL_NAV / AVOID_OBSTACLE / ESCAPE / RECOVER
        self.nav_state = "NORMAL_NAV"
        self.avoid_dir = 0.0           # 绕行方向锁定：+1=左(ENU +y侧)，-1=右(ENU -y侧)，0=未锁
        self.avoid_dir_lock_until = 0.0  # 方向锁定截止时间
        self.traj_hist = deque()       # 时间窗口轨迹 [(t, x, y, d_goal), ...]
        self.escape_sign = 0.0         # 脱困方向（+1/-1，锁定期间不变，ESC​APE 持久状态核心）
        self.escape_start_time = 0.0   # 进入 ESCAPE 的时间
        self.escape_lock_until = 0.0   # 脱困方向锁定截止时间
        self.escape_ref_prog = 0.0     # 进入 ESCAPE 时到目标的距离（用于判断是否脱困成功）
        self.escape_ref_pos = (0.0, 0.0)  # 进入 ESCAPE 时的位置（用于判断位移）
        self.escape_clear_cycles = 0   # ESCAPE 期间连续净空周期计数（出口识别）
        self.escape_enter_reason = ""  # ESCAPE 进入原因（日志输出）
        self.retreat_start_time = 0.0  # 进入 RETREAT 的时间（左右都堵时后退脱离）
        self.retreat_start_pos = (0.0, 0.0)  # 进入 RETREAT 时的位置（累计位移判定）
        self.retreat_dir = (0.0, 0.0)  # RETREAT 锁定方向（进入时选一次，期间不变）
        self.retreat_relock_until = 0.0  # 后退方向重选后的最短锁定截止时间（防横跳）
        self.recover_start_time = 0.0  # 进入 RECOVER 的时间
        self.escape_cooldown_until = 0.0  # 脱困冷却截止时间（防 ESCAPE↔RECOVER 抖振）
        self.exit_clear_cycles = 0     # 出口识别：连续净空周期计数（DWA 巡航时用）
        self.dwa_valid_count = 0       # DWA 本轮合法候选轨迹数（0=无路可走，需 RETREAT）
        self.last_dir = (0.0, 0.0)     # 上一周期输出方向（低通防翻转）
        self.path = []             # 全局路径（map 坐标）
        self.path_local = []       # 全局路径（ENU 局部坐标）
        self.rate = rospy.Rate(20)

    # ==================== 回调 ====================
    def _state_cb(self, msg):
        self.state = msg

    def _pose_cb(self, msg):
        p = msg.pose.position
        self.local = (p.x, p.y, p.z)
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def _ekf_cb(self, msg):
        # EKF 位姿方差来自 /mavros/local_position/odom 的 pose.covariance。
        # 36 元素行主序：索引 0=x方差, 7=y方差, 14=z方差。取水平最大者开方得标准差。
        cov = msg.pose.covariance
        vx = cov[0] if cov[0] > 0 else 0.0
        vy = cov[7] if cov[7] > 0 else 0.0
        self.ekf_std = math.sqrt(max(vx, vy))
        self.ekf_bad = self.ekf_std > self.ekf_acc_limit

    def _scan_cb(self, msg):
        self.scan = msg

    def _goal_cb(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.goal_updated = True
        rospy.loginfo("收到 /goal 新目标 %.2f,%.2f", self.goal[0], self.goal[1])

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

    def _dynamic_obstacles(self):
        """从 /gazebo/model_states 提取移动柱子的位置+速度，转 ENU 局部系。

        返回 [(x, y, vx, vy), ...]。柱子 = 外部动态障碍物，速度用于匀速轨迹预测。
        这是「提前绕开」的关键：激光只给当前位置，柱子 2m/s 时等激光看到已经很近；
        model_states 给真实速度，DWA 才能预测柱子未来位置、提前横向绕行。
        """
        if self.model_states is None or self.offset is None:
            return []
        out = []
        for i, name in enumerate(self.model_states.name):
            if "moving_obstacle" not in name:
                continue
            p = self.model_states.pose[i].position
            t = self.model_states.twist[i].linear
            out.append((p.x - self.offset[0], p.y - self.offset[1], t.x, t.y))
        return out

    @staticmethod
    def _seg_point_min_dist(sx, sy, ex, ey, px, py):
        """点 (px,py) 到线段 (sx,sy)-(ex,ey) 的最短距离。"""
        segx = ex - sx
        segy = ey - sy
        seg_len2 = segx * segx + segy * segy
        if seg_len2 < 1e-9:
            return math.hypot(px - sx, py - sy)
        t = ((px - sx) * segx + (py - sy) * segy) / seg_len2
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0
        cx = sx + segx * t
        cy = sy + segy * t
        return math.hypot(px - cx, py - cy)

    # ==================== 速度发布 + 高度控制 ====================
    def _send_vel(self, vx, vy, vz):
        t = TwistStamped()
        t.header.stamp = rospy.Time.now()
        t.twist.linear.x = vx
        t.twist.linear.y = vy
        t.twist.linear.z = vz
        self.vel_pub.publish(t)

    def _altitude_vz(self):
        """高度 P 控制器：返回竖直速度，让飞机保持在 self.altitude。"""
        if self.local is None:
            return 0.0
        z_err = self.altitude - self.local[2]
        if abs(z_err) < 0.05:
            return 0.0
        vz = self.alt_kp * z_err
        return max(-self.max_vz, min(self.max_vz, vz))

    # ==================== 激光 -> ENU 障碍物点 ====================
    def _scan_points_enu(self, dense=False):
        """把 /scan 转为 ENU 局部系障碍物点集 [(x,y),...]。

        假设：2D 激光水平安装、扫描角 0 指向机头（body +x），REP-103。
        先用航向 yaw 把 body 系点旋转到 ENU，再加当前 local 位置偏移。

        dense=False：降采样到 ~90 点（DWA 斥力/可视化够用，省算力）。
        dense=True ：全量束（ESDF 建场必须，否则远端漏障碍）。
        """
        if self.scan is None or self.local is None:
            return []
        pts = []
        a = self.scan.angle_min
        n = len(self.scan.ranges)
        step = 1 if dense else max(1, n // 90)  # 全量 / 降采样到 ~90 点
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        lx, ly = self.local[0], self.local[1]
        for idx, r in enumerate(self.scan.ranges):
            if idx % step == 0 and self.scan.range_min < r < self.scan.range_max:
                bx = r * math.cos(a)
                by = r * math.sin(a)
                ex = bx * cy - by * sy
                ey = bx * sy + by * cy
                pts.append((lx + ex, ly + ey))
            a += self.scan.angle_increment
        return pts

    # ==================== 局部引导目标 ====================
    def _pick_local_goal(self):
        """沿全局路径从当前位置找最近点，再往前累加 LOOKAHEAD 距离取引导目标。"""
        if self.local is None or not self.path_local:
            return self.goal
        best = 0
        best_dist = float("inf")
        for i, (px, py) in enumerate(self.path_local):
            d = math.hypot(px - self.local[0], py - self.local[1])
            if d < best_dist:
                best_dist = d
                best = i
        acc = 0.0
        for i in range(best, len(self.path_local) - 1):
            x0, y0 = self.path_local[i]
            x1, y1 = self.path_local[i + 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            if acc + seg >= LOOKAHEAD:
                frac = (LOOKAHEAD - acc) / max(seg, 1e-6)
                return (x0 + (x1 - x0) * frac, y0 + (y1 - y0) * frac)
            acc += seg
        return self.path_local[-1]

    # ==================== DWA 核心 ====================
    def _trajectory_clear_breakdown(self, vx, vy, obs_enu, dyn_obs):
        """预测轨迹净空拆分：返回 (static_clear, dynamic_clear, total_clear)。

        阶段A 改造：静态障碍不再用「点到线段距离」逐激光点遍历，改为在
        局部 ESDF 距离场上沿轨迹采样查表（O(1) 每点），距离更准、无梯度问题、
        且天然覆盖「拐弯扫过区域 + 线段端点外」的漏判。

        任一采样点距离为 UNKNOWN（地图外/未观测）→ 保守判 0（不安全）。

        obs_enu 仍由 _scan_points_enu() 提供（ENU 局部系，米），它既用于本函数
        的 ESDF 建场数据源，也兼容后续可能的原始点运算。dyn_obs 在
        USE_DYNAMIC_OBS=False 时为空，保留动态柱预测代码以备未来恢复。
        """
        lx, ly = self.local[0], self.local[1]
        static_clear = float("inf")
        dynamic_clear = float("inf")

        # ---- 静态障碍：ESDF 轨迹采样 ----
        traj_len = math.hypot(vx, vy) * PREDICT_TIME
        n_steps = max(1, int(math.ceil(traj_len / self.esdf_traj_step)))
        for k in range(n_steps + 1):
            t = PREDICT_TIME * k / float(n_steps)
            px = lx + vx * t
            py = ly + vy * t
            d = self.esdf.get_distance(px, py)
            if d < 0.0:
                d = 0.0   # UNKNOWN/越界 → 保守判 0（不安全）
            if d < static_clear:
                static_clear = d

        # ---- 动态柱子（本阶段去排，保留代码）----
        if dyn_obs:
            K = 10
            for ox, oy, ovx, ovy in dyn_obs:
                for k in range(K + 1):
                    t = PREDICT_TIME * k / float(K)
                    px = lx + vx * t
                    py = ly + vy * t
                    qx = ox + ovx * t
                    qy = oy + ovy * t
                    d = math.hypot(px - qx, py - qy) - self.obs_radius
                    if d < dynamic_clear:
                        dynamic_clear = d

        return static_clear, dynamic_clear, min(static_clear, dynamic_clear)

    def _trajectory_min_dist(self, vx, vy, obs_enu, dyn_obs):
        """预测轨迹到障碍物的最近距离（考虑动态柱子未来位置）。

        obs_enu: 静态障碍点 [(x,y)]（墙/激光点）→ 点到线段距离
        dyn_obs: 动态柱子 [(x, y, vx, vy)]（model_states）→ 相对匀速运动采样

        动态柱子是「提前绕开」的关键：柱子 2m/s 时，等激光看到它很近已经来不及。
        这里对每个采样速度 (vx,vy)，把无人机沿 (vx,vy)、柱子沿其真实速度 (ovx,ovy)
        都外推 PREDICT_TIME 秒，采样 K 步算相对距离，取最小。这样 DWA 能「预判」柱子
        未来位置，提前横向绕行，而不是撞到眼前才减速。
        """
        _, _, total = self._trajectory_clear_breakdown(vx, vy, obs_enu, dyn_obs)
        return total

    def _dwa_step(self, obs_enu, dyn_obs, d_goal_final):
        """单步 DWA：采样速度 -> 硬淘汰碰撞轨迹 -> 对安全轨迹评分 -> 选最优。

        d_goal_final：到最终目标点的距离（局部系），只用于「接近终点」时的减速。

        核心改动（根治撞墙/撞柱/震荡）：
        ① 碰撞判据用速度相关的安全半径 safe_radius(v) = ROBOT_RADIUS + SAFETY_MARGIN
           + |v|²/(2*BRAKING_DECEL)，即 footprint 膨胀 + 制动距离，而非固定 0.45m；
           任何候选预测轨迹最近距离 < safe_radius → continue 直接淘汰（碰撞轨迹不评分）。
        ② 出口识别：直冲目标的候选净空 > EXIT_CLEARANCE 且连续多周期 → 提高目标权重、
           降低障碍权重、提高前进速度，快速穿出，不原地试探。
        ③ 方向锁偏置：avoid_dir 锁定后惩罚反向横向速度（方向迟滞，消除左右震荡）。
        """
        lx, ly = self.local[0], self.local[1]
        gx, gy = self._pick_local_goal()

        # 采样窗口：全范围 [-max_speed, max_speed]。
        # 关键：横向 vy 必须全范围采样，否则全速前进时横向速度候选≈0，只能减速无法绕行。
        # 速度连续性由 smooth/accel 软约束保证，而非硬裁窗口。
        vx_min, vx_max = -self.max_speed, self.max_speed
        vy_min, vy_max = -self.max_speed, self.max_speed

        dx, dy = gx - lx, gy - ly
        gd = math.hypot(dx, dy) or 1e-6
        ux, uy = dx / gd, dy / gd       # 朝向目标的单位向量
        lat_ux, lat_uy = -uy, ux        # 左向单位向量（目标方向逆时针旋转90°）

        # ---- 出口识别（第六优先级）：直冲目标候选的净空足够大 → 出口可用 ----
        goal_clear = self._trajectory_min_dist(ux * self.max_speed, uy * self.max_speed,
                                               obs_enu, dyn_obs)
        if goal_clear > EXIT_CLEARANCE:
            self.exit_clear_cycles += 1
        else:
            self.exit_clear_cycles = 0
        exit_stable = self.exit_clear_cycles >= EXIT_CLEAR_CYCLES
        # 出口可用时：目标权重×2、障碍权重×0.3、速度权重×2 → 快速穿出
        w_heading = W_HEADING * (2.0 if exit_stable else 1.0)
        w_obstacle = W_OBSTACLE * (0.3 if exit_stable else 1.0)
        w_speed = W_SPEED * (2.0 if exit_stable else 1.0)
        # 接近最终目标：软障碍权重随距离线性放宽到 GOAL_RELAX_MIN_W 倍。
        # 口袋场景 goal 距墙 < SAFETY_DIST，软代价会阻止最后接近 → 用此放开；
        # 硬淘汰 safe_radius 不受影响（goal 距墙 1.15m > 0.6m，物理仍安全）。
        relax = self._goal_relax(d_goal_final)
        if relax > 0.0:
            w_obstacle *= 1.0 - relax * (1.0 - GOAL_RELAX_MIN_W)

        # ---- emergency collision constraint（问题6）：激光近距硬约束 ----
        # front/left/right 净空全部来自实时 LaserScan（obs_enu）+ 动态柱（dyn_obs），
        # 与 _side_clearances 完全同一数据源。某方向净空 < 阈值时，任何朝该方向
        # 运动的候选轨迹直接判 collision 淘汰（不评分），绝不依赖软代价逼近。
        front_clear = goal_clear
        left_clear = self._trajectory_min_dist(lat_ux * self.max_speed, lat_uy * self.max_speed,
                                               obs_enu, dyn_obs)
        right_clear = self._trajectory_min_dist(-lat_ux * self.max_speed, -lat_uy * self.max_speed,
                                                obs_enu, dyn_obs)
        self._scan_clear = (front_clear, left_clear, right_clear)

        best_vx, best_vy = 0.0, 0.0
        best_cost = float("inf")
        valid_count = 0               # 合法候选轨迹数（通过碰撞硬淘汰的数量）
        best_min_d = float("inf")     # 最优轨迹的最近障碍距离（collision clearance）
        best_braking = 0.0            # 最优轨迹对应的制动距离
        best_static_clear = float("inf")   # 最优轨迹的静态障碍净空（激光）
        best_dynamic_clear = float("inf")  # 最优轨迹的动态障碍净空（model_states）
        window_pts = []
        # 调试字段（日志 [DWA] 用）
        best_heading_cost = best_obs_cost = best_prog_cost = 0.0

        for i in range(VX_SAMPLES):
            vx = vx_min + (vx_max - vx_min) * (i / float(VX_SAMPLES - 1))
            for j in range(VY_SAMPLES):
                vy = vy_min + (vy_max - vy_min) * (j / float(VY_SAMPLES - 1))
                end_x = lx + vx * PREDICT_TIME
                end_y = ly + vy * PREDICT_TIME
                window_pts.append((end_x, end_y))

                # ---- 速度相关制动裕度（第三优先级）----
                # 每个候选速度的制动距离不同：越快越需要提前留空间
                spd = math.hypot(vx, vy)
                braking = spd * spd / (2.0 * BRAKING_DECEL)
                safe_radius = ROBOT_RADIUS + SAFETY_MARGIN + braking
                # 上限封顶到 0.8*safety_dist，避免狭窄通道里全样本被淘汰 → 悬停被困
                hard_radius = min(safe_radius, self.safety_dist * 0.8)

                # ---- 未来轨迹碰撞检测（第二优先级 footprint 膨胀）----
                min_d = self._trajectory_min_dist(vx, vy, obs_enu, dyn_obs)
                if min_d < hard_radius:
                    # 碰撞轨迹直接淘汰，不评分（不靠调低 obstacle 权重掩盖问题）
                    continue

                # ---- emergency collision constraint（问题6）：激光近距硬约束 ----
                # 前方净空不足 → 禁止任何朝前的轨迹；左/右净空不足 → 禁止朝该侧的轨迹。
                # 这是硬约束，不依赖软代价，保证「front=0.01m 时向前必判 collision」。
                fwd_v = vx * ux + vy * uy            # 目标方向速度分量（>0 朝前）
                lat_v = vx * lat_ux + vy * lat_uy    # 横向分量（>0 左，<0 右）
                if front_clear < EMERGENCY_FRONT and fwd_v > 0.0:
                    continue
                if left_clear < EMERGENCY_SIDE and lat_v > 0.0:
                    continue
                if right_clear < EMERGENCY_SIDE and lat_v < 0.0:
                    continue

                valid_count += 1   # 通过碰撞硬淘汰的合法候选

                # ---- 对安全轨迹评分 ----
                hx, hy = end_x - lx, end_y - ly
                hd = math.hypot(hx, hy) or 1e-6
                heading = abs(1.0 - (hx * dx + hy * dy) / (hd * gd))  # 0=对准 2=反向
                dist = math.hypot(end_x - gx, end_y - gy) / LOOKAHEAD
                # 目标进展：速度在目标方向上的投影（>0 表示向目标前进）
                prog_v = vx * ux + vy * uy
                progress_cost = 1.0 - max(0.0, prog_v) / self.max_speed  # 0=全速朝目标
                speed_cost = 1.0 - (spd / self.max_speed)
                # 平滑项归一化到 [0,1]，防止高速下 W_SMOOTH 盖过障碍物代价
                smooth = math.hypot(vx - self.cur_vx, vy - self.cur_vy) / (2.0 * self.max_speed)
                # 加速度软约束：超出物理加速能力的部分加惩罚
                max_dv = self.max_accel * CTRL_DT
                over = max(0.0, math.hypot(vx - self.cur_vx, vy - self.cur_vy) - max_dv)
                accel_penalty = over / self.max_speed

                # 软避障代价（safe_radius ~ warn_dist 之间连续施压，越近越陡）
                if min_d < self.warn_dist:
                    if min_d < self.safety_dist:
                        obstacle = (self.safety_dist - min_d) / (self.safety_dist - hard_radius)
                    else:
                        obstacle = (1.0 - min_d / self.warn_dist) * 0.5
                else:
                    obstacle = 0.0

                cost = (w_heading * heading + W_DIST * dist +
                        w_obstacle * obstacle + w_speed * speed_cost +
                        W_PROGRESS * progress_cost +
                        W_SMOOTH * smooth + W_ACCEL * accel_penalty)

                # 方向锁偏置（第一优先级配套）：惩罚横向速度与锁定方向相反的候选
                if self.avoid_dir != 0.0:
                    lat_v = vx * lat_ux + vy * lat_uy  # >0 朝左，<0 朝右
                    if self.avoid_dir * lat_v < 0.0:
                        cost += W_AVOID_DIR * abs(lat_v) / self.max_speed

                if cost < best_cost:
                    best_cost = cost
                    best_vx, best_vy = vx, vy
                    best_heading_cost = heading
                    best_obs_cost = obstacle
                    best_prog_cost = progress_cost
                    best_min_d = min_d
                    best_braking = braking
                    # 问题4/10：记录最优轨迹的静态/动态净空拆分
                    best_static_clear, best_dynamic_clear, _ = self._trajectory_clear_breakdown(
                        vx, vy, obs_enu, dyn_obs)

        # 靠近「最终目标」才按比例减速（用 d_goal_final，不是局部前瞻点距离）。
        decel_dist = self.max_speed * self.max_speed / (2.0 * self.max_accel) + 0.25
        if d_goal_final < decel_dist:
            scale = max(0.0, (d_goal_final - self.arrive_tol) / (decel_dist - self.arrive_tol))
            best_vx *= scale
            best_vy *= scale

        # 无合法候选：best_* 停留在初始 0，绝不能当作「正常 DWA 结果」返回
        if valid_count == 0:
            self.dwa_valid_count = 0
            rospy.logwarn_throttle(1.0, "[DWA] NO_VALID_TRAJECTORY (hard_radius 淘汰全部 %d 样本)",
                                   VX_SAMPLES * VY_SAMPLES)
            self._publish_window(window_pts)
            return 0.0, 0.0, False   # (vx, vy, has_valid)

        # 调试字段（主循环日志 [DWA] 用）
        self.dwa_valid_count = valid_count
        self._dbg_dwa = (best_vx, best_vy, best_heading_cost, best_obs_cost, best_prog_cost,
                         valid_count, best_min_d, best_braking,
                         best_static_clear, best_dynamic_clear)
        self._publish_window(window_pts)
        return best_vx, best_vy, True

    def _emergency_escape(self, obs_enu, dyn_obs, relax=0.0):
        """紧急逃逸兜底（第二层保险，防 DWA 主避障失效后贴脸碰撞）。

        双条件触发（同时满足才逃逸，避免过早误触发）：
          ① 距离阈值：动态柱子 < escape_dist(0.6m)；静态墙 < hard_brake_dist(1.2m)
          ② 碰撞预测：按当前速度外推 escape_predict(0.8s)，会与障碍物 < ROBOT_RADIUS
        触发后 → 反向满速拉开距离，暂停跟踪航点（纯逃逸，不引导回航线）。

        relax：接近最终目标的放宽系数 [0,1]。口袋场景 goal 距墙 1.15m < hard_brake_dist，
        若保持原阈值，刚接近 goal 就被「紧急逃逸」弹开，形成进袋→退→重进死循环。
        relax>0 时静态阈值从 hard_brake_dist 收紧到「真实碰撞兜底」级别（仍 > 机体半径），
        既放过 goal 处的合法贴近，又保留真失控冲墙时的兜底。

        只对动态柱子（外部障碍）生效兜底；静态墙由 DWA 主避障负责。
        队友无人机（未来 6 机集群）不在此列——上层集群协调，这里禁止触发。
        防抖：迟滞 + 方向低通 + 速度连续（受 max_accel*dt 限制）。
        """
        if self.local is None:
            self.escaping = False
            return 0.0, 0.0, False
        lx, ly = self.local[0], self.local[1]

        # ---- 汇总障碍物：静态点(速度0) + 动态柱子(带速度) ----
        near = []   # (x, y, vx, vy) —— 斥力方向源（激光点位置，方向正确）
        # 静态最近距离走 ESDF（与 DWA 轨迹净空同源，含建场膨胀，保守且一致）
        d_esdf = self.esdf.get_distance(lx, ly)
        static_min = d_esdf if d_esdf >= 0.0 else 0.0
        for ox, oy in obs_enu:
            d = math.hypot(ox - lx, oy - ly)
            if d < self.safety_dist:
                near.append((ox, oy, 0.0, 0.0))
        # 动态柱子：中心距减去柱子半径 = 到表面距离；同时算相对接近速度
        dyn_min = float("inf")       # 到最近柱子表面的距离
        dyn_rel_speed = 0.0          # 最近柱子的相对接近速度（>0 表示正在逼近）
        for ox, oy, ovx, ovy in dyn_obs:
            cx = ox - lx
            cy = oy - ly
            cdist = math.hypot(cx, cy)
            surf = cdist - self.obs_radius   # 到柱子表面距离
            if surf < dyn_min:
                dyn_min = surf
                # 相对接近速度：无人机速度 - 柱子速度，投影到「柱子→无人机」方向
                rvx = self.cur_vx - ovx
                rvy = self.cur_vy - ovy
                if cdist > 1e-3:
                    dyn_rel_speed = -(rvx * cx + rvy * cy) / cdist  # 逼近为正
                else:
                    dyn_rel_speed = 0.0
            if surf < self.safety_dist:
                near.append((ox, oy, ovx, ovy))

        # ---- 条件①：距离阈值（动态柱子按相对接近速度放大）----
        # 高速逼近时提前逃逸：相对速度 > escape_rel_speed → 阈值放大到 escape_dist_fast
        if dyn_rel_speed > self.escape_rel_speed:
            dyn_thresh = self.escape_dist_fast
        else:
            dyn_thresh = self.escape_dist
        # 静态阈值：接近目标时从 hard_brake_dist 收紧到「真实碰撞兜底」级别，
        # 避免 goal 距墙 1.15m < hard_brake_dist 时刚接近就被弹开。
        if relax > 0.0:
            static_thresh = ROBOT_RADIUS + SAFETY_MARGIN + 0.2   # ≈0.8m，仍 > 机体半径
            static_thresh = self.hard_brake_dist + relax * (static_thresh - self.hard_brake_dist)
        else:
            static_thresh = self.hard_brake_dist
        dist_ok = (dyn_min < dyn_thresh) or (static_min < static_thresh)

        # ---- 条件②：按当前速度预测 escape_predict 内会碰撞（到表面距离 < 速度相关安全半径）----
        # 安全半径 = ROBOT_RADIUS + SAFETY_MARGIN + v²/(2*decel)，即 footprint 膨胀 + 制动距离
        cur_spd = math.hypot(self.cur_vx, self.cur_vy)
        safe_radius = ROBOT_RADIUS + SAFETY_MARGIN + cur_spd * cur_spd / (2.0 * BRAKING_DECEL)
        will_collide = False
        if dist_ok:
            K = 8
            for ox, oy, ovx, ovy in near:
                for k in range(K + 1):
                    t = self.escape_predict * k / float(K)
                    px = lx + self.cur_vx * t
                    py = ly + self.cur_vy * t
                    qx = ox + ovx * t
                    qy = oy + ovy * t
                    # 动态柱子：中心距 - 柱子半径 = 表面距；静态点：直接距离
                    if ovy == 0.0 and ovx == 0.0:
                        surf = math.hypot(px - qx, py - qy)
                    else:
                        surf = math.hypot(px - qx, py - qy) - self.obs_radius
                    if surf < safe_radius:
                        will_collide = True
                        break
                if will_collide:
                    break

        # ---- 迟滞切换 ----
        exit_d = max(dyn_thresh, static_thresh) + 0.5
        min_surf = min(static_min, dyn_min if dyn_min < float("inf") else static_min)
        if not self.escaping:
            if not (dist_ok and will_collide):
                return 0.0, 0.0, False
            self.escaping = True
        else:
            if min_surf > exit_d:
                self.escaping = False
                self.escape_dir = (0.0, 0.0)
                return 0.0, 0.0, False

        # ---- 逃逸方向：所有近距障碍推斥合力（1/d²），纯逃逸不跟踪航点 ----
        fx = fy = 0.0
        for ox, oy, ovx, ovy in near:
            dx = lx - ox
            dy = ly - oy
            d = math.hypot(dx, dy)
            if d < self.safety_dist and d > 1e-3:
                w = 1.0 / (d * d)
                fx += (dx / d) * w
                fy += (dy / d) * w

        f = math.hypot(fx, fy)
        if f < 1e-6:
            # 合力为 0（罕见）：沿航线反方向后退
            gx, gy = self._pick_local_goal()
            fx, fy = lx - gx, ly - gy
            f = math.hypot(fx, fy) or 1e-6
        target_dir = (fx / f, fy / f)

        # 方向低通：指数平滑到 target_dir，避免逐帧翻转
        ALPHA = 0.4
        pd = self.escape_dir
        if pd == (0.0, 0.0):
            self.escape_dir = target_dir
        else:
            self.escape_dir = (pd[0] + ALPHA * (target_dir[0] - pd[0]),
                               pd[1] + ALPHA * (target_dir[1] - pd[1]))
        dx, dy = self.escape_dir
        dn = math.hypot(dx, dy) or 1e-6
        dx, dy = dx / dn, dy / dn

        # 满速逃逸，尽快拉开距离
        target_mag = self.max_speed
        tvx = dx * target_mag
        tvy = dy * target_mag
        # 速度连续：受 max_accel*dt 限制，从当前速度平滑逼近（不瞬时反向）
        max_dv = self.max_accel * CTRL_DT
        evx = self._approach(self.cur_vx, tvx, max_dv)
        evy = self._approach(self.cur_vy, tvy, max_dv)
        return evx, evy, True

    @staticmethod
    def _approach(cur, target, max_step):
        """把 cur 向 target 逼近，单步最大 max_step（限速收敛，防突跳）。"""
        if target > cur + max_step:
            return cur + max_step
        if target < cur - max_step:
            return cur - max_step
        return target

    # ==================== 状态机：方向迟滞 + 局部极小值检测 + 脱困 ====================
    def _nearest_obstacle_dist(self, obs_enu, dyn_obs):
        """最近障碍物距离（阶段A：静态部分走 ESDF 查表，与 front/left/right 同源）。

        动态柱子中心距减半径=表面距（USE_DYNAMIC_OBS=False 时 dyn_obs 为空）。
        ESDF 已含建场膨胀，返回的是「到膨胀后障碍表面」的距离，比裸激光点更保守
        且与 DWA 轨迹净空语义一致，根治「scan_min 与 front 数据源不一致」。
        """
        m = float("inf")
        if self.local is None:
            return m
        lx, ly = self.local[0], self.local[1]
        d = self.esdf.get_distance(lx, ly)
        if d < 0.0:
            d = 0.0   # UNKNOWN/越界 → 保守判 0
        m = d
        for ox, oy, ovx, ovy in dyn_obs:
            d = math.hypot(ox - lx, oy - ly) - self.obs_radius
            if d < m:
                m = d
        return m

    def _esdf_away_dir(self):
        """远离最近障碍的方向：ESDF 梯度指向距离增大方向（即远离障碍）。

        相比 1/d² 原始激光斥力，ESDF 梯度由「网格量化距离场 + 中心差分」给出，
        逐帧平滑稳定，不会在贴脸时被最近 1~2 个噪声点主导而剧烈振荡。在 U 形
        死胡同等场景，梯度天然指向距离场增大的方向（即开口/开阔区），正是脱困
        要去的方向。
        返回单位向量 (x,y)；梯度为 0（开放空间中心/越界/未就绪）时返回 None。
        """
        if self.local is None or not self.esdf.ready:
            return None
        lx, ly = self.local[0], self.local[1]
        gx, gy = self.esdf.get_gradient(lx, ly)
        g = math.hypot(gx, gy)
        if g > 1e-6:
            return gx / g, gy / g
        return None

    def _goal_relax(self, d_goal_final):
        """接近最终目标的放宽系数 [0,1]：0=远离目标不放宽，1=已到目标完全放宽。

        goal 距墙很近的口袋场景（如 U 形死角 goal 距墙 1.15m < SAFETY_DIST）里，
        软障碍代价/脱困触发/紧急逃逸会阻止无人机完成最后接近，形成「进袋→卡→退→
        重进」死循环。用此系数随接近目标线性放宽软逻辑；硬淘汰 safe_radius 不受影响，
        仍保证真实碰撞安全（goal 距墙 1.15m 远大于 safe_radius 0.6m）。
        """
        if d_goal_final >= GOAL_RELAX_DIST:
            return 0.0
        return 1.0 - d_goal_final / GOAL_RELAX_DIST

    def _terminal_approach(self, goal_local, d_goal, obs_enu, dyn_obs):
        """终末接近控制器：接近 goal 时直接速度控制逼近，完全绕过 DWA 采样/状态机。

        为什么需要独立控制器（根治「进袋→贴墙→TRAP→RETREAT→重进」死循环）：
          goal 在 U 形袋深处（距三面墙 1.15m < SAFETY_DIST）时，DWA 采样会被
          hard_radius 淘汰所有前进候选（只剩 hover），TRAP 检测又把「goal 处正常
          贴墙悬停」误判成卡死 → 拉进 ESCAPE/RETREAT → 退出重进，无限循环。
          goal 本身物理可达（1.15m > safe_radius 0.6m），问题不在避障，在「终点
          在墙边」这一场景根本不该走 DWA 绕行/脱困逻辑。

        做法：直接朝 goal 做速度 P 控制，速度上限用「刹停约束」v <= sqrt(2a·(d-tol))，
        保证在 goal 处速度→0、不 overshoot 贴墙；再叠加朝 goal 方向预测净空的硬保护
        （真的会撞墙才降速），最后速度平滑输出。
        """
        lx, ly = self.local[0], self.local[1]
        dx = goal_local[0] - lx
        dy = goal_local[1] - ly
        d = math.hypot(dx, dy) or 1e-6
        ux, uy = dx / d, dy / d
        # 速度上限：保证能在 goal 前刹停（匀减速 v²=2a·s，s = d - arrive_tol）
        v_limit = math.sqrt(2.0 * self.max_accel * max(0.0, d - self.arrive_tol))
        v = min(self.max_speed, v_limit)
        vx, vy = ux * v, uy * v
        # 硬保护：用「当前位置到障碍的距离」（ESDF 查表，非预测轨迹）判断是否真会撞。
        # ⚠️ 不能用 _trajectory_min_dist（2.4m 前瞻）：goal 距墙 1.15m，任何朝 goal 的
        # 2.4m 预测都必然越过 goal 撞到底墙 → clear 恒 0 → 速度被错误归零、到不了 goal。
        # 正确判据：只要当前位置距障碍 > safe_radius（机体+裕度+当前速度制动），就安全，
        # 可以继续逼近；逼近到 goal 前速度本来就受 v_limit 约束趋 0，不会 overshoot 撞墙。
        cur_clear = self._nearest_obstacle_dist(obs_enu, dyn_obs)
        safe_r = ROBOT_RADIUS + SAFETY_MARGIN + v * v / (2.0 * BRAKING_DECEL)
        if cur_clear < safe_r:
            # 当前位置就已贴得太近 → 降到能刹住的速度（或 0）
            v_ok = math.sqrt(max(0.0, (cur_clear - ROBOT_RADIUS - SAFETY_MARGIN)
                                * 2.0 * BRAKING_DECEL))
            v = max(0.0, min(v, v_ok))
            vx, vy = ux * v, uy * v
        # 速度平滑，不突跳
        max_dv = self.max_accel * CTRL_DT
        return (self._approach(self.cur_vx, vx, max_dv),
                self._approach(self.cur_vy, vy, max_dv))

    def _side_clearances(self, obs_enu, dyn_obs):
        """左/右净空：无人机沿左向/右向横移的预测轨迹到障碍的最近距离。

        ⚠️ 语义修正（根治数据不一致）：旧实现取「障碍点的横向坐标分量」lat，
        正前方有障碍(lat≈0)时左右净空被同时压到 ~0，导致误判「左右都堵」、
        触发 LEFT↔RIGHT 无限互换。现在统一用与 front 完全相同的几何定义——
        `_trajectory_min_dist`（沿某方向以 max_speed 前进 PREDICT_TIME 秒的
        预测轨迹到障碍的最近距离），left/right/front 三者语义彻底一致。
        """
        if self.local is None:
            return float("inf"), float("inf")
        gx, gy = self._pick_local_goal()
        lx, ly = self.local[0], self.local[1]
        dx, dy = gx - lx, gy - ly
        gd = math.hypot(dx, dy) or 1e-6
        ux, uy = dx / gd, dy / gd
        lft_x, lft_y = -uy, ux   # 左向单位向量
        # 左向横移的预测轨迹净空 / 右向横移的预测轨迹净空
        left_clear = self._trajectory_min_dist(lft_x * self.max_speed,
                                               lft_y * self.max_speed,
                                               obs_enu, dyn_obs)
        right_clear = self._trajectory_min_dist(-lft_x * self.max_speed,
                                                -lft_y * self.max_speed,
                                                obs_enu, dyn_obs)
        return left_clear, right_clear

    def _open_side_sign(self, obs_enu, dyn_obs):
        """返回更开阔的一侧：+1=左，-1=右（用于 ESCAPE 脱困方向）。"""
        left_clear, right_clear = self._side_clearances(obs_enu, dyn_obs)
        return 1.0 if left_clear >= right_clear else -1.0

    def _detect_trap(self, min_disp=None):
        """时间窗口检测局部极小值：窗口内位移小 且 到目标距离几乎没下降 → True。

        这是「卡死/局部极小值」的核心判据：不只看瞬时速度（速度可能来回抵消），
        而是统计 TRAP_WINDOW 秒内的净位移与目标进展，两者都小说明被障碍物困住。
        min_disp：位移阈值覆盖（接近目标时放宽，避免 goal 处正常贴墙悬停误触发）。
        """
        thresh = TRAP_MIN_DISP if min_disp is None else min_disp
        if len(self.traj_hist) < 2:
            return False
        t0, x0, y0, d0 = self.traj_hist[0]
        t1, x1, y1, d1 = self.traj_hist[-1]
        if t1 - t0 < TRAP_WINDOW * 0.5:   # 窗口未填满
            return False
        disp = math.hypot(x1 - x0, y1 - y0)
        prog = d0 - d1
        return disp < thresh and prog < TRAP_MIN_PROG

    def _update_avoid_dir(self, obs_enu, dyn_obs, min_obs, now):
        """方向迟滞：有障碍压力时锁定绕行方向，避免逐帧左右翻转。"""
        if min_obs > self.warn_dist:
            # 无障碍压力 → 解除方向锁
            if now >= self.avoid_dir_lock_until:
                self.avoid_dir = 0.0
            return
        lx, ly = self.local[0], self.local[1]
        gx, gy = self._pick_local_goal()
        dx, dy = gx - lx, gy - ly
        gd = math.hypot(dx, dy) or 1e-6
        lft_x, lft_y = -(dy / gd), (dx / gd)
        lat_v = self.cur_vx * lft_x + self.cur_vy * lft_y   # 当前横向速度 >0 左 <0 右
        left_clear, right_clear = self._side_clearances(obs_enu, dyn_obs)

        if self.avoid_dir == 0.0:
            # 首次锁定：若已有明显横向速度则跟随当前方向，否则取更开阔一侧
            if abs(lat_v) > 0.3:
                self.avoid_dir = 1.0 if lat_v > 0 else -1.0
            else:
                self.avoid_dir = 1.0 if left_clear >= right_clear else -1.0
            self.avoid_dir_lock_until = now + DIR_LOCK_TIME
        elif now >= self.avoid_dir_lock_until:
            # 锁到期：只有另一侧净空明显更大（> DIR_SWITCH_MARGIN）才允许切换，
            # 否则保持原方向，彻底消除「左绕一下又右绕回来」
            if self.avoid_dir > 0 and right_clear > left_clear + DIR_SWITCH_MARGIN:
                self.avoid_dir = -1.0
                self.avoid_dir_lock_until = now + DIR_LOCK_TIME
            elif self.avoid_dir < 0 and left_clear > right_clear + DIR_SWITCH_MARGIN:
                self.avoid_dir = 1.0
                self.avoid_dir_lock_until = now + DIR_LOCK_TIME

    def _escape_move(self, now, obs_enu, dyn_obs):
        """ESCAPE 脱困运动：沿 escape_sign 横移 + 轻微前进，速度 ESCAPE_SPEED。

        前进分量压到 0.3（保持对目标推进），横移 0.95 是主方向——脱困核心是「横向
        离开局部极小值」，不是直冲障碍。速度受 max_accel*dt 限制，方向平滑不突跳。
        若含前进分量的速度预测会碰撞，则退化为纯横向（避免脱困时又撞障碍）。
        """
        lx, ly = self.local[0], self.local[1]
        gx, gy = self._pick_local_goal()
        dx, dy = gx - lx, gy - ly
        gd = math.hypot(dx, dy) or 1e-6
        ux, uy = dx / gd, dy / gd
        # 左向单位向量
        lft_x, lft_y = -uy, ux

        # 主方向：锁定方向的横向 0.95 + 目标方向前进 0.3
        fwd = 0.3
        lat = 0.95
        dir_x = lft_x * self.escape_sign * lat + ux * fwd
        dir_y = lft_y * self.escape_sign * lat + uy * fwd
        dn = math.hypot(dir_x, dir_y) or 1e-6
        dir_x, dir_y = dir_x / dn, dir_y / dn

        # 碰撞守卫：含前进分量的速度预测会撞 → 退化为纯横向
        if self._trajectory_min_dist(dir_x * ESCAPE_SPEED, dir_y * ESCAPE_SPEED,
                                     obs_enu, dyn_obs) < SAFE_DIST:
            dir_x, dir_y = lft_x * self.escape_sign, lft_y * self.escape_sign

        max_dv = self.max_accel * CTRL_DT
        vx = self._approach(self.cur_vx, dir_x * ESCAPE_SPEED, max_dv)
        vy = self._approach(self.cur_vy, dir_y * ESCAPE_SPEED, max_dv)
        return vx, vy

    def _enter_escape(self, now, d_goal, reason, obs_enu, dyn_obs):
        """进入 ESCAPE（只在状态切换时调用一次）：选方向、锁定、记录基准。

        关键：escape_sign 只在「进入」时用 _open_side_sign 选一次，之后持久锁定，
        不再每帧重选（根治 LEFT→RIGHT→LEFT→RIGHT 震荡）。
        """
        self.nav_state = "ESCAPE"
        self.escape_start_time = now
        self.escape_ref_prog = d_goal
        self.escape_ref_pos = (self.local[0], self.local[1])
        self.escape_sign = self._open_side_sign(obs_enu, dyn_obs)
        self.escape_lock_until = now + ESCAPE_LOCK_TIME
        self.escape_clear_cycles = 0
        self.escape_enter_reason = reason
        rospy.logwarn("[ESCAPE ENTER] dir=%s reason=%s",
                      "LEFT" if self.escape_sign > 0 else "RIGHT", reason)

    def _exit_escape(self, now, reason, progressed):
        """退出 ESCAPE：日志 + 清理 + 把方向锁接力给 avoid_dir（RECOVER 沿用）。"""
        rospy.logwarn("[ESCAPE EXIT] reason=%s progressed=%.2fm", reason, progressed)
        self.avoid_dir = self.escape_sign   # 方向锁接力，RECOVER 期间继续沿同侧绕行
        self.escape_start_time = 0.0
        self.escape_sign = 0.0
        self.escape_clear_cycles = 0
        self.escape_enter_reason = ""

    def _pick_retreat_dir(self, obs_enu, dyn_obs):
        """选择后退方向：评估 back/back-left/back-right/left/right 共 5 个方向的
        预测净空，取净空最大的方向作为后退方向（墙角/死胡同时自动选斜后退脱困）。"""
        lx, ly = self.local[0], self.local[1]
        gx, gy = self._pick_local_goal()
        dx, dy = gx - lx, gy - ly
        gd = math.hypot(dx, dy) or 1e-6
        ux, uy = dx / gd, dy / gd          # 朝目标
        lft_x, lft_y = -uy, ux             # 左
        cands = (
            (-ux, -uy),                     # back
            (-ux + lft_x, -uy + lft_y),     # back-left
            (-ux - lft_x, -uy - lft_y),     # back-right
            (lft_x, lft_y),                 # left
            (-lft_x, -lft_y),               # right
        )
        best_dir = (-ux, -uy)
        best_clear = -1.0
        for cx, cy in cands:
            cn = math.hypot(cx, cy) or 1e-6
            cx, cy = cx / cn, cy / cn
            clr = self._trajectory_min_dist(cx * RETREAT_SPEED, cy * RETREAT_SPEED,
                                            obs_enu, dyn_obs)
            if clr > best_clear:
                best_clear = clr
                best_dir = (cx, cy)

        # 四面全堵（最大预测净空仍 < SAFE_DIST）→ 5 个离散方向都不可靠。
        # 改用「ESDF 梯度方向」（远离最近障碍、指向距离场增大的方向，即开口/开阔区）。
        # 相比 1/d² 原始激光斥力，ESDF 梯度逐帧平滑稳定，不会在贴脸时被最近
        # 1~2 个噪声激光点主导而剧烈振荡（那是上一版「斥力方向每帧横跳」的根因）。
        if best_clear < SAFE_DIST:
            away = self._esdf_away_dir()
            if away is not None:
                cand_clear = self._trajectory_min_dist(away[0] * RETREAT_SPEED,
                                                       away[1] * RETREAT_SPEED,
                                                       obs_enu, dyn_obs)
                min_safe = ROBOT_RADIUS + SAFETY_MARGIN
                # ① 梯度方向更开阔 → 采用（梯度稳定，无需 +0.2 防抖裕度，直接 > 即可）
                # ② 即便不更开阔，若 best_dir 本身净空 < 机体半径（朝它走必撞墙），
                #    也必须强制改走梯度方向——目标在身后时 back=-ux 会指向身后的墙，
                #    这是「retreat_dir 冲进 U 形死角」的根因。
                if cand_clear > best_clear or best_clear < min_safe:
                    best_dir = away
                    rospy.logwarn("[RETREAT] 四面堵，采用 ESDF 梯度方向(%.2f,%.2f) clear=%.2f (best=%.2f%s)",
                                  best_dir[0], best_dir[1], cand_clear, best_clear,
                                  "" if cand_clear > best_clear else "，原方向不安全强制改")
                else:
                    rospy.logwarn_throttle(2.0,
                        "[RETREAT] 四面堵，ESDF 梯度方向(%.2f,%.2f) clear=%.2f 不优于 best %.2f，保留",
                        away[0], away[1], cand_clear, best_clear)
        return best_dir

    def _enter_retreat(self, now, obs_enu, dyn_obs, reason=""):
        """进入 RETREAT（只在状态切换时调用一次）：选方向、锁定、记录基准位置。"""
        self.nav_state = "RETREAT"
        self.retreat_start_time = now
        self.retreat_start_pos = (self.local[0], self.local[1])
        self.retreat_dir = self._pick_retreat_dir(obs_enu, dyn_obs)
        self.retreat_relock_until = now + RETREAT_RELOCK_TIME  # 刚进入先锁一段，防立即横跳
        rospy.logwarn("[RETREAT ENTER] dir=(%.2f,%.2f) reason=%s",
                      self.retreat_dir[0], self.retreat_dir[1], reason)

    def _retreat_controller(self, now, obs_enu, dyn_obs):
        """RETREAT 独立控制器（最高优先级，DWA/ESCAPE 都不得覆盖）。

        持续沿锁定方向低速后退，直到：
          ① 最近障碍 > RETREAT_CLEAR 且 持续 ≥ RETREAT_MIN_TIME 且 累计位移 ≥ RETREAT_MIN_DIST
          ② 或超时 RETREAT_MAX_TIME（强制退出重规划，防死胡同死锁）
        退出时清方向锁 + 清紧急逃逸迟滞 + 触发重规划。
        """
        min_obs = self._nearest_obstacle_dist(obs_enu, dyn_obs)
        elapsed = now - self.retreat_start_time
        disp = math.hypot(self.local[0] - self.retreat_start_pos[0],
                          self.local[1] - self.retreat_start_pos[1])

        # ---- 安全检查：锁定方向净空已不足 → 重选方向 + 限速 ----
        # RETREAT 是最高优先级、完全绕开 DWA/紧急逃逸的「裸奔」控制器，若还死守
        # 进入时锁定的方向，一旦该方向在后退过程中被墙/柱堵住，就会全速撞上去。
        # 但重选必须满足两点，否则每帧在不同方向间横跳、实际原地抖动：
        #   ① 候选方向净空必须明显优于当前方向（> locked_clear + RETREAT_SWITCH_MARGIN）；
        #   ② 重选后锁定 RETREAT_RELOCK_TIME 秒，期间不再重选。
        dx, dy = self.retreat_dir
        locked_clear = self._trajectory_min_dist(dx * RETREAT_SPEED, dy * RETREAT_SPEED,
                                                 obs_enu, dyn_obs)
        min_safe = ROBOT_RADIUS + SAFETY_MARGIN  # 低速后退的最低安全半径
        relock_ok = now >= self.retreat_relock_until
        if locked_clear < min_safe and relock_ok:
            new_dir = self._pick_retreat_dir(obs_enu, dyn_obs)
            new_clear = self._trajectory_min_dist(new_dir[0] * RETREAT_SPEED,
                                                  new_dir[1] * RETREAT_SPEED,
                                                  obs_enu, dyn_obs)
            # 候选方向必须明显更开阔才切换，否则保留原方向（避免「换更差方向」横跳）
            if new_clear > locked_clear + RETREAT_SWITCH_MARGIN:
                self.retreat_dir = new_dir
                self.retreat_relock_until = now + RETREAT_RELOCK_TIME
                dx, dy = new_dir
                locked_clear = new_clear
                rospy.logwarn("[RETREAT] 锁定方向被堵(%.2f)，切换到(%.2f,%.2f) clear=%.2f",
                              min_safe, dx, dy, locked_clear)
            else:
                # 没有更开阔方向：保持原方向，但降速到能刹住的极限，避免原地横跳
                rospy.logwarn_throttle(1.0,
                    "[RETREAT] 当前方向被堵(%.2f)且无更优方向(候选%.2f)，保持原方向慢退",
                    locked_clear, new_clear)
        # 自适应限速：净空 vs 制动距离，保证锁定的后退速度能刹住不撞
        spd = RETREAT_SPEED
        braking = spd * spd / (2.0 * BRAKING_DECEL)
        if locked_clear < ROBOT_RADIUS + SAFETY_MARGIN + braking:
            spd = math.sqrt(max(0.0, (locked_clear - ROBOT_RADIUS - SAFETY_MARGIN)
                                * 2.0 * BRAKING_DECEL))
            spd = max(0.15, min(RETREAT_SPEED, spd))   # 下限 0.15，避免完全停死卡住

        # 退出条件：足够净空 + 最短时间 + 最小位移，或超时兜底
        if (min_obs > RETREAT_CLEAR and elapsed >= RETREAT_MIN_TIME and disp >= RETREAT_MIN_DIST) \
                or elapsed >= RETREAT_MAX_TIME:
            self._exit_retreat(now)
            return 0.0, 0.0   # 本帧已触发重规划，速度归零

        # ---- 输出速度 ----
        # 贴脸（min_obs < SAFE_DIST）：旧速度可能仍带着「朝墙」的分量，_approach 平滑
        # 会保留该分量把无人机继续送进墙（日志中 command 与 retreat_dir 相反即此因）。
        # 此时直接以低速爬离速度沿锁定方向运动，不做平滑滞后——先脱离接触，再谈平滑。
        if min_obs < SAFE_DIST:
            # 沿「远离最近障碍」方向低速爬离（比锁定方向更安全：锁定方向可能被选错）
            away = self._esdf_away_dir()
            if away is not None:
                dx, dy = away
            vx = dx * RETREAT_CREEP_SPEED
            vy = dy * RETREAT_CREEP_SPEED
        else:
            max_dv = self.max_accel * CTRL_DT
            vx = self._approach(self.cur_vx, dx * spd, max_dv)
            vy = self._approach(self.cur_vy, dy * spd, max_dv)
        return vx, vy

    def _exit_retreat(self, now):
        """退出 RETREAT：清方向锁 + 清紧急逃逸迟滞 + 强制重规划。

        关键：后退把无人机带离障碍膨胀区后，当前位置已不在 occupied 格内，
        此时重规划不会再报 START_OCCUPIED。清掉 escaping 迟滞，避免退出后
        又被贴脸紧急逃逸立刻拉回 ESCAPE。
        """
        self.nav_state = "NORMAL_NAV"
        self.avoid_dir = 0.0
        self.escape_sign = 0.0
        self.retreat_start_time = 0.0
        self.escaping = False
        self.escape_dir = (0.0, 0.0)
        rospy.logwarn("[RETREAT EXIT] 脱离死胡同，重新规划")
        cur_map = (self.local[0] + self.offset[0], self.local[1] + self.offset[1])
        if not self._plan_path(cur_map):
            # 仍失败（罕见，如仍贴着障碍）：刷新时间戳，让 DWA 继续把无人机带开
            rospy.logwarn("[RETREAT EXIT] 重规划失败，DWA 继续引导")
        self.best_progress = float("inf")
        self.last_progress_time = now

    def _escape_persist(self, now, d_goal, obs_enu, dyn_obs):
        """ESCAPE 持久状态：每帧沿锁定方向运动，直到满足退出条件之一。

        方向切换规则（根治 LEFT→RIGHT→LEFT→RIGHT 无限互换）：
          - 当前方向净空 >= HARD_BLOCK：继续沿当前方向；
          - 当前方向被堵(cur_clear < HARD_BLOCK)且锁到期：
              另一侧 >= EXIT_CLEARANCE → 切换一次并重新锁定；
              另一侧也 < HARD_BLOCK → 进 RETREAT（左右都堵，禁止互换）。
        退出条件（满足任一）：① 离开障碍影响区域 ② 出口稳定净空且有进展
        ③ 目标距离持续下降 ④ 达到最大脱困时间。
        """
        elapsed = now - self.escape_start_time
        min_obs = self._nearest_obstacle_dist(obs_enu, dyn_obs)
        left_clear, right_clear = self._side_clearances(obs_enu, dyn_obs)
        cur_clear = left_clear if self.escape_sign > 0 else right_clear
        other_clear = right_clear if self.escape_sign > 0 else left_clear

        # ---- 方向切换（只在锁到期后评估一次，且必须左右都判断）----
        if now >= self.escape_lock_until and cur_clear < HARD_BLOCK:
            if other_clear >= EXIT_CLEARANCE:
                # 另一侧有明确出口 → 切换一次并重新锁定
                new_sign = -self.escape_sign
                rospy.logwarn("[ESCAPE] 方向 %s 堵(%.2fm)，另一侧出口 %.2fm，切换 %s",
                              "LEFT" if self.escape_sign > 0 else "RIGHT", cur_clear,
                              other_clear, "LEFT" if new_sign > 0 else "RIGHT")
                self.escape_sign = new_sign
                self.escape_lock_until = now + ESCAPE_LOCK_TIME
            elif other_clear < HARD_BLOCK:
                # 左右都堵 → RETREAT（禁止 LEFT/RIGHT 无限互换）
                rospy.logwarn("[ESCAPE] 左右都堵(left=%.2f right=%.2f)，进入 RETREAT",
                              left_clear, right_clear)
                self._enter_retreat(now, obs_enu, dyn_obs, "BOTH_BLOCKED")
                return self._retreat_controller(now, obs_enu, dyn_obs)
            # 否则（另一侧介于 HARD_BLOCK~EXIT_CLEARANCE）：保持当前方向继续试探

        # ---- 目标方向出口净空（退出条件②用）----
        gx, gy = self._pick_local_goal()
        lx, ly = self.local[0], self.local[1]
        gdx, gdy = gx - lx, gy - ly
        ggd = math.hypot(gdx, gdy) or 1e-6
        goal_clear = self._trajectory_min_dist(gdx / ggd * self.max_speed,
                                               gdy / ggd * self.max_speed,
                                               obs_enu, dyn_obs)
        if goal_clear > EXIT_CLEARANCE:
            self.escape_clear_cycles += 1
        else:
            self.escape_clear_cycles = 0

        # ---- 退出条件判断 ----
        progressed = self.escape_ref_prog - d_goal   # 到目标距离下降量
        exit_reason = None
        if min_obs > self.warn_dist and elapsed >= ESCAPE_MIN_TIME:
            exit_reason = "CLEARED"                          # ① 离开障碍影响区域
        elif self.escape_clear_cycles >= EXIT_CLEAR_CYCLES and progressed > 0:
            exit_reason = "EXIT_AVAILABLE"                   # ② 出口稳定净空且有进展
        elif progressed >= RECOVER_PROG and elapsed >= ESCAPE_MIN_TIME:
            exit_reason = "PROGRESS"                         # ③ 目标距离持续下降
        elif elapsed >= ESCAPE_MAX_TIME:
            exit_reason = "TIMEOUT"                          # ④ 最大脱困时间

        if exit_reason is not None:
            self._exit_escape(now, exit_reason, progressed)
            self.nav_state = "RECOVER"
            self.recover_start_time = now
            self.escape_cooldown_until = now + ESCAPE_COOLDOWN
            vx, vy, _ = self._dwa_step(obs_enu, dyn_obs, d_goal)
            return vx, vy

        # ---- 持续沿锁定方向运动 ----
        rospy.loginfo_throttle(1.0,
            "[ESCAPE] locked_dir=%s elapsed=%.1fs left=%.2f right=%.2f goal_clear=%.2f",
            "LEFT" if self.escape_sign > 0 else "RIGHT", elapsed,
            left_clear, right_clear, goal_clear)
        return self._escape_move(now, obs_enu, dyn_obs)

    def _state_machine_step(self, obs_enu, dyn_obs, d_goal):
        """状态机主决策（优先级从高到低）：
            RETREAT > ESCAPE > RECOVER > 陷阱检测 > 方向锁 DWA

        控制权规则（问题9）：
          - RETREAT 拥有最高优先级，独立控制器输出，DWA/ESCAPE/紧急逃逸都不得覆盖；
          - ESCAPE 是持久状态，进入时锁方向，紧急逃逸只作为「进入触发器」；
          - 只有 NORMAL_NAV / AVOID_OBSTACLE 才走普通 DWA。
        """
        now = rospy.get_time()
        min_obs = self._nearest_obstacle_dist(obs_enu, dyn_obs)
        # 接近最终目标的放宽系数：口袋场景 goal 距墙 < SAFETY_DIST，软逻辑（TRAP/紧急逃逸/
        # 软障碍代价）会阻止最后接近 → 随距离放宽；硬淘汰 safe_radius 不受影响。
        relax = self._goal_relax(d_goal)

        # ---- 1) RETREAT 最高优先级：独立后退控制器，禁止任何其他逻辑覆盖 ----
        if self.nav_state == "RETREAT":
            return self._retreat_controller(now, obs_enu, dyn_obs)

        # ---- 2) 贴脸紧急逃逸（只在非 RETREAT 时作为进入 ESCAPE 的触发器）----
        evx, evy, escaped = self._emergency_escape(obs_enu, dyn_obs, relax)
        if escaped:
            if self.nav_state != "ESCAPE":
                # 首次进入：选方向、锁定，首帧用紧急逃逸速度快速拉开
                self._enter_escape(now, d_goal, "COLLISION", obs_enu, dyn_obs)
                return evx, evy
            # 已在 ESCAPE：忽略紧急逃逸重算的方向，沿用锁定方向
            return self._escape_persist(now, d_goal, obs_enu, dyn_obs)

        # ---- 3) ESCAPE 持久状态：沿锁定方向运动 + 判断退出 ----
        if self.nav_state == "ESCAPE":
            return self._escape_persist(now, d_goal, obs_enu, dyn_obs)

        # ---- 4) RECOVER 状态：方向锁接力 + DWA 恢复巡航 ----
        if self.nav_state == "RECOVER":
            if now - self.recover_start_time >= RECOVER_TIME:
                self.nav_state = "NORMAL_NAV"
                self.avoid_dir = 0.0
                rospy.loginfo("RECOVER 完成，回到 NORMAL_NAV")
            else:
                vx, vy, _ = self._dwa_step(obs_enu, dyn_obs, d_goal)
                return vx, vy

        # ---- 5) 局部极小值检测 → 进入 ESCAPE ----
        # 接近最终目标时放宽 TRAP：goal 处「悬停减速/微调」会被 TRAP 误判成局部极小值，
        # 拉进 ESCAPE/RETREAT 形成进袋→退→重进死循环。relax>0 时按比例提高 TRAP 触发
        # 阈值，接近目标处不再因正常贴墙悬停而误触发脱困。
        trap_thresh = TRAP_MIN_DISP * (1.0 + relax * 3.0)   # 接近目标时放宽到 4 倍
        if min_obs < self.warn_dist and self._detect_trap(trap_thresh):
            if now >= self.escape_cooldown_until:
                self._enter_escape(now, d_goal, "TRAP", obs_enu, dyn_obs)
                return self._escape_move(now, obs_enu, dyn_obs)

        # ---- 6) 方向迟滞：更新 avoid_dir 锁 ----
        self._update_avoid_dir(obs_enu, dyn_obs, min_obs, now)

        # ---- 7) 状态标注 + 正常 DWA（无合法轨迹 → 进 RETREAT 后退）----
        self.nav_state = "AVOID_OBSTACLE" if min_obs < self.warn_dist else "NORMAL_NAV"
        vx, vy, has_valid = self._dwa_step(obs_enu, dyn_obs, d_goal)
        if not has_valid:
            # DWA 无路可走（hard_radius 淘汰全部候选）→ 后退脱离，而不是悬停/原地撞
            rospy.logwarn_throttle(1.0, "[DWA] 无合法轨迹 → 进入 RETREAT")
            self._enter_retreat(now, obs_enu, dyn_obs, "NO_VALID_TRAJECTORY")
            return self._retreat_controller(now, obs_enu, dyn_obs)
        return vx, vy

    # ==================== RViz 可视化 ====================
    def _marker(self, ns, mid, mtype, r, g, b, a, scale):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale        # SPHERE 三维都要设，否则只设 x/y 会变成扁球/不显示
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
        # ⚠️ 四元数必须初始化：默认 w=0 会让 RViz 报 "Uninitialized quaternion"，
        # 导致 Marker 姿态异常甚至不显示。单位四元数 = (0,0,0,1)。
        m.pose.orientation.w = 1.0
        return m

    def _publish_global_path(self):
        m = self._marker("global_path", 0, Marker.LINE_STRIP, 0.0, 1.0, 0.0, 1.0, 0.05)
        for x, y in self.path:
            m.points.append(Point(x, y, self.altitude))
        self.path_pub.publish(m)

    def _publish_inflated(self):
        m = self._marker("inflated", 0, Marker.POINTS, 1.0, 0.0, 0.0, 0.6, self.grid.resolution)
        g = self.grid
        for iy in range(g.height):
            for ix in range(g.width):
                if not g.is_free((ix, iy)):
                    wx = g.origin[0] + ix * g.resolution
                    wy = g.origin[1] + iy * g.resolution
                    m.points.append(Point(wx, wy, self.altitude))
        self.inflated_pub.publish(m)

    def _publish_window(self, pts):
        m = self._marker("dwa_window", 0, Marker.POINTS, 1.0, 1.0, 0.0, 0.8, 0.04)
        for x, y in pts:
            m.points.append(Point(x + self.offset[0], y + self.offset[1], self.altitude))
        self.window_pub.publish(m)

    def _publish_scan(self, obs_enu):
        m = self._marker("laser", 0, Marker.POINTS, 0.0, 0.5, 1.0, 0.6, 0.04)
        for x, y in obs_enu:
            m.points.append(Point(x + self.offset[0], y + self.offset[1], self.altitude))
        self.scan_pub.publish(m)

    def _publish_pos(self):
        if self.local is None:
            return
        m = self._marker("current_pos", 0, Marker.SPHERE, 0.0, 1.0, 1.0, 0.9, 0.25)
        m.pose.position = Point(self.local[0] + self.offset[0],
                                self.local[1] + self.offset[1], self.altitude)
        self.pos_pub.publish(m)

    # ==================== A* 规划/重规划 ====================
    def _nearest_free_cell_world(self, start_map, max_radius_m=3.0):
        """在 start_map 附近搜索最近自由栅格（膨胀后），返回其世界坐标或 None。

        问题8：A* 报 START_OCCUPIED 时，不是无限重试，而是找最近的合法起点重规划。
        找不到才返回 None（调用方进 RETREAT）。
        """
        inflated = inflate_grid(self.grid, self.inflation_m)
        start_cell = inflated.world_to_cell(start_map)
        r_max = int(math.ceil(max_radius_m / inflated.resolution))
        for r in range(1, r_max + 1):
            # 按曼哈顿环逐层向外搜（螺旋近似），保证「最近优先」
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    if start_cell is None:
                        cell = (dx, dy)
                    else:
                        cell = (start_cell[0] + dx, start_cell[1] + dy)
                    if inflated.in_bounds(cell) and inflated.is_free(cell):
                        wx, wy = inflated.cell_to_world(cell)
                        # 还要校验这个点不贴近障碍物（用激光净空）——可选，保持简单：
                        return (wx, wy)
        return None

    def _plan_path(self, start_map):
        """A* 全局规划：先尝试原始 start，若 START_OCCUPIED 则找最近自由栅格重试。

        返回：True=规划成功；False=失败（调用方应进 RETREAT 后退脱离，而非反复重试）。
        """
        inflated = inflate_grid(self.grid, self.inflation_m)
        route = plan(inflated, start_map, self.goal, connectivity=8)
        if not route.success and route.reason == "START_OCCUPIED":
            # 起点落在膨胀障碍内 → 找最近自由栅格重试（问题8）
            nearest = self._nearest_free_cell_world(start_map)
            if nearest is not None:
                rospy.logwarn("[A*] START_OCCUPIED，改用最近自由栅格 (%.2f,%.2f) 重规划",
                              nearest[0], nearest[1])
                route = plan(inflated, nearest, self.goal, connectivity=8)
        if not route.success:
            rospy.logerr("A* 规划失败: %s", route.reason)
            return False
        self.path = smooth_path(route.points, samples_per_segment=6)
        self.path_local = [(x - self.offset[0], y - self.offset[1])
                           for x, y in self.path]
        self._publish_global_path()
        rospy.loginfo("A* 规划：%d 航点 -> %d 平滑点（膨胀 %.2fm）",
                      len(route.points), len(self.path), self.inflation_m)
        return True

    # ==================== PX4 参数/模式 ====================
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

    # ==================== 主流程 ====================
    def run(self):
        while not rospy.is_shutdown() and not self.state.connected:
            self.rate.sleep()
        rospy.loginfo("MAVROS 已连接")
        while self.local is None and not rospy.is_shutdown():
            self.rate.sleep()
        while self._iris_world() is None and not rospy.is_shutdown():
            self.rate.sleep()

        # ---- 坐标标定 ----
        iris = self._iris_world()
        self.offset = (iris[0] - self.local[0], iris[1] - self.local[1])
        spawn = (iris[0], iris[1])
        rospy.loginfo("出生点(map)=%.2f,%.2f | local=%.2f,%.2f | offset=%.2f,%.2f",
                      spawn[0], spawn[1], self.local[0], self.local[1],
                      self.offset[0], self.offset[1])

        # ---- A* 起点 ----
        sx = self.start[0] if self.start[0] is not None else spawn[0]
        sy = self.start[1] if self.start[1] is not None else spawn[1]

        # ---- 首次全局规划 ----
        if not self._plan_path((sx, sy)):
            raise SystemExit(1)
        self._publish_inflated()

        # ---- FCU 参数 ----
        self._configure_fcu_params()

        # ---- 预热 setpoint（原地零速）----
        for _ in range(10):
            self._send_vel(0.0, 0.0, 0.0)
            self.rate.sleep()

        # ---- 切 OFFBOARD ----
        if not self.mode_srv(0, "OFFBOARD").mode_sent:
            rospy.logerr("切换 OFFBOARD 失败，退出")
            raise SystemExit(1)
        for _ in range(50):
            if self.state.mode == "OFFBOARD":
                break
            self._send_vel(0.0, 0.0, 0.0)
            self.rate.sleep()
        if self.state.mode != "OFFBOARD":
            rospy.logerr("OFFBOARD 未生效，当前模式: %s", self.state.mode)
            raise SystemExit(1)
        rospy.loginfo("已进入 OFFBOARD")

        # ---- 解锁 ----
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

        # ---- 起飞：竖直速度爬升到目标高度 ----
        while not rospy.is_shutdown():
            vz = self._altitude_vz()
            self._send_vel(0.0, 0.0, vz)
            if self.local is not None and abs(self.altitude - self.local[2]) < 0.2:
                break
            self.rate.sleep()
        rospy.loginfo("已起飞到 %.2fm", self.local[2] if self.local else 0.0)

        # ---- DWA 主循环 ----
        self.cur_vx = self.cur_vy = 0.0
        self.best_progress = float("inf")
        self.last_progress_time = rospy.get_time()
        goal_local = (self.goal[0] - self.offset[0], self.goal[1] - self.offset[1])
        rospy.loginfo("开始 DWA 跟踪，目标(map)=%.2f,%.2f", self.goal[0], self.goal[1])

        while not rospy.is_shutdown():
            if self.local is None:
                self.rate.sleep()
                continue

            # 收到 /goal 新目标 -> 重规划
            if self.goal_updated:
                self.goal_updated = False
                goal_local = (self.goal[0] - self.offset[0], self.goal[1] - self.offset[1])
                cur_map = (self.local[0] + self.offset[0], self.local[1] + self.offset[1])
                self._plan_path(cur_map)
                self.best_progress = float("inf")
                self.last_progress_time = rospy.get_time()

            lx, ly = self.local[0], self.local[1]
            d_goal = math.hypot(lx - goal_local[0], ly - goal_local[1])

            # 到达目标
            if d_goal < self.arrive_tol:
                rospy.loginfo("到达终点 %.2f,%.2f，AUTO.LAND 降落", self.goal[0], self.goal[1])
                self._send_vel(0.0, 0.0, 0.0)
                self.mode_srv(0, "AUTO.LAND")
                break

            # EKF 漂移监测：方差过大 -> 悬停等待恢复
            if self.ekf_bad:
                rospy.logwarn_throttle(2.0,
                    "EKF 位姿方差过大 std=%.2f > %.2f，悬停等待恢复",
                    self.ekf_std, self.ekf_acc_limit)
                self._send_vel(0.0, 0.0, self._altitude_vz())
                self.cur_vx = self.cur_vy = 0.0
                self.rate.sleep()
                continue

            # 停滞检测 -> 自动重规划
            if d_goal < self.best_progress - STUCK_PROGRESS:
                self.best_progress = d_goal
                self.last_progress_time = rospy.get_time()
            elif rospy.get_time() - self.last_progress_time > STUCK_TIME:
                rospy.logwarn("停滞 %.1fs 无进展，触发重规划", STUCK_TIME)
                cur_map = (lx + self.offset[0], ly + self.offset[1])
                if self._plan_path(cur_map):
                    self.best_progress = d_goal
                    self.last_progress_time = rospy.get_time()
                else:
                    # 重规划失败（如 START_OCCUPIED）：保留旧路径继续走，
                    # 并刷新时间戳退避，避免每周期(20Hz)反复重试刷屏
                    self.best_progress = d_goal
                    self.last_progress_time = rospy.get_time()

            # ---- 状态机决策（方向迟滞 + 局部极小值检测 + 脱困）----
            # ESDF 建场用全量激光（dense=True，远端不漏障碍）；斥力/可视化用降采样
            obs_enu = self._scan_points_enu()
            obs_dense = self._scan_points_enu(dense=True)
            # 阶段A：每帧以当前位置为中心重建 ESDF 距离场（ENU 局部系）
            self.esdf.update((lx, ly), obs_dense)
            # 本阶段去排动态柱预测（静态未知环境）；保留代码，开关 USE_DYNAMIC_OBS
            dyn_obs = self._dynamic_obstacles() if self.use_dynamic_obs else []

            # ---- 终末接近：距 goal 足够近时直接刹停逼近，绕过 DWA/状态机脱困 ----
            # U 形袋等「终点在墙边」场景：DWA 采样会在 hard_radius 处淘汰所有前进候选
            # （只剩 hover），TRAP 检测再把正常贴墙悬停误判成卡死 → 死循环。终末控制器
            # 用刹停约束 v²=2a·(d-tol) 保证在 goal 处速度→0、不 overshoot 贴墙。
            if d_goal < TERMINAL_DIST:
                # 清掉残留的脱困状态，避免 RETREAT/ESCAPE 锁干扰终末接近输出
                if self.nav_state in ("RETREAT", "ESCAPE", "RECOVER"):
                    self.nav_state = "NORMAL_NAV"
                    self.avoid_dir = 0.0
                    self.escape_sign = 0.0
                    self.escaping = False
                    self.escape_dir = (0.0, 0.0)
                vx, vy = self._terminal_approach(goal_local, d_goal, obs_enu, dyn_obs)
            else:
                vx, vy = self._state_machine_step(obs_enu, dyn_obs, d_goal)

            # ---- 维护轨迹时间窗口（供局部极小值检测用）----
            now = rospy.get_time()
            self.traj_hist.append((now, lx, ly, d_goal))
            while self.traj_hist and now - self.traj_hist[0][0] > TRAP_WINDOW:
                self.traj_hist.popleft()

            vz = self._altitude_vz()
            self._send_vel(vx, vy, vz)
            self.cur_vx, self.cur_vy = vx, vy

            # RViz 可视化
            self._publish_scan(obs_enu)
            self._publish_pos()

            # ---- 结构化调试日志（问题10）----
            # scan_min：实时激光看到的最近障碍（static + dynamic 表面距，与 front/left/right 同源）
            scan_min = self._nearest_obstacle_dist(obs_enu, dyn_obs)
            left_clear, right_clear = self._side_clearances(obs_enu, dyn_obs)
            gx, gy = self._pick_local_goal()
            gdx, gdy = gx - lx, gy - ly
            ggd = math.hypot(gdx, gdy) or 1e-6
            front_clear = self._trajectory_min_dist(gdx / ggd * self.max_speed,
                                                    gdy / ggd * self.max_speed,
                                                    obs_enu, dyn_obs)
            cur_spd = math.hypot(self.cur_vx, self.cur_vy)
            braking_now = cur_spd * cur_spd / (2.0 * BRAKING_DECEL)
            safe_radius = ROBOT_RADIUS + SAFETY_MARGIN + braking_now
            # dbg = (best_vx, best_vy, heading_cost, obs_cost, prog_cost,
            #        valid, min_clear, braking, static_clear, dynamic_clear)
            dbg = getattr(self, "_dbg_dwa",
                          (0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, float("inf"), float("inf")))
            escape_dir_s = "LEFT" if self.escape_sign > 0 else ("RIGHT" if self.escape_sign < 0 else "NONE")
            rospy.loginfo_throttle(1.0,
                "[SAFETY] front=%.2f left=%.2f right=%.2f scan_min=%.2f "
                "robot_radius=%.2f safety_margin=%.2f braking=%.2f safe_radius=%.2f | "
                "[DWA] valid=%d static_clear=%.2f dynamic_clear=%.2f total_clear=%.2f "
                "best_vx=%.2f best_vy=%.2f | "
                "[STATE] state=%s retreat_dir=%s escape_dir=%s",
                front_clear, left_clear, right_clear, scan_min,
                ROBOT_RADIUS, SAFETY_MARGIN, braking_now, safe_radius,
                dbg[5], dbg[8], dbg[9], dbg[6],
                dbg[0], dbg[1],
                self.nav_state,
                ("(%.2f,%.2f)" % self.retreat_dir) if self.nav_state == "RETREAT" else "NONE",
                escape_dir_s)
            if self.nav_state == "RETREAT":
                r_elapsed = rospy.get_time() - self.retreat_start_time
                r_disp = math.hypot(lx - self.retreat_start_pos[0], ly - self.retreat_start_pos[1])
                rospy.loginfo_throttle(1.0,
                    "[RETREAT] command_vx=%.2f command_vy=%.2f locked_dir=(%.2f,%.2f) "
                    "elapsed=%.2fs distance_from_enter=%.2fm",
                    vx, vy, self.retreat_dir[0], self.retreat_dir[1], r_elapsed, r_disp)
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("dwa_avoidance")
    DwaAvoidance().run()
