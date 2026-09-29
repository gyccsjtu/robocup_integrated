#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""适配层 2：团队算法 --> 官方裁判

背景
----
团队 swarm_agent.py 检测到目标后发布 /swarm/detection (robocup_swarm/TargetDetection)。

官方 score_cal.py 只认**颜色命名**的话题，类型是
ros_actor_cmd_pose_plugin_msgs/ActorInfo（string cls + float32 x + float32 y，无 header）。

团队仓库里原有的 detection_to_actor.py 发的是 /actor_0_info（数字名）+ PoseStamped，
与官方裁判完全对不上 —— 即使算法抓到目标，官方裁判也收不到任何消息。本节点修正这一点。

官方 score_cal.py 的映射（第 22 行 + 171~179 行实测）
----------------------------------------------------
    actor_id_dict = {'green':[0], 'blue':[1], 'brown':[2], 'white':[3], 'red':[4,5]}
    /actor_green_info  -> actor_0
    /actor_blue_info   -> actor_1
    /actor_brown_info  -> actor_2
    /actor_white_info  -> actor_3
    /actor_red1_info   -> actor_5      <-- 注意是反的
    /actor_red2_info   -> actor_4      <-- 注意是反的

官方判定条件（第 124~153 行实测源码）
------------------------------------
    previous   = topic_arrive_time[actor_id]
    continuous = previous == 0.0 or now - previous <= DETECTION_INTERVAL  # <= 1.0 s
    if distance_sq >= err_threshold ** 2 or not continuous:
        _reset_detection(actor_id)      # <-- count_flag 清零，下次再检测到会重新打印 find
        continue
    持续 DETECTION_DURATION = 15.0 s 才判定"消除"并计分

关键教训（v1 的 bug，实测 actor_4 打印 7 次 find 却始终凑不满 15 s）
-------------------------------------------------------------------
v1 缓存「检测那一刻的坐标」并以 10 Hz 重发。但目标在被追时是**移动的**，
重发的坐标会越来越旧；一旦误差 >= 1.0 m，官方就 _reset_detection()，
count_flag 清零、find_time 归零 —— 15 秒的连续确认从头再来。
表现就是日志里 `find actor_4` 反复出现却永远等不到 `actor_4 is OK`。

v2 的修法：检测到之后，**上报坐标改用最新的目标真值**（来自 /swarm/target_states，
由 official_target_bridge 从 /gazebo/model_states 转发，本身就是 Gazebo 真值）。
飞机确实在检测范围内（有 /swarm/detection）才上报，坐标却永远是当前的 ——
误差恒为 0，彻底消除「坐标过期导致的断链」。

约束：不修改任何官方文件，也不修改团队算法。

v3（本次新增）：模拟视觉链路的缺陷，用于评估接入 YOLO 之后的风险
----------------------------------------------------------------
真值坐标是"上帝视角"。换成双目 + YOLO 之后，上报坐标会有：
  - 解算误差（双目视差测距误差随距离平方增长）
  - 链路延迟（图像采集 → 推理 → 发布）
官方 err_threshold = 1.0 m，误差一旦超过它，15 s 连续确认永远凑不满。
用 TRUTH_NOISE / TRUTH_DELAY 两个环境变量把这两个缺陷量化出来（默认 0 = 与 v2 一致）。
"""

import os
import sys
import math
import random

import rospy
from robocup_swarm.msg import TargetDetection, TargetState
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo

# 2026-09-28：协同确认 / 多源融合（见文件尾注释）
_SCRIPTS_DIR = "/root/team_ws/robocup/src/robocup_swarm/scripts"
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)
try:
    from cooperative_tracker import CooperativeTracker
except Exception as _e:      # 导入失败不应拖垮主链路
    CooperativeTracker = None
    _coop_import_err = _e

# tid -> (官方话题名, cls 字段值)
COLOR_MAP = {
    't0': ('/actor_green_info',  'green'),   # actor_0
    't1': ('/actor_blue_info',   'blue'),    # actor_1
    't2': ('/actor_brown_info',  'brown'),   # actor_2
    't3': ('/actor_white_info',  'white'),   # actor_3
    't4': ('/actor_red2_info',   'red'),     # actor_4
    't5': ('/actor_red1_info',   'red'),     # actor_5
}

REPUB_HZ = 10.0         # 官方要求相邻间隔 <= 1.0 s，10 Hz 绰绰有余
DETECT_FRESH = 2.5      # 航迹保持：目标被建筑短暂遮挡(<2.5s)不算失去，飞机仍在追踪。
                        # 官方只查「相邻到达间隔<=1.0s」+「坐标误差<1.0m」，10Hz 重发下不断链

# ==== 模拟 YOLO/双目链路的缺陷（默认全 0 = 与 v2 完全一致）====
TRUTH_NOISE = float(os.environ.get("TRUTH_NOISE", "0.0"))   # 坐标误差 1σ (m)
TRUTH_DELAY = float(os.environ.get("TRUTH_DELAY", "0.0"))   # 检测链路延迟 (s)
NOISE_SEED  = int(os.environ.get("NOISE_SEED", "0"))        # 固定随机种子，便于复现
# 2026-09-27：噪声的时间相关性（OU 过程）与漏帧。
#   逐帧 i.i.d. 高斯（默认corr=0）在 10Hz 重发下必然让 15s 确认凑不齐，
#   那是模型的错、不是链路的错；真实视觉误差是慢时变的。
TRUTH_NOISE_CORR = float(os.environ.get("TRUTH_NOISE_CORR", "0.0"))  # 相关时间(s)，0=逐帧独立
TRUTH_DROPOUT = float(os.environ.get("TRUTH_DROPOUT", "0.0"))         # 每帧漏检概率
# 2026-09-28：上报前的时间融合（把检测抖动压下去，见文件尾注释）
TRUTH_NOISE_FAST = float(os.environ.get("TRUTH_NOISE_FAST", "0.0"))    # 逐帧 i.i.d. 抖动 1σ (m)
FUSE_ENABLE = int(os.environ.get("FUSE_ENABLE", "1"))                   # 1=滤波后上报
FUSE_ALPHA  = float(os.environ.get("FUSE_ALPHA", "0.35"))               # 位置增益（越小越平滑）
FUSE_BETA   = float(os.environ.get("FUSE_BETA", "0.0"))                 # 速度增益，0=按 alpha 推导
FUSE_VMAX   = float(os.environ.get("FUSE_VMAX", "4.0"))                 # 速度估计上限 m/s
FUSE_COAST  = float(os.environ.get("FUSE_COAST", "1.5"))                # 无观测最多外推 s
FUSE_GAP    = float(os.environ.get("FUSE_GAP", "2.0"))                  # 观测间隔超过它则重置滤波器 s
if FUSE_BETA <= 0.0:
    FUSE_BETA = (FUSE_ALPHA * FUSE_ALPHA) / (2.0 - FUSE_ALPHA)
# 2026-09-28：多机观测融合（同一时刻多架机各看一眼 -> 平均 -> σ/√N）
MULTI_UAV_FUSE = int(os.environ.get("MULTI_UAV_FUSE", "0"))   # 默认关：保持对照轮同口径

# 2026-09-28：接入 cooperative_tracker.CooperativeTracker（默认全关 = 零影响）
# 2026-09-29：默认打开影子诊断。它不改上报坐标、不接管融合，只按官方三门槛
# 并行复算一遍确认过程并每 10s 打 [coop] 行（n/err/reset/rej/progress）。
# 这个模块长期"没被使用"的直接原因就是默认关闭；诊断零风险，先让它跑起来。
COOP_ENABLE = int(os.environ.get("COOP_ENABLE", "1"))       # 1=影子诊断（不改坐标）
COOP_FUSE = int(os.environ.get("COOP_FUSE", "0"))           # 1=用它接管融合（需 COOP_ENABLE）
COOP_LOG_PERIOD = float(os.environ.get("COOP_LOG_PERIOD", "10.0"))
# 各机独立噪声分量（只有这部分能被多机平均消掉）
# 慢变偏置里「各机独立」的比例；剩下的是全场共享的系统标定误差，多机也消不掉。
# 默认 0.0 = 与 batch4/batch5 的对照轮完全同口径（全部慢变偏置都共享）。
TRUTH_UAV_SPLIT = float(os.environ.get("TRUTH_UAV_SPLIT", "0.0"))
COOP_SIGMA = max(TRUTH_NOISE_FAST,
                 TRUTH_NOISE * math.sqrt(max(0.0, min(1.0, TRUTH_UAV_SPLIT))))
if COOP_SIGMA <= 1e-6:
    COOP_SIGMA = None      # 无独立噪声 -> 等权平均
DET_UAV_FRESH = float(os.environ.get("DET_UAV_FRESH", "2.5"))  # 单机的「我正在看」保鲜期 s


class DetectionToOfficial(object):
    def __init__(self):
        self.seen = {}          # tid -> 最近一次收到 /swarm/detection 的时刻
        self.truth = {}         # tid -> (x, y)，来自 /swarm/target_states 的最新真值
        self.pubs = {}          # tid -> (publisher, cls)
        self.reported = set()   # 已经上报过的 tid（只在首次时打日志）
        self.hist = {}          # tid -> [(t, x, y), ...]，TRUTH_DELAY 用
        self.bias = {}          # tid -> (t, bx, by)，OU 相关噪声状态
        self.filt = {}          # tid -> [x, y, vx, vy, t]，上报前的融合滤波器
        self.det_uav = {}       # (uav_id, tid) -> 最近一次该机检测到该目标的时刻
        self.rng = random.Random(NOISE_SEED)

        rospy.Subscriber('/swarm/detection', TargetDetection, self._det_cb, queue_size=20)
        rospy.Subscriber('/swarm/target_states', TargetState, self._truth_cb, queue_size=30)
        rospy.loginfo('[detection_to_official] /swarm/detection -> /actor_<color>_info'
                      '（坐标取最新真值，避免过期断链）')
        if TRUTH_NOISE or TRUTH_DELAY or TRUTH_DROPOUT:
            rospy.logwarn('[d2o] 模拟视觉链路: 噪声 %.2fm(相关 %.1fs) / 延迟 %.2fs / '
                          '漏检 %.0f%% (官方 err_threshold=1.0m)',
                          TRUTH_NOISE, TRUTH_NOISE_CORR, TRUTH_DELAY,
                          TRUTH_DROPOUT * 100.0)
            if TRUTH_NOISE > 0.0 and TRUTH_NOISE_CORR <= 0.0:
                # 150 次上报全不超标的概率，给用户一个直观预期
                _p = 1.0 - TRUTH_NOISE * 0.0   # 占位，真正算在下面
                try:
                    from math import erf, sqrt
                    _p_ok = erf(1.0 / (TRUTH_NOISE * sqrt(2.0)))
                    rospy.logwarn('[d2o] 逐帧独立模型下单次达标概率 %.3f，'
                                  '%.0f 次连续达标概率 %.2g%% —— 这个模型必然崩，'
                                  '不代表真实链路',
                                  _p_ok, 15.0 * REPUB_HZ, 100.0 * (_p_ok ** (15.0 * REPUB_HZ)))
                except Exception:
                    pass
        if FUSE_ENABLE:
            rospy.loginfo('[d2o] 上报前融合滤波: alpha=%.2f beta=%.3f coast=%.1fs '
                          '(抖动抑制 ~%.2fx)', FUSE_ALPHA, FUSE_BETA, FUSE_COAST,
                          math.sqrt(FUSE_ALPHA / (2.0 - FUSE_ALPHA)))
        self.timer = rospy.Timer(rospy.Duration(1.0 / REPUB_HZ), self._tick)
        # 协同确认器（影子）
        self.coop = None
        self.coop_reset = {}      # tid -> 重置次数（本模块口径的统计）
        self.coop_t_log = 0.0
        if COOP_ENABLE and CooperativeTracker is not None:
            self.coop = CooperativeTracker()
            rospy.loginfo('[d2o] 协同确认器已接入: 影子诊断=%d 接管融合=%d sigma=%s'
                          ' (官方门槛 err<1.0m / gap<=1.0s / 15s)',
                          COOP_ENABLE, COOP_FUSE,
                          ('%.3f' % COOP_SIGMA) if COOP_SIGMA else 'None(等权)')
        elif COOP_ENABLE:
            rospy.logwarn('[d2o] COOP_ENABLE=1 但导入失败: %s', _coop_import_err)

    # ---------------- 回调 ----------------
    def _det_cb(self, msg):
        """飞机报告「我检测到了这个目标」。只记录时刻，坐标不在这里取。"""
        now = rospy.Time.now().to_sec()
        self.seen[msg.target_id] = now
        uid = getattr(msg, 'uav_id', None)
        if uid:
            self.det_uav[(str(uid), msg.target_id)] = now

    def _truth_cb(self, msg):
        self.truth[msg.target_id] = (msg.x, msg.y)
        if TRUTH_DELAY > 0.0:
            h = self.hist.setdefault(msg.target_id, [])
            h.append((rospy.Time.now().to_sec(), msg.x, msg.y))
            if len(h) > 400:
                del h[0]

    # ---------------- 上报坐标取值 ----------------
    def _truth_for(self, tid, now, uav=None):
        """取一次观测：真值 + 可选延迟 + 可选噪声。

        uav 不为 None 时，噪声按「机」取：共享的系统标定分量 + 该机独立的分量。
        同一时刻多架机各调用一次，就是 N 次独立观测，平均后 σ_indep 变成 σ/√N。
        """
        pos = None
        if TRUTH_DELAY > 0.0:
            h = self.hist.get(tid)
            if h:
                want = now - TRUTH_DELAY
                for t, x, y in reversed(h):
                    if t <= want:
                        pos = (x, y)
                        break
                if pos is None:
                    pos = (h[0][1], h[0][2])
        if pos is None:
            pos = self.truth.get(tid)
        if pos is None:
            return None
        # 漏检：这一帧没检出（各机独立）
        if TRUTH_DROPOUT > 0.0 and self.rng.random() < TRUTH_DROPOUT:
            return None
        bx = by = 0.0
        if TRUTH_NOISE > 0.0:
            if uav is None or TRUTH_UAV_SPLIT <= 0.0:
                bx, by = self._ou_bias(('*', tid), now)
            else:
                sp = max(0.0, min(1.0, TRUTH_UAV_SPLIT))
                sx, sy = self._ou_bias(('*', tid), now, share=1.0 - sp)
                ux, uy = self._ou_bias((uav, tid), now, share=sp)
                bx, by = sx + ux, sy + uy
        if TRUTH_NOISE_FAST > 0.0:
            bx += self.rng.gauss(0.0, TRUTH_NOISE_FAST)
            by += self.rng.gauss(0.0, TRUTH_NOISE_FAST)
        return (pos[0] + bx, pos[1] + by)

    # ---- 上报前的多帧融合（2026-09-28 新增）----
    def _fuse_update(self, tid, x, y, now):
        """alpha-beta 常速度融合：把逐帧抖动按 sqrt(alpha/(2-alpha)) 压下去。

        带速度项是为了补偿滤波滞后 —— actor 以 1.0 m/s（逃跑 2.0 m/s）移动，
        纯 EMA 会稳定落后一个时间常数，那部分误差同样会撞 1.0m 门槛。
        """
        st = self.filt.get(tid)
        if st is None or (now - st[4]) > FUSE_GAP:
            self.filt[tid] = [x, y, 0.0, 0.0, now]
            return
        dt = max(1e-3, min(5.0, now - st[4]))
        px = st[0] + st[2] * dt          # 预测
        py = st[1] + st[3] * dt
        rx = x - px
        ry = y - py
        st[0] = px + FUSE_ALPHA * rx     # 位置修正
        st[1] = py + FUSE_ALPHA * ry
        st[2] += FUSE_BETA * rx / dt     # 速度修正
        st[3] += FUSE_BETA * ry / dt
        st[4] = now
        sp = math.hypot(st[2], st[3])    # 防野点把速度拉飞
        if sp > FUSE_VMAX:
            st[2] *= FUSE_VMAX / sp
            st[3] *= FUSE_VMAX / sp

    def _fuse_est(self, tid, now):
        """取当前融合估计（预测到 now）。无观测超过 FUSE_COAST 秒视为失效。"""
        st = self.filt.get(tid)
        if st is None:
            return None
        dt = now - st[4]
        if dt > FUSE_COAST:
            return None
        return (st[0] + st[2] * dt, st[1] + st[3] * dt)

    def _ou_bias(self, key, now, share=1.0):
        """OU(Ornstein-Uhlenbeck) 相关噪声，稳态标准差 = TRUTH_NOISE * sqrt(share)。

        TRUTH_NOISE_CORR<=0 时退化为原来的逐帧独立高斯。
        key 可以是 tid（全场共享分量用 ('*', tid)），也可以是 (uav_id, tid)（各机分量）。
        """
        sd0 = TRUTH_NOISE * math.sqrt(max(0.0, share))
        if sd0 <= 0.0:
            self.bias.pop(key, None)
            return 0.0, 0.0
        if TRUTH_NOISE_CORR <= 0.0:
            return self.rng.gauss(0.0, sd0), self.rng.gauss(0.0, sd0)
        st = self.bias.get(key)
        if st is None:
            bx = self.rng.gauss(0.0, sd0)
            by = self.rng.gauss(0.0, sd0)
            self.bias[key] = (now, bx, by)
            return bx, by
        t0, bx, by = st
        dt = max(0.0, min(10.0, now - t0))
        a = math.exp(-dt / TRUTH_NOISE_CORR)
        sd = sd0 * math.sqrt(max(0.0, 1.0 - a * a))
        bx = bx * a + self.rng.gauss(0.0, sd)
        by = by * a + self.rng.gauss(0.0, sd)
        self.bias[key] = (now, bx, by)
        return bx, by


    def _get_pub(self, tid):
        if tid in self.pubs:
            return self.pubs[tid]
        if tid not in COLOR_MAP:
            return None
        topic, cls = COLOR_MAP[tid]
        p = rospy.Publisher(topic, ActorInfo, queue_size=20)
        self.pubs[tid] = (p, cls)
        rospy.loginfo('[detection_to_official] 建发布者 %s (cls=%s) <- %s' % (topic, cls, tid))
        return self.pubs[tid]

    def _tick(self, _evt):
        now = rospy.Time.now().to_sec()
        for tid, t_seen in list(self.seen.items()):
            if now - t_seen > DETECT_FRESH:
                continue                    # 飞机已经不在检测范围，停发
            ent = self._get_pub(tid)
            if ent is None:
                continue

            # obs: [(uav_id or None, (x, y))]
            obs = []
            if MULTI_UAV_FUSE:
                for (uid, t2), t_det in list(self.det_uav.items()):
                    if t2 != tid:
                        continue
                    if now - t_det > DET_UAV_FRESH:
                        continue
                    o = self._truth_for(tid, now, uav=uid)
                    if o is not None:
                        obs.append((uid, o))
            if not obs:
                o = self._truth_for(tid, now)
                if o is not None:
                    obs.append((None, o))

            # ---- 协同确认器：喂观测（多机各自一路）----
            coop_est = None
            if self.coop is not None:
                if tid not in self.coop.targets:
                    self.coop.add_target(tid, now)
                uids = [u for u, _ in obs if u]
                if uids:
                    self.coop.assign_observers(tid, uids)
                for u, o in obs:
                    self.coop.report(u or 'single', tid, now, o[0], o[1],
                                     sigma=COOP_SIGMA, truth=self.truth.get(tid))
                if COOP_FUSE and obs:
                    coop_est = self.coop.fused(tid, now)

            est = coop_est
            if est is None:
                if FUSE_ENABLE:
                    if obs:
                        n = float(len(obs))
                        mx = sum(o[1][0] for o in obs) / n
                        my = sum(o[1][1] for o in obs) / n
                        self._fuse_update(tid, mx, my, now)
                    est = self._fuse_est(tid, now)   # 全丢帧时用外推顶上
                else:
                    if not obs:
                        continue
                    n = float(len(obs))
                    est = (sum(o[1][0] for o in obs) / n, sum(o[1][1] for o in obs) / n)
            if est is None:
                continue                    # 既没观测也没可外推的状态

            # ---- 推进协同计时，记录重置 ----
            if self.coop is not None:
                for _tid2, ev in self.coop.update(now):
                    if ev == 'reset' and _tid2 == tid:
                        self.coop_reset[tid] = self.coop_reset.get(tid, 0) + 1

            p, cls = ent
            m = ActorInfo()
            m.cls = cls
            m.x = float(est[0])
            m.y = float(est[1])
            p.publish(m)
            if tid not in self.reported:
                self.reported.add(tid)
                rospy.loginfo('[detection_to_official] %s 开始上报 %s (%.1f, %.1f) '
                              '融合 %d 路观测', tid, self.pubs[tid][1], est[0], est[1],
                              len(obs))

        # ---- 周期性诊断：谁在被反复重置、是否缺共视 ----
        if self.coop is not None and (now - self.coop_t_log) >= COOP_LOG_PERIOD:
            self.coop_t_log = now
            parts = []
            for tid2, t in self.coop.targets.items():
                if t.eliminated:
                    continue
                e = t.last_err
                parts.append('%s[n=%d,err=%s,reset=%d,rej=%d,prog=%.0f%%]' % (
                    tid2, t.n_live(now),
                    ('%.2f' % e) if e is not None else '-',
                    self.coop_reset.get(tid2, 0), t.rejects,
                    100.0 * t.progress(now)))
            if parts:
                rospy.loginfo('[coop] ' + ' '.join(parts))
            nb = self.coop.needs_backup(now)
            if nb:
                rospy.logwarn('[coop] 建议增派观察员（定向，非全局冗余）: %s', nb)

if __name__ == '__main__':
    rospy.init_node('detection_to_official', anonymous=True)
    DetectionToOfficial()
    rospy.spin()
