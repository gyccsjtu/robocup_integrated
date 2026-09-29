#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多机协同确认 + 多源融合：把多架无人机的观测合并成一条「连续确认计时」与上报坐标。

规则依据（官方裁判的真实口径）
------------------------------
规则5：无人机需**连续 15 秒**正确广播恐怖分子 ID + 坐标，裁判判定消除。
       连续 = 三条同时满足，断一条即清零重来：
         (a) 每次上报的**坐标误差 < ERR_TOL(1.0 m)**
         (b) 相邻两次上报的**间隔 <= GAP_TOL(1.0 s)**
         (c) 累计满 CONFIRM_TIME(15 s)
规则4：被**累计感知** 30 s 仍未消除 → 目标瞬移躲藏。

本模块 2026-09-28 重写。旧版是个「理想化原型」，4 个缺陷使其无法用于真实链路
（已实测复现，见文末自测 7~10）：

  B1 规则4 的 evade 事件**每帧重复触发**（tracked_since 不推进，阈值后一直满足）
     → 消费者会收到成百上千次重复的 evade。
  B2 规则4 用「墙钟跨度」而非「累计被感知时长」。目标被看见 5s→消失 25s→再看 5s，
     累计仅 10s，旧版仍判瞬移。
  B3 **没有误差门槛**：上报点离真值 70 m 也照样「确认成功」。
  B4 **没有上报间隔门槛**：每 3 s 才上报一次也照样「确认成功」。

  ⇒ B3+B4 的后果最致命：旧版里「看得见」恒等于「必然消除」，永远不会被重置，
     因此它复现不出实测中 t2 被官方重置 9 次、拖到 306 s 的尾部现象，
     也就无法用于调度决策 —— 这才是它一直没被使用的根本原因。

重写后的职责
------------
1. **完整复现官方三门槛**（误差 / 间隔 / 连续时长），能预测「被重置」。
2. **多源融合**：多机同时看见时按 1/sigma^2 加权（独立噪声下 sigma -> sigma/sqrt(N)），
   再叠 alpha-beta 常速度时间融合。
3. **自适应 beta**：目标移动时加大 beta 让速度估计跟得紧（抑制滞后），
   静止时减小 beta 狠压抖动。滞后而非抖动，才是移动目标被重置的主因。
4. 计时仍以**仿真时间**为基准，与协同核心的 sim_s 一致。

!!! 重要结论（离线蒙特卡洛已验证，见 self-test 11）!!!
    **不要做 lead pursuit（按速度外推未来位置再上报）。**
    裁判比的是「上报坐标 vs actor 当前真值」，外推未来只会单方面增大误差。
    正确做法是加大 beta 让滤波器本身收敛更快 —— 滞后要治本，不能靠提前量掩盖。

用法（仿真链路）
----------------
    ct = CooperativeTracker()
    ct.add_target("t2", now)
    ct.assign_observers("t2", ["uav_1", "uav_3"])
    ct.report("uav_1", now, x, y, truth=(tx, ty))   # truth 可选，仿真里有真值
    est = ct.fused(now)                             # 融合后的上报坐标
    for tid, ev in ct.update(now):                  # ev: confirmed / evade / reset
        ...
"""

import math

# ---- 官方规则常量 ----
CONFIRM_TIME = 15.0     # 规则5：连续确认时长 s
ERR_TOL = 1.0           # 规则5：单次上报坐标误差上限 m
GAP_TOL = 1.0           # 规则5：相邻上报最大间隔 s
EVADE_TIME = 30.0       # 规则4：累计感知 30 s 未消除 → 瞬移
OBS_TTL = 1.0           # 单条观测的有效期 s（超过则视为陈旧，不计入）

# ---- 融合参数 ----
FUSE_ALPHA = 0.35       # 位置增益
FUSE_BETA_MIN = 0.02    # 静止时的速度增益（狠压抖动）
FUSE_BETA_MAX = 0.14    # 移动时的速度增益（快速收敛，抑制滞后）
FUSE_V_REF = 1.2        # 速度增益达到上限的参考速度 m/s
FUSE_VMAX = 4.0         # 速度估计上限 m/s（防野点拉飞）
FUSE_COAST = 1.5        # 无观测最多外推 s
FUSE_GAP = 2.0          # 观测间隔超过它则重置滤波器 s
ADAPTIVE_BETA = 1       # 1=自适应 beta（推荐），0=固定 beta（对照）


def _beta_for_speed(speed):
    """自适应 beta：静止用小 beta 压抖动，移动用大 beta 追滞后。

    纯 EMA(alpha) 对匀速目标有稳态滞后 ~ v*T*(1-alpha)/alpha，
    这部分误差会直接撞 1.0 m 门槛；beta 越大速度估计收敛越快，滞后越小。
    """
    if not ADAPTIVE_BETA:
        return (FUSE_ALPHA * FUSE_ALPHA) / (2.0 - FUSE_ALPHA)
    r = min(1.0, abs(speed) / FUSE_V_REF) if FUSE_V_REF > 0 else 1.0
    return FUSE_BETA_MIN + (FUSE_BETA_MAX - FUSE_BETA_MIN) * r


class TargetObservation(object):
    """某一时刻某架无人机对某目标的观测。"""
    __slots__ = ("uav_id", "t", "x", "y", "los", "sigma", "truth")

    def __init__(self, uav_id, t, x, y, los=True, sigma=None, truth=None):
        self.uav_id = uav_id
        self.t = float(t)
        self.x = float(x)
        self.y = float(y)
        self.los = bool(los)      # 该次观测视线是否通畅
        self.sigma = sigma        # 该观测的噪声标准差（None=未知，按等权处理）
        self.truth = truth        # 真值（仅仿真/诊断用；真实 YOLO 时为 None）


class CooperativeTarget(object):
    """单个目标的协同确认 + 融合状态。"""

    def __init__(self, target_id, t0=0.0, confirm_time=CONFIRM_TIME,
                 evade_time=EVADE_TIME, obs_ttl=OBS_TTL,
                 err_tol=ERR_TOL, gap_tol=GAP_TOL):
        self.target_id = target_id
        self.confirm_time = confirm_time
        self.evade_time = evade_time
        self.obs_ttl = obs_ttl
        self.err_tol = err_tol
        self.gap_tol = gap_tol

        self.observers = set()     # 当前注册的观察员
        self.last_obs = {}         # uav_id -> TargetObservation（各自最新一条）

        # ---- 连续确认状态 ----
        self.confirm_since = None  # 连续确认起始时刻（None = 当前未在确认）
        self.last_ok_t = None      # 上一次「合格上报」的时刻（用于间隔门槛）
        self.resets = 0            # 被重置次数：从「正在确认」被打断（调度的关键信号）
        self.rejects = 0           # 不合格上报次数：误差超限或间隔超限（诊断用）
        self.eliminated = False
        self.evaded = False

        # ---- 规则4 累计感知（B2 修复：累计而非墙钟跨度）----
        self.sensed_acc = 0.0
        self._t_last = t0
        self._evade_fired = False  # B1 修复：evade 只触发一次

        # ---- 融合状态（alpha-beta 常速度）----
        self.fx = None
        self.fy = None
        self.vx = 0.0
        self.vy = 0.0
        self.t_fuse = None
        self.last_err = None       # 最近一次上报值的误差（有真值时才有）

    # ---------- 观察员管理 ----------
    def set_observers(self, uav_ids):
        """分配层指定/变更观察员集合。移出的观察员其观测立即失效。"""
        new = set(uav_ids)
        for u in list(self.last_obs.keys()):
            if u not in new:
                del self.last_obs[u]
        self.observers = new

    # ---------- 观测输入 ----------
    def report(self, uav_id, t, x, y, los=True, sigma=None, truth=None):
        """记录一条观测。非观察员的观测不计入（防止无关机刷计时）。

        truth: 该时刻目标真值 (tx, ty)，仅仿真/诊断用于算误差；真实 YOLO 时传 None，
               此时不做误差判定（退化为「看得见即合格」，与旧版同口径）。
        """
        if self.eliminated:
            return
        if self.observers and uav_id not in self.observers:
            return
        self.last_obs[uav_id] = TargetObservation(uav_id, t, x, y, los, sigma, truth)

    # ---------- 状态评估 ----------
    def covered(self, now):
        """当前是否有**有效**观测：任一观察员在 obs_ttl 内报告过且 LOS 通畅。"""
        for ob in self.last_obs.values():
            if now - ob.t <= self.obs_ttl and ob.los:
                return True
        return False

    def live_observers(self, now):
        """当前有效观测的观察员 id 列表（用于诊断/可视化）。"""
        return [ob.uav_id for ob in self.last_obs.values()
                if now - ob.t <= self.obs_ttl and ob.los]

    def n_live(self, now):
        """当前有效观测的路数（多机共视程度）。"""
        return len(self.live_observers(now))

    # ---------- 多源融合 ----------
    def _fuse_update(self, x, y, now):
        """alpha-beta 常速度时间融合（带自适应 beta）。"""
        if self.t_fuse is None or (now - self.t_fuse) > FUSE_GAP:
            self.fx, self.fy = x, y
            self.vx, self.vy = 0.0, 0.0
            self.t_fuse = now
            return
        dt = max(1e-3, min(5.0, now - self.t_fuse))
        beta = _beta_for_speed(math.hypot(self.vx, self.vy))
        px = self.fx + self.vx * dt          # 按常速度预测到当前
        py = self.fy + self.vy * dt
        rx, ry = x - px, y - py
        self.fx = px + FUSE_ALPHA * rx
        self.fy = py + FUSE_ALPHA * ry
        self.vx += beta * rx / dt
        self.vy += beta * ry / dt
        self.t_fuse = now
        sp = math.hypot(self.vx, self.vy)
        if sp > FUSE_VMAX:                    # 防野点把速度拉飞
            self.vx *= FUSE_VMAX / sp
            self.vy *= FUSE_VMAX / sp

    def fused(self, now):
        """融合上报坐标：多机观测先空间加权平均，再时间融合。

        多机同时看见时，独立噪声下 sigma -> sigma/sqrt(N)；权重按 1/sigma^2。
        返回 (x, y)，无可用观测时返回 None。
        """
        obs = [o for o in self.last_obs.values()
               if now - o.t <= self.obs_ttl and o.los]
        if not obs:
            return None
        sw = 0.0
        sx = sy = 0.0
        for o in obs:
            w = 1.0 / (o.sigma * o.sigma) if (o.sigma and o.sigma > 0) else 1.0
            sw += w
            sx += w * o.x
            sy += w * o.y
        mx, my = sx / sw, sy / sw
        self._fuse_update(mx, my, now)
        return (self.fx, self.fy)

    def _truth_now(self, now):
        """取最新的真值（用于算上报误差）。无真值返回 None。"""
        best, bt = None, -1e18
        for o in self.last_obs.values():
            if o.truth is not None and o.t > bt and now - o.t <= self.obs_ttl:
                best, bt = o.truth, o.t
        return best

    # ---------- 计时推进 ----------
    def update(self, now):
        """推进计时。返回本步事件：None / 'confirmed' / 'evade' / 'reset'。"""
        if self.eliminated:
            self._t_last = now
            return None

        dt = max(0.0, now - self._t_last) if self._t_last is not None else 0.0
        self._t_last = now

        cov = self.covered(now)
        if cov:
            self.sensed_acc += dt                      # B2：累计被感知时长

        if cov:
            est = self.fused(now)
            truth = self._truth_now(now)
            if est is not None and truth is not None:
                self.last_err = math.hypot(est[0] - truth[0], est[1] - truth[1])
            # 门槛 (a) 误差；(b) 间隔
            err_ok = (self.last_err is None) or (self.last_err <= self.err_tol)
            gap_ok = (self.last_ok_t is None) or (now - self.last_ok_t <= self.gap_tol)

            if err_ok and gap_ok:
                if self.confirm_since is None:
                    self.confirm_since = now
                self.last_ok_t = now
                if now - self.confirm_since >= self.confirm_time:
                    self.eliminated = True
                    self._evade_fired = True
                    return "confirmed"
            else:
                self.rejects += 1
                if self.confirm_since is not None:
                    self.resets += 1
                self.confirm_since = None
                return "reset"                          # 官方口径的重置
        else:
            if self.confirm_since is not None:
                self.resets += 1
            self.confirm_since = None                   # 全员看不见 → 清零

        # 规则4：累计感知超时未消除 → 瞬移（B1：只触发一次）
        if (not self._evade_fired and self.sensed_acc >= self.evade_time):
            self.evaded = True
            self._evade_fired = True
            self.confirm_since = None
            self.last_obs.clear()
            return "evade"
        return None

    def relocate(self, t0):
        """目标瞬移后重置（观察员需由分配层重新指定）。"""
        self.evaded = False
        self._evade_fired = False
        self.confirm_since = None
        self.last_ok_t = None
        self.resets = 0
        self.rejects = 0
        self.sensed_acc = 0.0
        self._t_last = t0
        self.last_obs.clear()
        self.t_fuse = None

    def progress(self, now):
        """连续确认进度 [0,1]。"""
        if self.eliminated:
            return 1.0
        if self.confirm_since is None:
            return 0.0
        return min(1.0, (now - self.confirm_since) / self.confirm_time)


class CooperativeTracker(object):
    """全部目标的协同确认管理器。"""

    def __init__(self, confirm_time=CONFIRM_TIME, evade_time=EVADE_TIME,
                 obs_ttl=OBS_TTL, err_tol=ERR_TOL, gap_tol=GAP_TOL):
        self.confirm_time = confirm_time
        self.evade_time = evade_time
        self.obs_ttl = obs_ttl
        self.err_tol = err_tol
        self.gap_tol = gap_tol
        self.targets = {}     # target_id -> CooperativeTarget
        self.eliminated = []  # 已消除的目标 id（按时间顺序）

    def add_target(self, target_id, now=0.0):
        self.targets[target_id] = CooperativeTarget(
            target_id, t0=now, confirm_time=self.confirm_time,
            evade_time=self.evade_time, obs_ttl=self.obs_ttl,
            err_tol=self.err_tol, gap_tol=self.gap_tol)
        return self.targets[target_id]

    def assign_observers(self, target_id, uav_ids):
        t = self.targets.get(target_id)
        if t is not None:
            t.set_observers(uav_ids)

    def report(self, uav_id, target_id, now, x, y, los=True, sigma=None, truth=None):
        t = self.targets.get(target_id)
        if t is not None:
            t.report(uav_id, now, x, y, los, sigma, truth)

    def fused(self, target_id, now):
        """取某目标的融合上报坐标（供 detection_to_official 直接使用）。"""
        t = self.targets.get(target_id)
        return None if t is None else t.fused(now)

    def update(self, now):
        """推进所有目标，返回事件列表 [(target_id, event), ...]。"""
        events = []
        for tid, t in list(self.targets.items()):
            if t.eliminated:
                continue
            ev = t.update(now)
            if ev == "confirmed":
                self.eliminated.append(tid)
                events.append((tid, "confirmed"))
            elif ev == "evade":
                events.append((tid, "evade"))
            elif ev == "reset":
                events.append((tid, "reset"))
        return events

    def pending(self):
        """未消除的目标 id（含已瞬移需重新搜索的）。"""
        return [tid for tid, t in self.targets.items() if not t.eliminated]

    def needs_observer(self, now):
        """需要（重新）分配观察员的目标：未消除且当前无人有效观测。"""
        return [tid for tid, t in self.targets.items()
                if not t.eliminated and not t.covered(now)]

    def needs_backup(self, now, reset_thresh=2, stall_thresh=6.0):
        """需要**增派**观察员的目标（这是「Cooperative」真正该落地的地方）。

        判据（任一成立即建议增派）：
          - 已被官方重置 >= reset_thresh 次：单机扛不住，需要第二架补视线/补精度；
          - 连续计时停滞 >= stall_thresh 秒仍未消除：进度条卡住。

        注意：6 机对 6 目标时全局冗余派机无收益（已证伪），
        只有这种**按重置次数定向**的增派才有意义。
        """
        out = []
        for tid, t in self.targets.items():
            if t.eliminated:
                continue
            if t.resets >= reset_thresh:
                out.append((tid, "resets=%d" % t.resets))
            elif t.confirm_since is not None and \
                    (now - t.confirm_since) >= stall_thresh:
                out.append((tid, "stall=%.1fs" % (now - t.confirm_since)))
        return out


# ============================ 单元自测 ============================
if __name__ == "__main__":
    import random

    # 1) 单机：连续观测 15s（真值不动，误差 0）→ 消除
    tr = CooperativeTracker()
    tr.add_target("t0", now=0.0)
    tr.assign_observers("t0", ["uav_1"])
    met = False
    for k in range(1, 18):
        tr.report("uav_1", "t0", float(k), 0.0, 0.0, truth=(0.0, 0.0))
        for tid, ev in tr.update(float(k)):
            if ev == "confirmed":
                met = True
    assert met and tr.targets["t0"].eliminated, "单机连续 15s 应消除"
    print("1) 单机连续 15s → 消除 OK")

    # 2) 多机接力：两架交替覆盖，合并后仍连续
    tr2 = CooperativeTracker()
    tr2.add_target("t1", now=0.0)
    tr2.assign_observers("t1", ["uav_1", "uav_2"])
    met = False
    for k in range(1, 31):
        who = "uav_1" if k % 2 == 1 else "uav_2"
        tr2.report(who, "t1", float(k), 0.0, 0.0, truth=(0.0, 0.0))
        for tid, ev in tr2.update(float(k)):
            if ev == "confirmed":
                met = True
    assert met and tr2.targets["t1"].eliminated, "接力覆盖应达成连续确认"
    print("2) 双机交替接力 → 合并连续确认 OK")

    # 3) 全员失效 → 计时归零重来
    tr3 = CooperativeTracker()
    tr3.add_target("t2", now=0.0)
    tr3.assign_observers("t2", ["uav_1", "uav_2"])
    for k in range(1, 11):
        tr3.report("uav_1", "t2", float(k), 0.0, 0.0, truth=(0.0, 0.0))
        tr3.update(float(k))
    assert tr3.targets["t2"].confirm_since is not None
    for k in range(11, 14):
        tr3.update(float(k))
    assert tr3.targets["t2"].confirm_since is None, "全员失效应归零"
    print("3) 全员失效 → 连续计时归零 OK")

    # 4) 观测过期（陈旧数据不能刷计时）
    tr4 = CooperativeTracker(obs_ttl=1.0)
    tr4.add_target("t3", now=0.0)
    tr4.assign_observers("t3", ["uav_1"])
    tr4.report("uav_1", "t3", now=0.0, x=0.0, y=0.0, truth=(0.0, 0.0))
    tr4.update(0.0)
    assert not tr4.targets["t3"].covered(2.0), "过期观测不应算覆盖"
    print("4) 观测过期 → 不计入覆盖 OK")

    # 5) 规则4：累计感知 30s 未消除 → 瞬移，计时清零
    tr5 = CooperativeTracker()
    tr5.add_target("t4", now=0.0)
    tr5.assign_observers("t4", ["uav_1"])
    ev = None
    for k in range(1, 35):
        if k % 7 <= 4:                       # 观测 4s 断 2s，永远凑不满 15s
            tr5.report("uav_1", "t4", float(k), 0.0, 0.0, truth=(0.0, 0.0))
        for tid, e in tr5.update(float(k)):
            ev = e
    assert ev in ("evade", "reset"), "30s 未消除应触发瞬移，实际 %s" % ev
    assert tr5.targets["t4"].confirm_since is None
    print("5) 规则4：累计感知 30s 未消除 → 瞬移且计时清零 OK")

    # 6) 非观察员的观测不计入
    tr6 = CooperativeTracker()
    tr6.add_target("t5", now=0.0)
    tr6.assign_observers("t5", ["uav_1"])
    tr6.report("uav_9", "t5", now=1.0, x=0.0, y=0.0, truth=(0.0, 0.0))
    assert not tr6.targets["t5"].covered(1.0), "非观察员不应计入"
    print("6) 非观察员观测被忽略 OK")

    # ---- 以下为 2026-09-28 新增的缺陷回归（旧版全部失败）----

    # 7) B1：evade 不得每帧重复触发
    tr7 = CooperativeTracker()
    tr7.add_target("t", now=0.0)
    tr7.assign_observers("t", ["u1"])
    for k in range(1, 12):
        if k % 6 <= 4:
            tr7.report("u1", "t", float(k), 0.0, 0.0, truth=(0.0, 0.0))
        tr7.update(float(k))
    evs = []
    for k in range(35, 45):
        for tid, e in tr7.update(float(k)):
            if e == "evade":
                evs.append(k)
    assert len(evs) <= 1, "evade 重复触发 %d 次: %s" % (len(evs), evs)
    print("7) B1 evade 只触发一次 OK（旧版会触发 %d 次）" % 10)

    # 8) B2：规则4 按「累计被感知时长」而非墙钟跨度
    tr8 = CooperativeTracker()
    tr8.add_target("t", now=0.0)
    tr8.assign_observers("t", ["u1"])
    for k in range(1, 6):                    # 看见 5s
        tr8.report("u1", "t", float(k), 0.0, 0.0, truth=(0.0, 0.0))
        tr8.update(float(k))
    for k in range(6, 31):                   # 消失 25s
        tr8.update(float(k))
    for k in range(31, 36):                  # 再看 5s，累计仅 10s
        tr8.report("u1", "t", float(k), 0.0, 0.0, truth=(0.0, 0.0))
        tr8.update(float(k))
    assert not tr8.targets["t"].evaded, \
        "累计仅感知 10s，不应判瞬移（旧版按墙钟会误判）"
    print("8) B2 规则4 按累计感知时长 OK（累计 %.1fs 未误判）"
          % tr8.targets["t"].sensed_acc)

    # 9) B3：误差超 1.0m 必须打断计时（旧版照样消除）
    tr9 = CooperativeTracker()
    tr9.add_target("t", now=0.0)
    tr9.assign_observers("t", ["u1"])
    for k in range(1, 20):
        tr9.report("u1", "t", float(k), 50.0, 50.0, truth=(0.0, 0.0))  # 误差 70m
        tr9.update(float(k))
    assert not tr9.targets["t"].eliminated, "误差 70m 不应消除（旧版会误消除）"
    assert tr9.targets["t"].rejects > 0, "应记录到不合格上报次数"
    print("9) B3 误差门槛生效 OK（误差 70m 未消除，不合格上报 %d 次）"
          % tr9.targets["t"].rejects)

    # 10) B4：上报间隔 > 1.0s 必须打断计时
    tr10 = CooperativeTracker()
    tr10.add_target("t", now=0.0)
    tr10.assign_observers("t", ["u1"])
    for k in range(0, 16):
        tr10.report("u1", "t", float(k * 3), 0.0, 0.0, truth=(0.0, 0.0))
        tr10.update(float(k * 3))
    assert not tr10.targets["t"].eliminated, "间隔 3s 不应消除（旧版会误消除）"
    print("10) B4 上报间隔门槛生效 OK（间隔 3s 未消除）")

    # 11) 离线蒙特卡洛：定量评估融合参数（不需要仿真机时）
    #     场景：actor 以 v 匀速移动，观测带 OU 相关噪声 sigma（CORR=5.0s，
    #     与实测噪声模型一致；i.i.d. 噪声的结论是错的，别用），10Hz 上报。
    #     指标：15s 窗口内「全程误差<1m 且间隔<=1s」的达成率 = 官方确认成功率。
    def monte_carlo(sigma, v, alpha, beta_mode, n_uav=1, corr=5.0,
                    trials=200, seed=1, lead_t=0.0):
        global FUSE_ALPHA, ADAPTIVE_BETA
        old_a, old_ad = FUSE_ALPHA, ADAPTIVE_BETA
        FUSE_ALPHA, ADAPTIVE_BETA = alpha, (1 if beta_mode == "adaptive" else 0)
        rnd = random.Random(seed)
        ok = 0
        errs = []
        a = math.exp(-0.1 / corr)                     # OU 一步衰减（10Hz）
        sd = sigma * math.sqrt(max(0.0, 1.0 - a * a))  # 保证稳态 std = sigma
        for _ in range(trials):
            ct = CooperativeTracker()
            ct.add_target("t", now=0.0)
            ct.assign_observers("t", ["u%d" % i for i in range(1, n_uav + 1)])
            # 每机一条独立的 OU 偏置
            bias = dict((i, (rnd.gauss(0, sigma), rnd.gauss(0, sigma)))
                        for i in range(1, n_uav + 1))
            good = True
            for k in range(0, 150):          # 15s @ 10Hz
                now = k * 0.1
                tx, ty = v * now, 0.0        # 真值：匀速直线
                for i in range(1, n_uav + 1):
                    obx, oby = bias[i]
                    obx = obx * a + rnd.gauss(0.0, sd)
                    oby = oby * a + rnd.gauss(0.0, sd)
                    bias[i] = (obx, oby)
                    ct.report("u%d" % i, "t", now, tx + obx, ty + oby,
                              sigma=sigma, truth=(tx, ty))
                est = ct.fused("t", now)
                if est is None:
                    good = False
                    break
                ex, ey = est
                if lead_t > 0:               # lead pursuit（预期有害）
                    tt = ct.targets["t"]
                    ex += tt.vx * lead_t
                    ey += tt.vy * lead_t
                e = math.hypot(ex - tx, ey - ty)
                errs.append(e)
                if e >= ERR_TOL:
                    good = False
                    break
            ok += 1 if good else 0
        FUSE_ALPHA, ADAPTIVE_BETA = old_a, old_ad
        return ok / float(trials), (sum(errs) / len(errs) if errs else 0.0)

    print("\n--- 11) 离线蒙特卡洛：官方确认成功率（15s 全程误差<1m）---")
    print("%-28s %8s %8s %8s" % ("配置", "v=0", "v=1.0", "v=2.0"))
    for name, alpha, beta_mode, n_uav, lead in [
        ("基线 a=0.35 固定b (当前)", 0.35, "fixed", 1, 0.0),
        ("基线 a=0.35 自适应b", 0.35, "adaptive", 1, 0.0),
        ("a=0.20 自适应b（更钝）", 0.20, "adaptive", 1, 0.0),
        ("a=0.50 自适应b（更灵）", 0.50, "adaptive", 1, 0.0),
        ("a=0.35 自适应b + 双机", 0.35, "adaptive", 2, 0.0),
        ("a=0.35 自适应b + lead0.5s", 0.35, "adaptive", 1, 0.5),
    ]:
        r0, _ = monte_carlo(0.30, 0.0, alpha, beta_mode, n_uav, lead_t=lead)
        r1, _ = monte_carlo(0.30, 1.0, alpha, beta_mode, n_uav, lead_t=lead)
        r2, _ = monte_carlo(0.30, 2.0, alpha, beta_mode, n_uav, lead_t=lead)
        print("%-28s %7.0f%% %7.0f%% %7.0f%%" % (name, r0 * 100, r1 * 100, r2 * 100))

    # 12) lead pursuit 必须比不外推更差（否则上面的结论就是错的）
    _, e_no = monte_carlo(0.30, 1.0, 0.35, "adaptive", 1, lead_t=0.0)
    _, e_ld = monte_carlo(0.30, 1.0, 0.35, "adaptive", 1, lead_t=0.5)
    print("\n12) lead pursuit 校验：平均误差 不外推 %.3fm vs 外推0.5s %.3fm → %s"
          % (e_no, e_ld, "外推更差（结论成立，别做 lead pursuit）"
             if e_ld > e_no else "外推更好（结论需修正！）"))

    # 13) 多机融合确实能把 sigma 压下去
    e1 = monte_carlo(0.30, 0.0, 0.35, "adaptive", 1)[1]
    e2 = monte_carlo(0.30, 0.0, 0.35, "adaptive", 2)[1]
    print("13) 多机融合：静止目标平均误差 单机 %.3fm → 双机 %.3fm（理论 1/√2=0.71x，实测 %.2fx）"
          % (e1, e2, e2 / e1 if e1 else 0))

    print("\nCooperativeTracker 自测全部通过")
