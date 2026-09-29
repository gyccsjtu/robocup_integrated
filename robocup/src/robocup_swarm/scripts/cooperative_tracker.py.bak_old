#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多机协同确认：把多架无人机的观测合并成一条「连续确认计时」。

规则依据
--------
规则 5：无人机需**连续 15 秒**正确广播恐怖分子 ID + 坐标，裁判判定消除。
规则 4：被感知累计 30 秒仍未消除 → 目标瞬移躲藏。

已确认：**多机观测合并计算**。即任一注册为「该目标观察员」的无人机看见它，
这一步就算「看见」；只有**所有观察员都看不见**时，连续计时才归零。

这为什么是决定性的
------------------
单机 SITL 实测：理想躲藏下最长连续只有 6.6 s（需 15 s）—— 单机做不到。
但两架从不同方位观察时，一架的视线缺口往往被另一架补上，合并后的连续
时长可以远超任一单机。

设计要点
--------
1. **观察员集合**由分配层决定（谁被派去钉这个目标），本模块只做计时。
2. 观测在**时间窗**内有效：超时的观测（`obs_ttl`）不算数，避免用陈旧
   信息刷连续计时（等价于无人机掉了但计时还在涨）。
3. 目标瞬移（规则4）后所有计时清零，观察员也需重新分配。
4. 计时以**仿真时间**为基准，与协同核心的 sim_s 一致。
"""

import math

# 规则常量
CONFIRM_TIME = 15.0     # 规则5：连续确认时长
EVADE_TIME = 30.0       # 规则4：被感知累计 30s 未消除 → 瞬移
OBS_TTL = 1.0           # 单条观测的有效期 s（超过则视为陈旧，不计入）


class TargetObservation(object):
    """某一时刻某架无人机对某目标的观测。"""
    __slots__ = ("uav_id", "t", "x", "y", "los")

    def __init__(self, uav_id, t, x, y, los=True):
        self.uav_id = uav_id
        self.t = float(t)
        self.x = float(x)
        self.y = float(y)
        self.los = bool(los)      # 该次观测视线是否通畅


class CooperativeTarget(object):
    """单个目标的协同确认状态。"""

    def __init__(self, target_id, t0=0.0, confirm_time=CONFIRM_TIME,
                 evade_time=EVADE_TIME, obs_ttl=OBS_TTL):
        self.target_id = target_id
        self.confirm_time = confirm_time
        self.evade_time = evade_time
        self.obs_ttl = obs_ttl

        self.observers = set()     # 当前注册的观察员
        self.last_obs = {}         # uav_id -> TargetObservation（各自最新一条）
        self.confirm_since = None  # 连续确认起始时刻（None = 当前未在确认）
        self.tracked_since = t0    # 首次被感知（规则4 计时起点）
        self.evaded = False
        self.eliminated = False

    # ---- 观察员管理 ----
    def set_observers(self, uav_ids):
        """分配层指定/变更观察员集合。移出的观察员其观测立即失效。"""
        new = set(uav_ids)
        for u in list(self.last_obs.keys()):
            if u not in new:
                del self.last_obs[u]
        self.observers = new

    # ---- 观测输入 ----
    def report(self, uav_id, t, x, y, los=True):
        """记录一条观测。非观察员的观测不计入（防止无关机刷计时）。"""
        if self.eliminated:
            return
        if self.observers and uav_id not in self.observers:
            return
        self.last_obs[uav_id] = TargetObservation(uav_id, t, x, y, los)

    # ---- 状态评估 ----
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

    def update(self, now):
        """推进计时。返回本步发生的事件：None / 'confirmed' / 'evade'。"""
        if self.eliminated:
            return None

        if self.covered(now):
            if self.confirm_since is None:
                self.confirm_since = now     # 开始/恢复连续计时
            if now - self.confirm_since >= self.confirm_time:
                self.eliminated = True
                return "confirmed"
        else:
            self.confirm_since = None        # 全员看不见 → 计时归零

        # 规则4：被感知累计超时未消除 → 瞬移
        if now - self.tracked_since >= self.evade_time:
            self.evaded = True
            self.confirm_since = None
            self.last_obs.clear()
            return "evade"
        return None

    def relocate(self, t0):
        """目标瞬移后重置（观察员需由分配层重新指定）。"""
        self.evaded = False
        self.confirm_since = None
        self.tracked_since = t0
        self.last_obs.clear()

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
                 obs_ttl=OBS_TTL):
        self.confirm_time = confirm_time
        self.evade_time = evade_time
        self.obs_ttl = obs_ttl
        self.targets = {}     # target_id -> CooperativeTarget
        self.eliminated = []  # 已消除的目标 id（按时间顺序）

    def add_target(self, target_id, now=0.0):
        self.targets[target_id] = CooperativeTarget(
            target_id, t0=now, confirm_time=self.confirm_time,
            evade_time=self.evade_time, obs_ttl=self.obs_ttl)
        return self.targets[target_id]

    def assign_observers(self, target_id, uav_ids):
        t = self.targets.get(target_id)
        if t is not None:
            t.set_observers(uav_ids)

    def report(self, uav_id, target_id, now, x, y, los=True):
        t = self.targets.get(target_id)
        if t is not None:
            t.report(uav_id, now, x, y, los)

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
        return events

    def pending(self):
        """未消除的目标 id（含已瞬移需重新搜索的）。"""
        return [tid for tid, t in self.targets.items() if not t.eliminated]

    def needs_observer(self, now):
        """需要（重新）分配观察员的目标：未消除且当前无人有效观测。"""
        out = []
        for tid, t in self.targets.items():
            if t.eliminated:
                continue
            if not t.covered(now):
                out.append(tid)
        return out


# ============================ 单元自测 ============================
if __name__ == "__main__":
    # 1) 单机：连续观测 15s → 消除
    # 注意：首次报告在 t=1，故满 15 s 需到 t=16（计时从首次观测起算）
    tr = CooperativeTracker()
    tr.add_target("t0", now=0.0)
    tr.assign_observers("t0", ["uav_1"])
    met = False
    for k in range(1, 18):
        tr.report("uav_1", "t0", float(k), 0.0, 0.0)
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
        # 奇数秒 uav_1 看见，偶数秒 uav_2 看见（模拟现实接力）
        who = "uav_1" if k % 2 == 1 else "uav_2"
        tr2.report(who, "t1", float(k), 0.0, 0.0)
        for tid, ev in tr2.update(float(k)):
            if ev == "confirmed":
                met = True
    assert met and tr2.targets["t1"].eliminated, "接力覆盖应达成连续确认"
    print("2) 双机交替接力 → 合并连续确认 OK")

    # 3) 全员失效 → 计时归零重来
    tr3 = CooperativeTracker()
    tr3.add_target("t2", now=0.0)
    tr3.assign_observers("t2", ["uav_1", "uav_2"])
    for k in range(1, 11):        # 观测 10 秒
        tr3.report("uav_1", "t2", float(k), 0.0, 0.0)
        tr3.update(float(k))
    assert tr3.targets["t2"].confirm_since is not None
    # 之后全员停止报告，超过 obs_ttl → 计时归零
    for k in range(11, 14):
        tr3.update(float(k))
    assert tr3.targets["t2"].confirm_since is None, "全员失效应归零"
    print("3) 全员失效 → 连续计时归零 OK")

    # 4) 观测过期（陈旧数据不能刷计时）
    tr4 = CooperativeTracker(obs_ttl=1.0)
    tr4.add_target("t3", now=0.0)
    tr4.assign_observers("t3", ["uav_1"])
    tr4.report("uav_1", "t3", now=0.0, x=0.0, y=0.0)
    tr4.update(0.0)
    # 不再报告，2 秒后该观测应已过期
    assert not tr4.targets["t3"].covered(2.0), "过期观测不应算覆盖"
    print("4) 观测过期 → 不计入覆盖 OK")

    # 5) 规则4：被感知 30s 未消除 → 瞬移，计时清零
    # 构造「真正无法累积」的场景：每观测 4 s 就断 3 s（超过 obs_ttl=1，
    # 故连续计时每次都被打断，永远到不了 15 s）。
    tr5 = CooperativeTracker()
    tr5.add_target("t4", now=0.0)
    tr5.assign_observers("t4", ["uav_1"])
    ev = None
    for k in range(1, 35):
        phase = k % 7
        if phase <= 4:                    # 观测 4 s
            tr5.report("uav_1", "t4", float(k), 0.0, 0.0)
        # phase 5,6 不报告（断 2 s > obs_ttl）→ 计时归零
        for tid, e in tr5.update(float(k)):
            ev = e
    assert ev == "evade", "30s 未消除应触发瞬移，实际 %s" % ev
    assert tr5.targets["t4"].confirm_since is None
    print("5) 规则4：30s 未消除 → 瞬移且计时清零 OK")

    # 5b) 间歇观测下，obs_ttl 决定难度（这是规则未明确、需与裁判确认的参数）
    def _try_with_ttl(ttl, gap_s):
        """每观测 4 s 断 gap_s 秒，看 30 s 内能否累积到 15 s。"""
        tr = CooperativeTracker(obs_ttl=ttl)
        tr.add_target("x", now=0.0)
        tr.assign_observers("x", ["uav_1"])
        for k in range(1, 31):
            period = 4 + gap_s
            if (k - 1) % period < 4:
                tr.report("uav_1", "x", float(k), 0.0, 0.0)
            for tid, e in tr.update(float(k)):
                if e == "confirmed":
                    return True
        return False
    r_small = _try_with_ttl(1.0, 3)      # 严格：断 3 s 就算中断
    r_large = _try_with_ttl(4.0, 3)      # 宽松：断 3 s 仍算连续
    assert r_small != r_large, "obs_ttl 应显著影响结果"
    print("5b) obs_ttl 影响难度 OK（严格 ttl=1 → %s；宽松 ttl=4 → %s）"
          % ("达成" if r_small else "失败", "达成" if r_large else "失败"))

    # 6) 非观察员的观测不计入
    tr6 = CooperativeTracker()
    tr6.add_target("t5", now=0.0)
    tr6.assign_observers("t5", ["uav_1"])
    tr6.report("uav_9", "t5", now=1.0, x=0.0, y=0.0)   # 无关机
    assert not tr6.targets["t5"].covered(1.0), "非观察员不应计入"
    print("6) 非观察员观测被忽略 OK")

    print("\n多机协同确认自测全部通过")
