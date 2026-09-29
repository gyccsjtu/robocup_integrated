#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务分配器：回答"6 架无人机现在该去打哪个目标、各投几架"。

它只做一件事
------------
输入：已发现且未消除的目标 + 每架当前位置 + 每个目标的追踪状态
输出：{目标: [分到的无人机]}，**架数由目标状态动态决定**，不写死 3 架/目标

主线：动态性价比分配（--strategy dynamic，默认）
------------------------------------------------
每个候选「给目标 T 加到 n 架」算一个**性价比**：

    性价比(T, n) = 边界价值(T, n) * 边际系数(n) / 距离成本(T)

    边界价值 = w_urgent * 瞬移倒计时进度      # 越接近 30 s 越急
             + w_finish * 确认进度^2          # 凸：快到手的目标值得加码
             + w_dead   * (无人值守 ? 1 : 0)  # 兼顾搜索覆盖面，见下
             + sticky   * 在位加成            # 已在编组里的机，留住（防抖动）

    边际系数(1..3) = 1.0 / 0.45 / 0.20       # 第 2、3 架的边际收益递减
    距离成本 = 1 + 最近可用机距离 / 60 m      # 距离越远越不值

然后**全局贪心**：先算"每个目标该配几架"，再实际挑人。三道闸门控制投入度：

    attack_frac  值不值得**打**？价值须达"最值目标"的 frac 倍，否则暂不追
    cut          值不值得**加**第 2/3 架？
    KEEP_FRAC    值不值得**留**住现有编组？（滞回下界 = cut * KEEP_FRAC）

搜索覆盖面：`w_dead` 与"暂不追"的取舍
-------------------------------------
`w_dead` 是这套公式里**唯一**权衡"集中追踪"与"搜索发现"的项，而且是
**目标数相关**的，不能拍脑袋定：

    3 目标  dead=0.8 → 2.62/3 (67.5%)   dead=0   → 2.65/3 (70.0%)   持平
    6 目标  dead=0.8 → 5.55/6 (65.0%)   dead=0   → 4.72/6 (32.5%)   决定性

6 目标下差 20 个百分点（z≈2.9，p≈0.004）。原因是**兵力守恒**：`w_dead=0`
时无人值守的目标价值≈0，机全压到已发现的目标上；目标一多，被搁置的就多，
最后没有任何一架在搜索，后面的目标根本发现不了 —— 看「同时追踪」那一列最
直接：6 目标下 dead=0 只有 0.59，dead=0.8 有 0.71，而"分散"策略能到 1.15。

所以：**目标数 ≤ 交战能力时，两种都对；目标数超过能力时，必须保证搜索面。**
本项目是 6 目标 6 机，所以要 dead=0.8。若要实现"T3 暂不追"，把它设 0，
但**只在目标数少时成立**。

（`attack_frac` 仍然是"相对最值目标"的比例，这是为了让 `frac` 在目标数
变化时不用重调；但闸门 1 的作用在实测中有限 —— 6 目标下 `frac` 只影响
"先打哪几个"，不影响最终完成数。）

对照 baseline（--compare 里一并跑，用来验证动态是否真的更好）
------------------------------------------------------------
  spread      分散：每目标 1 架（UAV_i → T_i）
  focus       集中：1 个重点目标 × 3 架 + 其余搜索
  concentrate K 目标 × M 架

实测（离线，n=40，默认配置）
----------------------------
  3 目标：
    方法                 完成数   全清率   首次消除   均单目标   同时追踪   丢失    冲突
    动态（默认）          2.65/3   70.0%    41.1 s    41.1 s    0.50      8.7   251.1
    2 目标 × 3 架         2.25/3   42.5%    56.4 s    49.6 s    0.53     18.2   605.0
    1 目标 × 3 架         2.00/3   35.0%    61.6 s    55.8 s    0.47     17.9   569.2
    3 目标 × 2 架         2.00/3   30.0%    64.6 s    53.4 s    0.62     36.4   264.2
    分散（每目标 1 架）     0.95/3    2.5%    92.5 s    61.4 s    0.56     41.6    20.9

  6 目标：
    方法                 完成数   全清率   首次消除   均单目标   同时追踪   丢失    冲突
    动态（默认）          5.55/6   65.0%    32.4 s    58.9 s    0.71     23.5   690.9
    3 目标 × 2 架         4.62/6   22.5%    42.9 s    81.1 s    0.86     86.5   649.6
    2 目标 × 3 架         4.60/6   27.5%    42.4 s    72.7 s    0.78     52.4  1699.8
    1 目标 × 3 架         3.27/6    7.5%    57.8 s   101.4 s    0.58     39.9  1144.1
    分散（每目标 1 架）     2.27/6    0.0%   103.1 s   122.3 s    1.15    137.2    69.0

动态在**每一个**指标上都优于所有固定编组。注意两个反直觉之处：
* 「分散」与「3 目标 × 2 架」的**同时追踪数最高**（1.15 / 0.86）、LOS 也最高
  （67.3% / 62.2%），但完成数最低/偏低 —— 看得多不等于看得够久，
  **15 s 连续确认**要的是深度不是广度。所以必须同时报完成数，不能只看 LOS。
* 「2 目标 × 3 架」的冲突数高达 1699.8（动态只有 690.9）：固定编组反复把 3 架
  派到同一个目标上，机间挤在一起。

调试记录（踩过的坑，别再踩）
----------------------------
1. **保留判据不能要求 LOS**（`_valid_members`）。派出去的机在飞的路上本来
   就没 LOS，要求 LOS 会把它们全判为无效 → 每帧重新 `choose()` → 整个接近
   过程编组都在抖，阶段1（保留）这个唯一的稳定机制等于失效。
   迷惑之处：它让**所有阈值旋钮都失效** —— attack_frac 0.6→0.95、cut
   0.02→0.25、dead 项开关，完成数全在 1.88~1.93 之间纹丝不动。修好后
   1.93 → 2.65（3 目标，dead=0 配置下测的）。
   **旋钮扫不出差别，通常说明有与阈值无关的 bug。**
2. **无人值守判据不能用 `covered()`**。会有正反馈：到站 covered=True →
   价值跌破阈值 → 撤编 → 机回去搜索 → 又没 LOS → 重新派机。实测改派
   428 架次/局、同时有效追踪仅 0.43 个。改用"在位数"后降到 33.6。
3. **单阈值会让编组在阈值附近抖**，需要 `KEEP_FRAC` 做滞回（两个门槛）。
4. **消融必须在修完 bug 之后做**。`dead` 项第一次消融（bug 修好前）得出
   "关掉更好"（1.65→1.52），修好后再测结论**反转**（6 目标 4.72→5.55）。
   在已知有 bug 的代码上得到的消融结论是不可信的。
5. 已排除的假设（实测无效，别再试）：
   * 「按 `choose()` 的方位互补挑人」优于「按最近挑」—— 1.65 vs 1.62，噪声内。
   * 「同时打的目标数」是主要瓶颈 —— frac 0.6→0.95 几乎无差别（1.93→1.88）。

指标（用户指定的全部）
----------------------
  完成目标数 / 全清率(记作 15s 成功率) / 平均单目标完成时间 / 首次消除
  / 同时有效追踪目标数 / LOS 占比 / 目标丢失次数 / 无人机冲突次数
  另加：改派架次（调度抖动）、目标无人可派的时间占比（量化"暂不追"）

用法
----
    python3 task_allocator.py --strategy dynamic --targets 3 --runs 40
    python3 task_allocator.py --compare --targets 3 --runs 40
    python3 task_allocator.py --compare --targets 6 --runs 40 --max-t 300
"""

import argparse
import math
import os
import random
import sys

WS = os.environ.get("ROBOCUP_WORKSPACE",
                    os.environ.get("ROBOCUP_WS", os.path.expanduser("~/team_ws/robocup")))
sys.path.insert(0, os.path.join(WS, "scripts", "vm"))
sys.path.insert(0, os.path.join(WS, "src", "robocup_swarm", "scripts"))

from strategy_compare import METADATA, SENSE_R  # noqa: E402
import mission_time as MT  # noqa: E402

# ---- 动态分配：边界价值权重 ----
W_URGENT = 1.0      # 瞬移倒计时（t - tracked_since，30 s 到点）
W_FINISH = 1.2      # 确认进度的**凸**奖励 prog^2（快完成的目标优先加码）
W_DEAD = 0.8        # 无人值守奖励（默认开）。关掉=允许放掉不值得的目标，见文件头
STICKY = 0.15       # 在位加成：保住现有编组，防止来回改派

# ---- 动态分配：结构参数 ----
MAX_TEAM = 3        # 每目标架数上限（离线 n=200 实测 3 tracker 最优）
EFF_DECAY = 0.45    # eff(n) = 1 - EFF_DECAY^(n-1) → 边际 1.0 / 0.45 / 0.20
COST_DIST = 60.0    # 距离成本尺度 m
ATTACK_FRAC = 0.6   # 值不值得**打**：价值须达到"最值目标"的此比例，否则暂不追
DEFAULT_CUT = 0.02  # 值不值得**加第 2/3 架**：低于此值不加机
KEEP_FRAC = 0.5     # 滞回系数：留人门槛 = cut * KEEP_FRAC，低于它才释放编组

# 两个旋钮的方向（别搞反，反了会把"集中/分散"读成相反的结论）：
#   attack_frac 高 → 只有最值的目标够格被打（集中）；低 → 已知目标都打（分散）
#   cut 低       → 在打的目标容易加到第 2/3 架（集中）；高 → 每目标只留 1 架（分散）
# 实测 cut=0.25 时完成数掉到 0.55~0.78/3 —— 机都在搜索、没人加码，是过度分散。

# ---- 通用 ----
CLASH_R = 3.0       # 机间小于此距离记一次"冲突"
EVADE_JUDGE_S = 30.0   # 规则4：被感知累计 30 s 未消除 → 瞬移
DIAG_TRAIL = 6      # 近距诊断保留的历史帧数


class AllocSim(MT.Sim):
    """在 mission_time.Sim 上替换分配决策，其余（规则/运动/判定）完全复用。

    复用父类的 run() / _search_step() / _engaged() / 目标运动 / 确认计时 ——
    规则实现只有一份，对照实验才成立。
    """

    def __init__(self, g, strategy="dynamic", targets=6, zoom_k=2, zoom_m=3,
                 cut=DEFAULT_CUT, attack_frac=ATTACK_FRAC, w_dead=W_DEAD, **kw):
        self.strategy = strategy
        self.zoom_k = int(zoom_k)     # concentrate: 同时打几个目标
        self.zoom_m = int(zoom_m)     # concentrate: 每目标几架
        self.cut = float(cut)
        self.attack_frac = float(attack_frac)
        self.w_dead = float(w_dead)
        self.n_targets = int(targets)
        # 指标计数
        self.n_los_hit = 0
        self.n_los_tot = 0
        self.n_lost = 0               # 目标丢失次数（有效观测中断）
        self.min_dist = float("inf")   # 全程机间最小距离（最坏贴近程度）
        self.n_clash = 0
        self.n_clash15 = 0             # < 1.5 m 的帧数
        self.n_clash10 = 0             # < 1.0 m 的帧数
        self.diag = []                 # 近距事件的逐帧记录
        self.segments = []             # 近距事件的连续段汇总
        self._prev = {}                # uav -> 上一帧位置（算相对速度用）
        self._trail = {}               # uav -> 最近几帧位置
        self._dall = {}                # 机对 -> 本帧距离（所有机对）
        self._seg_entry = {}           # 机对 -> 进入 <1.5 m 时的距离
        self._in_close = set()         # 当前处于近距的机对
        self._seg_start = {}           # 机对 -> 本段起始时刻
        self.live_targets_hist = []   # 同时有效追踪的目标数
        self.n_idle_target = 0        # 有目标但一架都没派的帧数（量化"暂不追"）
        self.was_covered = {}         # tid -> 上一帧是否被覆盖
        self.assign_hist = []         # 每帧的 {tid: 架数}，用于量化改派抖动
        self.n_switch = 0             # 编组成员变动次数
        self._last_members = {}       # tid -> set(uav)
        MT.Sim.__init__(self, g, n_targets=targets, **kw)

    # ---------------- 旧公式（保留，仅 concentrate/focus 的排序用） ----------------
    def priority(self, tid, n_assigned):
        """四项加权优先级（用户最初指定的形式）。

        注意语义：`tracked_since` 由 `add_target`/`relocate` 设置，**不是**
        逐帧累加的被感知时长，而是「自发现或自上次瞬移以来」——
        即**规则 4 的瞬移倒计时**。用它当"紧迫度"是对的（快瞬移代价最大），
        但不能说成"被感知累计时长"。
        """
        tgt = self.tr.targets[tid]
        tg = self.tgt[tid]
        urgent = min(1.0, max(0.0, (self.t - tgt.tracked_since) / EVADE_JUDGE_S))
        ux = min(self.uavs.values(),
                 key=lambda p: math.hypot(p[0] - tg["x"], p[1] - tg["y"]))
        d = math.hypot(ux[0] - tg["x"], ux[1] - tg["y"])
        near = max(0.0, 1.0 - d / 100.0)
        return (W_URGENT * urgent + W_FINISH * tgt.progress(self.t) ** 2
                + 0.6 * near + 0.4 * min(1.0, n_assigned / 6.0))

    # ---------------- 动态性价比 ----------------
    def _pool_dist(self, x, y, pool):
        """目标到**当前可用机**的最近距离（m）。空池返回 inf。"""
        if not pool:
            return float("inf")
        return min(math.hypot(p[0] - x, p[1] - y) for p in pool.values())

    def _base_value(self, tid, cur, dist):
        """单个目标的边界价值（不含边际系数与距离成本）。

        cur  —— 该目标当前**在位**的成员数（0 = 无人值守）
        dist —— 到可用机的最近距离（m），用于算距离成本
        """
        tgt = self.tr.targets[tid]

        # 1) 紧迫度：规则4 的瞬移倒计时（越接近 30 s 越急）
        urgent = min(1.0, max(0.0, (self.t - tgt.tracked_since) / EVADE_JUDGE_S))
        # 2) 接近完成的凸奖励：进度 10 s/15 s 比 0 s 值钱得多
        finish = tgt.progress(self.t) ** 2
        # 3) 无人值守奖励。
        #
        # **这里绝对不能用 `not tgt.covered()`** —— 派出去的机在飞过去的路上
        # 同样没有 LOS，于是"已经派人"和"没人管"无法区分：一到站 covered 变
        # True、价值骤降跌破阈值 → 撤编 → 机回去搜索 → 又没 LOS → 重新派机。
        # 实测这个正反馈把改派推到 428 架次/局、同时有效追踪只有 0.43 个。
        # 用"在位数"来看守，派出去就算有人管。
        #
        # `self.w_dead = 0` 会关掉这一项，效果是：不再强制"凡是够格打的目标都
        # 得有人看"，机可以集中到快完成的目标上 —— 即用户说的"T3 暂不追"。
        # 消融用它来分辨"动态打不过固定"到底是调度逻辑的问题还是这一项的问题。
        dead = 1.0 if (cur == 0 and self.w_dead > 0.0) else 0.0

        v = W_URGENT * urgent + W_FINISH * finish + self.w_dead * dead
        # 4) 在位加成：让拆散现有编组变得更不划算（防抖动）
        if cur > 0:
            v += STICKY * (min(cur, MAX_TEAM) / float(MAX_TEAM))
        # 距离写进**分母**：它该摊薄整个价值，而不是和进度相加
        return v / (1.0 + dist / COST_DIST)

    def _valid_members(self, tid, require_los=False):
        """上一帧编组里**仍然算数**的成员。

        require_los=False（默认）：只看"没有掉队太远"，**不要求当前有 LOS**。
        require_los=True ：沿用父类判据，还要求此刻能看见目标。

        为什么默认不带 LOS 判据
        ----------------------
        派出去的机**在飞过去的路上**本来就没有 LOS。如果保留判据要求 LOS，
        那么阶段1（保留）会把这些机全部判为"无效"→ 释放 → 阶段2重新
        `choose()` 一遍 → 可能挑到别的机 → 下一帧又重复。整个接近过程中
        编组每帧都在抖，而阶段1 本来是唯一的稳定机制。

        这个 bug 的表现很有迷惑性：它让**所有阈值旋钮都失效** —— 实测
        attack_frac 从 0.6 扫到 0.95、cut 从 0.02 扫到 0.25、dead 项开关，
        完成数全在 1.88~1.93 之间纹丝不动，因为抖动与这些阈值无关。
        改成"只看距离"后，阶段1 才真正起到粘住编组的作用。
        """
        tg = self.tgt[tid]
        out = []
        for u in list(tg.get("roles", {})):
            if u not in self.uavs:
                continue
            ux, uy = self.uavs[u]
            if math.hypot(tg["x"] - ux, tg["y"] - uy) > SENSE_R * 1.5:
                continue
            if require_los and not self._visible(ux, uy, tg["x"], tg["y"]):
                continue
            out.append(u)
        return out

    def _dynamic(self, pool):
        """两阶段滞回贪心 + 三道闸门。返回 {tid: [(uav, "tracker"), ...]}。

        三道闸门（对应"投入程度"的三个问题）
        -----------------------------------
          ATTACK_BAR  值不值得**打**？  base < bar → 一架都不派 = 暂不追
          cut         值不值得**加**第 2/3 架？  v < cut → 不再加
          KEEP_FRAC   值不值得**留**住现有编组？  v < cut*KEEP_FRAC → 释放

        为什么"要不要打"必须和"要不要加机"分开
        --------------------------------------
        用户要的动态行为是"T3 暂不追"。但如果只有 `cut` 一个阈值，贪心会先
        给**每个**已知目标铺 1 架（第 1 架的边际系数 1.0 最大），T3 永远有
        一架在看 —— 那就退化成"分散"，而不是"暂不追"。实测这正是动态
        (1.90/3) 打不过固定 2×3 (2.25/3) 的原因：兵力被摊到不值得的目标上。

        为什么必须两阶段
        ----------------
        给"留人"和"加人"用**两个不同的门槛**，否则性价比会随"机到没到站"
        在阈值附近上下跳，编组每帧重算（见 `_base_value` 里 dead 项的注释）。
        滞回：价值掉到 cut*KEEP_FRAC 以上就一直留着，跌破才释放。

        目标完成时父类清空其编组，在位成员自然消失，剩余机下一帧流向别的
        目标 —— 用户要的"T1 完成 → 释放 → 重新计算 → T3/T4"就是这个流程。

        阶段1 里"保留"优先于"扩张"：一架机不能同时属于两个目标，所以先把
        老编组占的机从池子里扣掉，再拿剩下的谈扩张。
        """
        pending = [tid for tid in self.tr.pending() if self.tgt[tid]["known"]]
        if not pending:
            return {}
        prev = {tid: self._valid_members(tid) for tid in pending}
        pool = dict(pool)
        teams = {}

        def reach(tid, pool_):
            tg = self.tgt[tid]
            return min(math.hypot(pool_[u][0] - tg["x"], pool_[u][1] - tg["y"])
                       for u in pool_)

        # ---- 闸门 1：值不值得打 ----
        #
        # 门槛必须是**相对**最好的目标，不能是绝对常数：`_base_value` 里
        # `dead`（无人值守）是加法奖励，任何刚发现的目标都凭空拿 1.0，于是
        # 任何绝对门槛都拦不住它 —— 实测 attack_bar 从 0.30 提到 0.50 结果
        # 逐位相同，就是因为它从来没真正过滤过谁。
        #
        # 取"相对最值目标的比例"也更贴合用户的原话：T3 该暂不追，理由是它
        # **比 T1 差**，而不是它低于某个绝对分数线。
        raw = {}
        for tid in pending:
            if pool:
                # 用 cur=0 且统一按"到可用机的距离"算 —— 闸门 1 只看目标**本身**
                # 值不值得打，不应被"当前有没有人在追"影响（那是闸门 3 的事）。
                raw[tid] = self._base_value(tid, 0, reach(tid, pool))
        if not raw:
            return {}
        thr = max(raw.values()) * self.attack_frac
        engage = [tid for tid in pending if raw.get(tid, 0.0) >= thr]

        # ---- 阶段 1：保留在位编组（滞回下界）----
        for tid in engage:
            mem = [u for u in prev[tid] if u in pool]
            if not mem:
                continue
            v = self._base_value(tid, len(mem), reach(tid, {u: pool[u]
                                                            for u in mem}))
            # 边际系数按在位人数取（1 架→1.0，2 架→0.45，3 架→0.20），
            # 与阶段2的 step 对应，保证第一架和"第一架"可比。
            v *= EFF_DECAY ** (len(mem) - 1)
            if v >= self.cut * KEEP_FRAC:
                teams[tid] = list(mem)
                for u in mem:
                    pool.pop(u, None)

        # ---- 阶段 2：扩张（给无人值守的目标补位 / 给够值的目标加机）----
        #
        # 分两步：先用贪心决定**每个目标要几架**（这是动态策略的核心逻辑），
        # 再实际挑人。
        #
        # 为什么不顺手用"最近的机"挑完就算：`ObserverAssigner.score()` 里有
        # `w_spread * sin(方位差)`（权重 2.5，见 observer_assign.py），会偏好
        # 与已选机**方位互补**的候选；纯按距离挑会让同一目标的几架挤在同一
        # 侧，观测冗余。固定编组那几行走的正是 `choose`，所以这个差别会直接
        # 体现在完成数上（实测 2×3 固定 = 2.25/3 高于纯距离贪心）。
        want = {tid: len(teams.get(tid, [])) for tid in engage}
        shadow = dict(pool)          # 只用来算距离，不决定挑谁
        for step in range(MAX_TEAM):
            marg = EFF_DECAY ** step          # 1.0 / 0.45 / 0.20
            best, best_v = None, 0.0
            for tid in engage:
                if want[tid] >= MAX_TEAM or not shadow:
                    continue
                v = self._base_value(tid, want[tid] + 1, reach(tid, shadow)) * marg
                if v > best_v:
                    best, best_v = tid, v
            if best is None or best_v < self.cut:
                break
            want[best] += 1
            near_u = min(shadow, key=lambda u: math.hypot(
                shadow[u][0] - self.tgt[best]["x"],
                shadow[u][1] - self.tgt[best]["y"]))
            shadow.pop(near_u, None)

        # 按贪心给出的架数挑人。
        #
        # 已经在位/在途的老成员优先留用 —— 否则 `choose()` 可能把一只已经在
        # 飞的机换掉，新机又得从头飞，编组永远凑不齐。用 `prev[tid]`（=阶段1
        # 那批没通过滞回下界的机）做优先名单，剩下的名额才交给 `choose()`。
        for tid in engage:
            need = want.get(tid, 0)
            held = teams.get(tid, [])
            need -= len(held)
            if need <= 0 or not pool:
                continue
            picks = [u for u in prev[tid] if u in pool][:need]
            for u in picks:
                pool.pop(u, None)
            if len(picks) < need:
                more = self.assigner.choose(
                    (self.tgt[tid]["x"], self.tgt[tid]["y"]), pool,
                    n=need - len(picks))
                if not more:
                    # 兜底：choose 挑不出来时退回最近机，不能让贪心的决定落空
                    more = []
                    for _ in range(min(need - len(picks), len(pool))):
                        u = min(pool, key=lambda k: math.hypot(
                            pool[k][0] - self.tgt[tid]["x"],
                            pool[k][1] - self.tgt[tid]["y"]))
                        more.append(u)
                        pool.pop(u, None)
                picks = picks + list(more)
                for u in more:
                    pool.pop(u, None)
            teams.setdefault(tid, []).extend(picks)

        return {tid: [(u, "tracker") for u in us]
                for tid, us in teams.items() if us}

    # ---------------- 分配入口 ----------------
    def _assign(self, free_uavs):
        pending = [tid for tid in self.tr.pending() if self.tgt[tid]["known"]]
        pool = dict(self.uavs)

        if self.strategy == "spread":
            teams = self._spread(pool)
        elif self.strategy == "focus":
            teams = self._focus(pool)
        elif self.strategy == "concentrate":
            teams = self._concentrate(pool)
        else:
            teams = self._dynamic(pool)

        # 清空未入选目标 —— 否则父类 run() 不动它们，陈旧 observers 会让
        # "已放弃"的目标继续占着无人机（这个坑在 concentrate.py 里踩过）。
        # 动态策略下这一步尤其关键：目标随时可能因为性价比不足被撤编。
        for tid in pending:
            if tid not in teams:
                self.tgt[tid]["observers"] = []
                self.tgt[tid]["roles"] = {}
                self.tr.assign_observers(tid, [])

        # 改派抖动统计（动态策略的代价，必须量化）
        for tid in pending:
            mem = set(u for u, _ in teams.get(tid, []))
            old = self._last_members.get(tid)
            if old is not None and mem != old:
                self.n_switch += len(mem ^ old)
            self._last_members[tid] = mem
        return teams

    def _spread(self, pool):
        """分散：每个目标 1 架，按优先级高的先挑最近机。"""
        pending = [tid for tid in self.tr.pending() if self.tgt[tid]["known"]]
        order = sorted(pending, key=lambda t: -self.priority(t, 0))
        teams = {}
        for tid in order:
            if not pool:
                break
            picks = self.assigner.choose(
                (self.tgt[tid]["x"], self.tgt[tid]["y"]), pool, n=1)
            if not picks:
                continue
            teams[tid] = [(u, "tracker") for u in picks]
            for u in picks:
                pool.pop(u, None)
        return teams

    def _focus(self, pool):
        """集中：选**一个**重点目标，投 3 架；其余机继续搜索。

        全部压上会让其余目标再也无人发现（实测平均消除从 2.5 掉到 1.57），
        所以这里明确留出搜索兵力。
        """
        pending = [tid for tid in self.tr.pending() if self.tgt[tid]["known"]]
        if not pending:
            return {}
        # 已有编组且仍有效 → 不换目标（防抖动）
        for tid in sorted(pending, key=lambda t: -self.priority(t, 3)):
            tg = self.tgt[tid]
            roles = tg.get("roles", {})
            keep = {}
            for u in list(roles):
                if u not in pool:
                    continue
                ux, uy = self.uavs[u]
                if math.hypot(tg["x"] - ux, tg["y"] - uy) > SENSE_R * 1.5:
                    continue
                if not self._visible(ux, uy, tg["x"], tg["y"]):
                    continue
                keep[u] = "tracker"
            if len(keep) >= 2:                  # 还有至少 2 架在位 → 继续打
                teams = {tid: list(keep.items())}
                for u in keep:
                    pool.pop(u, None)
                return teams

        best = max(pending, key=lambda t: self.priority(t, 0))
        picks = self.assigner.choose(
            (self.tgt[best]["x"], self.tgt[best]["y"]), pool, n=PER_FOCUS)
        if not picks:
            return {}
        for u in picks:
            pool.pop(u, None)
        return {best: [(u, "tracker") for u in picks]}

    def _concentrate(self, pool):
        """一般形式：同时打 zoom_k 个目标，每目标 zoom_m 架。"""
        pending = [tid for tid in self.tr.pending() if self.tgt[tid]["known"]]
        order = sorted(pending, key=lambda t: -self.priority(t, 0))
        teams = {}
        for tid in order[:self.zoom_k]:
            if not pool:
                break
            picks = self.assigner.choose(
                (self.tgt[tid]["x"], self.tgt[tid]["y"]), pool, n=self.zoom_m)
            if not picks:
                continue
            teams[tid] = [(u, "tracker") for u in picks]
            for u in picks:
                pool.pop(u, None)
        return teams

    # ---------------- 指标采集 ----------------
    # ---------------- 近距事件诊断 ----------------
    def _diag_pair(self, u, v, dd, tid_u, tid_v, pu, pv_):
        """记录一次近距事件，并区分「正常接近」与「坐标跳变」。

        为什么要区分（用户提的第一个量）
        ------------------------------
        只看位置分不出这两种情况，但它们的修法完全不同：

          情况A 正常接近：5.0 → 4.0 → 3.0 → 2.0 → 1.0 → 0.5
              排斥不够强 / 角度策略在互相打架 → 调参数
          情况B 本帧跳变：前一帧 8 m，这一帧 0.04 m
              **不是避碰问题** —— 是坐标异常（Gazebo 跳点 / 状态同步 / 目标
              瞬移后坐标没跟上），调参数永远修不好

        **必须分两个判据，不能只看"距离跌幅"**
        ------------------------------------
        第一版只用"两机距离单帧跌幅 > 2·v·DT"判跳变，结果把两种完全不同的
        情况混在一起，而且我自己都被误导了：

          * 两机**相向全速**接近：每架各动 v·DT，距离缩短 2·v·DT —— 合法。
          * **单机**瞬移 35 m：距离一步就掉 35 m —— 这才是坐标异常。

        两者都表现为"距离暴跌"，但前者是正常的。所以现在分开判：

          uav_teleport : 某一架**自身**单帧位移 > v_max·DT（物理不可能）
          closing      : 两机各自位移都合法，但距离跌幅超过收拢上限

        另外把两架的坐标**显式记进记录里**（`pu`/`pv_`），不再只存航迹 ——
        上一版报告只能靠读 `_trail` 末帧推断，一旦采样顺序有偏差就没法自证，
        害我无法判断那 35 m 是真异常还是仪器问题。

        相对速度用 `||v_i − v_j||`（用户给的形式），即两机位移/DT 之差的模。
        """
        prev_u = self._prev.get(u)
        prev_v = self._prev.get(v)
        # 上限取追踪与搜索速度的**较大者**。
        #
        # 原来只写了 UAV_TRACK_SPEED*DT = 0.47 m，于是搜索机每帧 0.50 m
        # （UAV_SEARCH_SPEED=5.0）的正常位移全被报成"坐标异常" —— 复核段里
        # 5 例"异常"全是 0.50/0.47 m 的合法搜索步，纯属**狼来了**。
        # 判据要卡的是"物理不可能"，不是"比追踪速度快的都算异常"。
        lim = max(MT.UAV_TRACK_SPEED, MT.UAV_SEARCH_SPEED) * MT.DT

        def step(p_prev, p_now):
            return (math.hypot(p_now[0] - p_prev[0], p_now[1] - p_prev[1])
                    if p_prev is not None else float("nan"))

        du, dv = step(prev_u, pu), step(prev_v, pv_)
        # 单机位移超限 → 无歧义的坐标异常
        teleport = (not math.isnan(du) and du > lim + 1e-6) or \
                   (not math.isnan(dv) and dv > lim + 1e-6)

        if not math.isnan(du) and not math.isnan(dv):
            rel_v = math.hypot(
                (pu[0] - prev_u[0]) / MT.DT - (pv_[0] - prev_v[0]) / MT.DT,
                (pu[1] - prev_u[1]) / MT.DT - (pv_[1] - prev_v[1]) / MT.DT)
        else:
            rel_v = float("nan")

        prev_d = self._dall.get((u, v))
        drop = (prev_d - dd) if prev_d is not None else float("nan")
        # 距离跌幅超过"两机相向全速收拢"的上限 —— 只作**参考量**记录。
        # 注意它**不能单独当跳变判据**：合速度上限是 (v_i + v_j)·DT，
        # "两机各 4.7 迎面"恰好就是 0.94，拿它当阈值必然误判（踩过）。
        # 真正的跳变判据只有 `teleport`：单机自身位移超限，无歧义。
        closing = (prev_d is not None) and (drop > 2.0 * lim + 0.06)

        # `same` 不能直接比字符串：两架都**不在追踪**时 tid 都是 "-"，
        # "-" == "-" 会判成"同目标"，把搜索阶段的相遇全算成"编组协调"。
        # 实测踩到过：加了空间管理后剩下的 9 段全报"同目标 100%"，其实那 9 段
        # 两架都在搜索（相对速度 ≈ 2×5.0 = 搜索速度，而非 2×4.7）。
        both_known = (tid_u != "-" and tid_v != "-")
        self.diag.append(dict(
            t=self.t, u=u, v=v, d=dd, prev_d=prev_d, drop=drop,
            jump=bool(teleport), teleport=bool(teleport), closing=bool(closing),
            rel_v=rel_v, spd_u=(du / MT.DT if not math.isnan(du) else float("nan")),
            spd_v=(dv / MT.DT if not math.isnan(dv) else float("nan")),
            pu=tuple(pu), pv=tuple(pv_),
            prev_u=(tuple(prev_u) if prev_u is not None else None),
            prev_v=(tuple(prev_v) if prev_v is not None else None),
            tgt_u=tid_u, tgt_v=tid_v,
            same=(both_known and tid_u == tid_v),
            both_search=(not both_known),
            trail_u=list(self._trail.get(u, [])),
            trail_v=list(self._trail.get(v, []))))

    def _observe_metrics(self):
        """每帧采集 LOS / 丢失 / 冲突 / 同时有效追踪目标数。

        由 mission_time.Sim.run(on_step=...) 每帧调用，用的是**同一套仿真**，
        所以这些指标与完成数出自同一次运行，不存在两份实现分叉的问题。
        """
        pending = [tid for tid in self.tr.pending()
                   if self.tgt[tid]["known"] and tid not in self.done]
        live = 0
        n_assigned_now = 0
        for tid in pending:
            tgt = self.tr.targets[tid]
            cov = tgt.covered(self.t)
            if cov:
                live += 1
            # 丢失：上一帧有覆盖，这一帧断了
            if self.was_covered.get(tid, False) and not cov:
                self.n_lost += 1
            self.was_covered[tid] = cov
            if self.tgt[tid].get("observers"):
                n_assigned_now += 1
            # LOS：每个当前在读的观察员记一次命中/未命中。
            # 用 last_obs（每个 uav 一条），不是所有 uav —— 没在读的机
            # 不该计入分母，否则 LOS 会被"没参与追踪的机"稀释。
            for u, ob in tgt.last_obs.items():
                self.n_los_tot += 1
                if ob.los:
                    self.n_los_hit += 1
        # 有已知目标却一架都没派 → 这就是"暂不追"（用户要的 T3 行为）
        if pending and n_assigned_now == 0:
            self.n_idle_target += 1
        self.live_targets_hist.append(live)
        self.assign_hist.append({tid: len(self.tgt[tid].get("observers", []))
                                 for tid in pending})

        # 冲突：按**严重程度**分档统计，并把最小值记下来。
        #
        # 只报一个"<3m 的次数"没法判断排斥项有没有用：690 次里可能全是 2.9 m
        # 的擦肩（无害），也可能全是 0.5 m 的贴近（危险）。分档后才知道排斥项
        # 到底是把近的推远了，还是只推了远的。
        us = list(self.uavs.values())
        for i in range(len(us)):
            for j in range(i + 1, len(us)):
                dd = math.hypot(us[i][0] - us[j][0], us[i][1] - us[j][1])
                self.min_dist = min(self.min_dist, dd)
                # 注意**不能 break** —— 三档是累积计数（<3.0 / <1.5 / <1.0），
                # dd=0.5 应该同时计入三档，而不是只记进最宽的那档。
                if dd < 3.0:
                    self.n_clash += 1
                if dd < 1.5:
                    self.n_clash15 += 1
                if dd < 1.0:
                    self.n_clash10 += 1

        # 近距诊断：逐帧记录 <1.5 m 的机对。
        #
        # **必须跟踪所有机对的距离**（不只近距的那些）：要区分"逐渐接近"和
        # "突然出现"，就得知道它们进入 <1.5 m **之前**有多远。一开始我只在
        # <1.5 m 时才开始记，结果"起始d"永远是 ≤1.5，这个问题根本没法回答。
        engaged = self._engaged()
        close_now = set()
        all_pairs = {}
        for ui in sorted(self.uavs):
            for vi in sorted(self.uavs):
                if vi <= ui:
                    continue
                p, q = self.uavs[ui], self.uavs[vi]
                dd = math.hypot(p[0] - q[0], p[1] - q[1])
                all_pairs[(ui, vi)] = dd
                if dd < 1.5:
                    close_now.add((ui, vi))
                    self._diag_pair(ui, vi, dd, engaged.get(ui, "-"),
                                    engaged.get(vi, "-"), tuple(p), tuple(q))

        # 段的开始/结束
        for pair in close_now - self._in_close:
            self._seg_start[pair] = self.t
            self._seg_entry[pair] = all_pairs.get(pair, float("nan"))
        for pair in sorted(self._in_close - close_now):
            seg = [d for d in self.diag if (d["u"], d["v"]) == pair
                   and d["t"] >= self._seg_start.get(pair, 0.0)]
            if seg:
                self.segments.append(dict(
                    pair=pair, t0=seg[0]["t"], t1=seg[-1]["t"], n=len(seg),
                    d_min=min(d["d"] for d in seg), d_first=seg[0]["d"],
                    entry=self._seg_entry.get(pair, float("nan")),
                    any_jump=any(d["jump"] for d in seg),
                    same_target=seg[0]["same"],
                    tgt=(seg[0]["tgt_u"], seg[0]["tgt_v"]),
                    # 把逐帧记录整段留住，供 report_diag 直接读 ——
                    # 一开始我在 report 里回头查 sims[0].diag，结果绝大多数
                    # 段的相对速度/跌幅都打成 "-"（那些段不在 sims[0] 里）。
                    frames=list(seg)))
        self._dall = all_pairs
        self._in_close = close_now

        # 保存本帧位置（下一帧算相对速度）与最近几帧航迹
        for u in self.uavs:
            self._prev[u] = tuple(self.uavs[u])
            tr = self._trail.setdefault(u, [])
            tr.append((self.t, tuple(self.uavs[u])))
            if len(tr) > DIAG_TRAIL:
                tr.pop(0)
        del all_pairs


PER_FOCUS = 3      # focus 策略投几架


def run_once(g, strategy, targets, seed, max_t, zoom_k=2, zoom_m=3,
             cut=DEFAULT_CUT, attack_frac=ATTACK_FRAC, w_dead=W_DEAD):
    sim = AllocSim(g, strategy=strategy, targets=targets, zoom_k=zoom_k,
                   zoom_m=zoom_m, cut=cut, attack_frac=attack_frac,
                   w_dead=w_dead, seed=seed)
    # 逐帧指标靠 on_step 钩子采集 —— 与完成数出自同一次运行。
    t_end, done, evs = sim.run(max_t, on_step=lambda s: s._observe_metrics())
    n = len(done)
    # 平均单目标完成时间 = 消除时刻 - 首次发现时刻（含搜索与重搜）
    disc = {}
    elim = {}
    for (t, kind, tid) in evs:
        if kind == "discover":
            disc.setdefault(tid, t)
        elif kind == "eliminated":
            elim[tid] = t
    spans = [elim[t] - disc[t] for t in elim if t in disc]
    # === 新增搜索统计 ===
    stats = sim.get_search_stats()
    return dict(
        done=n, targets=targets, full=(n == targets),
        evades=sum(1 for e in evs if e[1] == "evade"),
        first=(min(elim.values()) if elim else None),
        span=(sum(spans) / len(spans)) if spans else None,
        los=(100.0 * sim.n_los_hit / sim.n_los_tot) if sim.n_los_tot else 0.0,
        lost=sim.n_lost,
        clash=sim.n_clash,
        clash15=sim.n_clash15,
        clash10=sim.n_clash10,
        mind=(sim.min_dist if sim.min_dist < float("inf") else None),
        switch=sim.n_switch,
        idle=100.0 * sim.n_idle_target / max(1, len(sim.live_targets_hist)),
        live=(sum(sim.live_targets_hist) / len(sim.live_targets_hist)
              if sim.live_targets_hist else 0.0),
        used=t_end, t_end=t_end, evs=evs,
        # === 搜索阶段统计 ===
        first_detect_time=stats["first_detect_time"],  # tid -> 首次发现时间
        first_assign_time=stats["first_assign_time"],    # tid -> 首次分配时间
        detect_to_assign_delay=stats["detect_to_assign_delay"],  # tid -> 发现到分配的延迟
        first_region_time=stats["first_region_time"],    # (rx,ry) -> 首次被搜索时间
        region_coverage=stats["region_coverage"],        # (rx,ry) -> 累计覆盖时间
        remaining_search_ratio=stats["remaining_search_ratio"],  # 剩余搜索面积比例
    )


def _mean(vals):
    vals = [v for v in vals if v is not None]
    return (sum(vals) / len(vals)) if vals else None


def _min(vals):
    """全场最小 —— 用来报最坏贴近程度（最坏情况比均值更能说明安全性）。"""
    vals = [v for v in vals if v is not None]
    return min(vals) if vals else None


def summarize(rows):
    n = len(rows)
    full = sum(1 for r in rows if r["full"])
    return dict(
        n=n, full=full, full_rate=100.0 * full / n,
        avg_done=sum(r["done"] for r in rows) / n,
        first=_mean([r["first"] for r in rows]),
        span=_mean([r["span"] for r in rows]),
        los=sum(r["los"] for r in rows) / n,
        lost=sum(r["lost"] for r in rows) / n,
        clash=sum(r["clash"] for r in rows) / n,
        clash15=sum(r["clash15"] for r in rows) / n,
        clash10=sum(r["clash10"] for r in rows) / n,
        mind=_min([r["mind"] for r in rows]),
        switch=sum(r["switch"] for r in rows) / n,
        idle=sum(r["idle"] for r in rows) / n,
        live=sum(r["live"] for r in rows) / n,
        evades=sum(r["evades"] for r in rows) / n,
    )


STRATS = [
    ("dynamic", "动态（默认）", dict()),
    ("dynamic", "动态 dead=.8（强制看守）", dict(w_dead=0.8)),
    ("dynamic", "动态 cut=.25（过度分散）", dict(cut=0.25)),
    ("concentrate", "2 目标 × 3 架", dict(zoom_k=2, zoom_m=3)),
    ("concentrate", "1 目标 × 3 架", dict(zoom_k=1, zoom_m=3)),
    ("concentrate", "3 目标 × 2 架", dict(zoom_k=3, zoom_m=2)),
    ("spread", "分散（每目标 1 架）", {}),
]

# ---- 编组内空间管理消融（用户指定的四行 + 变体）----
#
# 通过在**模块级**改 mission_time 的全局量来切换，所以每次只变一个因素。
# 注意：这些是模块级全局量，无法并行跑，必须串行。
#
# 三行分别是三个**正交**的机制，第四行检查它们是否叠加：
#   软距离约束 = FRIEND_SAFE（速度层排斥）
#   角度去重   = ANGLE_DEDUP_DEG（按实际方位去重，不是固定 slot）
#   成员滞回   = 已在 _dynamic 的 KEEP_FRAC + 不要求 LOS 的保留里，默认开
SPACE_ABLATION = [
    ("基线（仅成员滞回）", 0.0, 0.0, "chase"),
    ("+ 软距离约束 d=3.0", 3.0, 0.0, "chase"),
    ("+ 软距离约束 d=5.0", 5.0, 0.0, "chase"),
    ("+ 角度去重 60°", 0.0, 60.0, "dedup"),
    ("+ 角度去重 100°", 0.0, 100.0, "dedup"),
    ("+ 距离 d=3.0 + 角度 60°", 3.0, 60.0, "dedup"),
    ("[否证复现] 固定 slot 0/120/240", 0.0, 0.0, "abs"),
    ("[否证复现] slot + 距离 d=3.0", 3.0, 0.0, "abs"),
]


def report_diag(sims, top=14):
    """打印近距事件诊断：相对速度、两机归属目标、以及 A/B 分类。

    这是用户要求补的两个量，目的是把"为什么贴到 0"从猜测变成证据：
      * relative_v = ||v_i − v_j||：区分「慢慢挤过来」和「本帧跳变」
      * tgt_u / tgt_v：区分「同目标 → 编组协调问题」和
        「不同目标 → 全局转场/任务切换冲突」
    """
    segs = []
    for s in sims:
        segs.extend(s.segments)
    if not segs:
        print("近距诊断：无 <1.5 m 事件。")
        return
    n_jump = sum(1 for g in segs if g["any_jump"])
    n_same = sum(1 for g in segs if g["same_target"])
    n_search = sum(1 for g in segs if g["frames"][0]["both_search"])
    n_cross = len(segs) - n_same - n_search
    print("近距诊断（<1.5 m 事件，共 %d 段；%d 局）" % (len(segs), len(sims)))
    print("  含跳变的段 %d (%.0f%%)" % (n_jump, 100.0 * n_jump / len(segs)))
    print("  同目标(编组内) %d (%.0f%%) | **两架都在搜索** %d (%.0f%%) | 不同目标(转场) %d (%.0f%%)"
          % (n_same, 100.0 * n_same / len(segs),
             n_search, 100.0 * n_search / len(segs),
             n_cross, 100.0 * n_cross / len(segs)))
    print("")
    print("%-14s | %-7s | %-6s | %-6s | %-7s | %-7s | %-8s | %-9s | %s"
          % ("机对", "进入d", "最小d", "持续s", "相对速度", "单帧跌幅", "归属目标", "同目标", "判定"))
    print("-" * 108)
    for g in sorted(segs, key=lambda x: x["d_min"])[:top]:
        rel = [f["rel_v"] for f in g["frames"] if not math.isnan(f["rel_v"])]
        drops = [f["drop"] for f in g["frames"] if not math.isnan(f["drop"])]
        rel_s = ("%.2f" % max(rel)) if rel else "-"
        drop_s = ("%.2f" % max(drops)) if drops else "-"
        if g["any_jump"]:
            verdict = "跳变 → 坐标异常（非避碰）"
        elif g["frames"][0]["both_search"]:
            verdict = "搜索相遇 → 无编组约束"
        elif g["same_target"]:
            verdict = "渐近 → 编组协调"
        else:
            verdict = "渐近 → 全局转场冲突"
        tag = g["tgt"] if not g["frames"][0]["both_search"] else ("搜索", "搜索")
        print("%-14s | %7.2f | %6.2f | %6.1f | %7s | %8s | %-9s | %-7s | %s"
              % ("%s-%s" % g["pair"], g["entry"], g["d_min"],
                 g["t1"] - g["t0"], rel_s, drop_s,
                 ("%s/%s" % tag), ("是" if g["same_target"] else "否"), verdict))
    print("-" * 100)
    print("")
    print("  判据：「单帧跌幅 > %.2f m」= 物理上不可能。DT=%.1f s，两机相向各顶速"
          % (2 * MT.UAV_TRACK_SPEED * MT.DT, MT.DT))
    print("        %.1f m/s，一帧最多缩短 %.2f m。超过它只可能是坐标跳变。"
          % (MT.UAV_TRACK_SPEED, 2 * MT.UAV_TRACK_SPEED * MT.DT))
    print("  「进入d」= 刚跌破 1.5 m 时的距离；「相对速度」= ||v_i − v_j||，段内最大。")
    print("  「相对速度 ≈ 2×%.1f = %.1f」说明两机**迎面全速对穿**。注意两个速度"
          % (MT.UAV_SEARCH_SPEED, 2 * MT.UAV_SEARCH_SPEED))
    print("        要分开看：搜索速度 %.1f（2→%.1f），追踪速度 %.1f（2→%.1f）。"
          % (MT.UAV_SEARCH_SPEED, 2 * MT.UAV_SEARCH_SPEED,
             MT.UAV_TRACK_SPEED, 2 * MT.UAV_TRACK_SPEED))
    print("        对穿时从 1.9 m 到 0 只要 0.2 s，而排斥项上限 2 m/s，0.2 s 只能")
    print("        改 0.4 m —— **反应式局部排斥物理上拦不住全速对穿**，必须在更远")
    print("        处就介入（或改搜索路径让纵带不重叠）。")
    print("  「相对速度」= ||v_i − v_j||（用户给的形式），取段内最大值。")
    print("")
    print("  长持续段（>=3 s）= **卡死/伴飞**，不是擦肩而过。看航迹确认：")
    longs = [g for g in segs if g["t1"] - g["t0"] >= 3.0]
    if not longs:
        print("    无。")
    for g in sorted(longs, key=lambda x: -(x["t1"] - x["t0"]))[:6]:
        f0 = g["frames"][0]
        print("    %s  持续 %.1f s  最小 %.2f m  目标 %s/%s"
              % ("%s-%s" % g["pair"], g["t1"] - g["t0"], g["d_min"], *g["tgt"]))
        for nm, tr in (("       ", f0["trail_u"]), ("       ", f0["trail_v"])):
            pts = ["(%.2f,%.2f)" % (p[1][0], p[1][1]) for p in tr][-6:]
            print("    %s%s" % (nm, " ".join(pts) if pts else "(无航迹)"))
    print("")
    # 跳变复核：把原始坐标原样打出来，不再靠读航迹末帧推断。
    #
    # 上一版这里只打了 `_trail` 的末几帧，结果我看到 "uav_5 末3帧全是
    # (-40.13,12.99)" 就判成 35 m 瞬移 —— 但那可能只是采样顺序造成的读数
    # 偏差，无法自证。现在记录里直接存了**两机的本帧坐标与上一帧坐标**，
    # 且跳变判据只看"单机自身位移是否超过 v_max·DT"（无歧义），
    # 两机收拢过快只作为参考量。
    print("  坐标异常复核（判据：单机单帧位移 > v_max·DT = %.2f m，"
          "v_max = max(track %.1f, search %.1f)）："
          % (max(MT.UAV_TRACK_SPEED, MT.UAV_SEARCH_SPEED) * MT.DT,
             MT.UAV_TRACK_SPEED, MT.UAV_SEARCH_SPEED))
    tele = [d for s in sims for d in s.diag if d["teleport"]]
    if not tele:
        print("    无。基线里的 0 段跳变成立。")
    for d in sorted(tele, key=lambda x: -x["drop"])[:5]:
        print("    t=%6.1f %s-%s  单帧位移 %s=%.2f m / %s=%.2f m"
              % (d["t"], d["u"], d["v"], d["u"], d["spd_u"] * MT.DT,
                 d["v"], d["spd_v"] * MT.DT))
        print("        %s: %s → %s" % (d["u"],
              ("(%.2f,%.2f)" % d["prev_u"]) if d["prev_u"] else "None",
              "(%.2f,%.2f)" % d["pu"]))
        print("        %s: %s → %s" % (d["v"],
              ("(%.2f,%.2f)" % d["prev_v"]) if d["prev_v"] else "None",
              "(%.2f,%.2f)" % d["pv"]))
    # 收拢过快但单机都合法：这是"两机相向全速"，属正常，单独列出来避免误读
    fast = [d for s in sims for d in s.diag if d.get("closing") and not d["teleport"]]
    print("    另有 %d 帧「收拢快于相向全速上限」但单机位移合法 —— "
          "属正常対穿，非异常。" % len(fast))
    print("")
    print("  两类问题的修法完全不同：")
    print("    跳变（坐标异常）—— 调扩散/排斥参数永远修不好，要查状态同步或")
    print("      目标瞬移后的坐标处理。**Gazebo 跳点是 SITL 才有、离线不会有的**，")
    print("      所以离线测出跳变才说明算法侧真有问题。")
    print("    渐近 + 同目标 —— 编组内协调不够，调软距离/角度去重。")
    print("    渐近 + 不同目标 —— 转场/任务切换撞车，要改分配层的路径或时序。")


def run_space_ablation(g, args):
    """编组内空间管理消融。每行只改一个因素，其余与 baseline 完全一致。

    通过模块级全局量切换 `mission_time` 的 FRIEND_SAFE / ANGLE_DEDUP_DEG /
    SLOT_MODE，所以**必须串行**（并行会互相污染）。跑完恢复原值。
    """
    old = (MT.FRIEND_SAFE, MT.ANGLE_DEDUP_DEG, MT.SLOT_MODE)
    print("=" * 124)
    print("编组内空间管理消融：软距离约束 / 角度去重 / 成员滞回（不引入固定 slot）")
    print("=" * 124)
    print("分配策略固定为「动态（默认）」| 地图 %s | %d 目标 × %d 局 | 追踪 %.1f m/s"
          % (os.path.basename(METADATA), args.targets, args.runs, MT.UAV_TRACK_SPEED))
    print("")
    print("%-32s | %-9s | %-13s | %-8s | %-8s | %-8s | %-8s | %-9s"
          % ("方法", "完成数", "15s成功率", "同时追踪", "冲突<3m", "<1.5m", "<1.0m", "最小间距"))
    print("-" * 124)
    rows_all = []
    try:
        for label, safe, dedup, slot in SPACE_ABLATION:
            MT.FRIEND_SAFE, MT.ANGLE_DEDUP_DEG, MT.SLOT_MODE = safe, dedup, slot
            rows = [run_once(g, "dynamic", args.targets, args.seed + k, args.max_t)
                    for k in range(args.runs)]
            s = summarize(rows)
            rows_all.append((label, s))
            print("%-32s | %4.2f/%d  | %6.1f%% (%2d/%2d) | %8.2f | %6.1f | %6.1f | %6.1f | %8s"
                  % (label, s["avg_done"], args.targets, s["full_rate"], s["full"], s["n"],
                     s["live"], s["clash"], s["clash15"], s["clash10"],
                     ("%.2f m" % s["mind"]) if s["mind"] is not None else "  -  "))
    finally:
        MT.FRIEND_SAFE, MT.ANGLE_DEDUP_DEG, MT.SLOT_MODE = old
    print("-" * 124)
    print("")
    print("判读（相对第一行基线）：")
    base = rows_all[0][1]
    for label, s in rows_all[1:]:
        print("  %-32s 完成 %+.2f  冲突<3m %+7.1f  <1.0m %+6.1f  最小间距 %s"
              % (label, s["avg_done"] - base["avg_done"],
                 s["clash"] - base["clash"], s["clash10"] - base["clash10"],
                 ("%.2f m" % s["mind"]) if s["mind"] is not None else "-"))
    print("")
    print("  * 「最小间距」是**全场最坏**贴近程度，比均值更能说明安全裕度。")
    print("    飞行栈要求 min_separation_m = 3.0 m，所以看的是这一列能不能站住。")
    print("  * 最后两行是**复现既有否证**，不是候选方案：固定 slot 在「同一走廊")
    print("    接近」时会把几架挤到同一侧（tracker_blocker.py 消融表 60.5% → 35.0%）。")
    print("  * 角度去重与固定 slot 的区别：前者只在**方位已经撞车**时才各让一半，")
    print("    方位本来散开时完全不动；后者不管现状一律拽到指定角度。")
    print("=" * 124)
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--strategy", default="dynamic",
                    choices=["dynamic", "spread", "focus", "concentrate"])
    ap.add_argument("--targets", type=int, default=6)
    ap.add_argument("--runs", type=int, default=30)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--max-t", type=float, default=300.0)
    ap.add_argument("--cut", type=float, default=DEFAULT_CUT,
                    help="动态分配性价比阈值：高=集中，低=分散")
    ap.add_argument("--attack-frac", type=float, default=ATTACK_FRAC,
                    help="值不值得打：价值须达最值目标的此比例，否则暂不追")
    ap.add_argument("--compare", action="store_true", help="跑完整对照表")
    ap.add_argument("--diag", action="store_true",
                    help="跑诊断：打印近距事件的相对速度/归属目标/跳变分类")
    ap.add_argument("--space-ablation", action="store_true",
                    help="跑编组内空间管理消融（软距离约束 / 角度去重 / slot）")
    ap.add_argument("--space", default="off",
                    choices=["off", "on", "bearing", "sep1", "sep2", "sep3", "sep5", "sepall", "los"],
                    help="on = 软距离 d=3.0 + 角度去重 60°；"
                         "bearing = 接近射线分离（接近段恒等变换，已否证）；"
                         "sepN = 连续角度分离，期望机间距 N 米；sepall = 推荐全开")
    args = ap.parse_args()

    # 推荐的编组内空间管理组合。**注意：下面这组数是我修掉 FRIEND_MAX 限幅
    # bug 之后重测的**（旧的 "-96% / 最小间距 0.04 m" 是在 bug 代码上量的，
    # 那 13 个"瞬移段"全是我自己的 bug 制造的假冲突，已作废）。
    # 干净版 n=40、6 目标：
    #   基线        5.55/6  65.0%  冲突<3m 690.9  最小间距 0.01 m
    #   d=3.0+60°   5.40/6  52.5%  冲突<3m 103.9  最小间距 0.00 m
    # 即：冲突 -85%，但 15s 成功率 -12.5 点，且**最小间距没有改善**
    # （要求 3.0 m，实测仍是 0.00 m）。所以默认关闭。
    #
    # 现场（probe_form.py --trace，seed=1 t=19.0）：远机落点=目标本体，
    # 扇形旋转作用在零向量上 → 恒等变换 → 同目标各机接近段零分离。
    # probe_phase.py n=4：编组内重合占近距事件 96.1%、贴身 97.3%，
    # 搜索阶段仅 3.5%。所以修在编组内是对的。
    if args.space == "on":
        MT.FRIEND_SAFE, MT.ANGLE_DEDUP_DEG, MT.SLOT_MODE = 3.0, 60.0, "dedup"
        print("[space] 启用编组内空间管理：软距离 d=3.0 m + 角度去重 60°")
    elif args.space == "bearing":
        MT.FRIEND_SAFE, MT.ANGLE_DEDUP_DEG, MT.SLOT_MODE = 0.0, 0.0, "bearing"
        print("[space] 启用接近射线分离（SLOT_MODE=bearing，纯几何，无排斥）")
    elif args.space.startswith("sep"):
        if args.space == "sepall":
            MT.FRIEND_SAFE, MT.ANGLE_DEDUP_DEG, MT.SLOT_MODE = 1.5, 0.0, "chase"
            MT.SEP_TARGET_M, MT.SEARCH_PHASE = 3.0, 6
            print("[space] 启用推荐全开：SEP=3 + phase6 + 排斥1.5")
        elif args.space == "los":
            MT.FRIEND_SAFE, MT.ANGLE_DEDUP_DEG, MT.SLOT_MODE = 3.0, 0.0, "chase"
            MT.SEP_TARGET_M, MT.SEARCH_PHASE = 3.0, 6
            print("[space] 启用 LOS 安全模式：SEP=3 + phase6 + fr3 + LOS 保持检查")
        else:
            sep = float(args.space[3:])
            MT.FRIEND_SAFE, MT.ANGLE_DEDUP_DEG, MT.SLOT_MODE = 0.0, 0.0, "chase"
            MT.SEP_TARGET_M = sep
            print("[space] 启用连续角度分离：期望机间距 %.1f m"
                  "（Δθ=2·asin(SEP/2r) 反解，远时自动变小，保持 chase 径向几何）"
                  % MT.SEP_TARGET_M)

    g = MT.MapGrid(METADATA)

    if args.diag:
        sims = []
        for k in range(args.runs):
            sim = AllocSim(g, strategy="dynamic", targets=args.targets, seed=args.seed + k)
            sim.run(args.max_t, on_step=lambda s: s._observe_metrics())
            sims.append(sim)
        report_diag(sims)
        return 0

    if args.space_ablation:
        return run_space_ablation(g, args)

    print("=" * 112)
    print("任务分配器：动态性价比 vs 固定编组")
    print("=" * 112)
    print("地图 %s | 搜索 %.1f m/s | 追踪 %.1f m/s | 躲藏=理想 | %d 目标 × %d 局"
          % (os.path.basename(METADATA), MT.UAV_SEARCH_SPEED, MT.UAV_TRACK_SPEED,
             args.targets, args.runs))
    print("规则：游走 1 / 逃跑 2 / 确认 15 s / 瞬移 30 s | 计时含搜索与重搜")
    print("")

    if not args.compare:
        rows = [run_once(g, args.strategy, args.targets, args.seed + k, args.max_t,
                         cut=args.cut)
                for k in range(args.runs)]
        s = summarize(rows)
        print("策略 %s | 完成 %.2f/%d | 全清率 %.1f%% | 首次消除 %s | 平均单目标 %s"
              % (args.strategy, s["avg_done"], args.targets, s["full_rate"],
                 ("%.1f s" % s["first"]) if s["first"] else "-",
                 ("%.1f s" % s["span"]) if s["span"] else "-"))
        print("LOS %.1f%% | 丢失 %.1f 次/局 | 冲突 %.1f 次/局 | 同时有效追踪 %.2f 个"
              % (s["los"], s["lost"], s["clash"], s["live"]))
        print("改派 %.1f 架次/局 | 目标无人可派的时间占比 %.1f%% | 瞬移 %.1f 次/局"
              % (s["switch"], s["idle"], s["evades"]))
        return 0

    print("%-24s | %-9s | %-13s | %-9s | %-9s | %-7s | %-7s | %-7s | %-7s"
          % ("方法", "完成数", "15s成功率", "首次消除", "均单目标",
             "同时追踪", "LOS", "丢失", "冲突"))
    print("-" * 112)
    for key, label, kw in STRATS:
        rows = [run_once(g, key, args.targets, args.seed + k, args.max_t, **kw)
                for k in range(args.runs)]
        s = summarize(rows)
        print("%-24s | %4.2f/%d  | %6.1f%% (%2d/%2d) | %7s | %7s | %7.2f | %5.1f%% | %6.1f | %6.1f"
              % (label, s["avg_done"], args.targets, s["full_rate"], s["full"], s["n"],
                 ("%.1f s" % s["first"]) if s["first"] else "  -  ",
                 ("%.1f s" % s["span"]) if s["span"] else "  -  ",
                 s["live"], s["los"], s["lost"], s["clash"]))
    print("-" * 112)
    print("")
    print("判读：")
    print("  * 两个旋钮（方向别搞反）：attack_frac 高 = 只有最值的目标够格被打（集中）；")
    print("    cut 低 = 在打的目标容易加到第 2/3 架（集中）。两者都往低了调 = 更分散。")
    print("  * 「同时追踪」与「完成数」可能此消彼长：集中会压低头者，")
    print("    但只有当完成数真的变高，集中才算赢。")
    print("  * 「均单目标」= 消除时刻 - 首次发现时刻（含重新搜索的时间）。")
    print("=" * 112)
    return 0


if __name__ == "__main__":
    sys.exit(main())
