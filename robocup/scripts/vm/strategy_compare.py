#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""追踪策略离线对比：严格 15 秒连续 LOS 下，哪种追踪方式更优。

问题背景（2026 RoboCup 集群搜索规则）
------------------------------------
规则 5 要求无人机对目标**连续 15 秒**正确广播 ID + 坐标才算消除。
严格解释：LOS 一旦被遮挡，连续计时立刻归零。

规则 4：目标被感知累计 30 秒仍未消除 → 瞬移躲藏。所以必须在 30 秒内
累积满 15 秒**连续**观测，中途断一次就前功尽弃。

规则 3：目标感知到无人机后以 2 m/s 逃跑，方向为**远离最近的无人机**。

这带来一个关键性质：**目标的逃跑方向由我们的站位决定** —— 我们站在哪，
它就往反方向跑。于是"站立点选择"成为可优化的策略空间，而不只是"追"。

本脚本在真实地图上离线比较若干追踪策略，无需 ROS/Gazebo。

地图事实（training_city_full_s7）
--------------------------------
* 38 栋建筑，高度 9.2~17.9 m，**全部高于 6 m 飞行限高** → 完全遮挡
* 184 根灯杆，高 7.5 m，同样高于 6 m → 也是遮挡物
* 故 6 m 高度下 LOS 是**纯 2D 平面遮挡**，可按栅格射线判定

用法：
    python3 strategy_compare.py [--trials 200] [--seed 42]
"""

import argparse
import json
import math
import os
import random
import sys
from collections import deque

WS = os.environ.get("ROBOCUP_WORKSPACE",
                    os.environ.get("ROBOCUP_WS", os.path.expanduser("~/team_ws/robocup")))
METADATA = os.path.join(
    WS, "src/robocup_training_worlds/worlds/generated/training_city_full_s7.json")

# ---- 规则常量 ----
UAV_SPEED_MAX = 6.0      # 规则：无人机水平速度**上限**（可调，非必须跑满）
UAV_SPEED = UAV_SPEED_MAX  # 当前试验用的速度（可被 --uav-speed 覆盖）
TARGET_WALK = 1.0        # 规则2：未感知时游走
TARGET_FLEE = 2.0        # 规则3：感知后逃跑
CONFIRM_S = 15.0         # 规则5：连续确认时长
EVADE_S = 30.0           # 规则4：被感知 30s 未消除 → 瞬移
SENSE_R = 20.0           # 感知半径（与 agent 的 DETECT_RADIUS 一致）
DT = 0.1                 # 仿真步长 s
MAX_T = 60.0             # 单次试验最长仿真时间 s


# ============================ 地图与 LOS ============================
class MapGrid(object):
    """占据栅格 + 射线遮挡判定（纯 2D，因所有障碍都高于飞行高度）。"""

    def __init__(self, metadata_path, resolution=None):
        with open(metadata_path) as fh:
            md = json.load(fh)
        g = md["grid"]
        self.res = float(resolution or g["resolution_m"])
        self.origin = (float(g["origin"][0]), float(g["origin"][1]))
        self.w, self.h = int(g["width"]), int(g["height"])
        # 用 metadata 自带的 RLE 栅格（已含全部障碍的平面投影）
        self.cells = self._decode(g["data"], self.w * self.h)
        self.md = md
        # BFS 距离场缓存：{目标格: {格: 步数}}，LRU。
        # 目标以 ~1 m/s 移动，多架机在同一帧/相邻帧常用同一个目标格，
        # 命中率高。主连通域 63859 格，每次现算太贵，必须缓存。
        # cap 小一点，避免一个长局把几百个目标格的距离场都留在内存里。
        self._field_cache = {}
        self._field_cap = 8

    @staticmethod
    def _decode(data, expected):
        """rle_pairs 解码：[[value, count], ...]。"""
        out = bytearray(expected)
        idx = 0
        for pair in data:
            v, n = int(pair[0]), int(pair[1])
            for _ in range(n):
                if idx >= expected:
                    break
                out[idx] = v
                idx += 1
        return out

    def to_cell(self, x, y):
        return (int(math.floor((x - self.origin[0]) / self.res)),
                int(math.floor((y - self.origin[1]) / self.res)))

    def in_bounds(self, c):
        return 0 <= c[0] < self.w and 0 <= c[1] < self.h

    def free_cell(self, c):
        return self.in_bounds(c) and self.cells[c[1] * self.w + c[0]] == 0

    def free(self, x, y):
        return self.free_cell(self.to_cell(x, y))

    def visible(self, ax, ay, bx, by):
        """(ax,ay) 能否看见 (bx,by) —— DDA 射线，端点格豁免。"""
        ix, iy = self.to_cell(ax, ay)
        gx, gy = self.to_cell(bx, by)
        if (ix, iy) == (gx, gy):
            return True
        dx, dy = bx - ax, by - ay
        sx = 1 if dx > 0 else (-1 if dx < 0 else 0)
        sy = 1 if dy > 0 else (-1 if dy < 0 else 0)
        INF = float("inf")
        if sx:
            nx = self.origin[0] + (ix + (1 if sx > 0 else 0)) * self.res
            tmax_x = (nx - ax) / dx
            tdx = self.res / abs(dx)
        else:
            tmax_x, tdx = INF, INF
        if sy:
            ny = self.origin[1] + (iy + (1 if sy > 0 else 0)) * self.res
            tmax_y = (ny - ay) / dy
            tdy = self.res / abs(dy)
        else:
            tmax_y, tdy = INF, INF
        guard = 0
        limit = int(math.hypot(gx - ix, gy - iy)) + 4
        while (ix, iy) != (gx, gy) and guard < limit:
            guard += 1
            if tmax_x < tmax_y:
                ix += sx
                tmax_x += tdx
            else:
                iy += sy
                tmax_y += tdy
            if (ix, iy) == (gx, gy):
                break
            if not self.free_cell((ix, iy)):
                return False
        return True

    def los_clearance(self, ax, ay, bx, by):
        """视线被遮挡处到观察者的距离；全程无遮挡返回 None（表示无限远）。"""
        ix, iy = self.to_cell(ax, ay)
        gx, gy = self.to_cell(bx, by)
        if (ix, iy) == (gx, gy):
            return None
        dx, dy = bx - ax, by - ay
        sx = 1 if dx > 0 else (-1 if dx < 0 else 0)
        sy = 1 if dy > 0 else (-1 if dy < 0 else 0)
        INF = float("inf")
        if sx:
            nx = self.origin[0] + (ix + (1 if sx > 0 else 0)) * self.res
            tmax_x = (nx - ax) / dx
            tdx = self.res / abs(dx)
        else:
            tmax_x, tdx = INF, INF
        if sy:
            ny = self.origin[1] + (iy + (1 if sy > 0 else 0)) * self.res
            tmax_y = (ny - ay) / dy
            tdy = self.res / abs(dy)
        else:
            tmax_y, tdy = INF, INF
        guard = 0
        limit = int(math.hypot(gx - ix, gy - iy)) + 4
        while (ix, iy) != (gx, gy) and guard < limit:
            guard += 1
            if tmax_x < tmax_y:
                ix += sx
                tmax_x += tdx
                t_hit = tmax_x - tdx
            else:
                iy += sy
                tmax_y += tdy
                t_hit = tmax_y - tdy
            if (ix, iy) == (gx, gy):
                break
            if not self.free_cell((ix, iy)):
                return max(0.0, t_hit) * math.hypot(dx, dy)
        return None

    def _bfs_field(self, goal_cell, max_cells=70000):
        """以 goal_cell 为源、在可通行格上做 BFS，返回 {cell: 步数}。

        结果按 goal_cell 做 LRU 缓存：目标只以 ~1 m/s 走，一帧内多架机、
        相邻帧之间常常共用同一个目标格，缓存命中率很高。缓存是必要的 ——
        主连通域有 63859 格，每次现算在 Python 里是几十毫秒级。

        **这是判断"这一步是不是真的朝目标去"的唯一可靠依据**：窄缝袋口处
        直线距离会骗人（往袋里走反而更接近目标），BFS 距离不会。
        """
        if goal_cell in self._field_cache:
            f = self._field_cache.pop(goal_cell)
            self._field_cache[goal_cell] = f          # LRU: 移到末尾
            return f
        if not self.free_cell(goal_cell):
            return None
        dist = {goal_cell: 0}
        q = deque([goal_cell])
        while q and len(dist) < max_cells:
            c = q.popleft()
            d0 = dist[c] + 1
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                n = (c[0] + dx, c[1] + dy)
                if n in dist or not self.free_cell(n):
                    continue
                dist[n] = d0
                q.append(n)
        if len(self._field_cache) >= self._field_cap:
            # dict 是有序的（3.7+ 保证插入序），淘汰最早插入的那个。
            # **不能用 popitem(last=False)** —— 那是 OrderedDict 的签名，
            # 普通 dict 的 popitem() 不收参数，运行时会 TypeError。
            # 单元测试没覆盖到这里（只走了 cache-miss 路径），是跑真仿真
            # 才暴露的。
            self._field_cache.pop(next(iter(self._field_cache)))
        self._field_cache[goal_cell] = dist
        return dist

    def _escape_dir(self, x, y, tx, ty):
        """沿 BFS 距离场下降的第一步方向（单位向量）。无解返回 None。"""
        f = self._bfs_field(self.to_cell(tx, ty))
        if not f:
            return None
        c = self.to_cell(x, y)
        if c not in f:
            return None
        best, bd = None, f[c]
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (c[0] + dx, c[1] + dy)
            if n in f and f[n] < bd:
                bd, best = f[n], n
        if best is None:
            return None
        px = self.origin[0] + (best[0] + 0.5) * self.res
        py = self.origin[1] + (best[1] + 0.5) * self.res
        vx, vy = px - x, py - y
        n = math.hypot(vx, vy)
        if n < 1e-9:
            return None
        return (vx / n, vy / n)

    def _field_dist(self, x, y, tx, ty):
        """当前位置的 BFS 距离（步数）；不在场里返回 None。"""
        f = self._bfs_field(self.to_cell(tx, ty))
        if not f:
            return None
        return f.get(self.to_cell(x, y))

    def _clear_run(self, x, y, ang, maxd):
        """沿 ang 方向从 (x,y) 出发能连续走多远（m），上限 maxd。

        脱困方向不能按"离目标多近"排序 —— 第一版就是这么写的，结果
        在窄缝里**来回振荡**：位置抖一下，"最朝向目标"的方向就翻面，
        实测 50 帧走了 25 m 累计位移、净位移只有 0.13 m。按"前方还有
        多长可通行"排序是个**位置的光滑函数**，不会因为抖动翻面。
        """
        ca, sa = math.cos(ang), math.sin(ang)
        n = max(1, int(maxd / max(self.res, 1e-6)))
        run = 0.0
        for i in range(1, n + 1):
            px, py = x + ca * self.res * i, y + sa * self.res * i
            if not self.free(px, py):
                break
            run += self.res
        return run

    def speed_toward(self, x, y, tx, ty, speed):
        """朝目标移动一步。

        返回 (新x, 新y, 受阻标志)。受阻时**尝试转向**而不是原地不动 ——
        否则贴着建筑的目标会卡死（实测：目标一出生就被建筑夹住时，
        位移恒为 0，导致所有策略的"达成率"都是假的）。

        ---- 脱困逻辑（相对最初版本的新增）----
        最初版本只试「直行 + ±20…±120° 侧滑」，**全失败就 return x, y**
        —— 返回当前位置。这个函数的输出会被写回 `self.uavs[u] = [ax, ay]`，
        于是"留在原地"是个**恒等映射**：下一帧输入同一个点、输出同一个点，
        **永远出不来**。实测（probe_stuck.py, seed=9）：uav_2 冻结 67.1 s、
        uav_4 冻结 68.7 s，四周围 0.5 m 的 12 个方向里 9 个不可通行，
        唯一出口在 ±120° 锥角**之外** → 每帧都失败 → 位置逐帧按位相同。

        改三层：
          2 级 全向 ±180°（原锥角只到 ±120°，出口常在锥角外）
          3 级 步长递减（缝隙比 step 窄时，半格挪动也胜过不动）
          4 级 相邻格兜底，且**位移不得超过 step**（否则又变成瞬移）

        排序用 `_clear_run`（前方可通行长度）而不是"离目标近" —— 后者在
        缝隙口会随抖动翻面，导致原地振荡。
        """
        if speed <= 0.0:
            return x, y, False
        d = math.hypot(tx - x, ty - y)
        if d < 1e-6:
            return x, y, False
        ux, uy = (tx - x) / d, (ty - y) / d
        step = speed * DT

        # 第 1 级：直奔目标 + 沿墙滑动。
        #
        # **但滑动必须在 BFS 距离场里"真的在靠近目标"才算数**。
        # 这是本函数最关键的一处修正。原来的滑动只看"落点可通行"，
        # 结果在袋口会**主动往回走**：实测（probe_st_dir.py 逐帧表）
        #   帧18 (-45.761,-3.522) 级1全失败 → BFS → (-46.214,-3.733)
        #   帧19 (-46.214,-3.733) 级1侧滑-20 → 拉回 → (-45.761,-3.522)
        #   帧20/21 重复 … 完美的 2-周期，永远出不去。
        # BFS 明明把机挪出去了，下一帧"侧滑 -20°"又把它拽回袋里 ——
        # 因为侧滑的判据只有"落点 free"，它对"这一步是朝袋底还是朝出口"
        # 一无所知。累计位移 25 m、净位移 0.13 m 就是这么来的。
        #
        # 判据用 BFS 步数而不是欧氏距离：袋口处欧氏距离会骗人（往袋里走
        # 反而更接近目标），BFS 步数不会。
        nx, ny = x + ux * step, y + uy * step
        if self.free(nx, ny):
            return nx, ny, False
        f = self._bfs_field(self.to_cell(tx, ty))
        here = f.get(self.to_cell(x, y)) if f else None
        for ang in (20, -20, 40, -40, 60, -60, 80, -80, 100, -100, 120, -120):
            r = math.radians(ang)
            vx = ux * math.cos(r) - uy * math.sin(r)
            vy = ux * math.sin(r) + uy * math.cos(r)
            cx, cy = x + vx * step, y + vy * step
            if not self.free(cx, cy):
                continue
            if here is not None:
                # 只在"确实拉近了 BFS 距离"时才接受这个滑动
                cd = f.get(self.to_cell(cx, cy))
                if cd is None or cd >= here:
                    continue
            return cx, cy, True

        # 第 2 级：沿 BFS 距离场下降（真正的脱困）。
        #
        # 这里原本是"全向 12 个方向按 clear_run 排序再挑一个"的贪婪扫描，
        # 我试过并**否掉了它** —— 在袋口它必然振荡：
        #   实测起点 (-45.67,-3.55)：36 个方向里只有 2 个可通行，且**几乎
        #   相反**（相对目标 +150° 与 +180°，clear_run 都是 2.00，并列第一）。
        #   改成按 clear_run 排序也没用 —— 两者都是 2.00，仍并列，谁排前面
        #   取决于浮点误差，照样翻面。
        # 根因是"每帧独立决策"的局部性：只看得到一步，看不到那个 2 m 的
        # clear_run 其实是**袋底**而不是出口。
        #
        # BFS 连通域分析（probe_conn.py）证明这里不是封闭口袋：该点属于
        # 15965 m² 的主连通域，到航点存在 **67 步**栅格最短路（欧氏 17.2 m）。
        # 路是通的、还很近，只是局部贪婪看不到。
        # 实测沿 BFS 距离场下降：**第 65 帧抵达航点**（0.5 m 内），
        # 而 greedy 版 400 帧都出不去。
        p = self._escape_dir(x, y, tx, ty)
        if p is not None:
            ca, sa = p
            for frac in (1.0, 0.5, 0.25):
                cx, cy = x + ca * step * frac, y + sa * step * frac
                if self.free(cx, cy):
                    return cx, cy, True

        # 第 3 级：BFS 也失败（目标被围死/超出搜索上限）。朝相邻可通行格
        # 的方向迈一步，保证**非恒等**，避免永久冻结。
        #
        # 注意不能要求"走到格中心"：speed=4.7 时 step=0.47 < res=0.5，
        # 格中心永远超出限幅 → 全部被拒 → 返回原地 → U1 失败（实测 2 例）。
        # 正确做法是**朝格中心方向**走 min(step, 到中心距离)，只保证方向对、
        # 位移合法。
        c = self.to_cell(x, y)
        best = None
        for ddy in (-1, 0, 1):
            for ddx in (-1, 0, 1):
                if ddx == 0 and ddy == 0:
                    continue
                cc = (c[0] + ddx, c[1] + ddy)
                if not self.free_cell(cc):
                    continue
                px = self.origin[0] + (cc[0] + 0.5) * self.res
                py = self.origin[1] + (cc[1] + 0.5) * self.res
                vx, vy = px - x, py - y
                n = math.hypot(vx, vy)
                if n < 1e-9:
                    continue                     # 已经在格中心
                m = min(step * 0.25, n)          # 小步，且 <= step
                qx, qy = x + vx / n * m, y + vy / n * m
                if not self.free(qx, qy):
                    continue
                dd = math.hypot(px - tx, py - ty)
                if best is None or dd < best[0]:
                    best = (dd, qx, qy)
        if best is not None:
            return best[1], best[2], True
        return x, y, True


    def open_space_score(self, x, y, radius_cells=8):
        """(x,y) 周围的可通行程度 [0,1]：1 = 全开阔，0 = 被障碍包围。

        用于筛选「目标出生点」—— 贴在建筑缝隙里的点会让目标一逃跑就卡住，
        使实验失真。radius_cells=8 即 4 m 半径。
        """
        c = self.to_cell(x, y)
        free_n = total = 0
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                total += 1
                if self.free_cell((c[0] + dx, c[1] + dy)):
                    free_n += 1
        return free_n / float(total) if total else 0.0


# ============================ 速度策略 ============================
# 关键认识：任务目标不是「抓住」目标，而是「持续看见 15 秒」。
# 因此速度不该恒定，而应按「距离/视线状态」分阶段调整 ——
# 冲太快会过冲（实测 5 m/s 时断线从 0.45 激增到 4.32：冲过目标 →
# 目标按规则掉头 → 再冲过 → LOS 反复切换），冲太慢则追不上。
SPEED_POLICY_CONST = "const"
SPEED_POLICY_ADAPTIVE = "adaptive"

# 尾随段的目标间距：保持在这个距离既能稳定看见，又不易被建筑切断视线
TAIL_STANDOFF_M = 7.0


def compute_speed(g, uav, tgt, los_ok, base_speed, policy,
                  target_speed=TARGET_FLEE, tgt_vel=(0.0, 0.0),
                  time_left=EVADE_S):
    """按当前态势给出无人机速度。

    policy=const: 恒定 base_speed。
    policy=adaptive: 只在**确实该减速**的时机减速 ——

    上一版把「接近目标」当作减速信号，那是错的：规则4 的 30 秒是硬预算，
    减速直接等于浪费掉本就紧张的时间。实测上一版让理想躲藏下的达成率从
    94% 掉到 20%。

    本版只保留两类减速，其余情况一律全速：
      1) **过冲**：目标正在往我身后跑（说明我冲过头了）→ 减速掉头
      2) 目标已在我前方但速度方向与我的接近方向严重背离 → 轻微减速
    距离本身**不再是减速依据** —— 越近越要咬住。
    """
    if policy != SPEED_POLICY_ADAPTIVE:
        return base_speed

    d = math.hypot(tgt[0] - uav[0], tgt[1] - uav[1])
    if d < 1e-6:
        return base_speed

    # 视线断了：全速去补（此时任何减速都在浪费 30 秒预算）
    if not los_ok:
        return base_speed

    # 目标相对我的方位（单位向量）
    to_tgt = ((tgt[0] - uav[0]) / d, (tgt[1] - uav[1]) / d)
    tv = math.hypot(tgt_vel[0], tgt_vel[1])

    if tv > 1e-6:
        # 过冲判据：目标速度方向与「我看向目标」的方向点积 < 0
        # 意味着目标正在往我背后跑 —— 我已经冲过去了。
        dot = (tgt_vel[0] / tv) * to_tgt[0] + (tgt_vel[1] / tv) * to_tgt[1]
        if dot < -0.2 and d < 6.0:
            return max(1.0, base_speed * 0.45)   # 冲过头了，减速掉头
        if dot < 0.0 and d < 3.0:
            return max(1.0, base_speed * 0.6)    # 极近且目标在侧后，稍减

    # 时间紧迫（接近 30 s 瞬移阈值）时更要全速
    if time_left < 8.0:
        return base_speed

    return base_speed


# ============================ 目标躲藏行为 ============================
# 躲藏聪明度分档，用于验证「按最坏情况设计」的鲁棒性：
#   HIDE_NONE(0)  纯「远离无人机」，不利用建筑（规则3 的字面最小实现）
#   HIDE_LOCAL(1) 会避开眼前的墙，但**不知道**地图，无法主动切断视线
#                 —— 这是最接近真实恐怖分子的模型
#   HIDE_IDEAL(2) 拥有地图上帝视角，精确选择能切断无人机视线的方向
#                 —— 这是**最坏情况**，作为设计基准
#
# 单调性：HIDE_IDEAL 最难 → 策略若在此达标，在 LOCAL / NONE 下必然也达标。
HIDE_NONE = 0
HIDE_LOCAL = 1
HIDE_IDEAL = 2

# HIDE_LOCAL 的「视力」：只能看到前方这么多米内的障碍
LOCAL_SIGHT_M = 5.0


def pick_flee_dir(g, tgt, uav, hide_level, prev_dir=(0.0, 0.0)):
    """目标选择逃跑方向。

    返回单位方向向量；若无可行方向则返回 None。
    """
    away = (tgt[0] - uav[0], tgt[1] - uav[1])
    n = math.hypot(away[0], away[1]) or 1.0
    away = (away[0] / n, away[1] / n)

    if hide_level <= HIDE_NONE:
        return away

    best, best_score = None, -1e9
    # 候选方向：以「远离无人机」为中心向两侧展开
    # （目标不会反着朝无人机冲过来，这也符合规则的"逃离"语义）
    for k in range(-6, 7):
        a = math.radians(k * 15.0)
        ca, sa = math.cos(a), math.sin(a)
        dx = away[0] * ca - away[1] * sa
        dy = away[0] * sa + away[1] * ca
        # 必须可通行（不能往墙里跑）
        if not g.free(tgt[0] + dx * 3.0, tgt[1] + dy * 3.0):
            continue
        # 评分1：仍在远离无人机（遵守规则3）
        sep = dx * away[0] + dy * away[1]
        # 评分3：方向连续性（避免每步抖动）
        cont = dx * prev_dir[0] + dy * prev_dir[1] if prev_dir != (0.0, 0.0) else 0.0

        if hide_level == HIDE_IDEAL:
            # 上帝视角：逃到 12 m 外后，无人机是否看不见
            fx, fy = tgt[0] + dx * 12.0, tgt[1] + dy * 12.0
            hide_bonus = 1.0 if not g.visible(uav[0], uav[1], fx, fy) else 0.0
            score = 1.0 * sep + 2.0 * hide_bonus + 0.6 * cont
        else:
            # HIDE_LOCAL：只用有限的"视力"看前方有没有墙，
            # 会自然钻进附近能遮挡的建筑，但无法主动规划切断视线。
            fx, fy = tgt[0] + dx * LOCAL_SIGHT_M, tgt[1] + dy * LOCAL_SIGHT_M
            # 前方越通畅越好走；若前方就是墙，说明会撞上，扣分
            path_pen = 0.0
            for frac in (0.4, 0.7, 1.0):
                px = tgt[0] + dx * LOCAL_SIGHT_M * frac
                py = tgt[1] + dy * LOCAL_SIGHT_M * frac
                if not g.free(px, py):
                    path_pen -= 0.8
            score = 1.4 * sep + 1.0 * path_pen + 0.6 * cont

        if score > best_score:
            best_score, best = score, (dx, dy)
    return best if best else away


# ============================ 策略 ============================
def pick_uav_target(g, uav, tgt, tgt_vel, strategy, uav_speed=UAV_SPEED_MAX):
    """各策略给出的无人机目标点（世界坐标）。

    `uav_speed` 只影响需要解算到达时间/代价的策略（intercept、los_guard、
    herd）—— 追逐类策略不依赖它。
    """
    ux, uy = uav
    tx, ty = tgt

    if strategy == "chase":
        # 直追目标当前位置
        return tx, ty

    if strategy == "lead":
        # 追预测位置（按目标当前逃跑方向外推 1.5 s）
        return tx + tgt_vel[0] * 1.5, ty + tgt_vel[1] * 1.5

    if strategy == "intercept":
        # 拦截：解算「我在 t 秒后到达，目标也在 t 秒后到达」的交点
        # 目标逃跑方向恒定（远离无人机），故可用相对速度闭式近似
        best, best_t = (tx, ty), 1e9
        for t in [x * 0.2 for x in range(1, 26)]:
            fx, fy = tx + tgt_vel[0] * t, ty + tgt_vel[1] * t
            d = math.hypot(fx - ux, fy - uy) / max(uav_speed, 1e-6)
            if d <= t and t < best_t:
                best_t, best = t, (fx, fy)
        return best

    if strategy == "los_guard":
        # 保持 LOS：在目标周围一圈候选站位里，选「现在有 LOS 且未来一段时间
        # 也不易被遮」的那个。用 los_clearance 作为遮挡余量度量。
        r = 6.0        # 观察半径（保持在感知范围内且不太近）
        best, best_score = (tx, ty), -1e9
        for k in range(16):
            a = 2 * math.pi * k / 16.0
            px, py = tx + r * math.cos(a), ty + r * math.sin(a)
            if not g.free(px, py):
                continue
            clr = g.los_clearance(px, py, tx, ty)
            margin = 1e6 if clr is None else clr
            # 同时考虑到达代价：越近越容易及时站住
            arrive = math.hypot(px - ux, py - uy) / max(uav_speed, 1e-6)
            score = margin - 2.0 * arrive
            if score > best_score:
                best_score, best = score, (px, py)
        return best

    if strategy == "herd":
        # 驱赶：绕到目标「逃跑方向的反侧」，把它往开阔处赶。
        # 逃跑方向 = 远离无人机；站到目标与开阔区之间，逼它改向。
        away = (tx - ux, ty - uy)
        n = math.hypot(*away) or 1.0
        ux_, uy_ = away[0] / n, away[1] / n
        # 候选：目标周围若干站位，选「站过去之后目标的新逃跑方向 LOS 最好」
        r = 5.0
        best, best_score = (tx, ty), -1e9
        for k in range(16):
            a = 2 * math.pi * k / 16.0
            px, py = tx + r * math.cos(a), ty + r * math.sin(a)
            if not g.free(px, py):
                continue
            # 若我站在 (px,py)，目标会朝远离我的方向逃
            na = (tx - px, ty - py)
            nn = math.hypot(*na) or 1.0
            fdx, fdy = na[0] / nn, na[1] / nn
            # 沿该方向看 25 m 内 LOS 是否通畅（越通畅越好）
            fx, fy = tx + fdx * 25.0, ty + fdy * 25.0
            clr = g.los_clearance(tx, ty, fx, fy)
            margin = 25.0 if clr is None else clr
            arrive = math.hypot(px - ux, py - uy) / max(uav_speed, 1e-6)
            score = margin - 1.5 * arrive
            if score > best_score:
                best_score, best = score, (px, py)
        return best

    return tx, ty


# ============================ 单次试验 ============================
def run_trial(g, strategy, start_xy, tgt_xy, max_t=MAX_T, uav_speed=UAV_SPEED_MAX,
              speed_policy=SPEED_POLICY_CONST, hide=HIDE_NONE):
    """跑一次追踪，返回统计量。

    speed_policy: const（恒定速度）或 adaptive（按距离/LOS 分段调速）
    hide: 躲藏聪明度 HIDE_NONE / HIDE_LOCAL / HIDE_IDEAL

    返回 dict：
      achieved      是否在 max_t 内累积满 15 s 连续确认
      t_confirm     达成耗时（未达成 = None）
      los_breaks    连续计时被打断的次数
      max_streak    最长连续确认时长
      ended_by      结束原因：confirmed / evade / timeout
    """
    uav = list(start_xy)
    tgt = list(tgt_xy)
    tgt_vel = (0.0, 0.0)

    streak = 0.0          # 当前连续确认时长
    max_streak = 0.0
    los_breaks = 0
    tracked_total = 0.0   # 被感知累计（规则4）
    seen_last = False
    t = 0.0
    blocked_streak = 0            # 目标连续受阻步数
    escape_dir = (0.0, 0.0)       # 受阻后锁定的替代逃跑方向
    away_n = (0.0, 0.0)           # 单位化的「远离无人机」方向

    while t < max_t:
        t += DT
        d = math.hypot(tgt[0] - uav[0], tgt[1] - uav[1])
        sensed = d < SENSE_R and g.visible(uav[0], uav[1], tgt[0], tgt[1])

        # ---- 规则4：被感知累计 30 s 未消除 → 瞬移（本实验记为失败）----
        if sensed:
            tracked_total += DT
            if tracked_total >= EVADE_S:
                return dict(achieved=False, t_confirm=None, los_breaks=los_breaks,
                            max_streak=max_streak, ended_by="evade")

        # ---- 规则5：连续确认 ----
        if sensed:
            streak += DT
            max_streak = max(max_streak, streak)
            if streak >= CONFIRM_S:
                return dict(achieved=True, t_confirm=t, los_breaks=los_breaks,
                            max_streak=max_streak, ended_by="confirmed")
        else:
            if seen_last and streak > 0:
                los_breaks += 1
            streak = 0.0
        seen_last = sensed

        # ---- 目标运动（规则3：感知到就以 2 m/s 远离无人机逃跑）----
        if d < SENSE_R:
            away = (tgt[0] - uav[0], tgt[1] - uav[1])
            n = math.hypot(away[0], away[1]) or 1.0
            away_n = (away[0] / n, away[1] / n)
            tgt_vel = (away_n[0] * TARGET_FLEE, away_n[1] * TARGET_FLEE)
        else:
            tgt_vel = (0.0, 0.0)     # 未感知则原地（保守：不游走，考验追踪能力）

        if tgt_vel != (0.0, 0.0):
            # 逃跑方向：按躲藏聪明度选择（0=纯远离 1=钻附近建筑 2=理想躲藏）
            # 撞墙时目标改向（真实目标不会自己撞墙卡死）。
            if blocked_streak > 0:
                fdx, fdy = escape_dir
            else:
                fd = pick_flee_dir(g, tgt, uav, hide, escape_dir)
                fdx, fdy = fd[0], fd[1]
                escape_dir = (fdx, fdy)
            nx, ny, hit = g.speed_toward(tgt[0], tgt[1],
                                         tgt[0] + fdx * 10, tgt[1] + fdy * 10,
                                         TARGET_FLEE)
            if hit:
                blocked_streak += 1
                if blocked_streak >= 3:
                    # 连续受阻 → 换一个「远离无人机且可通行」的方向
                    best, best_d = None, -1.0
                    for k in range(16):
                        a = 2 * math.pi * k / 16.0
                        cand = (math.cos(a), math.sin(a))
                        px, py = tgt[0] + cand[0] * 2.0, tgt[1] + cand[1] * 2.0
                        if not g.free(px, py):
                            continue
                        sep = -cand[0] * away_n[0] - cand[1] * away_n[1]
                        if sep > best_d:
                            best_d, best = sep, cand
                    if best:
                        escape_dir = best
                    blocked_streak = 0
            else:
                blocked_streak = 0
            tgt[0], tgt[1] = nx, ny

        # ---- 无人机运动（速度按策略自适应）----
        goal = pick_uav_target(g, uav, tgt, tgt_vel, strategy, uav_speed)
        spd = compute_speed(g, uav, tgt, sensed, uav_speed, speed_policy,
                            tgt_vel=tgt_vel if d < SENSE_R else (0.0, 0.0),
                            time_left=EVADE_S - tracked_total)
        ux, uy, _ = g.speed_toward(uav[0], uav[1], goal[0], goal[1], spd)
        uav[0], uav[1] = ux, uy

    return dict(achieved=False, t_confirm=None, los_breaks=los_breaks,
                max_streak=max_streak, ended_by="timeout")


# ============================ 主流程 ============================
# ============================ 主流程 ============================
def _run_matrix(g, cases, strategies, spd, policy, hide, max_t):
    """跑一轮策略×速度矩阵，返回 {策略: dict(rate, avg_t, avg_b, reasons)}。"""
    out = {}
    for st in strategies:
        ok = 0
        times, breaks = [], []
        reasons = {}
        for (a, b) in cases:
            r = run_trial(g, st, a, b, max_t, uav_speed=spd,
                          speed_policy=policy, hide=hide)
            if r["achieved"]:
                ok += 1
                times.append(r["t_confirm"])
            breaks.append(r["los_breaks"])
            reasons[r["ended_by"]] = reasons.get(r["ended_by"], 0) + 1
        n = len(cases)
        out[st] = dict(rate=ok / float(n),
                       avg_t=(sum(times) / len(times)) if times else None,
                       avg_b=sum(breaks) / float(n), reasons=reasons)
    return out


def _print_matrix(rows, strategies, title):
    print(title)
    print("%-12s | %-9s | %-10s | %-9s | %-9s" %
          ("策略", "达成率", "平均耗时", "平均断线", "主要结束原因"))
    print("-" * 74)
    for st in strategies:
        r = rows[st]
        mr = max(r["reasons"].items(), key=lambda kv: kv[1])[0] if r["reasons"] else "-"
        print("%-12s | %6.1f%%   | %8s | %8.2f | %s (%d)" %
              (st, 100.0 * r["rate"],
               ("%.1fs" % r["avg_t"]) if r["avg_t"] else "  -  ",
               r["avg_b"], mr, r["reasons"].get(mr, 0)))
    print("")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=120)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--max-t", type=float, default=MAX_T)
    ap.add_argument("--uav-speed", type=float, default=None,
                    help="单一无人机速度；不给则扫描 2.0~6.0 全区间")
    ap.add_argument("--sweep", action="store_true",
                    help="速度扫描模式：每个速度跑一遍全部策略")
    ap.add_argument("--hide", action="store_true",
                    help="目标使用主动躲藏行为（优先逃向能切断视线的方向）")
    ap.add_argument("--hide-compare", action="store_true",
                    help="对比目标「会躲」与「不会躲」两种情况")
    args = ap.parse_args()

    random.seed(args.seed)
    g = MapGrid(METADATA)
    print("=" * 78)
    print("追踪策略对比：严格 15 s 连续 LOS（LOS 断即归零）")
    print("=" * 78)
    print("地图: %s" % os.path.basename(METADATA))
    print("无人机速度上限 %.0f m/s（可调）| 目标逃跑 %.0f m/s | 感知半径 %.0f m | 限高 6 m"
          % (UAV_SPEED_MAX, TARGET_FLEE, SENSE_R))
    print("确认 %.0f s | 瞬移阈值 %.0f s | 步长 %.1f s | 单次上限 %.0f s"
          % (CONFIRM_S, EVADE_S, DT, args.max_t))

    strategies = ["chase", "lead", "intercept", "los_guard", "herd"]

    # 生成随机初始条件（要求：初始可见 + 目标周围开阔）
    random.seed(args.seed)
    cases = []
    tries = 0
    while len(cases) < args.trials and tries < args.trials * 300:
        tries += 1
        ax = random.uniform(-90, 90)
        ay = random.uniform(-42, 42)
        if not g.free(ax, ay) or g.open_space_score(ax, ay) < 0.55:
            continue
        d0 = random.uniform(15.0, 25.0)
        ang = random.uniform(0, 2 * math.pi)
        bx = ax + d0 * math.cos(ang)
        by = ay + d0 * math.sin(ang)
        if abs(bx) > 95 or abs(by) > 45 or not g.free(bx, by):
            continue
        if g.open_space_score(bx, by) < 0.55:
            continue
        if not g.visible(ax, ay, bx, by):
            continue
        cases.append(((ax, ay), (bx, by)))

    print("有效初始条件: %d 组（采样尝试 %d 次）\n" % (len(cases), tries))

    speeds = ([args.uav_speed] if args.uav_speed
              else [2.0, 2.5, 3.0, 4.0, 5.0, 6.0])

    if args.hide_compare:
        # 核心对比：三档躲藏聪明度 × 速度策略，验证「按最坏情况设计」的鲁棒性
        print("=" * 78)
        print("对比：目标躲藏聪明度 × 速度策略（同一批初始条件）")
        print("=" * 78)
        print("躲藏分档：0=不躲(纯远离) 1=会钻附近建筑(最接近真实) 2=理想躲藏(最坏情况)")
        print("")
        for hide, hname in ((HIDE_NONE, "不躲"),
                            (HIDE_LOCAL, "会钻建筑（最接近真实）"),
                            (HIDE_IDEAL, "理想躲藏（最坏情况）")):
            for policy in (SPEED_POLICY_CONST, SPEED_POLICY_ADAPTIVE):
                spd = args.uav_speed or UAV_SPEED_MAX
                rows = _run_matrix(g, cases, strategies, spd, policy, hide, args.max_t)
                pol = "自适应调速" if policy == SPEED_POLICY_ADAPTIVE else "恒速 %.0f m/s" % spd
                print("─" * 74)
                _print_matrix(rows, strategies, "【%s × %s】" % (hname, pol))
        return 0

    for spd in speeds:
        for policy in ([SPEED_POLICY_CONST, SPEED_POLICY_ADAPTIVE]
                       if args.sweep else [SPEED_POLICY_CONST]):
            rows = _run_matrix(g, cases, strategies, spd, policy, args.hide, args.max_t)
            pol = "自适应调速" if policy == SPEED_POLICY_ADAPTIVE else "恒速"
            print("─" * 78)
            extra = " | 目标会躲" if args.hide else ""
            _print_matrix(rows, strategies,
                          "无人机 %s %.1f m/s%s" % (pol, spd, extra))
    return 0


if __name__ == "__main__":
    sys.exit(main())
