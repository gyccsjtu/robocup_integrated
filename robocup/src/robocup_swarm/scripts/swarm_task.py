#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""集群协同搜索任务分配层（纯逻辑，原创实现，仅依赖标准库）。

把 200×100 m 场地划分为 10×10 m 搜索格，用「集中式拍卖」给每架无人机分配
不重复的未搜索格，并用「任务租约」防止重复搜索/掉线卡死。发现目标后进入
「追踪接力」：只派一架机做 15s 确认，其余继续搜索，追踪机失能时按「最快
到达观察点」选接力机。

设计约束（答辩/查重）：
  - 只依赖标准库（math），无 ROS / numpy，可独立单元测试。
  - 不复制 Crazyswarm2 的节点/消息/类/配置/API；其「多机状态同步、集中调度、
    可观测性」思想仅作概念参考。本文件自建数据结构与效用函数。
  - 拍卖效用 U_{i,c} = w1·未搜索收益 − w2·预计飞行时间 − w3·与他机重复率 − w4·路径风险。

坐标系约定：世界/地图坐标（米），与生成器 metadata 一致（ENU，x:-100~100，y:-50~50）。
"""

import math

# ============================ 参数 ============================
GRID_SIZE_M   = 10.0   # 搜索格边长 m（200×100 场地 → 20×10 格）
# 拍卖效用权重
W_GAIN        = 1.0    # 未搜索收益权重
W_FLIGHT      = 0.35   # 预计飞行时间权重（秒级，与收益量纲对齐）
W_OVERLAP     = 0.8    # 与他机重复率权重
W_RISK        = 0.5    # 路径风险权重
W_DISTANCE    = 0.3    # 机间距离惩罚权重（避免相邻格分配给不同机）

# 机间安全距离（与格子边长对齐）
MIN_UAV_DIST  = 12.0   # 最小机间距 m（格子边长10m + 缓冲2m）
# 任务租约
LEASE_DURATION    = 30.0   # 租约时长 s（超过未完成 → 任务重新分配）
CONFIRM_TIME      = 15.0   # 规则5：连续 15s 正确广播 ID+坐标 → 判定消除
EVADE_TIME        = 30.0   # 规则4：被感知 30s 仍未消除 → 目标瞬移躲藏
# 追踪接力
RELAY_SAFE_DIST   = 5.0    # 观察环安全半径 m（追踪机需保持在目标此距离内）
RELAY_LOSE_DIST   = 12.0   # 丢失判定距离 m（追踪机距目标超过此值 → 触发接力）
# 目标检测（规则3：几何判定，比赛主判定方式）
DETECT_RADIUS     = 20.0   # 无人机感知目标的水平半径 m（与激光量程一致）
TARGET_WALK_SPEED = 1.0    # 规则2：未感知无人机时的随机走动速度 m/s
TARGET_FLEE_SPEED = 2.0    # 规则3：感知到无人机后的逃跑速度 m/s
# 网格状态枚举
STATE_FREE      = 0    # 未分配
STATE_ASSIGNED  = 1    # 执行中（有租约）
STATE_COVERED   = 2    # 已覆盖
STATE_REVIEW    = 3    # 需复查（置信度不足）


class SearchCell(object):
    """单个搜索格：状态 + 租约 + 覆盖置信度。"""
    __slots__ = ("cx", "cy", "state", "owner", "lease_until", "confidence")

    def __init__(self, cx, cy):
        self.cx = cx          # 格中心 x（世界坐标 m）
        self.cy = cy          # 格中心 y
        self.state = STATE_FREE
        self.owner = None     # 执行机 id
        self.lease_until = 0.0
        self.confidence = 0.0 # 覆盖置信度 [0,1]


class CoverageGrid(object):
    """200×100 覆盖栅格：20×10 个 10×10 m 搜索格。"""

    def __init__(self, x_min, x_max, y_min, y_max, cell_m=GRID_SIZE_M):
        self.x_min, self.x_max = float(x_min), float(x_max)
        self.y_min, self.y_max = float(y_min), float(y_max)
        self.cell_m = float(cell_m)
        self.nx = int(math.ceil((self.x_max - self.x_min) / self.cell_m))
        self.ny = int(math.ceil((self.y_max - self.y_min) / self.cell_m))
        self.cells = {}
        for ix in range(self.nx):
            for iy in range(self.ny):
                c = SearchCell(self.x_min + (ix + 0.5) * self.cell_m,
                               self.y_min + (iy + 0.5) * self.cell_m)
                self.cells[(ix, iy)] = c

    # ---- 坐标 ↔ 格索引 ----
    def world_to_cell(self, x, y):
        ix = int(math.floor((x - self.x_min) / self.cell_m))
        iy = int(math.floor((y - self.y_min) / self.cell_m))
        if 0 <= ix < self.nx and 0 <= iy < self.ny:
            return (ix, iy)
        return None

    def cell_index(self, key):
        return key

    # ---- 查询 ----
    def cell(self, key):
        return self.cells.get(key)

    def uncovered_cells(self):
        """返回所有「未分配 / 需复查」的格 key 列表（可拍卖候选）。"""
        out = []
        for key, c in self.cells.items():
            if c.state in (STATE_FREE, STATE_REVIEW):
                out.append(key)
        return out

    def is_covered(self, x, y, radius_m):
        """(x,y) 处观测半径 radius_m 覆盖到的所有格标记为已覆盖，返回覆盖的格列表。"""
        covered = []
        for key, c in self.cells.items():
            d = math.hypot(c.cx - x, c.cy - y)
            if d <= radius_m and c.state != STATE_COVERED:
                c.state = STATE_COVERED
                c.confidence = 1.0
                c.owner = None
                c.lease_until = 0.0
                covered.append(key)
        return covered

    def mark_review(self, key, confidence):
        """置信度不足 → 标记需复查。"""
        c = self.cells.get(key)
        if c is not None:
            c.state = STATE_REVIEW
            c.confidence = confidence
            c.owner = None
            c.lease_until = 0.0


class TaskAllocator(object):
    """集中式拍卖任务分配器（每机独立效用，管理器按最高效用分配）。

    U_{i,c} = w1·收益 − w2·飞行时间 − w3·重复率 − w4·风险
    """

    def __init__(self, grid, w_gain=W_GAIN, w_flight=W_FLIGHT,
                 w_overlap=W_OVERLAP, w_risk=W_RISK, w_distance=W_DISTANCE, cruise_speed=3.0):
        self.grid = grid
        self.w_gain = w_gain
        self.w_flight = w_flight
        self.w_overlap = w_overlap
        self.w_risk = w_risk
        self.w_distance = w_distance
        self.cruise_speed = cruise_speed

    def _flight_time(self, uav_x, uav_y, cell):
        """预计飞行时间 = 直线距离 / 巡航速度（秒）。"""
        return math.hypot(cell.cx - uav_x, cell.cy - uav_y) / self.cruise_speed

    def _overlap_rate(self, cell, assigned_positions):
        """与他机重复率：本格到「其他机已分配格」的平均接近度 [0,1]。

        用「离最近已分配格中心的距离」做软度量：越近重复率越高。
        """
        if not assigned_positions:
            return 0.0
        nearest = min(math.hypot(cell.cx - px, cell.cy - py)
                      for px, py in assigned_positions)
        # 10m 格距内视为重复：距离 0 → 重复率 1，距离 ≥ 2×cell_m → 0
        return max(0.0, 1.0 - nearest / (2.0 * self.grid.cell_m))

    def _path_risk(self, cell, risk_map):
        """路径风险：risk_map 给出每格风险值 [0,1]（建筑/密集区风险高）。"""
        if not risk_map:
            return 0.0
        return risk_map.get((cell.cx, cell.cy), 0.0)

    def utility(self, uav_id, uav_x, uav_y, cell_key, assigned_positions, risk_map=None, other_uavs=None):
        """单机对单格效用 U_{i,c}。收益：未搜索=1，需复查=0.5。

        other_uavs: {uav_id: (x,y)} 其他飞机的当前位置，用于计算机间距离惩罚。
        """
        cell = self.grid.cell(cell_key)
        if cell is None:
            return float("-inf")
        gain = 1.0 if cell.state == STATE_FREE else (0.5 if cell.state == STATE_REVIEW else 0.0)
        if gain == 0.0:
            return float("-inf")   # 已覆盖/执行中不可再分配
        ft = self._flight_time(uav_x, uav_y, cell)
        ov = self._overlap_rate(cell, assigned_positions)
        rk = self._path_risk(cell, risk_map)

        # 机间距离惩罚：距离其他飞机太近扣分
        dist_penalty = 0.0
        if other_uavs:
            for oid, (ox, oy) in other_uavs.items():
                if oid == uav_id:
                    continue
                d = math.hypot(cell.cx - ox, cell.cy - oy)
                if d < MIN_UAV_DIST:
                    # 距离越近惩罚越大：线性衰减到 MIN_UAV_DIST 时为0
                    dist_penalty += (MIN_UAV_DIST - d) / MIN_UAV_DIST

        return (self.w_gain * gain
                - self.w_flight * ft
                - self.w_overlap * ov
                - self.w_risk * rk
                - self.w_distance * dist_penalty)

    def allocate(self, uavs, risk_map=None):
        """集中式分配一轮。uavs: {id: (x,y)}，返回 {id: cell_key 或 None}。

        贪心：按当前「收益最高」逐机分配（每机取剩余格中自身效用最大者），
        已分配格即时从候选池移除（降低重复率），实现轻量拍卖。
        """
        remaining = set(self.grid.uncovered_cells())
        assigned = {}          # uav_id -> cell_key
        assigned_positions = []  # [(cx,cy)] 已分配格中心，供重复率计算

        # 预先获取所有飞机的位置，用于计算机间距离惩罚
        all_uavs = dict(uavs)

        for uav_id, (ux, uy) in uavs.items():
            best_key = None
            best_u = float("-inf")

            # 本机分配时，排除已分配格的位置，但不排除其他飞机的当前位置
            other_uavs = {oid: pos for oid, pos in all_uavs.items() if oid != uav_id}

            for key in remaining:
                u = self.utility(uav_id, ux, uy, key, assigned_positions, risk_map, other_uavs)
                if u > best_u:
                    best_u = u
                    best_key = key
            if best_key is not None:
                assigned[uav_id] = best_key
                c = self.grid.cell(best_key)
                c.state = STATE_ASSIGNED
                c.owner = uav_id
                remaining.discard(best_key)
                assigned_positions.append((c.cx, c.cy))
            else:
                assigned[uav_id] = None
        return assigned


class LeaseManager(object):
    """任务租约：掉线/卡死/超时 → 租约到期自动重分配，防重复搜索。"""

    def __init__(self, grid, duration=LEASE_DURATION):
        self.grid = grid
        self.duration = duration

    def grant(self, uav_id, cell_key, now):
        c = self.grid.cell(cell_key)
        if c is None:
            return False
        c.state = STATE_ASSIGNED
        c.owner = uav_id
        c.lease_until = now + self.duration
        return True

    def renew(self, uav_id, cell_key, now):
        """执行机持续上报 → 续租。"""
        c = self.grid.cell(cell_key)
        if c is not None and c.owner == uav_id and c.state == STATE_ASSIGNED:
            c.lease_until = now + self.duration
            return True
        return False

    def expire(self, now):
        """收集所有租约到期的格，回退为未分配，返回待重分配的格 key 列表。"""
        expired = []
        for key, c in self.grid.cells.items():
            if c.state == STATE_ASSIGNED and c.lease_until < now:
                c.state = STATE_FREE
                c.owner = None
                c.lease_until = 0.0
                expired.append(key)
        return expired


class LineOfSight(object):
    """视线（LOS）遮挡判定：无人机与目标之间是否被建筑挡住。

    规则3 的「几何判定」核心：仅当水平距离 < DETECT_RADIUS **且** 视线无遮挡
    才算感知到目标（否则隔着楼也算看到，不符合实际）。

    实现用自写的栅格 DDA 射线步进（Amanatides-Woo 思路，变量名与注释自建）：
    从观测者沿连线逐格步进，途中若遇到障碍栅格 → 判定被遮挡。
    只依赖「障碍栅格查询函数」+ 栅格原点/分辨率，不绑定具体地图实现，便于单测。

    注意 origin：地图栅格原点通常不是 (0,0)（本项目为 (-100,-50)），
    坐标→栅格必须减去 origin，否则会整体错位（曾导致 LOS 恒为 False）。
    """

    def __init__(self, is_blocked, cell_size=0.5, origin=(0.0, 0.0)):
        """
        is_blocked : callable(ix, iy) -> bool，判断栅格是否被占用
        cell_size  : 栅格边长 m（与调用方地图一致）
        origin     : 栅格原点 (x0, y0)（与调用方地图一致）
        """
        self.is_blocked = is_blocked
        self.cell_size = float(cell_size)
        self.origin = (float(origin[0]), float(origin[1]))

    def _to_cell(self, x, y):
        """世界坐标 -> 栅格索引（必须减 origin）。"""
        return (int(math.floor((x - self.origin[0]) / self.cell_size)),
                int(math.floor((y - self.origin[1]) / self.cell_size)))

    def visible(self, ox, oy, tx, ty):
        """(ox,oy) 能否看见 (tx,ty)（无遮挡返回 True）。

        端点所在格不算遮挡（目标/观测者可能贴着障碍边缘）。
        """
        ix, iy = self._to_cell(ox, oy)
        gx, gy = self._to_cell(tx, ty)
        if (ix, iy) == (gx, gy):
            return True   # 同一格必然可见

        dx, dy = tx - ox, ty - oy
        step_x = 1 if dx > 0 else (-1 if dx < 0 else 0)
        step_y = 1 if dy > 0 else (-1 if dy < 0 else 0)

        # 到下一条格边界的参数距离（用「格边界的绝对坐标」减去起点，故含 origin）
        inf = float("inf")
        if step_x != 0:
            next_x = self.origin[0] + (ix + (1 if step_x > 0 else 0)) * self.cell_size
            t_max_x = (next_x - ox) / dx
            t_delta_x = self.cell_size / abs(dx)
        else:
            t_max_x, t_delta_x = inf, inf
        if step_y != 0:
            next_y = self.origin[1] + (iy + (1 if step_y > 0 else 0)) * self.cell_size
            t_max_y = (next_y - oy) / dy
            t_delta_y = self.cell_size / abs(dy)
        else:
            t_max_y, t_delta_y = inf, inf

        # 逐格步进，直到抵达目标格（含端点豁免逻辑）
        guard = 0
        max_steps = int(math.hypot(gx - ix, gy - iy)) + 4
        while (ix, iy) != (gx, gy) and guard < max_steps:
            guard += 1
            if t_max_x < t_max_y:
                ix += step_x
                t_max_x += t_delta_x
            else:
                iy += step_y
                t_max_y += t_delta_y
            if (ix, iy) == (gx, gy):
                break          # 到达目标格，不算遮挡
            if self.is_blocked(ix, iy):
                return False   # 中途撞障碍 → 被遮挡
        return True


class TargetTracker(object):
    """目标观测 + 确认消除 + 追踪接力（对应比赛规则 3/4/5）。

    规则映射：
      规则 3  感知到无人机 → 目标逃跑 2m/s 并向同伙广播（仿真侧实现，本类只记录）
      规则 4  被持续感知 30s 仍未消除 → 目标瞬移躲藏（本类超时判定 → 清空确认计时）
      规则 5  连续 15s 正确广播 ID + 坐标 → 消除（本类累计「连续确认时长」）

    关键：确认计时是**连续**的。中途丢失视野（超过 lose_dist 或无人观测）
    必须清零重来，否则规则 5 的「连续 15 秒」就形同虚设。
    """

    def __init__(self, confirm_time=CONFIRM_TIME, safe_dist=RELAY_SAFE_DIST,
                 lose_dist=RELAY_LOSE_DIST, evade_time=EVADE_TIME):
        self.confirm_time = confirm_time     # 规则5：连续确认 15s 判定消除
        self.safe_dist = safe_dist
        self.lose_dist = lose_dist
        self.evade_time = evade_time         # 规则4：被感知 30s 未消除 → 瞬移
        self.targets = {}   # target_id -> dict(...)

    # ---------------- 目标生命周期 ----------------
    def add_target(self, target_id, x, y, vx=0.0, vy=0.0, conf=1.0, tracker=None, now=0.0):
        """登记一个新目标。`now` 视为**首次被感知时刻**（管理器是因检测才得知它），
        故连续确认计时从此刻起算 —— 否则规则5 的 15s 会被推迟一个采样周期。"""
        self.targets[target_id] = {
            "x": x, "y": y, "vx": vx, "vy": vy,
            "conf": conf, "tracker": tracker,
            "confirm_until": now + self.confirm_time,   # 兼容旧接口：名义确认截止
            "confirm_since": now,    # 连续确认起始时刻（已被感知）
            "tracked_since": now,    # 首次被感知时刻（规则4 的 30s 计时起点）
            "evaded": False,         # 是否已触发瞬移（需重新搜索）
            "eliminated": False,     # 是否已判定消除
        }

    def remove_target(self, target_id):
        return self.targets.pop(target_id, None) is not None

    # ---------------- 规则 5：连续 15s 确认 → 消除 ----------------
    def observe(self, target_id, now, observer=None):
        """记录一次「本时刻有无人机正确观测到该目标」。

        返回 True 表示本次观测使该目标达成连续确认（可判定消除）。
        """
        t = self.targets.get(target_id)
        if t is None or t["eliminated"]:
            return False
        if t["confirm_since"] is None:
            t["confirm_since"] = now      # 开始/重新开始连续计时
        if observer is not None:
            t["tracker"] = observer
        held = now - t["confirm_since"]
        t["conf"] = min(1.0, held / self.confirm_time)   # 确认进度作为置信度
        if held >= self.confirm_time:
            t["eliminated"] = True
            t["conf"] = 1.0
            return True
        return False

    def lose(self, target_id, now):
        """本时刻没有观测到该目标 → 连续确认计时清零（规则5 要求「连续」）。"""
        t = self.targets.get(target_id)
        if t is None or t["eliminated"]:
            return
        t["confirm_since"] = None
        t["conf"] = 0.0

    def confirm_progress(self, target_id, now):
        """当前连续确认进度 [0,1]，用于可视化/日志。"""
        t = self.targets.get(target_id)
        if t is None or t["confirm_since"] is None:
            return 0.0
        return min(1.0, (now - t["confirm_since"]) / self.confirm_time)

    # ---------------- 规则 4：被感知 30s 未消除 → 瞬移 ----------------
    def check_evade(self, target_id, now):
        """规则4：目标被感知累计超过 30s 仍未消除 → 应瞬移。

        注意：这里的 30s 是「被感知」的累计时长（tracked_since 起算），
        与规则 5 的「连续确认」是两个独立计时。
        返回 True 表示本次调用触发了瞬移（调用方负责搬移目标并重置状态）。
        """
        t = self.targets.get(target_id)
        if t is None or t["eliminated"] or t["evaded"]:
            return False
        if now - t["tracked_since"] >= self.evade_time:
            t["evaded"] = True
            t["confirm_since"] = None    # 瞬移后确认计时必须清零
            t["conf"] = 0.0
            return True
        return False

    def relocate(self, target_id, x, y, now):
        """瞬移目标到新位置，并重置所有计时（重新开始搜索/确认）。"""
        t = self.targets.get(target_id)
        if t is None:
            return False
        t["x"], t["y"] = x, y
        t["vx"] = t["vy"] = 0.0
        t["tracker"] = None
        t["confirm_since"] = None
        t["tracked_since"] = now     # 重新开始 30s 计时
        t["evaded"] = False
        t["conf"] = 0.0
        return True

    def pending_targets(self):
        """返回所有「未被消除」的目标 id（仍在场上的）。"""
        return [tid for tid, t in self.targets.items() if not t["eliminated"]]

    # ---------------- 追踪接力（保留原有设计） ----------------
    def predict(self, target_id, dt):
        """卡尔曼预测位置的简化版：匀速外推（自写，非完整卡尔曼）。"""
        t = self.targets.get(target_id)
        if t is None:
            return None
        return (t["x"] + t["vx"] * dt, t["y"] + t["vy"] * dt, t["vx"], t["vy"])

    def needs_relay(self, target_id, now):
        """追踪机是否需要接力：确认超时 或 追踪机为 None。"""
        t = self.targets.get(target_id)
        if t is None:
            return False
        if t["tracker"] is None:
            return True
        return now > t["confirm_until"]

    def pick_relay(self, target_id, uavs, cruise_speed=3.0):
        """选「预计最快到达观察点」的无人机接力（返回 uav_id 或 None）。

        uavs: {id: (x,y)}，观察点取目标预测位置处。
        """
        t = self.targets.get(target_id)
        if t is None:
            return None
        tx, ty, _, _ = self.predict(target_id, 0.0)
        best_id, best_t = None, float("inf")
        for uav_id, (ux, uy) in uavs.items():
            if uav_id == t["tracker"]:
                continue
            ft = math.hypot(tx - ux, ty - uy) / cruise_speed
            if ft < best_t:
                best_t = ft
                best_id = uav_id
        return best_id

    def handover(self, target_id, new_tracker, now):
        """接力：新机继承同一目标 ID 与确认计时（重新计时 15s）。"""
        t = self.targets.get(target_id)
        if t is None:
            return False
        t["tracker"] = new_tracker
        t["confirm_until"] = now + self.confirm_time
        return True


# ============================ 单元自测 ============================
if __name__ == "__main__":
    # 建 200×100 栅格（20×10 格）
    g = CoverageGrid(-100, 100, -50, 50)
    assert g.nx == 20 and g.ny == 10, (g.nx, g.ny)
    print("栅格尺寸 OK: %d×%d 格，共 %d 格" % (g.nx, g.ny, len(g.cells)))

    # ---- 1) 拍卖分配：两机不重复搜索 ----
    alloc = TaskAllocator(g)
    uavs = {"uav_1": (-50.0, 0.0), "uav_2": (50.0, 0.0)}
    assign = alloc.allocate(uavs)
    keys = list(assign.values())
    assert len(set(keys)) == 2 and None not in keys, assign
    # 各机应被分到离自己近的格
    c1, c2 = g.cell(assign["uav_1"]), g.cell(assign["uav_2"])
    assert c1.cx < c2.cx, (c1.cx, c2.cx)
    print("拍卖分配 OK: uav_1→(%.0f,%.0f) uav_2→(%.0f,%.0f)，不重复" %
          (c1.cx, c1.cy, c2.cx, c2.cy))

    # ---- 2) 任务租约：到期重分配 ----
    lm = LeaseManager(g)
    # 拍卖只置 state/owner，租约需管理器随后授予（两机都授）
    lm.grant("uav_1", assign["uav_1"], now=0.0)
    lm.grant("uav_2", assign["uav_2"], now=0.0)
    assert g.cell(assign["uav_1"]).owner == "uav_1"
    # 续租 → 未到期
    lm.renew("uav_1", assign["uav_1"], now=20.0)
    assert lm.expire(25.0) == [], "续租后不应到期"
    # 不续租 → 30s 到期（uav_1 续到 50s 后断联，55s 时到期）
    expired = lm.expire(55.0)
    assert assign["uav_1"] in expired, expired
    print("任务租约 OK: 续租后不到期，断联 %ds 后到期重分配" % LEASE_DURATION)

    # ---- 3) 覆盖标记：观测半径覆盖 ----
    # 观测点 (0,0) 半径 8m：格中心 (±5,±5) 距原点 7.07m，应覆盖 4 个相邻格
    covered = g.is_covered(0.0, 0.0, radius_m=8.0)
    assert len(covered) == 4, covered
    assert all(g.cell(k).state == STATE_COVERED for k in covered)
    print("覆盖标记 OK: 观测半径 8m 覆盖 %d 格" % len(covered))

    # ---- 4) 追踪接力：选最快到达机 + 继承确认计时 ----
    tt = TargetTracker()
    tt.add_target("t0", x=0.0, y=0.0, tracker="uav_1", now=100.0)
    # uav_1 距目标远（丢失），uav_2 更近
    relay = tt.pick_relay("t0", {"uav_2": (2.0, 0.0), "uav_3": (30.0, 0.0)})
    assert relay == "uav_2", relay
    assert tt.handover("t0", "uav_2", now=105.0)
    assert tt.targets["t0"]["tracker"] == "uav_2"
    assert tt.targets["t0"]["confirm_until"] == 105.0 + CONFIRM_TIME
    print("追踪接力 OK: 最快到达机 uav_2 继承目标 t0，确认计时重置 %.0fs" % CONFIRM_TIME)

    # ---- 5) 规则5：连续 15s 确认 → 消除（中断必须清零） ----
    tt2 = TargetTracker()
    tt2.add_target("t1", x=10.0, y=10.0, now=0.0)
    # 连续观测 15s（每 1s 一次）→ 最后一刻达成消除
    met = False
    for k in range(1, int(CONFIRM_TIME) + 1):
        met = tt2.observe("t1", now=float(k), observer="uav_1")
    assert met and tt2.targets["t1"]["eliminated"], "连续 15s 应判定消除"
    print("规则5 OK: 连续观测 %ds → 目标消除" % CONFIRM_TIME)

    # 中断清零：观测到 10s 后丢失，再从头计时
    tt3 = TargetTracker()
    tt3.add_target("t2", x=0.0, y=0.0, now=0.0)
    for k in range(1, 11):
        tt3.observe("t2", now=float(k), observer="uav_1")
    tt3.lose("t2", now=11.0)
    assert tt3.targets["t2"]["confirm_since"] is None, "丢失后计时必须清零"
    for k in range(12, 22):   # 只再观测 10s，不应消除
        met = tt3.observe("t2", now=float(k), observer="uav_1")
    assert not tt3.targets["t2"]["eliminated"], "中断后重新计时，10s 不应消除"
    print("规则5 OK: 中途丢失 → 计时清零，不误判消除")

    # ---- 6) 规则4：被感知 30s 未消除 → 瞬移躲藏 ----
    tt4 = TargetTracker()
    tt4.add_target("t3", x=0.0, y=0.0, now=0.0)
    assert not tt4.check_evade("t3", now=20.0), "20s 未到瞬移阈值"
    assert tt4.check_evade("t3", now=EVADE_TIME), "30s 应触发瞬移"
    assert tt4.targets["t3"]["evaded"]
    # 瞬移后必须重置所有计时
    assert tt4.relocate("t3", x=50.0, y=-30.0, now=EVADE_TIME)
    t3 = tt4.targets["t3"]
    assert (t3["x"], t3["y"]) == (50.0, -30.0) and t3["tracked_since"] == EVADE_TIME
    assert not t3["evaded"] and t3["confirm_since"] is None
    print("规则4 OK: 被感知 %.0fs 未消除 → 瞬移到新位置并重置计时" % EVADE_TIME)

    # ---- 7) 视线遮挡：中间有建筑 → 不可见 ----
    # 造一个 0.5m 栅格的小地图（原点 0,0），在 x=5 处放一堵竖墙
    def _blocked(ix, iy):
        return ix == 10   # x = 5.0m 处的整列障碍
    los = LineOfSight(_blocked, cell_size=0.5, origin=(0.0, 0.0))
    assert not los.visible(0.0, 0.0, 10.0, 0.0), "穿墙应判为不可见"
    assert los.visible(0.0, 0.0, 4.0, 0.0), "墙前应可见"
    assert los.visible(6.0, 0.0, 10.0, 0.0), "墙后同侧应可见"
    print("视线遮挡 OK: 穿墙不可见，无遮挡可见")

    # ---- 8) 视线遮挡：原点非 (0,0) 时坐标转换必须正确 ----
    # 复现真实地图情形（origin=(-100,-50)）：x=-100 处的墙
    def _blocked_off(ix, iy):
        return ix == 20   # origin -100 + 20*0.5 = -90.0m 处的墙
    los2 = LineOfSight(_blocked_off, cell_size=0.5, origin=(-100.0, -50.0))
    assert not los2.visible(-95.0, 0.0, -85.0, 0.0), "穿墙应不可见（含 origin 偏移）"
    assert los2.visible(-95.0, 0.0, -91.0, 0.0), "墙前应可见（含 origin 偏移）"
    print("LOS 原点偏移 OK: 非 (0,0) 原点下坐标转换正确")

    print("\nswarm_task 自测全部通过")
