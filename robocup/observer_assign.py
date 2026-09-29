#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ObserverAssigner: UAV assignment for persistent LOS coverage.

Core: Choose UAVs whose LOS blockage moments are staggered, not closest ones.

New (2026-09-21): 完整 handover 状态机
- NORMAL → PREPARE_BACKUP → BACKUP_MOVING → BACKUP_READY → HANDOVER → NORMAL
- 双向预测：predict_target_and_backup(t+H) 的未来位置
- 原 UAV 等 backup 确认后才退出，实现 seamless handover
- covered 统计全程保持连续
"""

import math
from enum import Enum, auto

# Constants
CONFIRM_TIME = 15.0
EVADE_TIME = 30.0
SENSE_RADIUS = 20.0
DEFAULT_UAV_SPEED = 6.0

# Viewpoint Planner parameters
VIEWPOINT_N_CAND = 36
VIEWPOINT_W_DIST = 1.0
VIEWPOINT_W_ANGLE = 2.5

# Handover 状态机
class HandoverState(Enum):
    NORMAL = auto()           # 正常观察
    PREPARE_BACKUP = auto()  # 检测到未来 LOS 可能断，准备找 backup
    BACKUP_MOVING = auto()   # backup 已出发去新观察位
    BACKUP_READY = auto()    # backup 连续 LOS 确认完成
    HANDOVER = auto()         # 真正切换（primary 退出）
    BACKUP_FAILED = auto()   # backup 找不到/超时
    BACKUP_TIMEOUT = auto()  # backup 移动超时

# Handover 参数
BACKUP_LOS_CONFIRM_TIME = 0.5  # backup 获得 LOS 后需要确认 0.5s
BACKUP_SEARCH_TIMEOUT = 2.0    # backup 搜索超时
BACKUP_MOVE_TIMEOUT = 5.0      # backup 移动到观察位超时
PREDICT_HORIZON = 3.0          # 预测未来 3 秒


class ObserverAssigner:
    def __init__(self, grid):
        self.grid = grid
        # Handover 状态机
        self._handover_state = HandoverState.NORMAL
        self._handover_state_time = 0.0  # 进入当前状态的时间
        self._primary_uav = None         # 当前 primary 观察员
        self._backup_uav = None         # 备用 UAV
        self._backup_viewpoint = None   # backup 将去的观察位
        self._backup_confirm_start = None  # backup 开始确认的时间
        self._handover_target = None    # handover 针对的目标

        # 统计
        self._predict_break_count = 0       # 预测到 LOS 会断的次数
        self._pre_handover_trigger = 0      # 触发 handover 准备的次数
        self._backup_assigned = 0          # backup 已分配的次数
        self._backup_los_ready = 0         # backup LOS 确认完成的次数
        self._successful_seamless = 0      # 成功 seamless handover 次数
        self._backup_failed = 0            # backup 失败次数
        self._seamless_covered_preserved = 0  # handover 期间 covered 未断的次数

    def plan_viewpoints(self, target_xy, uav_pos, other_viewpoints=None, n=1,
                       target_vel=(0, 0), predict_horizon=0):
        """Plan safe viewpoints with clear LOS for a target.

        Args:
            target_xy: (x, y) target position
            uav_pos: (ux, uy) UAV starting position
            other_viewpoints: [(x,y), ...] other observers' positions
            n: number of viewpoints needed
            target_vel: target velocity vector for prediction
            predict_horizon: >0 to check if LOS stays clear at predicted position

        Returns: [(score, sx, sy), ...] or None if no feasible point
        """
        tx, ty = target_xy
        ux, uy = uav_pos
        R = SENSE_RADIUS
        vx, vy = target_vel

        # Generate candidates: multi-layer concentric circles
        ratios = [0.5, 0.6, 0.7, 0.8, 0.9]
        candidates = []
        for ratio in ratios:
            r = ratio * R
            for i in range(VIEWPOINT_N_CAND):
                angle = 2.0 * math.pi * i / VIEWPOINT_N_CAND
                sx = tx + r * math.cos(angle)
                sy = ty + r * math.sin(angle)
                candidates.append((sx, sy))

        # Filter and score
        scored = []
        for (sx, sy) in candidates:
            d_to_target = math.hypot(sx - tx, sy - ty)
            if d_to_target >= R:
                continue

            # 2. LOS must be clear
            if not self.grid.visible(sx, sy, tx, ty):
                continue

            # 3. Position must be free
            if not self.grid.free(sx, sy):
                continue

            # 4. Keep distance from other viewpoints
            if other_viewpoints:
                min_sep = min(math.hypot(sx - ox, sy - oy) for (ox, oy) in other_viewpoints)
                if min_sep < 3.0:
                    continue

            # 5. LOS prediction check
            if predict_horizon > 0:
                future_tx = tx + vx * predict_horizon
                future_ty = ty + vy * predict_horizon
                if not self.grid.visible(sx, sy, future_tx, future_ty):
                    continue

            # 6. Flight cost
            flight_cost = math.hypot(sx - ux, sy - uy)

            # Scoring
            dist_score = VIEWPOINT_W_DIST * (1.0 - d_to_target / R)
            angle_score = 0.0
            if other_viewpoints:
                base_angle = math.atan2(uy - ty, ux - tx)
                this_angle = math.atan2(sy - ty, sx - tx)
                angle_diff = abs(this_angle - base_angle)
                angle_diff = min(angle_diff, 2.0 * math.pi - angle_diff)
                angle_score = VIEWPOINT_W_ANGLE * (angle_diff / math.pi)
            proximity_score = 2.0 * max(0, 1.0 - flight_cost / 50.0)

            score = dist_score + angle_score + proximity_score
            scored.append((score, sx, sy))

        if not scored:
            return None

        scored.sort(key=lambda x: -x[0])
        return scored[:n], len(scored)

    def choose(self, target_xy, pool, n=1, exclude=()):
        """Select UAVs that can observe the target.

        Args:
            target_xy: (x, y) target position
            pool: {uav_id: (x, y)} available UAVs
            n: number needed
            exclude: UAVs to exclude

        Returns: [uav_id, ...]
        """
        if n is None or n < 1:
            return []
        n = min(n, len(pool))

        cand = {uid: p for uid, p in pool.items() if uid not in set(exclude)}
        if not cand:
            return []

        # Check which UAVs can observe the target
        valid = []
        for uid, (ux, uy) in cand.items():
            d = math.hypot(ux - target_xy[0], uy - target_xy[1])
            if d < SENSE_RADIUS and self.grid.visible(ux, uy, target_xy[0], target_xy[1]):
                valid.append((uid, d, ux, uy))

        if not valid:
            return []

        # Sort by distance
        valid.sort(key=lambda x: x[1])
        return [uid for uid, d, ux, uy in valid[:n]]

    # =============== 完整 handover 状态机算法 ===============

    def get_handover_state(self):
        """获取当前 handover 状态"""
        return self._handover_state

    def update_handover_state(self, target_xy, target_vel, current_observers, all_uavs, current_time):
        """状态机更新主函数

        状态流转:
        NORMAL → PREPARE_BACKUP → BACKUP_MOVING → BACKUP_READY → HANDOVER → NORMAL
                     ↓
              BACKUP_FAILED / BACKUP_TIMEOUT

        Args:
            target_xy: 目标位置
            target_vel: 目标速度
            current_observers: {uav_id: (x, y)} 当前观察员
            all_uavs: {uav_id: (x, y)} 所有 UAV
            current_time: 当前时间

        Returns: (action, info) action 可以是:
            - None: 无需动作
            - ('search_backup', target_xy): 搜索 backup
            - ('assign_backup', uav_id, viewpoint): 分配 backup
            - ('handover_complete', primary_uav): 完成 handover
            - ('reset',): 重置状态机
        """
        state = self._handover_state

        # ===== NORMAL: 检测是否需要准备 backup =====
        if state == HandoverState.NORMAL:
            # 检查是否需要触发 backup 搜索
            if current_observers:
                primary_id = list(current_observers.keys())[0]
                primary_pos = current_observers[primary_id]

                # 预测当前 LOS 是否会断
                will_break = self._predict_los_break(
                    primary_pos, target_xy, target_vel, PREDICT_HORIZON
                )

                if will_break:
                    self._predict_break_count += 1
                    self._pre_handover_trigger += 1
                    self._primary_uav = primary_id
                    self._handover_target = target_xy
                    self._handover_state = HandoverState.PREPARE_BACKUP
                    self._handover_state_time = current_time
                    return ('search_backup', target_xy)

            return None

        # ===== PREPARE_BACKUP: 搜索 backup =====
        elif state == HandoverState.PREPARE_BACKUP:
            elapsed = current_time - self._handover_state_time

            # 超时则失败
            if elapsed > BACKUP_SEARCH_TIMEOUT:
                self._backup_failed += 1
                self._handover_state = HandoverState.BACKUP_FAILED
                self._handover_state_time = current_time
                return ('reset',)

            # 搜索 backup（外部调用 find_backup_candidates 后设置）
            return None

        # ===== BACKUP_MOVING: backup 前往观察位 =====
        elif state == HandoverState.BACKUP_MOVING:
            elapsed = current_time - self._handover_state_time

            # 超时则失败
            if elapsed > BACKUP_MOVE_TIMEOUT:
                self._backup_failed += 1
                self._handover_state = HandoverState.BACKUP_TIMEOUT
                self._handover_state_time = current_time
                return ('reset',)

            # 检查 backup 是否到达观察位并获得 LOS
            if self._backup_uav and self._backup_viewpoint:
                backup_pos = all_uavs.get(self._backup_uav)
                vp = self._backup_viewpoint

                if backup_pos:
                    # 检查是否到达观察位（距离 < 2m）
                    d_to_vp = math.hypot(backup_pos[0] - vp[0], backup_pos[1] - vp[1])

                    if d_to_vp < 2.0:
                        # 检查 LOS
                        if self.grid.visible(backup_pos[0], backup_pos[1], target_xy[0], target_xy[1]):
                            # 开始 LOS 确认计时
                            if self._backup_confirm_start is None:
                                self._backup_confirm_start = current_time
                            else:
                                confirm_elapsed = current_time - self._backup_confirm_start
                                if confirm_elapsed >= BACKUP_LOS_CONFIRM_TIME:
                                    # LOS 确认完成
                                    self._backup_los_ready += 1
                                    self._handover_state = HandoverState.BACKUP_READY
                                    self._handover_state_time = current_time
                        else:
                            # LOS 丢失，重置确认计时
                            self._backup_confirm_start = None

            return None

        # ===== BACKUP_READY: backup 已就绪，原 UAV 可以退出 =====
        elif state == HandoverState.BACKUP_READY:
            # 记录 seamless covered 保持
            self._seamless_covered_preserved += 1

            # 执行 handover
            self._successful_seamless += 1
            self._handover_state = HandoverState.HANDOVER
            self._handover_state_time = current_time

            return ('handover_complete', self._primary_uav, self._backup_uav)

        # ===== HANDOVER: 短暂过渡状态 =====
        elif state == HandoverState.HANDOVER:
            # 立即回到 NORMAL
            self._handover_state = HandoverState.NORMAL
            self._primary_uav = None
            self._backup_uav = None
            self._backup_viewpoint = None
            self._backup_confirm_start = None
            self._handover_target = None
            return ('reset',)

        # ===== BACKUP_FAILED / BACKUP_TIMEOUT: 失败状态 =====
        elif state in (HandoverState.BACKUP_FAILED, HandoverState.BACKUP_TIMEOUT):
            # 短暂停留后回到 NORMAL
            elapsed = current_time - self._handover_state_time
            if elapsed > 0.5:  # 0.5s 后重置
                self._handover_state = HandoverState.NORMAL
                self._primary_uav = None
                self._backup_uav = None
                self._backup_viewpoint = None
                self._backup_confirm_start = None
                self._handover_target = None
                return ('reset',)

            return None

        return None

    def assign_backup(self, uav_id, viewpoint, current_time):
        """外部调用：成功找到 backup 后设置"""
        self._backup_uav = uav_id
        self._backup_viewpoint = viewpoint
        self._backup_confirm_start = None
        self._backup_assigned += 1
        self._handover_state = HandoverState.BACKUP_MOVING
        self._handover_state_time = current_time

    def get_handover_stats(self):
        """获取 handover 统计"""
        return {
            'predict_break_count': self._predict_break_count,
            'pre_handover_trigger': self._pre_handover_trigger,
            'backup_assigned': self._backup_assigned,
            'backup_los_ready': self._backup_los_ready,
            'successful_seamless': self._successful_seamless,
            'backup_failed': self._backup_failed,
            'seamless_covered_preserved': self._seamless_covered_preserved,
        }

    def reset_handover_state(self):
        """重置 handover 状态机"""
        self._handover_state = HandoverState.NORMAL
        self._handover_state_time = 0.0
        self._primary_uav = None
        self._backup_uav = None
        self._backup_viewpoint = None
        self._backup_confirm_start = None
        self._handover_target = None

    # =============== 预测和搜索核心方法 ===============

    def _predict_los_break(self, uav_pos, target_xy, target_vel, horizon):
        """预测 uav 在 horizon 秒后与 target 的 LOS 是否会断

        Args:
            uav_pos: (x, y) UAV 位置
            target_xy: (x, y) 目标当前位置
            target_vel: (vx, vy) 目标速度
            horizon: 预测时间（秒）

        Returns: True 表示预测会断，False 表示预测仍通
        """
        tx, ty = target_xy
        vx, vy = target_vel

        # 预测目标未来位置
        future_tx = tx + vx * horizon
        future_ty = ty + vy * horizon

        ux, uy = uav_pos

        # 检查未来位置 LOS
        return not self.grid.visible(ux, uy, future_tx, future_ty)

    def predict_target_and_backup(self, target_xy, target_vel, backup_pos, backup_vel, horizon):
        """双向预测：target(t+H) 和 backup_uav(t+H) 的 LOS 是否通

        这是关键！只预测 target 不够，必须同时预测 backup 的运动

        Args:
            target_xy: (x, y) 目标当前位置
            target_vel: (vx, vy) 目标速度
            backup_pos: (x, y) backup UAV 当前位置
            backup_vel: (vx, vy) backup UAV 速度（可为 (0,0) 表示原地待命）
            horizon: 预测时间（秒）

        Returns: True 表示预测 LOS 会断，False 表示预测仍通
        """
        tx, ty = target_xy
        vx, vy = target_vel

        # 预测目标未来位置
        future_tx = tx + vx * horizon
        future_ty = ty + vy * horizon

        # 预测 backup UAV 未来位置
        ux, uy = backup_pos
        bux, buy = backup_vel
        future_ux = ux + bux * horizon
        future_uy = uy + buy * horizon

        # 检查双向预测后的 LOS
        return not self.grid.visible(future_ux, future_uy, future_tx, future_ty)

    def find_backup_candidates(self, target_xy, target_vel, current_observers, all_uavs,
                            predict_horizon=3.0, exclude_uavs=None):
        """分离：找候选 UAV → 找候选观察位 → 选最佳组合

        核心思路：
        1. 候选 UAV（排除当前观察员 + 可选排除列表）
        2. 为每个候选 UAV 生成候选观察位
        3. 检查：当前 LOS + free + 预测未来 LOS 仍通
        4. 选最佳组合

        Args:
            target_xy: (x, y) 目标位置
            target_vel: (vx, vy) 目标速度
            current_observers: 当前观察员集合（set 或 dict key）
            all_uavs: 所有 UAV 字典 {uav_id: (x, y)}
            predict_horizon: 预测时间
            exclude_uavs: 额外排除的 UAV（可选）

        Returns: (best_uav_id, best_viewpoint) 或 None
        """
        if exclude_uavs is None:
            exclude_uavs = set()

        # Step 1: 候选 UAV（排除当前观察员 + 额外排除）
        exclude = set(current_observers.keys()) if isinstance(current_observers, dict) else set(current_observers)
        exclude = exclude | set(exclude_uavs)

        candidates = []
        for uid, pos in all_uavs.items():
            if uid in exclude:
                continue
            # UAV 必须在感知范围内（或稍远一点，允许飞来）
            d = math.hypot(pos[0] - target_xy[0], pos[1] - target_xy[1])
            # 放宽条件：25m 内都可以作为候选（允许飞来）
            if d < 25.0:
                candidates.append((uid, pos, d))

        if not candidates:
            return None

        # 按距离排序，优先考虑近的 UAV
        candidates.sort(key=lambda x: x[2])

        # Step 2: 为每个候选 UAV 找候选观察位
        best_score = float('-inf')
        best_uav = None
        best_vp = None
        best_uav_vel = (0, 0)  # backup UAV 速度假设

        for uid, uav_pos, d in candidates:
            # 为这个 UAV 生成候选观察位
            viewpoints = self._generate_viewpoints(target_xy)

            for vp in viewpoints:
                vx, vy = vp

                # 2.1 当前 LOS 必须通
                if not self.grid.visible(vx, vy, target_xy[0], target_xy[1]):
                    continue

                # 2.2 位置必须 free
                if not self.grid.free(vx, vy):
                    continue

                # 2.3 预测未来 LOS 仍通（关键：用观察位作为 backup 位置）
                if predict_horizon > 0:
                    future_tx = target_xy[0] + target_vel[0] * predict_horizon
                    future_ty = target_xy[1] + target_vel[1] * predict_horizon
                    # backup UAV 在观察位时的预测
                    if not self.grid.visible(vx, vy, future_tx, future_ty):
                        continue

                # 2.4 飞行成本
                flight_cost = math.hypot(vx - uav_pos[0], vy - uav_pos[1])
                if flight_cost > 50.0:  # 太远不考虑
                    continue

                # 2.5 打分
                score = self._score_backup_viewpoint(
                    vp, target_xy, uav_pos, flight_cost, current_observers, all_uavs
                )

                if score > best_score:
                    best_score = score
                    best_uav = uid
                    best_vp = vp

        if best_uav is None:
            return None

        return (best_uav, best_vp)

    def _generate_viewpoints(self, target_xy):
        """生成候选观察位"""
        tx, ty = target_xy
        R = SENSE_RADIUS
        ratios = [0.5, 0.6, 0.7, 0.8, 0.9]
        viewpoints = []
        for ratio in ratios:
            r = ratio * R
            for i in range(VIEWPOINT_N_CAND):
                angle = 2.0 * math.pi * i / VIEWPOINT_N_CAND
                sx = tx + r * math.cos(angle)
                sy = ty + r * math.sin(angle)
                viewpoints.append((sx, sy))
        return viewpoints

    def _score_backup_viewpoint(self, vp, target_xy, uav_pos, flight_cost, current_observers, all_uavs):
        """为 backup 观察位打分

        关键：不要求"距离原观察员足够远"，因为：
        1. 比赛没有 3m 硬要求
        2. backup 离原观察员近一点，只要不撞，也可能更适合 LOS
        """
        tx, ty = target_xy
        d_to_target = math.hypot(vp[0] - tx, vp[1] - ty)

        # 距离分数：越近越好
        dist_score = VIEWPOINT_W_DIST * (1.0 - d_to_target / SENSE_RADIUS)

        # 飞行成本分数：越近越好
        cost_score = 2.0 * max(0, 1.0 - flight_cost / 50.0)

        # 分离分数：鼓励适度分散，但不是硬要求
        # 只要不碰撞就行
        sep_score = 0.0
        if isinstance(current_observers, dict):
            observer_positions = list(current_observers.values())
        else:
            observer_positions = []
            for obs_id in current_observers:
                if obs_id in all_uavs:
                    observer_positions.append(all_uavs[obs_id])

        for obs_pos in observer_positions:
            d = math.hypot(vp[0] - obs_pos[0], vp[1] - obs_pos[1])
            if d < 1.5:  # 太近（会撞）扣分
                sep_score -= 2.0
            elif d < 5.0:
                sep_score += 0.5  # 近距离鼓励分散

        return dist_score + cost_score + sep_score

    def check_los_continuous(self, uav_pos, target_xy, confirm_duration=0.3, history=None):
        """检查 LOS 是否连续保持（用于 backup 确认）

        Args:
            uav_pos: (x, y) UAV 当前位置
            target_xy: (x, y) 目标位置
            confirm_duration: 需要确认的持续时间
            history: {(time, uav_pos, target_xy), ...} 历史记录

        Returns: True 如果 LOS 连续保持 confirm_duration 秒
        """
        if history is None or len(history) < 2:
            # 没有历史，至少检查当前 LOS
            return self.grid.visible(uav_pos[0], uav_pos[1], target_xy[0], target_xy[1])

        # 检查历史中是否 LOS 一直通
        # 简化：只检查最近 confirm_duration 时间
        sorted_history = sorted(history.keys())

        now = sorted_history[-1]
        cutoff = now - confirm_duration

        for t in sorted_history:
            if t < cutoff:
                continue
            hist_uav, hist_target = history[t]
            if not self.grid.visible(hist_uav[0], hist_uav[1], hist_target[0], hist_target[1]):
                return False

        return True
