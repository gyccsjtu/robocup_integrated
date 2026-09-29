#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""裁判评分系统 - 计算协同搜索比赛的各项指标（含漏检诊断）

订阅消息：
  - /swarm/uav_status (UavStatus) - 无人机状态
  - /swarm/target_state (TargetState) - 恐怖分子真值
  - /swarm/target_detection (TargetDetection) - 检测事件
  - /swarm/assignment (SearchAssignment) - 任务分配

诊断功能：
  - 每个目标的发现/消除历史
  - 未被发现的目标列表及原因分析
  - 覆盖区域 vs 目标轨迹对比
"""

import os
import rospy
import json
import time
import math
from collections import defaultdict
from datetime import datetime

from std_msgs.msg import String

from robocup_swarm.msg import UavStatus, TargetState, TargetDetection, SearchAssignment


class JudgeSystem:
    def __init__(self):
        rospy.init_node("judge_system")

        # ===== 比赛配置 =====
        self.num_targets = 6  # 规则：6名恐怖分子
        self.total_time_limit = 300.0  # 比赛时长限制（秒）
        self.detect_radius = 20.0  # 无人机感知半径（与 manager 一致）

        # ===== 状态记录 =====
        self.start_time = None
        self.end_time = None
        self.running = False
        self._real_start_time = None  # 使用真实时间计时

        # 无人机状态历史
        self.uav_positions = defaultdict(list)  # uav_id -> [(time, x, y, z), ...]
        self.uav_status = {}  # uav_id -> latest UavStatus
        self.covered_regions = []  # [(time, x, y, radius), ...] 飞机覆盖过的区域

        # 目标状态历史
        self.target_states = {}  # target_id -> latest TargetState
        self.target_history = defaultdict(list)  # target_id -> [(time, state), ...]
        self.target_positions = defaultdict(list)  # target_id -> [(time, x, y), ...] 目标轨迹
        self.detections = []  # [(time, uav_id, target_id, x, y), ...]
        self.eliminations = []  # [(time, target_id), ...]

        # 任务分配记录
        self.assignments = defaultdict(list)  # uav_id -> [(time, cell_ix, cell_iy), ...]

        # 冲突检测
        self.conflicts = []  # [(time, target_id, [uav_ids]), ...]

        # ===== 订阅 =====
        rospy.Subscriber("/swarm/start", String, self.cb_start)  # 监听开始信号
        rospy.Subscriber("/swarm/uav_status", UavStatus, self.cb_uav_status)
        rospy.Subscriber("/swarm/target_states", TargetState, self.cb_target_state)
        rospy.Subscriber("/swarm/detection", TargetDetection, self.cb_target_detection)
        rospy.Subscriber("/swarm/assignment", SearchAssignment, self.cb_assignment)
        rospy.Subscriber("/swarm/finish", String, self.cb_finish)

        # 使用真实时间记录开始时间（避免 ROS 时间戳跳变问题）
        self._real_start_time = None
        rospy.loginfo("裁判系统启动（带漏检诊断）...")
        self.start_time = rospy.Time.now()
        self.running = True

    def cb_uav_status(self, msg):
        """无人机状态回调"""
        # 使用真实时间作为基准，减去开始时间得到相对时间
        if self._real_start_time is not None:
            t = time.time() - self._real_start_time
        else:
            t = msg.header.stamp.to_sec()
        uav_id = msg.uav_id

        # 记录位置
        self.uav_positions[uav_id].append((t, msg.x, msg.y, msg.z))
        self.uav_status[uav_id] = msg

        # 记录覆盖区域（用于漏检诊断）
        self.covered_regions.append((t, msg.x, msg.y, self.detect_radius))

        # 检测物理冲突：记录当前时刻所有无人机位置，用于后续检测
        # 按时间戳存储：t -> {uav_id: (x, y)}
        # 只记录每个时间戳的第一次位置，避免同一秒内多次计数
        if not hasattr(self, '_uav_positions_at_t'):
            self._uav_positions_at_t = {}
        t_key = int(t)
        if t_key not in self._uav_positions_at_t:
            self._uav_positions_at_t[t_key] = {}
            # 只在第一次到达该时间戳时记录，后续同一秒内的更新忽略
            self._uav_positions_at_t[t_key][uav_id] = (msg.x, msg.y, msg.z)
        elif uav_id not in self._uav_positions_at_t[t_key]:
            # 该时间戳已存在，但该无人机尚未记录
            self._uav_positions_at_t[t_key][uav_id] = (msg.x, msg.y, msg.z)

    def cb_target_state(self, msg):
        """目标状态回调"""
        if self._real_start_time is not None:
            t = time.time() - self._real_start_time
        else:
            t = msg.header.stamp.to_sec()
        target_id = msg.target_id

        old_state = self.target_states.get(target_id)
        self.target_states[target_id] = msg
        self.target_history[target_id].append((t, msg.state))

        # 记录目标轨迹（用于漏检诊断）
        if not msg.eliminated:
            self.target_positions[target_id].append((t, msg.x, msg.y))

        # 检测消除
        if msg.eliminated and (old_state and not old_state.eliminated):
            self.eliminations.append((t, target_id))
            rospy.loginfo("目标 %s 被消除", target_id)

    def cb_target_detection(self, msg):
        """检测事件回调"""
        if self._real_start_time is not None:
            t = time.time() - self._real_start_time
        else:
            t = msg.header.stamp.to_sec()
        self.detections.append((t, msg.uav_id, msg.target_id, msg.x, msg.y))

    def cb_assignment(self, msg):
        """任务分配回调"""
        if self._real_start_time is not None:
            t = time.time() - self._real_start_time
        else:
            t = msg.header.stamp.to_sec()
        if hasattr(msg, 'cell_ix') and hasattr(msg, 'cell_iy'):
            self.assignments[msg.uav_id].append((t, msg.cell_ix, msg.cell_iy))

    def cb_start(self, msg):
        """开始信号回调：收到 /swarm/start 后记录真实时间起点"""
        if self._real_start_time is None:
            self._real_start_time = time.time()
            self.start_time = rospy.Time.now()
            self.running = True
            rospy.loginfo("[judge] 收到开始信号，使用真实时间计时")

    def cb_finish(self, msg):
        """完赛信号回调：收到 /swarm/finish 即冻结计时并出报告"""
        if self.end_time is not None:
            return
        self.end_time = rospy.Time.now()
        self.running = False
        rospy.loginfo("[judge] 收到完赛信号：%s", msg.data)
        self.print_report()
        self.save_report()

    def _is_covered(self, tx, ty, t):
        """检查目标在时刻 t 是否处于飞机的覆盖区域内"""
        for ct, cx, cy, radius in self.covered_regions:
            if ct > t:
                break  # 只看 t 之前的覆盖记录
            if math.hypot(tx - cx, ty - cy) <= radius:
                return True
        return False

    def diagnose_missed_targets(self):
        """漏检诊断：分析每个目标为什么没被发现"""
        if not self.start_time:
            return {}

        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        start_t = self.start_time.to_sec()

        # 统计每个目标的发现情况
        target_diagnosis = {}
        discovered_targets = set()

        # 从检测记录中统计发现的目标
        for t, uav_id, target_id, x, y in self.detections:
            discovered_targets.add(target_id)

        # 分析每个目标
        for target_id in ["t%d" % i for i in range(self.num_targets)]:
            positions = self.target_positions.get(target_id, [])
            detection_times = [t for t, uid, tid, x, y in self.detections if tid == target_id]

            if target_id in discovered_targets:
                # 已发现
                first_detect = min(detection_times) - start_t
                total_detects = len(detection_times)
                last_pos = positions[-1] if positions else (None, None, None)
                target_diagnosis[target_id] = {
                    "status": "discovered",
                    "first_detect_time": first_detect,
                    "total_detections": total_detects,
                    "last_position": last_pos[1:] if last_pos else None,
                    "reason": "正常发现"
                }
            else:
                # 未发现，分析原因
                never_in_range = 0
                in_range_missed = 0

                for pt, px, py in positions:
                    covered = self._is_covered(px, py, pt)
                    if covered:
                        in_range_missed += 1  # 在覆盖区内但没被发现
                    else:
                        never_in_range += 1  # 从未被覆盖

                # 判断原因
                if never_in_range == len(positions):
                    reason = "目标始终在飞机感知范围外（覆盖盲区）"
                elif in_range_missed > 0:
                    reason = f"目标{in_range_missed}次在感知范围内但未被检测（几何判定失败）"
                else:
                    reason = "未知原因"

                target_diagnosis[target_id] = {
                    "status": "never_discovered",
                    "total_positions": len(positions),
                    "times_in_range": len(positions) - never_in_range,
                    "times_missed_in_range": in_range_missed,
                    "reason": reason,
                    "first_position": positions[0][1:] if positions else None,
                    "last_position": positions[-1][1:] if positions else None,
                }

        # 汇总
        never_discovered = [tid for tid, d in target_diagnosis.items()
                          if d["status"] == "never_discovered"]
        always_out_of_range = sum(1 for d in target_diagnosis.values()
                                  if "覆盖盲区" in d.get("reason", ""))
        missed_in_range = sum(1 for d in target_diagnosis.values()
                             if "几何判定失败" in d.get("reason", ""))

        return {
            "target_diagnosis": target_diagnosis,
            "never_discovered_targets": never_discovered,
            "never_discovered_count": len(never_discovered),
            "always_out_of_range": always_out_of_range,
            "missed_in_range": missed_in_range,
        }

    def compute_metrics(self):
        """计算所有指标"""
        if not self.start_time:
            return {}

        # 已完赛则冻结计时，后续指标不再随时间漂移
        now = self.end_time if self.end_time is not None else rospy.Time.now()
        elapsed = (now - self.start_time).to_sec()

        # ===== 1. 搜索效率指标 =====
        discovered = set()
        eliminated = set()

        # detections 存的是 (t, uav_id, target_id, x, y)
        for t, uav_id, target_id, x, y in self.detections:
            discovered.add(target_id)

        for t, target_id in self.eliminations:
            eliminated.add(target_id)

        total = self.num_targets
        discovery_rate = len(discovered) / total if total > 0 else 0
        elimination_rate = len(eliminated) / len(discovered) if discovered else 0

        # 第一个发现时间
        first_discovery_time = None
        if self.detections:
            first_discovery_time = self.detections[0][0] - self.start_time.to_sec()

        # 全部发现时间
        all_discovery_time = None
        if len(discovered) == total:
            # 最后一个目标被发现的时间
            last_discovery = max([d[0] for d in self.detections if d[2] in discovered])
            all_discovery_time = last_discovery - self.start_time.to_sec()

        # ===== 2. 协同效率指标 =====
        # 任务分配均衡（每个无人机分配到的任务数）
        task_counts = {}
        for uav_id, assigns in self.assignments.items():
            task_counts[uav_id] = len(assigns)

        # 计算方差
        if task_counts:
            mean_tasks = sum(task_counts.values()) / len(task_counts)
            variance = sum((c - mean_tasks) ** 2 for c in task_counts.values()) / len(task_counts)
        else:
            variance = 0

        # 冲突次数：基于物理冲突（两机距离 < 8m），而不是"同时发现目标"
        # 统计同一时刻两机距离 < 安全距离的次数
        SAFE_DIST = 8.0
        physical_conflicts = 0
        conflict_details = []  # 记录冲突详情用于调试
        if hasattr(self, '_uav_positions_at_t'):
            for t_key, positions in self._uav_positions_at_t.items():
                uav_ids = list(positions.keys())
                for i in range(len(uav_ids)):
                    for j in range(i + 1, len(uav_ids)):
                        p1 = positions[uav_ids[i]]
                        p2 = positions[uav_ids[j]]
                        dz = abs(p1[2] - p2[2]) if len(p1) > 2 and len(p2) > 2 else 0
                        if dz > 1.0:  # 高度差>1m认为不在同一层，跳过
                            continue
                        d = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
                        if d < SAFE_DIST:
                            physical_conflicts += 1
                            conflict_details.append((t_key, uav_ids[i], uav_ids[j], d))
        
        # 输出冲突日志
        if conflict_details:
            print(f"【物理冲突详情】共 {len(conflict_details)} 次冲突:")
            for t, u1, u2, dist in conflict_details[:10]:  # 只打印前10条
                print(f"  时间={t}s, {u1} <-> {u2}, 距离={dist:.1f}m")
        
        conflict_count = physical_conflicts

        # 空闲率（估计：没有检测的时间段）
        # 创建副本避免迭代过程中被修改导致的 ValueError
        detections_snapshot = list(self.detections)
        if detections_snapshot:
            detection_times = [d[0] for d in detections_snapshot]
            detection_times.sort()
            idle_time = 0
            for i in range(len(detection_times) - 1):
                gap = detection_times[i + 1] - detection_times[i]
                if gap > 5.0:  # 超过5秒无检测视为空闲
                    idle_time += gap
            idle_rate = idle_time / elapsed if elapsed > 0 else 0
        else:
            idle_rate = 1.0

        # ===== 3. 飞行质量指标 =====
        total_distance = 0.0
        for uav_id, positions in self.uav_positions.items():
            # 先排序，避免在迭代过程中修改列表
            sorted_positions = sorted(positions, key=lambda p: p[0])
            for i in range(1, len(sorted_positions)):
                dx = sorted_positions[i][1] - sorted_positions[i-1][1]
                dy = sorted_positions[i][2] - sorted_positions[i-1][2]
                dz = sorted_positions[i][3] - sorted_positions[i-1][3]
                total_distance += (dx**2 + dy**2 + dz**2) ** 0.5

        # ===== 4. 综合评分 =====
        search_score = (discovery_rate * 0.4 + elimination_rate * 0.4 +
                       (1 - (total - len(discovered))/total) * 0.2) * 100
        coordination_score = (1 - min(conflict_count / 10, 1.0)) * 100  # 假设10次冲突为满分
        flight_score = 100  # 碰撞由仿真器检测，这里简化

        total_score = search_score * 0.4 + coordination_score * 0.3 + flight_score * 0.2 + 100 * 0.1

        # 漏检诊断
        diagnosis = self.diagnose_missed_targets()

        return {
            "elapsed_time": elapsed,

            # 搜索效率
            "discovery_rate": discovery_rate,
            "elimination_rate": elimination_rate,
            "discovered_count": len(discovered),
            "eliminated_count": len(eliminated),
            "first_discovery_time": first_discovery_time,
            "all_discovery_time": all_discovery_time,

            # 协同效率
            "task_variance": variance,
            "conflict_count": conflict_count,
            "idle_rate": idle_rate,
            "task_counts": task_counts,

            # 飞行质量
            "total_distance": total_distance,

            # 综合评分
            "search_score": search_score,
            "coordination_score": coordination_score,
            "flight_score": flight_score,
            "total_score": total_score,

            # 漏检诊断
            "diagnosis": diagnosis,
        }

    def print_report(self):
        """打印评分报告"""
        m = self.compute_metrics()

        print("\n" + "="*60)
        print("           RoboCup 集群协同搜索 - 裁判评分报告")
        print("="*60)
        print(f"比赛时长: {m['elapsed_time']:.1f} 秒")
        print()

        print("【搜索效率】")
        print(f"  发现率: {m['discovery_rate']*100:.1f}% ({m['discovered_count']}/{self.num_targets})")
        print(f"  消除率: {m['elimination_rate']*100:.1f}% ({m['eliminated_count']}/{m['discovered_count']})")
        if m['first_discovery_time']:
            print(f"  首次发现: {m['first_discovery_time']:.1f} 秒")
        if m['all_discovery_time']:
            print(f"  全部发现: {m['all_discovery_time']:.1f} 秒")
        print()

        print("【协同效率】")
        print(f"  任务分配方差: {m['task_variance']:.2f}")
        print(f"  冲突次数: {m['conflict_count']}")
        print(f"  空闲率: {m['idle_rate']*100:.1f}%")
        print(f"  各机任务数: {m['task_counts']}")
        print()

        print("【飞行质量】")
        print(f"  总飞行距离: {m['total_distance']:.1f} 米")
        print()

        # 漏检诊断
        diag = m.get('diagnosis', {})
        if diag:
            print("【漏检诊断】")
            print(f"  未发现目标数: {diag.get('never_discovered_count', 0)}/6")
            print(f"  始终在覆盖区外: {diag.get('always_out_of_range', 0)} 个")
            print(f"  覆盖区内漏检: {diag.get('missed_in_range', 0)} 个")
            print()
            # 详细列出每个目标的状态
            target_diag = diag.get('target_diagnosis', {})
            for tid in ["t%d" % i for i in range(self.num_targets)]:
                d = target_diag.get(tid, {})
                status = d.get('status', 'unknown')
                if status == "discovered":
                    print(f"  {tid}: 已发现 (首次: {d.get('first_detect_time', 0):.1f}s, "
                          f"检测{d.get('total_detections', 0)}次)")
                else:
                    print(f"  {tid}: 未发现 - {d.get('reason', '未知')}")
                    if d.get('first_position'):
                        print(f"       初始位置: ({d['first_position'][0]:.1f}, {d['first_position'][1]:.1f})")
            print()

        print("【综合评分】")
        print(f"  搜索分 (40%): {m['search_score']:.1f}")
        print(f"  协同分 (30%): {m['coordination_score']:.1f}")
        print(f"  飞行分 (20%): {m['flight_score']:.1f}")
        print(f"  鲁棒分 (10%): 100.0")
        print()
        print(f"  >>> 总分: {m['total_score']:.1f} <<<")
        print("="*60)

    def save_report(self, filename=None):
        """保存报告到文件"""
        m = self.compute_metrics()

        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"judge_report_{timestamp}.json"

        report = {
            "timestamp": datetime.now().isoformat(),
            "elapsed_time": m['elapsed_time'],
            "search": {
                "discovery_rate": m['discovery_rate'],
                "elimination_rate": m['elimination_rate'],
                "discovered_count": m['discovered_count'],
                "eliminated_count": m['eliminated_count'],
                "first_discovery_time": m['first_discovery_time'],
                "all_discovery_time": m['all_discovery_time'],
            },
            "coordination": {
                "task_variance": m['task_variance'],
                "conflict_count": m['conflict_count'],
                "idle_rate": m['idle_rate'],
                "task_counts": m['task_counts'],
            },
            "flight": {
                "total_distance": m['total_distance'],
            },
            "scores": {
                "search_score": m['search_score'],
                "coordination_score": m['coordination_score'],
                "flight_score": m['flight_score'],
                "total_score": m['total_score'],
            },
            "diagnosis": {
                "never_discovered_count": m.get('diagnosis', {}).get('never_discovered_count', 0),
                "always_out_of_range": m.get('diagnosis', {}).get('always_out_of_range', 0),
                "missed_in_range": m.get('diagnosis', {}).get('missed_in_range', 0),
                "target_diagnosis": m.get('diagnosis', {}).get('target_diagnosis', {}),
            }
        }

        with open(filename, 'w') as f:
            json.dump(report, f, indent=2)

        rospy.loginfo("评分报告已保存: %s", filename)
        return filename


def main():
    judge = JudgeSystem()

    # 实时显示指标
    rate = rospy.Rate(1)  # 1Hz
    while not rospy.is_shutdown():
        if judge.end_time is not None:
            break       # 已收到完赛信号，报告由 cb_finish 输出过，不重复
        m = judge.compute_metrics()
        if m.get('elapsed_time', 0) > 5:  # 启动5秒后开始显示
            print(f"\r时间: {m['elapsed_time']:.0f}s | "
                  f"发现: {m['discovered_count']}/{judge.num_targets} | "
                  f"消除: {m['eliminated_count']} | "
                  f"冲突: {m['conflict_count']} | "
                  f"总分: {m['total_score']:.1f}", end='')
        rate.sleep()

    # 未收到完赛信号（手动 Ctrl-C 终止）时补一份报告
    if judge.end_time is None:
        print("\n\n未收到完赛信号（手动终止）")
        judge.print_report()
        judge.save_report()


if __name__ == "__main__":
    main()
