#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
monitor_coordination.py —— 三机协同搜索「体检仪」

用法:
    python3 monitor_coordination.py              # 持续输出，每 3s 一屏
    python3 monitor_coordination.py --once       # 只输出一屏
    python3 monitor_coordination.py -i 1 -n 30   # 1s 一次，共 30 屏

输出:
  A. 真值/GPS  : Gazebo 真值位置、速度、高度
  B. 飞控状态  : armed / mode / 上报坐标 / 与真值误差
  C. 任务分配  : 被分配格、格中心、到中心距离、分配距今时长
  D. 覆盖进度  : 来自 /swarm/debug（manager 内部网格统计，需 manager 打过补丁）
  E. 目标状态  : 各目标 state / tracked_time / confirm_time
  F. 异常告警  : 卡死 / 无任务 / 撞机风险 / 掉高 / 坐标跳变

异常判定阈值:
  STALE_ASSIGN  分配超过 30s 仍未到位（距格中心 > 2m）
  NO_ASSIGN     超过 60s 没收到任何分配
  STUCK         速度 < 0.3 m/s 持续 10s 以上
  LOW_ALT       z < 4.0 m
  TOO_CLOSE     机间距 < 2.0 m
  POS_ERR       上报坐标与真值差 > 2.0 m
"""

import argparse
import math
import sys
import time

import rospy

from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import State as MavState
from robocup_swarm.msg import (SearchAssignment, TargetState, UavStatus)
from std_msgs.msg import String

UAV_IDS = ["uav_1", "uav_2", "uav_3", "uav_4", "uav_5", "uav_6"]
MODEL_OF = {"uav_%d" % i: "iris_%d" % i for i in range(1, 7)}

# 异常阈值
T_STALE_ASSIGN = 30.0     # s
T_NO_ASSIGN = 60.0        # s
T_STUCK_V = 0.3           # m/s
T_STUCK_T = 10.0          # s
T_LOW_ALT = 4.0           # m
T_TOO_CLOSE = 2.0         # m
T_POS_ERR = 2.0           # m
T_ARRIVE = 2.0            # m  「未到位」判定


class Monitor(object):
    def __init__(self):
        self.truth = {}          # uav -> (x,y,z)
        self.truth_t = {}        # uav -> time
        self.prev_truth = {}
        self.speed = {}          # uav -> m/s（真值差分）
        self.mav = {}            # uav -> MavState
        self.status = {}         # uav -> UavStatus
        self.assign = {}         # uav -> (SearchAssignment, recv_time)
        self.targets = {}        # tid -> TargetState
        self.debug = ""          # /swarm/debug 文本
        self.warns = {}          # uav -> {code: since_time}
        self.t0 = time.time()

        rospy.Subscriber("/gazebo/model_states", ModelStates, self._truth_cb, queue_size=1)
        for u in UAV_IDS:
            rospy.Subscriber("/%s/mavros/state" % u, MavState,
                             lambda m, u=u: self._mav_cb(u, m), queue_size=1)
        rospy.Subscriber("/swarm/uav_status", UavStatus, self._status_cb, queue_size=10)
        rospy.Subscriber("/swarm/assignment", SearchAssignment, self._assign_cb, queue_size=10)
        rospy.Subscriber("/swarm/target_states", TargetState, self._target_cb, queue_size=10)
        rospy.Subscriber("/swarm/debug", String, self._debug_cb, queue_size=1)

    # ---------------- 回调 ----------------
    def _truth_cb(self, msg):
        now = time.time()
        for name, pose in zip(msg.name, msg.pose):
            u = None
            for k, m in MODEL_OF.items():
                if m == name:
                    u = k
                    break
            if u is None:
                continue
            p = (pose.position.x, pose.position.y, pose.position.z)
            if u in self.truth:
                dt = now - self.truth_t.get(u, now)
                if dt > 1e-3:
                    ox, oy, _ = self.truth[u]
                    v = math.hypot(p[0] - ox, p[1] - oy) / dt
                    # 平滑，抑制仿真抖动
                    self.speed[u] = 0.6 * self.speed.get(u, v) + 0.4 * v
            self.truth[u] = p
            self.truth_t[u] = now

    def _mav_cb(self, u, msg):
        self.mav[u] = msg

    def _status_cb(self, msg):
        self.status[msg.uav_id] = msg

    def _assign_cb(self, msg):
        self.assign[msg.uav_id] = (msg, time.time())

    def _target_cb(self, msg):
        self.targets[msg.target_id] = msg

    def _debug_cb(self, msg):
        self.debug = msg.data

    # ---------------- 告警 ----------------
    def _flag(self, u, code, on):
        d = self.warns.setdefault(u, {})
        if on:
            if code not in d:
                d[code] = time.time()
        else:
            d.pop(code, None)

    def _update_warns(self):
        now = time.time()
        for u in UAV_IDS:
            # 掉高
            t = self.truth.get(u)
            self._flag(u, "LOW_ALT", bool(t) and t[2] < T_LOW_ALT)

            # 卡死
            v = self.speed.get(u, 0.0)
            st = self.warns.setdefault(u, {})
            if v < T_STUCK_V:
                if "STUCK_T" not in st:
                    st["STUCK_T"] = now
            else:
                st.pop("STUCK_T", None)
                st.pop("STUCK", None)
            if "STUCK_T" in st and now - st["STUCK_T"] > T_STUCK_T:
                self._flag(u, "STUCK", True)

            # 无任务 / 任务超时
            a = self.assign.get(u)
            if a is None:
                self._flag(u, "NO_ASSIGN", now - self.t0 > T_NO_ASSIGN)
                self._flag(u, "STALE_ASSIGN", False)
            else:
                msg, ts = a
                age = now - ts
                s = self.status.get(u)
                d = 9e9
                if s is not None:
                    d = math.hypot(s.x - msg.target_x, s.y - msg.target_y)
                self._flag(u, "STALE_ASSIGN", age > T_STALE_ASSIGN and d > T_ARRIVE)
                self._flag(u, "NO_ASSIGN", False)

            # 坐标误差
            s = self.status.get(u)
            if s is not None and t is not None:
                self._flag(u, "POS_ERR", math.hypot(s.x - t[0], s.y - t[1]) > T_POS_ERR)
            else:
                self._flag(u, "POS_ERR", False)

        # 机间距
        for i in range(len(UAV_IDS)):
            for j in range(i + 1, len(UAV_IDS)):
                a, b = UAV_IDS[i], UAV_IDS[j]
                pa, pb = self.truth.get(a), self.truth.get(b)
                close = False
                if pa and pb:
                    close = math.hypot(pa[0] - pb[0], pa[1] - pb[1]) < T_TOO_CLOSE
                self._flag(a, "TOO_CLOSE", close)
                self._flag(b, "TOO_CLOSE", close)

    # ---------------- 渲染 ----------------
    def render(self):
        now = time.time()
        self._update_warns()
        L = []
        L.append("=" * 78)
        L.append("t=%.0fs  %s" % (now - self.t0, time.strftime("%H:%M:%S")))

        # A+B+C 每机一行
        L.append("%-6s %-22s %-16s %-24s" % ("机", "真值(x,y,z) v", "飞控", "任务/格"))
        for u in UAV_IDS:
            t = self.truth.get(u)
            m = self.mav.get(u)
            s = self.status.get(u)
            a = self.assign.get(u)
            ts = "%-22s" % ("无真值" if not t else "(%6.1f,%6.1f,%5.2f) %4.1f" %
                            (t[0], t[1], t[2], self.speed.get(u, 0.0)))
            ms = "%-16s" % ("--" if not m else "%s/%s%s" %
                            ("ARM" if m.armed else "DIS", m.mode,
                             "" if m.connected else " NC"))
            if a is None:
                at = "无分配"
            else:
                msg, rt = a
                d = -1.0
                if s is not None:
                    d = math.hypot(s.x - msg.target_x, s.y - msg.target_y)
                at = "(%d,%d)->(%.0f,%.0f) d=%.1fm %ds前" % (
                    msg.cell_ix, msg.cell_iy, msg.target_x, msg.target_y, d, now - rt)
            L.append("%-6s %s %s %-24s" % (u, ts, ms, at))

            # 上报 vs 真值
            if s is not None and t is not None:
                err = math.hypot(s.x - t[0], s.y - t[1])
                L.append("      └ 上报(%6.1f,%6.1f) 误差 %.2fm | 格(%d,%d) conf=%.2f | 指配额(%d,%d)"
                         % (s.x, s.y, err, s.cell_ix, s.cell_iy, s.confidence,
                            s.assigned_cell_ix, s.assigned_cell_iy))

        # 机间距
        sep = []
        for i in range(len(UAV_IDS)):
            for j in range(i + 1, len(UAV_IDS)):
                pa, pb = self.truth.get(UAV_IDS[i]), self.truth.get(UAV_IDS[j])
                if pa and pb:
                    sep.append("%s-%s %.1fm" % (UAV_IDS[i][-1], UAV_IDS[j][-1],
                                                math.hypot(pa[0] - pb[0], pa[1] - pb[1])))
        L.append("机间距: " + "  ".join(sep))

        # D 覆盖
        if self.debug:
            L.append("网格: " + self.debug)

        # E 目标
        if self.targets:
            st_name = {0: "游走", 1: "逃跑", 2: "瞬移", 3: "已消除"}
            parts = []
            for tid in sorted(self.targets):
                g = self.targets[tid]
                parts.append("%s(%5.0f,%4.0f)%s tr=%.0f cf=%.0f" %
                             (tid, g.x, g.y, st_name.get(g.state, "?"),
                              g.tracked_time, g.confirm_time))
            L.append("目标: " + " | ".join(parts))

        # F 告警
        wl = []
        for u in UAV_IDS:
            for code, since in sorted(self.warns.get(u, {}).items()):
                if code == "STUCK_T":
                    continue
                wl.append("%s:%s(%.0fs)" % (u[-1], code, now - since))
        L.append("告警: " + (", ".join(wl) if wl else "无"))
        return "\n".join(L)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--once", action="store_true")
    ap.add_argument("-i", "--interval", type=float, default=3.0)
    ap.add_argument("-n", "--count", type=int, default=0, help="0=不限")
    args = ap.parse_args()

    rospy.init_node("monitor_coordination", anonymous=True)
    m = Monitor()
    # 等数据灌入
    rospy.sleep(2.0)
    k = 0
    while not rospy.is_shutdown():
        print(m.render())
        sys.stdout.flush()
        k += 1
        if args.once or (args.count and k >= args.count):
            break
        rospy.sleep(args.interval)


if __name__ == "__main__":
    main()
