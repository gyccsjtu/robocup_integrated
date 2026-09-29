#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""局部 ESDF 距离场模块（阶段 A：DWA 避障增强，原创实现）。

以无人机当前位置为中心维护一个 side_m × side_m 的滚动局部占据栅格，
对障碍执行安全膨胀后，用「两遍一维欧氏距离变换（EDT）」计算每个栅格到
最近障碍的欧氏距离。对外提供 O(1) 距离查询与数值梯度。

设计约束：
  - 只依赖标准库（math），无 ROS / numpy，可独立单元测试、易答辩。
  - 坐标统一为「规划坐标系」= ENU 局部系（米），与 DWA 的
    `_scan_points_enu()` 输出、轨迹采样点完全同一坐标系，无需再变换。
  - 保守策略：窗口外 / 越界 / 未观测区域返回 UNKNOWN 哨兵，调用方必须
    当作「不安全」处理，绝不把未知当作安全空间。

算法来源说明：
  欧氏距离变换（EDT）是图像处理领域的教科书级公开算法，本文件为原创
  实现（自写变量名与注释），未复制 PX4-Avoidance / Fast-Planner 等开源
  项目的代码、类结构或参数。
"""

import math

# 未知/越界哨兵：距离恒为非负，用 -1.0 表示「未知」。
# DWA 对任何 < safe_radius 的值判为不安全，因此 UNKNOWN 天然触发轨迹淘汰。
UNKNOWN = -1.0

_INF = 1e18   # 距离变换中「自由格」的初始值（远大于窗口内最大距离）


def _edt_1d(values):
    """一维平方欧氏距离变换（下包络法，O(n)）。

    values: 长度 n 的序列；0.0 表示障碍格，正数(通常 _INF)表示自由格。
    返回：长度 n 的列表，第 q 项 = 到最近障碍格下标之差的平方。

    思路：每个障碍格 i 定义一条抛物线 (q - i)^2 + values[i]，答案就是这组
    抛物线下包络在 q 处的取值。用「交点法」顺序维护下包络，把 O(n^2) 的
    两两比较降为 O(n)。
    """
    n = len(values)
    out = [0.0] * n
    if 0.0 not in values:
        # 整条无障碍（正常不会发生：窗口边界已被标记为占据），保守返回大值
        return [_INF] * n

    vtx = [0] * n              # 下包络中每条抛物线对应的障碍下标
    bound = [0.0] * (n + 1)    # bound[k] = 第 k 条抛物线的右边界（交点横坐标）
    k = 0                      # 当前下包络条数 - 1
    vtx[0] = 0
    bound[0] = -_INF
    bound[1] = _INF

    for q in range(1, n):
        # 新抛物线(q)与当前最后一条抛物线(vtx[k])的交点
        fq = values[q] + float(q * q)
        while True:
            fv = values[vtx[k]] + float(vtx[k] * vtx[k])
            denom = 2.0 * float(q - vtx[k])
            s = (fq - fv) / denom if denom != 0.0 else _INF
            if s > bound[k]:
                break
            k -= 1
        k += 1
        vtx[k] = q
        bound[k] = s
        bound[k + 1] = _INF

    k = 0
    for q in range(n):
        while bound[k + 1] < q:
            k += 1
        dq = float(q - vtx[k])
        out[q] = dq * dq + values[vtx[k]]
    return out


class LocalEsdf(object):
    """滚动局部欧氏距离场。"""

    def __init__(self, resolution_m=0.25, side_m=20.0, inflate_m=0.25):
        self._res = float(resolution_m)
        self._side = float(side_m)
        self._w = int(round(self._side / self._res))
        self._h = self._w
        # 膨胀半径（格数）：填补 512 束激光在远端的间隙 + 障碍边缘不确定。
        # 注意：真正的安全裕度在 DWA 的 safe_radius 里，这里只负责建场准确。
        self._inflate = max(0, int(math.ceil(inflate_m / self._res)))
        self._dist = [0.0] * (self._w * self._h)   # 每格到最近障碍的欧氏距离(米)
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._center = (0.0, 0.0)
        self._ready = False

    # ---- 只读属性 ----
    @property
    def resolution(self):
        return self._res

    @property
    def side(self):
        return self._side

    @property
    def width(self):
        return self._w

    @property
    def height(self):
        return self._h

    @property
    def ready(self):
        return self._ready

    # ---- 坐标换算 ----
    def _snap(self, v):
        """把窗口原点吸附到整格（减少逐帧抖动）。"""
        return math.floor(v / self._res) * self._res

    def _world_to_cell(self, x, y):
        cx = int(math.floor((x - self._origin_x) / self._res))
        cy = int(math.floor((y - self._origin_y) / self._res))
        if 0 <= cx < self._w and 0 <= cy < self._h:
            return (cx, cy)
        return None

    # ---- 栅格操作 ----
    def _mark_border(self, occ):
        """把窗口外圈标记为占据：未知区/地图外 = 不安全（保守）。"""
        w, h = self._w, self._h
        for x in range(w):
            occ[x] = 1                    # y = 0
            occ[(h - 1) * w + x] = 1      # y = h-1
        for y in range(h):
            occ[y * w] = 1                # x = 0
            occ[y * w + w - 1] = 1        # x = w-1

    def _dilate(self, occ):
        """对占据格做圆盘膨胀（半径 self._inflate 格），填补激光束间隙。"""
        r = self._inflate
        if r <= 0:
            return occ
        w, h = self._w, self._h
        out = bytearray(occ)
        occupied = [i for i, v in enumerate(occ) if v]
        for idx in occupied:
            cx = idx % w
            cy = idx // w
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if dx * dx + dy * dy > r * r:
                        continue
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        out[ny * w + nx] = 1
        return out

    def _compute_esdf(self, occ):
        """两遍一维 EDT 求欧氏距离场（先沿列，再沿行）。"""
        w, h = self._w, self._h
        col = [0.0] * (w * h)
        # 第一遍：沿列（y 方向）对每一列做 1D EDT
        for x in range(w):
            vals = [0.0 if occ[y * w + x] else _INF for y in range(h)]
            d = _edt_1d(vals)
            for y in range(h):
                col[y * w + x] = d[y]
        # 第二遍：沿行（x 方向）对每一行做 1D EDT
        for y in range(h):
            vals = [col[y * w + x] for x in range(w)]
            d = _edt_1d(vals)
            for x in range(w):
                self._dist[y * w + x] = math.sqrt(d[x]) * self._res

    # ---- 对外接口 ----
    def update(self, center_xy, scan_points):
        """以 center_xy 为中心重建距离场。

        center_xy : (x, y) 无人机当前位置（规划坐标系，米）
        scan_points : [(x, y), ...] 障碍点集（同一坐标系，米），可为空
        """
        cx, cy = float(center_xy[0]), float(center_xy[1])
        self._center = (cx, cy)
        self._origin_x = self._snap(cx - self._side / 2.0)
        self._origin_y = self._snap(cy - self._side / 2.0)

        occ = bytearray(self._w * self._h)
        self._mark_border(occ)
        for px, py in scan_points:
            cell = self._world_to_cell(px, py)
            if cell is not None:
                occ[cell[1] * self._w + cell[0]] = 1
        occ = self._dilate(occ)
        self._compute_esdf(occ)
        self._ready = True

    def is_inside(self, x, y):
        """坐标是否落在窗口内（未落在外 = 未知）。"""
        return self._world_to_cell(x, y) is not None

    def get_distance(self, x, y):
        """返回 (x,y) 到最近障碍的欧氏距离(米)；越界/未知返回 UNKNOWN(-1.0)。"""
        cell = self._world_to_cell(x, y)
        if cell is None:
            return UNKNOWN
        return self._dist[cell[1] * self._w + cell[0]]

    def get_gradient(self, x, y):
        """返回距离场在 (x,y) 的数值梯度 (gx, gy)，指向距离增大的方向。

        中心差分，步长半格；任一侧越界（未知）则返回 (0,0)。
        """
        eps = self._res * 0.5
        dxp = self.get_distance(x + eps, y)
        dxm = self.get_distance(x - eps, y)
        dyp = self.get_distance(x, y + eps)
        dym = self.get_distance(x, y - eps)
        if dxp < 0.0 or dxm < 0.0 or dyp < 0.0 or dym < 0.0:
            return (0.0, 0.0)
        gx = (dxp - dxm) / (2.0 * eps)
        gy = (dyp - dym) / (2.0 * eps)
        return (gx, gy)


# ============================ 单元自测 ============================
if __name__ == "__main__":
    # 造一个已知障碍：x=0 处的竖直墙（激光点沿 y 从 -8 到 8），中心放无人机。
    esdf = LocalEsdf(resolution_m=0.25, side_m=20.0, inflate_m=0.25)
    wall = [(0.0, y) for y in [i * 0.25 - 8.0 for i in range(65)]]
    esdf.update((2.0, 0.0), wall)

    # 1) 距墙越远距离越大（单调性）
    d_near = esdf.get_distance(0.6, 0.0)
    d_mid = esdf.get_distance(1.5, 0.0)
    d_far = esdf.get_distance(4.0, 0.0)
    assert d_near < d_mid < d_far, (d_near, d_mid, d_far)
    print("单调性 OK: near=%.3f mid=%.3f far=%.3f" % (d_near, d_mid, d_far))

    # 2) 距离量级正确（墙在 x=0，膨胀 0.25，故 x=1 处距表面约 0.75）
    d1 = esdf.get_distance(1.0, 0.0)
    assert 0.5 < d1 < 1.0, d1
    print("量级 OK: get_distance(1,0)=%.3f (期望~0.75)" % d1)

    # 3) 越界返回 UNKNOWN
    assert esdf.get_distance(100.0, 0.0) == UNKNOWN
    assert esdf.get_distance(0.0, 100.0) == UNKNOWN
    print("越界保守 OK: 返回 UNKNOWN")

    # 4) 梯度指向远离墙（+x 方向）
    gx, gy = esdf.get_gradient(1.5, 0.0)
    assert gx > 0.0, (gx, gy)
    print("梯度 OK: grad=(%.3f,%.3f) 指向远离墙" % (gx, gy))

    # 5) 更新中心后窗口滚动
    esdf.update((-2.0, 0.0), wall)
    assert esdf.is_inside(-2.0, 0.0)
    print("滚动 OK: 中心更新后窗口正常")

    print("\nLocalEsdf 自测全部通过")
