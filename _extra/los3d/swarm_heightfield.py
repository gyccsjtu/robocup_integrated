# -*- coding: utf-8 -*-
"""建筑高度场 + 三维视线（LOS）判定支持。

背景
----
原 `swarm_task.LineOfSight.visible(ox, oy, tx, ty)` 是**纯二维**判定：
水平射线穿过任意障碍栅格就判「看不见」，完全不关心飞机飞多高、
楼有多高。这在空世界里没问题，到了真实城市场景会失真——
楼只有 4m 高而飞机在 5.5m 时物理上明明看得见，却被判遮挡。

本模块补上缺失的高度维。数据来源是地图生成器
（`robocup_training_worlds`）已经输出的 metadata JSON —— 每个障碍都带
`z_max` / `height` 和真实 `footprint`（polygon 或 circle）。
**本模块只读取这些既有字段，不修改地图生成器、不修改 actor、不修改裁判系统。**

坐标系：与 metadata `grid` 一致（ENU, origin=(-100,-50), resolution=0.5m）。

用法
----
    hf = HeightField.from_metadata(md)
    los = LineOfSight(is_blocked, cell_size=..., origin=...,
                      height_at=hf.height_at_cell)
    los.visible(px, py, tx, ty, oz=uav_z, tz=TARGET_KEYPOINT_Z)

若 `height_at` 为 None 或 oz/tz 为 None，LineOfSight 自动退回原有二维逻辑，
行为与改动前完全一致（向后兼容）。
"""

import math
import os

__all__ = [
    "HeightField",
    "TARGET_KEYPOINT_Z",
    "los_3d_enabled",
]

# 目标（actor）可见关键点的高度 m。
# 取自 Gazebo actor 的 init_pose z（map_generator 生成 "<init_pose>a b 1.25 ..."）。
# 判定越「低」越保守：取脚踝高度会更容易被矮物遮挡。
# 需要更保守时改为 0.5，需要更宽松时改为 1.7（头顶）。
TARGET_KEYPOINT_Z = 1.25

# metadata 缺 z_max / height 时使用的兜底高度（保守取高值）。
DEFAULT_OBSTACLE_HEIGHT_M = 8.0


def los_3d_enabled():
    """三维 LOS 总开关，默认开。

    出意外时可不停机回退：export ROBOCUP_LOS_3D=0
    """
    v = os.environ.get("ROBOCUP_LOS_3D", "1").strip().lower()
    return v not in ("0", "false", "no", "off")


def _point_in_polygon(px, py, pts):
    """射线法：点 (px,py) 是否在多边形 pts 内（pts 为 [(x,y),...]，已含 yaw 旋转）。"""
    inside = False
    n = len(pts)
    if n < 3:
        return False
    j = n - 1
    for i in range(n):
        xi, yi = pts[i][0], pts[i][1]
        xj, yj = pts[j][0], pts[j][1]
        # 只统计与水平射线相交且交点在 px 右侧的边
        if (yi > py) != (yj > py):
            denom = (yj - yi)
            if denom != 0.0:
                xint = (xj - xi) * (py - yi) / denom + xi
                if px < xint:
                    inside = not inside
        j = i
    return inside


class HeightField(object):
    """栅格化的建筑高度场：每格记录该处障碍的顶面高度 z_max（无障碍为 0）。

    与飞行用的栅格（A* / 避障）刻意区分开：
      * 飞行栅格按 robot_radius + safety_margin 向外膨胀过（本项目 0.6m），
        目的是让飞机离墙远一点，跟「看不看得见」没有关系；
      * 本高度场用障碍的**真实 footprint**（不膨胀），否则会把建筑边缘
        0.6m 内的空地误判成遮挡。
    两者职责不同，不能混用。
    """

    def __init__(self, cells, width, height, cell_size, origin):
        self.cells = cells          # list[float]，长度 width*height
        self.width = int(width)
        self.height = int(height)
        self.cell_size = float(cell_size)
        self.origin = (float(origin[0]), float(origin[1]))

    # ---- 构造 ----

    @classmethod
    def from_metadata(cls, md, ignore_types=(), extra_margin_m=0.0):
        """从 metadata dict 构建高度场。

        md : load_metadata() 返回的 dict（robocup_training_worlds/metadata/v1）
        ignore_types : 需要忽略的障碍类型，如 ("lamp_post",)
        extra_margin_m : 额外外扩，正常保持 0（>0 会让判定变保守）
        """
        g = md.get("grid", {})
        cell = float(g.get("resolution_m", 0.5))
        origin = tuple(g.get("origin", (0.0, 0.0)))
        width = int(g.get("width", 0))
        height = int(g.get("height", 0))
        if width <= 0 or height <= 0:
            # 没有 grid 描述时用 bounds 兜底
            b = md.get("bounds", {})
            width = int(math.ceil((b.get("x_max", 0) - b.get("x_min", 0)) / cell))
            height = int(math.ceil((b.get("y_max", 0) - b.get("y_min", 0)) / cell))
            origin = (float(b.get("x_min", 0.0)), float(b.get("y_min", 0.0)))

        cells = [0.0] * (width * height)
        hf = cls(cells, width, height, cell, origin)
        hf._rasterize(md.get("obstacles", []), ignore_types, extra_margin_m)
        return hf

    def _rasterize(self, obstacles, ignore_types, extra_margin_m):
        for ob in obstacles:
            if ob.get("type") in ignore_types:
                continue
            if not ob.get("blocking", True):
                continue
            z = ob.get("z_max", ob.get("height", DEFAULT_OBSTACLE_HEIGHT_M))
            try:
                z = float(z)
            except (TypeError, ValueError):
                z = DEFAULT_OBSTACLE_HEIGHT_M
            fp = ob.get("footprint") or {}
            kind = fp.get("kind")
            if kind == "polygon":
                self._raster_polygon(fp.get("points", []), z, extra_margin_m)
            elif kind == "circle":
                self._raster_circle(fp.get("center"), float(fp.get("radius", 0.0)),
                                    z, extra_margin_m)
            elif "bbox" in ob:
                # 兜底：只有 bbox 时按矩形处理
                x0, y0, x1, y1 = ob["bbox"]
                self._raster_polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)],
                                     z, extra_margin_m)

    def _raster_polygon(self, pts, z, margin):
        if len(pts) < 3:
            return
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        self._fill_box(min(xs) - margin, max(xs) + margin,
                       min(ys) - margin, max(ys) + margin,
                       lambda px, py: _point_in_polygon(px, py, pts), z)

    def _raster_circle(self, center, radius, z, margin):
        if not center or radius <= 0.0:
            return
        cx, cy = float(center[0]), float(center[1])
        # 细长障碍（灯柱等）半径常小于半个格：加半格保证至少落到 1 格上，
        # 否则 0.15m 的杆在 0.5m 栅格里会被整格「漏掉」，判定失真。
        reach = radius + margin + self.cell_size * 0.5
        r2 = reach * reach
        self._fill_box(cx - reach, cx + reach, cy - reach, cy + reach,
                       lambda px, py: (px - cx) ** 2 + (py - cy) ** 2 <= r2, z)

    def _fill_box(self, x0, x1, y0, y1, test_xy, z):
        """在 bbox 覆盖的格范围内，对满足 test_xy(格中心) 的格取 max(z)。"""
        ix0, iy0 = self._to_cell(x0, y0)
        ix1, iy1 = self._to_cell(x1, y1)
        ix0 = max(0, min(self.width - 1, ix0))
        ix1 = max(0, min(self.width - 1, ix1))
        iy0 = max(0, min(self.height - 1, iy0))
        iy1 = max(0, min(self.height - 1, iy1))
        half = self.cell_size * 0.5
        for iy in range(iy0, iy1 + 1):
            wy = self.origin[1] + (iy + 0.5) * self.cell_size
            row = iy * self.width
            for ix in range(ix0, ix1 + 1):
                wx = self.origin[0] + (ix + 0.5) * self.cell_size
                # 与 bbox 相交判定（格中心 + 半格），避免漏掉边缘格
                if not (x0 - half <= wx <= x1 + half and y0 - half <= wy <= y1 + half):
                    continue
                if not test_xy(wx, wy):
                    continue
                k = row + ix
                if z > self.cells[k]:
                    self.cells[k] = z

    # ---- 查询 ----

    def _to_cell(self, x, y):
        return (int(math.floor((x - self.origin[0]) / self.cell_size)),
                int(math.floor((y - self.origin[1]) / self.cell_size)))

    def in_bounds(self, ix, iy):
        return 0 <= ix < self.width and 0 <= iy < self.height

    def height_at_cell(self, ix, iy):
        """格 (ix,iy) 的建筑顶面高度 m；越界或无障碍返回 0.0。"""
        if not self.in_bounds(ix, iy):
            return 0.0
        return self.cells[iy * self.width + ix]

    def height_at_xy(self, x, y):
        ix, iy = self._to_cell(x, y)
        return self.height_at_cell(ix, iy)

    def blocks(self, ix, iy, z_ray):
        """视线在高度 z_ray 穿过格 (ix,iy) 时是否被挡。"""
        return self.height_at_cell(ix, iy) > z_ray

    # ---- 统计/自测辅助 ----

    def occupancy(self):
        """有建筑覆盖的格数、最高建筑高度。"""
        n = 0
        mx = 0.0
        for v in self.cells:
            if v > 0.0:
                n += 1
                if v > mx:
                    mx = v
        return n, mx
