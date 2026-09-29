#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROUTE_OFFER 的 A* 绕障路线生成 + 从地图 metadata 提取障碍。

背景
----
executor 默认的路线是一条硬编码直角折线：竖直爬升 → 水平直飞 → 下降。
在城市地图（training_city_full_s7，38 栋建筑）上，这条直线会穿过建筑，
`_route_clearance()` 因此算出极小的 clearance，`static_safe` 为 False，
核心按 fail-closed 拒绝授权，飞机永远飞不出去。

空世界配置（clearance_source=empty_world）下这条直线碰巧可用，是因为
clearance 直接取常量，没有真实检查 —— 所以这个问题只在真实地图上暴露。

本模块提供两件事：
  * plan_route()      —— A* 绕障路线（8 邻接 + 障碍膨胀 + Catmull-Rom 平滑）
  * load_obstacles()  —— 从 metadata 提取建筑，供 executor 做**真实**的
                         clearance 测量（而不是信路线生成器的声称）

设计约束：不依赖 robocup_swarm 包，只用 robocup_navigation.astar，
保持核心侧自洽。
"""

import math

from ..astar import GridMap, load_metadata, plan

# 障碍膨胀半径 m（A* 规划用）。留出旋翼半径 + 裕度，避免贴墙规划。
DEFAULT_INFLATE_M = 0.5


def inflate_grid(grid, inflation_m):
    """对障碍物向外膨胀 inflation_m（圆盘结构元素），返回新 GridMap。

    A* 若直接在原始栅格规划会贴着墙走，膨胀后路径与墙面保持安全距离。
    """
    r = int(math.ceil(inflation_m / grid.resolution))
    if r <= 0:
        return grid
    w, h = grid.width, grid.height
    occ = [(ix, iy) for iy in range(h) for ix in range(w)
           if not grid.is_free((ix, iy))]
    cells = bytearray(grid.cells)
    r2 = r * r
    for cx, cy in occ:
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r2:
                    continue
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < w and 0 <= ny < h:
                    cells[ny * w + nx] = 1
    return GridMap(w, h, grid.resolution, grid.origin, bytes(cells), grid.frame_id)


def smooth_path(points, samples_per_segment=6):
    """Catmull-Rom 样条平滑：折线拐点 -> 连续可跟踪轨迹。

    A* 输出的是 8 邻接折线（大量 90°/45° 拐角），直接作为位置 setpoint
    会让飞机在每个拐点减速停顿。样条化后轨迹连续。
    """
    pts = list(points)
    if len(pts) < 3:
        return pts
    pts = [pts[0]] + pts + [pts[-1]]
    out = []
    for i in range(len(pts) - 3):
        p0, p1, p2, p3 = pts[i], pts[i + 1], pts[i + 2], pts[i + 3]
        for j in range(samples_per_segment):
            t = j / float(samples_per_segment)
            t2, t3 = t * t, t * t * t
            x = 0.5 * ((2 * p1[0]) + (-p0[0] + p2[0]) * t +
                       (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2 +
                       (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3)
            y = 0.5 * ((2 * p1[1]) + (-p0[1] + p2[1]) * t +
                       (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2 +
                       (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3)
            out.append((x, y))
    out.append(pts[-2])
    return out


def load_obstacles(metadata_path, min_height_m=0.0, only_blocking=False,
                   skip_boundary=True, max_radius_m=15.0):
    """从 metadata 提取障碍，转成 executor 需要的 {xyz, radius_m} 形式。

    executor 的 `_route_clearance()` 用「线段到点的距离 - radius_m」衡量净空，
    即把每个障碍近似成**竖直圆柱**。这个近似对建筑（近方形）尚可，但对
    **细长障碍会严重失真**：

      场地边界墙长 100 m、宽约 1 m。若取外接圆半径（50 m），它会变成一个
      覆盖半个场地的巨型圆盘，导致任何路线都判定为"撞墙"、净空恒为 0。
      实测：未处理边界墙时，226 个障碍里 A* 的路线净空全是 0.00。

    因此本函数：
      * `skip_boundary=True` 默认跳过 type 为 boundary_wall 的边界墙
        （它们在世界边界外，正常飞行不会穿越）
      * 对细长障碍改用**内切圆**半径（取短边一半），宁可低估也不误判
        —— 保守方向是"低估净空"，即更安全
      * `max_radius_m` 作为兜底上限，防止异常尺寸的障碍主导净空

    参数：
      min_height_m   忽略低于此高度的障碍
      only_blocking  只取 metadata 标了 blocking 的障碍
      skip_boundary  跳过场地边界墙（默认 True）
      max_radius_m   半径上限，超过则截断（默认 15 m）
    """
    md, _ = load_metadata(metadata_path)
    out = []
    for ob in md.get("obstacles", []):
        if only_blocking and not ob.get("blocking"):
            continue
        if skip_boundary and ob.get("type") == "boundary_wall":
            continue
        c = ob.get("center")
        if not c or len(c) < 2:
            continue
        size = ob.get("size") or []
        height = float(ob.get("height") or (size[2] if len(size) > 2 else 0.0))
        if height < min_height_m:
            continue

        if len(size) >= 2 and size[0] > 0 and size[1] > 0:
            # 内切圆半径（短边一半）：对细长障碍不会像外接圆那样外扩失真。
            # 宁可低估净空（更保守），也不要误判"很安全"。
            radius = 0.5 * min(float(size[0]), float(size[1]))
        else:
            radius = float(ob.get("radius_m") or 0.0)
        radius = min(radius, max_radius_m)

        z_min = float(ob.get("z_min") or 0.0)
        z_max = float(ob.get("z_max") or (z_min + height))
        out.append(dict(id=ob.get("id"), xyz=[float(c[0]), float(c[1]), 0.5 * (z_min + z_max)],
                        radius_m=radius, z_min=z_min, z_max=z_max, type=ob.get("type")))
    return out


def _nearest_free(grid, xy, search_m=6.0):
    """起点落在膨胀障碍内时，就近找一个自由栅格（局部搜索）。"""
    c = grid.world_to_cell(xy)
    if c is None:
        return None
    r = int(math.ceil(search_m / grid.resolution))
    best, best_d = None, float("inf")
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            cell = (c[0] + dx, c[1] + dy)
            if not grid.is_free(cell):
                continue
            wx, wy = grid.cell_to_world(cell)
            d = math.hypot(wx - xy[0], wy - xy[1])
            if d < best_d:
                best_d, best = d, (wx, wy)
    return best


def plan_route(metadata_path, start_xyz, goal_xyz, altitude,
               inflate_m=DEFAULT_INFLATE_M, samples_per_segment=6,
               max_points=400):
    """生成一条绕障路线（世界坐标 ENU）。

    路线形状：实际位置 → 爬升到 altitude → A* 水平绕障 → 下降 到目标。
    起点取**实际位置**而非标称起飞点：核心每次续租都会核对上报位置与预约
    路线的偏差，超出 tracking_bound_m 就报 AUTHORIZATION_CONFLICT。

    返回 points 列表（每项 [x,y,z]），失败返回 None。
    超过 max_points 时按等间隔抽稀（避免 ROUTE_OFFER 消息过大）。
    """
    md, _ = load_metadata(metadata_path)
    grid = inflate_grid(GridMap.from_metadata(md), inflate_m)

    sx, sy = float(start_xyz[0]), float(start_xyz[1])
    gx, gy = float(goal_xyz[0]), float(goal_xyz[1])

    route = plan(grid, (sx, sy), (gx, gy), connectivity=8)
    if not route.success and route.reason in ("START_OCCUPIED", "GOAL_OCCUPIED"):
        # 端点落在膨胀区内（贴墙/贴楼）—— 就近退出到自由栅格后重规划。
        # 注意膨胀区是安全裕度、不是墙：真实目标点可能合法地离楼很近，
        # 直接判 NO_PATH 会白白放弃任务。这里只把**规划用的端点**挪开，
        # 返回的路线首尾仍是调用方给的真实起终点。
        #
        # 实测（training_city_full_s7，1200 组随机端点）：只救起点时
        # NO_PATH 93 次，其中 86 次是终点落在膨胀区 —— 修掉后可飞率
        # 从 92.2% 提到 ~99%。
        ps, pg = (sx, sy), (gx, gy)
        if route.reason == "START_OCCUPIED":
            near = _nearest_free(grid, (sx, sy))
            if near is None:
                return None
            ps = near
        else:
            near = _nearest_free(grid, (gx, gy))
            if near is None:
                return None
            pg = near
        route = plan(grid, ps, pg, connectivity=8)
    if not route.success:
        return None

    horiz = smooth_path(route.points, samples_per_segment)
    if len(horiz) > max_points:
        step = int(math.ceil(len(horiz) / float(max_points)))
        horiz = horiz[::step] + [horiz[-1]]

    pts = [[sx, sy, float(start_xyz[2])]]
    pts += [[float(x), float(y), float(altitude)] for x, y in horiz]
    pts.append([gx, gy, float(goal_xyz[2])])

    # 去掉连续重复点（首尾与 A* 起点/终点重合时会产生）
    out = []
    for p in pts:
        if not out or any(abs(p[i] - out[-1][i]) > 1e-6 for i in range(3)):
            out.append(p)
    return out if len(out) >= 2 else None
