#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Render top-down SVG previews of generated training maps.

The preview reuses ``training_city_lib.build_world`` so the drawn geometry is
exactly what ends up in the ``.world`` collision bodies. It is a read-only
visualisation helper - it never writes a world or metadata file.
"""
import os
import sys
import base64

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import training_city_lib as lib  # noqa: E402

CFG = lib.load_config()
MAX_PX = 720.0

COLORS = {
    "free": "#fbfbf9",
    "blocked": "#ffd9cc",
    "boundary": "#3a3f47",
    "building": "#9aa0a8",
    "building_stroke": "#5b6068",
    "lamp": "#e0a93b",
    "spawn": "#2e9e5b",
    "goal_ok": "#d23b3b",
    "goal_bad": "#9b9b9b",
    "road": "#eef0f2",
}


def _scale(bounds, max_px):
    w = bounds[2] - bounds[0]
    h = bounds[3] - bounds[1]
    s = max_px / max(w, h)
    return s, w * s, h * s


def _tx(x, bounds, s):
    return (x - bounds[0]) * s


def _ty(y, bounds, s, h_px):
    return h_px - (y - bounds[1]) * s


def _decode_grid(meta):
    g = meta["grid"]
    return lib.decode_rle(g["data"], g["width"], g["height"]), g


def _grid_svg(meta, bounds, s, w_px, h_px, factor):
    """Draw the inflated occupancy grid as a background of blocked cells."""
    cells, g = _decode_grid(meta)
    W, H = g["width"], g["height"]
    if W * H <= 0:
        return ""
    cw = (bounds[2] - bounds[0]) / W
    ch = (bounds[3] - bounds[1]) / H
    # display cell size in px
    dcw = cw * s
    dch = ch * s
    if dcw < 0.6:  # too dense, downsample by skipping factor
        step = max(1, int(round(factor)))
    else:
        step = 1
    parts = []
    for iy in range(0, H, step):
        for ix in range(0, W, step):
            blocked = 0
            total = 0
            for dy in range(step):
                for dx in range(step):
                    if ix + dx < W and iy + dy < H:
                        total += 1
                        if cells[(iy + dy) * W + (ix + dx)]:
                            blocked += 1
            if total and blocked >= (total + 1) // 2:
                x0 = bounds[0] + ix * cw
                y0 = bounds[1] + iy * ch
                px = _tx(x0, bounds, s)
                py = _ty(y0 + ch * step, bounds, s, h_px)
                parts.append('<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="%s"/>'
                             % (px, py, dcw * step + 0.5, dch * step + 0.5, COLORS["blocked"]))
    return "".join(parts)


def render_svg(meta, show_grid=True, factor=4):
    bounds = [meta["bounds"]["x_min"], meta["bounds"]["y_min"],
              meta["bounds"]["x_max"], meta["bounds"]["y_max"]]
    s, w_px, h_px = _scale(bounds, MAX_PX)
    grid_layer = _grid_svg(meta, bounds, s, w_px, h_px, factor) if show_grid else ""

    parts = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %.1f %.1f" '
             'style="max-width:100%%;background:%s" font-family="monospace">'
             % (w_px, h_px, COLORS["free"])]

    # road strips (unit has none; city presets get a faint road tint)
    for poly in ():
        pass

    parts.append(grid_layer)

    # obstacles
    for o in meta["obstacles"]:
        if o["type"] == "boundary_wall":
            fill = COLORS["boundary"]
        elif o["type"] == "building":
            fill = COLORS["building"]
        elif o["type"] == "lamp_post":
            fill = COLORS["lamp"]
        elif o["type"] == "wall":
            fill = COLORS["building"]
        else:
            fill = "#cccccc"
        if o["shape"] == "box":
            cx, cy = o["center"]
            half_x = o["size"][0] / 2.0
            half_y = o["size"][1] / 2.0
            x0 = _tx(cx - half_x, bounds, s)
            y0 = _ty(cy + half_y, bounds, s, h_px)
            w = o["size"][0] * s
            h = o["size"][1] * s
            stroke = COLORS["building_stroke"] if o["type"] in ("building", "wall", "boundary_wall") else "none"
            if o["type"] == "lamp_post":
                # lamp is a cylinder; draw as circle for clarity
                r = max(o["radius"] * s, 1.6)
                parts.append('<circle cx="%.2f" cy="%.2f" r="%.2f" fill="%s"/>'
                             % (_tx(cx, bounds, s), _ty(cy, bounds, s, h_px), r, fill))
            else:
                parts.append('<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" '
                             'fill="%s" stroke="%s" stroke-width="1"/>'
                             % (x0, y0, w, h, fill, stroke))
        else:
            cx, cy = o["center"]
            r = max(o["radius"] * s, 1.6)
            parts.append('<circle cx="%.2f" cy="%.2f" r="%.2f" fill="%s"/>'
                         % (_tx(cx, bounds, s), _ty(cy, bounds, s, h_px), r, fill))

    # spawn
    sp = meta["spawn"]
    sx = _tx(sp["center"][0], bounds, s)
    sy = _ty(sp["center"][1], bounds, s, h_px)
    parts.append('<circle cx="%.2f" cy="%.2f" r="%.1f" fill="none" stroke="%s" '
                 'stroke-width="2.2" stroke-dasharray="4 3"/>'
                 % (sx, sy, max(sp["radius_m"] * s, 6), COLORS["spawn"]))
    parts.append('<text x="%.2f" y="%.2f" fill="%s" font-size="11" font-weight="bold">S</text>'
                 % (sx - 4, sy + 4, COLORS["spawn"]))

    # goals
    for gi, g in enumerate(meta["goal_candidates"]):
        gx = _tx(g["center"][0], bounds, s)
        gy = _ty(g["center"][1], bounds, s, h_px)
        color = COLORS["goal_ok"] if (g["valid"] and g["reachable"]) else COLORS["goal_bad"]
        parts.append('<circle cx="%.2f" cy="%.2f" r="4.5" fill="none" stroke="%s" stroke-width="2.2"/>'
                     % (gx, gy, color))
        if not (g["valid"] and g["reachable"]):
            d = 4.5
            parts.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="%s" stroke-width="1.6"/>'
                         % (gx - d, gy - d, gx + d, gy + d, color))
            parts.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="%s" stroke-width="1.6"/>'
                         % (gx + d, gy - d, gx - d, gy + d, color))
        parts.append('<text x="%.2f" y="%.2f" fill="%s" font-size="9">G%d</text>'
                     % (gx + 6, gy + 3, color, gi))

    parts.append('</svg>')
    return "".join(parts)


def caption(meta):
    b = meta["bounds"]
    nb = sum(1 for o in meta["obstacles"] if o["type"] == "building")
    nl = sum(1 for o in meta["obstacles"] if o["type"] == "lamp_post")
    return ("预设 %s%s | 边界 x[%.0f,%.0f] y[%.0f,%.0f] | 建筑 %d 灯杆 %d | "
            "出生净空 %.2f m | 自由空间 %.1f%% | 校验 %s"
            % (meta["generator"]["preset"],
               ("/" + meta["generator"]["category"]) if meta["generator"]["category"] else "",
               b["x_min"], b["x_max"], b["y_min"], b["y_max"],
               nb, nl, meta["spawn"]["clearance_m"],
               meta["connectivity"]["free_cell_ratio"] * 100,
               "OK" if meta["validation"]["ok"] else "FAIL"))


def build_preview(output_path, light=False):
    sections = []
    # three headline presets
    for preset, seed, cat in (("full", 7, None), ("small", 2026, None), ("unit", 42, "single_wall")):
        sdf, meta = lib.build_world(preset, seed, CFG, category=cat, world_name="preview")
        factor = 6 if preset == "full" else 3
        sections.append('<h3>%s</h3><div class="cap">%s</div>%s'
                        % (preset.upper(), caption(meta),
                           render_svg(meta, show_grid=not light, factor=factor)))

    # unit category gallery
    gallery = []
    for cat in lib.UNIT_CATEGORIES:
        sdf, meta = lib.build_world("unit", 42, CFG, category=cat, world_name="preview")
        gallery.append('<div class="cell"><div class="gtitle">%s</div>%s</div>'
                       % (cat, render_svg(meta, show_grid=not light, factor=2)))
    sections.append('<h3>unit 全类别画廊 (10×10 m, 固定类别/随机坐标)</h3>'
                    '<div class="gallery">%s</div>' % "".join(gallery))

    legend = (
        '<div class="legend">'
        '<span><i style="background:%s"></i>建筑/墙</span>'
        '<span><i style="background:%s"></i>灯杆</span>'
        '<span><i style="background:%s"></i>边界墙</span>'
        '<span><i style="background:%s"></i>出生区(S)</span>'
        '<span><i style="background:%s"></i>目标(G,可达)</span>'
        '<span><i style="background:%s"></i>目标(不可达/室内)</span>'
        '<span><i style="background:%s"></i>膨胀后障碍(不可通行)</span>'
        '</div>' % (COLORS["building"], COLORS["lamp"], COLORS["boundary"],
                    COLORS["spawn"], COLORS["goal_ok"], COLORS["goal_bad"], COLORS["blocked"])
    )

    html = (
        '<!DOCTYPE html><html lang="zh"><head><meta charset="utf-8">'
        '<title>RoboCup 训练地图预览</title>'
        '<style>'
        'body{font-family:-apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif;'
        'margin:0;background:#f4f6f8;color:#222;padding:24px}'
        'h2{margin:0 0 4px}h3{margin:28px 0 6px;font-size:16px}'
        '.cap{font-size:12px;color:#666;margin-bottom:8px}'
        '.legend{margin:18px 0;font-size:12px;display:flex;flex-wrap:wrap;gap:14px}'
        '.legend span{display:inline-flex;align-items:center;gap:5px}'
        '.legend i{width:13px;height:13px;border-radius:3px;display:inline-block}'
        '.card{background:#fff;border:1px solid #e3e6ea;border-radius:10px;'
        'padding:14px;margin:10px 0;box-shadow:0 1px 3px rgba(0,0,0,.05)}'
        '.gallery{display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));'
        'gap:12px;margin-top:8px}'
        '.cell{background:#fff;border:1px solid #e3e6ea;border-radius:8px;padding:8px}'
        '.gtitle{font-size:11px;color:#555;margin-bottom:4px;text-align:center}'
        'svg{display:block;margin:0 auto}'
        'p.note{font-size:12px;color:#777;max-width:880px}'
        '</style></head><body>'
        '<h2>RoboCup 多旋翼集群协同搜索 · 训练地图预览</h2>'
        '<p class="note">由 <code>robocup_training_worlds</code> 的确定性生成器实时渲染，'
        '几何与 Gazebo Classic SDF 中的 collision/visual 完全一致。地图是<b>训练假设</b>，'
        '非官方 world：道路宽度、建筑尺寸/数量/高度、灯杆间距、出生点与坐标系原点 '
        '(full 用 x[-100,100]/y[-50,50]) 均为团队约定。红/橙底表示按安全边距膨胀后的不可通行区。</p>'
        + legend +
        "".join('<div class="card">%s</div>' % s for s in sections) +
        '<p class="note">渲染说明：full 预设在 200×100 m 下含约 40 栋建筑与 180 根灯杆，'
        '栅格背景按 6× 降采样显示；small / unit 显示更精细。所有布局由 seed 决定，'
        '同 seed 逐字节复现。legend 中“目标”为室外候选区，不是规则规定的 6 个恐怖分子位置。</p>'
        '</body></html>'
    )
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(html)
    return output_path


if __name__ == "__main__":
    light = "--light" in sys.argv
    out_args = [a for a in sys.argv[1:] if a != "--light"]
    out = out_args[0] if out_args else os.path.join(HERE, "worlds", "generated", "preview.html")
    path = build_preview(out, light=light)
    print("wrote", path, "(light=%s)" % light)
