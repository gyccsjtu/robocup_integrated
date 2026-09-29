# -*- coding: utf-8 -*-
"""把 training_city_full_s7 的真实地图元数据 + 实机 6 机快照，渲染成俯视态势图(HTML)。"""
import json, base64, zlib, struct, os

META = r"C:\Users\gycsjtu\team_ws\robocup\src\robocup_training_worlds\worlds\generated\training_city_full_s7.json"
OUT  = r"C:\Users\gycsjtu\WorkBuddy\2026-09-28-19-13-02\实机运行_俯视态势.html"

# 实机快照（仿真时钟 1089 s / 2026-09-28 23:12，来自 /gazebo/model_states + local_position）
# id, x, y, z, roll, pitch, A*规划次数, ORCA无解次数
SNAP = [
    (1,  37.284,  -4.868, 2.471,  -5.0, -0.1, 22,   0),
    (2, -69.390,  32.027, 3.191,   4.7,  2.8, 23, 142),
    (3, -41.490, -29.932, 0.241,  -3.1, -75.0, 15, 116),
    (4,  23.810,  32.342, 4.076,   0.9,  5.0, 22,  87),
    (5,  85.052,  24.287, 2.627,   8.0,  1.5, 21,   0),
    (6,  27.055, -29.204, 3.326,   1.3,  4.4, 17,  13),
]
CLOCK, RTF, FPS = 1089, 0.40, 3.76
ORCA_TOTAL = 358

d = json.load(open(META, encoding='utf-8'))
g = d['grid']
GW, GH = g['width'], g['height']
b = d['bounds']
mw, mh = b['x_max'] - b['x_min'], b['y_max'] - b['y_min']

# ---------- 1. 占用栅格 -> PNG（纯标准库） ----------
data = g['data']
if isinstance(data, str):
    data = json.loads(data)
cells = []
for pair in data:
    cells.extend([int(pair[0])] * int(pair[1]))
assert len(cells) == GW * GH, (len(cells), GW * GH)

FREE, BLOCK = (247, 249, 252), (203, 214, 228)
rows = []
for iy in range(GH - 1, -1, -1):
    row = []
    for ix in range(GW):
        row.extend(BLOCK if cells[iy * GW + ix] else FREE)
    rows.append(row)

def png_bytes(rgb_rows, w, h):
    raw = b''.join(b'\x00' + bytes(r) for r in rgb_rows)
    def chunk(t, payload):
        return (struct.pack('>I', len(payload)) + t + payload +
                struct.pack('>I', zlib.crc32(t + payload) & 0xffffffff))
    ihdr = struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', ihdr) +
            chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))

png_b64 = base64.b64encode(png_bytes(rows, GW, GH)).decode()

# ---------- 2. SVG ----------
MAP_X, MAP_Y, SCALE = 70.0, 62.0, 4.2
MAP_W, MAP_H = mw * SCALE, mh * SCALE
SVG_W, SVG_H = 980, 552

def sx(x): return MAP_X + (x - b['x_min']) * SCALE
def sy(y): return MAP_Y + (b['y_max'] - y) * SCALE

def bcol(h):
    t = max(0.0, min(1.0, (h - 7.5) / (18.0 - 7.5)))
    a, c = (168, 196, 228), (44, 82, 130)
    return '#%02x%02x%02x' % tuple(int(a[i] + (c[i] - a[i]) * t) for i in range(3))

p = []
p.append(f'<svg viewBox="0 0 {SVG_W} {SVG_H}" xmlns="http://www.w3.org/2000/svg" '
         f'font-family="Segoe UI, Microsoft YaHei, sans-serif">')
p.append('<rect width="100%" height="100%" fill="#ffffff"/>')
p.append(f'<text x="{MAP_X}" y="40" font-size="14.5" fill="#4b5563">'
         f'robocup_training_city_full_s7 ｜ 200 m × 100 m ｜ 0.5 m 栅格 ｜ '
         f'38 栋楼 + 184 灯柱 ｜ 仿真时钟 {CLOCK} s</text>')
p.append(f'<image href="data:image/png;base64,{png_b64}" x="{MAP_X}" y="{MAP_Y}" '
         f'width="{MAP_W}" height="{MAP_H}" preserveAspectRatio="none" '
         f'style="image-rendering:pixelated" opacity="0.9"/>')
p.append(f'<rect x="{MAP_X}" y="{MAP_Y}" width="{MAP_W}" height="{MAP_H}" '
         f'fill="none" stroke="#9aa4b2" stroke-width="1.5"/>')

for o in d['obstacles']:
    if o.get('footprint', {}).get('kind') == 'circle':
        cx, cy = o['center']
        p.append(f'<circle cx="{sx(cx):.1f}" cy="{sy(cy):.1f}" r="1.9" fill="#b6c0cc"/>')

for o in d['obstacles']:
    fp = o.get('footprint')
    if not fp or fp.get('kind') != 'polygon':
        continue
    pts = ' '.join(f'{sx(px):.1f},{sy(py):.1f}' for px, py in fp['points'])
    h = o.get('height', 0)
    p.append(f'<polygon points="{pts}" fill="{bcol(h)}" fill-opacity="0.92" '
             f'stroke="#2c5282" stroke-width="0.9"/>')
    cx, cy = o['center']
    if o.get('size', [0, 0])[0] > 11:
        p.append(f'<text x="{sx(cx):.1f}" y="{sy(cy)+3:.1f}" font-size="8.5" '
                 f'fill="#ffffff" text-anchor="middle" font-weight="600">{h:.1f}m</text>')

sc = d['spawn']
p.append(f'<circle cx="{sx(sc["center"][0]):.1f}" cy="{sy(sc["center"][1]):.1f}" '
         f'r="{sc["radius_m"]*SCALE:.1f}" fill="none" stroke="#0f766e" '
         f'stroke-width="1.8" stroke-dasharray="5 3"/>')
p.append(f'<text x="{sx(sc["center"][0])+10:.1f}" y="{sy(sc["center"][1])-8:.1f}" '
         f'font-size="11.5" fill="#0f766e" font-weight="600">起飞点</text>')

for gc in d.get('goal_candidates', []):
    gx, gy = gc['center']
    p.append(f'<circle cx="{sx(gx):.1f}" cy="{sy(gy):.1f}" r="5.5" fill="none" '
             f'stroke="#b45309" stroke-width="1.6"/>')
    p.append(f'<circle cx="{sx(gx):.1f}" cy="{sy(gy):.1f}" r="1.6" fill="#b45309"/>')

def tw(s, fs=12.5):
    return sum(fs if ord(c) > 0x2e80 else fs * 0.55 for c in s)

NUDGE = {1: -14, 2: -14, 3: 30, 4: 0, 5: 0, 6: -14}
SPLIT = MAP_X + MAP_W * 0.62

for i, x, y, z, roll, pitch, astar, orca in SNAP:
    crashed = pitch < -40
    col = '#b91c1c' if crashed else '#1d4ed8'
    px, py = sx(x), sy(y)
    p.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="10" fill="#ffffff" fill-opacity="0.75"/>')
    p.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="9.5" fill="none" stroke="{col}" stroke-width="3"/>')
    p.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="2.6" fill="{col}"/>')
    if crashed:
        p.append(f'<line x1="{px-13:.1f}" y1="{py-13:.1f}" x2="{px+13:.1f}" y2="{py+13:.1f}" '
                 f'stroke="{col}" stroke-width="3"/>')
        p.append(f'<line x1="{px-13:.1f}" y1="{py+13:.1f}" x2="{px+13:.1f}" y2="{py-13:.1f}" '
                 f'stroke="{col}" stroke-width="3"/>')

    label = (f'uav_{i} 已坠机  pitch {pitch:.0f}°  z={z:.2f}m' if crashed
             else f'uav_{i}  {z:.2f} m')
    fs = 13.0
    w = tw(label, fs)
    left = px > SPLIT
    lx = px - 16 if left else px + 16
    ly = py + NUDGE[i] + 4.5
    rx = lx - w - 6 if left else lx - 5
    p.append(f'<line x1="{px:.1f}" y1="{py:.1f}" x2="{lx - (2 if left else -2):.1f}" '
             f'y2="{ly-5:.1f}" stroke="{col}" stroke-width="1.3" opacity="0.85"/>')
    p.append(f'<rect x="{rx:.1f}" y="{ly-15:.1f}" width="{w+11:.1f}" height="20" rx="4.5" '
             f'fill="{"#fef2f2" if crashed else "#ffffff"}" fill-opacity="0.95" '
             f'stroke="{col}" stroke-width="{1.6 if crashed else 0.9}"/>')
    p.append(f'<text x="{lx:.1f}" y="{ly:.1f}" font-size="{fs}" font-weight="700" '
             f'fill="{col}" text-anchor="{"end" if left else "start"}">{label}</text>')
    if orca:
        ot = f'ORCA 无解 ×{orca}'
        ow = tw(ot, 11.0)
        ox = lx - ow - 4 if left else lx - 1
        p.append(f'<rect x="{ox:.1f}" y="{ly+6:.1f}" width="{ow+8:.1f}" height="17" rx="4" '
                 f'fill="#fff7ed" fill-opacity="0.95" stroke="#fdba74" stroke-width="0.8"/>')
        p.append(f'<text x="{lx:.1f}" y="{ly+18:.1f}" font-size="11" fill="#9a3412" '
                 f'text-anchor="{"end" if left else "start"}">{ot}</text>')

bx, by = MAP_X + 24, MAP_Y + MAP_H + 34
p.append(f'<line x1="{bx}" y1="{by}" x2="{bx+20*SCALE}" y2="{by}" stroke="#374151" stroke-width="2"/>')
p.append(f'<line x1="{bx}" y1="{by-5}" x2="{bx}" y2="{by+5}" stroke="#374151" stroke-width="2"/>')
p.append(f'<line x1="{bx+20*SCALE}" y1="{by-5}" x2="{bx+20*SCALE}" y2="{by+5}" stroke="#374151" stroke-width="2"/>')
p.append(f'<text x="{bx+10*SCALE}" y="{by-9}" font-size="12" fill="#374151" text-anchor="middle">20 m</text>')
p.append(f'<text x="{bx}" y="{by+30}" font-size="12.5" fill="#374151">'
         f'■ 建筑（越深越高 9.8–17.9 m）　· 灯柱 7.5 m　◇ 目标候选点　'
         f'<tspan fill="#1d4ed8">⊙ 正常在飞</tspan>　<tspan fill="#b91c1c">⊗ 已坠机</tspan></text>')
p.append('</svg>')
svg = '\n'.join(p)

# ---------- 3. HTML ----------
tr = []
for i, x, y, z, roll, pitch, astar, orca in SNAP:
    crashed = pitch < -40
    st = ('<span class="bad">已坠机 · 停止搜图</span>' if crashed
          else '<span class="ok">正常搜索</span>')
    oc = (f'<span class="bad">{orca}</span>' if orca >= 50 else
          (f'<span class="warn">{orca}</span>' if orca else '<span class="ok">0</span>'))
    tr.append(
        f'<tr><td class="mono">uav_{i}</td><td class="mono">{x:7.1f}</td>'
        f'<td class="mono">{y:7.1f}</td><td class="mono strong">{z:.2f}</td>'
        f'<td class="mono">{pitch:+.0f}°</td>'
        f'<td class="mono">{astar}</td><td>{oc}</td><td>{st}</td></tr>')

html = f'''<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>协同搜索实机运行 · 俯视态势</title>
<style>
:root{{--bd:#e5e7eb;--tx:#111827;--sub:#6b7280}}
*{{box-sizing:border-box}}
body{{margin:0;background:#f3f4f6;color:var(--tx);
 font-family:"Segoe UI","Microsoft YaHei",system-ui,sans-serif}}
.wrap{{max-width:1560px;margin:0 auto;padding:22px 20px 40px}}
h1{{font-size:21px;margin:0 0 4px}}
.sub{{color:var(--sub);font-size:13px;margin-bottom:18px}}
.grid{{display:grid;grid-template-columns:1fr 470px;gap:18px;align-items:start}}
.card{{background:#fff;border:1px solid var(--bd);border-radius:10px;padding:14px}}
.card h2{{font-size:14.5px;margin:0 0 10px;padding-bottom:8px;border-bottom:1px solid var(--bd)}}
svg{{width:100%;height:auto;display:block}}
table{{width:100%;border-collapse:collapse;font-size:12.5px}}
th,td{{padding:6px 4px;border-bottom:1px solid #f1f3f5;text-align:right}}
th:first-child,td:first-child{{text-align:left}}
th{{color:var(--sub);font-weight:600;font-size:11.5px;background:#fafbfc}}
.mono{{font-variant-numeric:tabular-nums;font-family:Consolas,monospace}}
.strong{{font-weight:700}}
.ok{{color:#047857}}.warn{{color:#b45309}}.bad{{color:#dc2626;font-weight:700}}
.kpi{{display:grid;grid-template-columns:1fr 1fr;gap:9px;margin-bottom:12px}}
.kpi div{{background:#f9fafb;border:1px solid var(--bd);border-radius:7px;padding:8px 10px}}
.kpi b{{display:block;font-size:17px;margin-top:1px}}
.kpi span{{font-size:11.5px;color:var(--sub)}}
.note{{font-size:12.5px;line-height:1.75;color:#374151}}
.note li{{margin-bottom:7px}}
code{{background:#f1f5f9;padding:1px 5px;border-radius:4px;
 font-family:Consolas,monospace;font-size:11.5px}}
.alert{{background:#fef2f2;border:1px solid #fecaca;border-radius:8px;
 padding:10px 12px;font-size:12.5px;line-height:1.7;color:#7f1d1d;margin-bottom:12px}}
</style></head><body><div class="wrap">
<h1>协同搜索实机运行 · 城市场景俯视态势</h1>
<div class="sub">云服务器 · PX4 SITL + Gazebo 11 + ROS Noetic ｜ 6 架 iris ｜
 抓取时刻 2026-09-28 23:12（仿真时钟 {CLOCK} s）</div>
<div class="grid">
 <div class="card"><h2>真实地图 + 实机位置（来自 /gazebo/model_states 真值）</h2>{svg}</div>
 <div>
  <div class="card" style="margin-bottom:18px"><h2>运行工况</h2>
   <div class="kpi">
    <div><span>实时因子 RTF</span><b>{RTF}</b></div>
    <div><span>渲染帧率</span><b>{FPS} FPS</b></div>
    <div><span>仿真时钟</span><b>{CLOCK} s</b></div>
    <div><span>有效在飞</span><b>5 / 6</b></div>
   </div>
   <div class="note">CPU 配额 12 核，其中 gzclient 软件渲染独占 <b>~10.5 核（1048%）</b>，
    gzserver 仅 51%。RTF {RTF} 的根因在这里。</div>
  </div>
  <div class="card" style="margin-bottom:18px"><h2>逐机状态</h2>
   <table>
    <tr><th>机体</th><th>x (m)</th><th>y (m)</th><th>z (m)</th><th>pitch</th>
        <th>A*次数</th><th>ORCA无解</th><th>判定</th></tr>
    {''.join(tr)}
   </table>
   <div class="note" style="margin-top:9px">uav_3 的 x/y 与 20 分钟前一模一样（−41.490, −29.932），
    20 s 内水平位移 0.10 m —— 完全静止。</div>
  </div>
  <div class="card"><h2>本轮结论</h2>
   <div class="alert"><b>uav_3 已坠机并彻底停摆</b>：机头 −75°，高度 0.24 m，
    在空地上（最近建筑 6.5 m，非撞楼）。20+ 分钟内持续收到
    <b>满载 +1.00 m/s 爬升指令</b>却一米不升；PX4 侧 <code>armed:True</code>、
    <code>OFFBOARD</code>、<code>landed_state=2（在空）</code>、
    <code>MPC_Z_VEL_MAX_UP=3.0</code> 全部正常，指令链上没有第二个发布者。
    机体已经摔在地上，所以拉不起来。
   </div>
   <ol class="note">
    <li><b>坠机与 ORCA 爆发在时间上咬合</b>：22:42:16–22:42:36 连续 116 条
        「ORCA 无解，强制排斥」，紧接着 20 s 内高度从 4.06 m 掉到 0.13 m，
        此后再没起来。</li>
    <li><b>ORCA 的半平面求解是死代码（可证明）</b>：约束要求
        <code>sep_vel ≥ 6.0 m/s</code>（= (10 − d₃)/1.0，d₃ ≤ SAFE_3D = 4.0），
        而候选速度上限只有 <code>MAX_SPEED = 5.0 m/s</code>。投影不可能超过模长，
        所以<b>只要进入求解必然无解</b>，每次都退化到裸排斥。
        6 架里 4 架中招，累计 {ORCA_TOTAL} 次。</li>
    <li><b>常量自相矛盾</b>：巡航目标上限 <code>ALT_TARGET_CAP=4.5</code> 高于
        硬顶 <code>ALT_HARD_CEIL=4.2</code>，而最高高度层就是 4.6 m。
        uav_4 实测被钉在 4.03–4.22 m（1138 个采样里 195 个越过 4.2），
        是 P 控制与硬护栏长期互搏的限环。</li>
    <li><b>另一处不可达分支</b>：<code>ALT_PANIC=5.0</code> 与
        <code>ALT_EMERG_CEIL=5.0</code> 相等，EMERG 在后无条件覆盖，
        PANIC 那一档永远执行不到。</li>
    <li><b>静默改写</b>：那几道护栏直接覆写 <code>cmd.twist.linear.z</code>，
        一行日志都不打 —— 这就是掉高那一刻日志里干干净净的原因。</li>
    <li>地图里<b>没有恐怖分子 actor 和裁判节点</b>：training_city_full_s7 是纯训练图，
        那两样只在官方 <code>base.world</code> 那套里。</li>
   </ol></div>
 </div>
</div></div></body></html>'''

open(OUT, 'w', encoding='utf-8').write(html)
print('已写出', OUT, os.path.getsize(OUT), 'bytes')
