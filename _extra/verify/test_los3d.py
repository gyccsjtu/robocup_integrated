# -*- coding: utf-8 -*-
"""三维 LOS 判定测试：合成场景（断言）+ 真实地图（集成/量化）。

运行：
  python test_los3d.py
"""
import json
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
# 源码目录：默认取同级 src/core，可用 SWARM_SRC 覆盖
SRC = os.environ.get("SWARM_SRC", os.path.join(HERE, "..", "core"))
sys.path.insert(0, SRC)

from swarm_task import LineOfSight          # noqa: E402
from swarm_heightfield import HeightField, TARGET_KEYPOINT_Z  # noqa: E402

# 地图 metadata：默认取 ROS 工作区里的那份，可用 ROBOCUP_METADATA 覆盖
META = os.environ.get(
    "ROBOCUP_METADATA",
    os.path.expanduser("~/team_ws/robocup/src/robocup_training_worlds"
                       "/worlds/generated/training_city_full_s7.json"))

TZ = TARGET_KEYPOINT_Z
OK = []


def check(name, cond):
    OK.append((name, bool(cond)))
    print(("  PASS  " if cond else "  FAIL  ") + name)


# ============================================================
# 1) 合成场景：墙高可控，直接验证三维几何
# ============================================================
def synth(wall_h):
    """在 x∈[10,10.5) 放一堵 wall_h 米高的墙，其余为空。"""
    def is_blocked(ix, iy):
        return ix == 20

    def height_at(ix, iy):
        return wall_h if ix == 20 else 0.0

    return LineOfSight(is_blocked, cell_size=0.5, origin=(0.0, 0.0),
                       height_at=height_at)


print("\n[1] 合成墙场景（origin=0,0；墙在 x=10m，格 ix=20）")

# 8m 高墙 —— 飞机 5.5m 应该被挡（三维和二维结论一致）
los8 = synth(8.0)
r = los8.visible(-5.0, 0.0, 15.0, 0.0, 5.5, TZ)
check("8m 墙 / 飞机 5.5m / 20m 外 → 遮挡（3D）", not r)
check("8m 墙 / 不传高度 → 遮挡（2D 回退，行为不变）",
      not los8.visible(-5.0, 0.0, 15.0, 0.0))

# 同一堵墙，飞机飞高到能整段越过墙顶 → 可见。
# 墙占 x∈[10,10.5]，视线最低点在墙远端 x=10.5（t=0.775）：
#   oz*(1-0.775) + 1.25*0.775 > 8  →  oz > 31.2m
# 所以 30m 不够（那里视线只有 7.99m，物理上确实被挡），40m 才行。
check("8m 墙 / 飞机 30m → 仍被挡（墙远端视线仅 7.99m，物理正确）",
      not los8.visible(-5.0, 0.0, 15.0, 0.0, 30.0, TZ))
check("8m 墙 / 飞机 40m → 可见（整段视线高过墙顶）",
      los8.visible(-5.0, 0.0, 15.0, 0.0, 40.0, TZ))

# 墙前 10m 内，视线根本不碰墙 → 可见
check("8m 墙 / 目标在墙同侧 10m → 可见",
      los8.visible(-5.0, 0.0, 5.0, 0.0, 5.5, TZ))

# ---- 核心回归：矮墙（2m）不该挡住 5.5m 的飞机 ----
# 视线在 x=10 处的高度 = 5.5 + (1.25-5.5)*0.75 = 2.31m > 2m 墙顶 → 应可见。
# 这正是原二维判定的 bug：只看到「射线穿过障碍格」就一律判死。
los2 = synth(2.0)
z_at_wall = 5.5 + (TZ - 5.5) * 0.75
check("2m 矮墙 / 飞机 5.5m → 可见（3D 修正了 2D 的过度遮挡）",
      los2.visible(-5.0, 0.0, 15.0, 0.0, 5.5, TZ))
check("  同上：2D 判定仍判死（说明差异确实来自高度维）",
      not los2.visible(-5.0, 0.0, 15.0, 0.0))
print("       视线过墙时高度 %.2fm vs 墙高 2.00m" % z_at_wall)

# 极近目标：视线很陡，几乎不会被挡
check("2m 矮墙 / 贴近观察（距离 4m）→ 可见",
      los2.visible(8.0, 0.0, 12.0, 0.0, 5.5, TZ))

# ============================================================
# 2) 真实地图：高度场栅格化是否正确
# ============================================================
print("\n[2] 真实地图 training_city_full_s7")
if not os.path.exists(META):
    print("  跳过：找不到 metadata")
else:
    md = json.load(open(META))
    hf = HeightField.from_metadata(md)
    n_occ, h_max = hf.occupancy()
    print("  栅格 %dx%d @ %.2fm, origin=%s" % (hf.width, hf.height, hf.cell_size, hf.origin))
    print("  有建筑格数 %d / %d，最高 %.2fm" % (n_occ, hf.width * hf.height, h_max))

    # 每栋楼的中心点高度应等于其 z_max
    blds = [o for o in md["obstacles"] if o["type"] == "building"]
    hit = sum(1 for b in blds
              if abs(hf.height_at_xy(b["center"][0], b["center"][1]) - b["z_max"]) < 1e-6)
    check("38 栋楼中心高度均等于 z_max（%d/%d）" % (hit, len(blds)), hit == len(blds))

    # 空地必须为 0（取一个远离障碍的 goal_candidate）
    g = md["goal_candidates"][0]["center"]
    check("goal 点 (%.1f, %.1f) 高度为 0（空地）" % (g[0], g[1]),
          hf.height_at_xy(g[0], g[1]) == 0.0)

    los_real = LineOfSight(
        lambda ix, iy: False, cell_size=hf.cell_size, origin=hf.origin,
        height_at=hf.height_at_cell)

    # ---- 真实场景：视线穿过建筑 → 必须判遮挡 ----
    b = blds[0]
    cx, cy = b["center"]
    rad = max(b["size"]) / 2.0 + 2.0
    px, py = cx - rad - 6.0, cy
    tx, ty = cx + rad + 6.0, cy
    check("楼后目标：飞机 5.5m → 被 %.2fm 高的楼挡住" % b["z_max"],
          not los_real.visible(px, py, tx, ty, 5.5, TZ))

    # ---- 交叉验证：DDA 结果 vs 逐点暴力采样（步长 0.25m）----
    # 这是三维判定正确性的核心把关：DDA 若写错（步进/参数 t 算错），
    # 会与暴力采样在大量样本上对不上。
    def brute_visible(px, py, pz, tx, ty, tz, step=0.25):
        d = math.hypot(tx - px, ty - py)
        if d <= 0:
            return True
        n = max(2, int(d / step))
        for i in range(1, n):          # 跳过两端（端点豁免）
            t = i / float(n)
            x = px + (tx - px) * t
            y = py + (ty - py) * t
            z = pz + (tz - pz) * t
            if hf.height_at_xy(x, y) > z:
                return False
        return True

    print("\n[3] DDA vs 暴力采样 交叉验证（真实地图，随机 8000 组）")
    random.seed(11)
    agree = 0
    tot = 0
    tries = 0
    disagree_ex = []
    while tot < 8000 and tries < 200000:
        tries += 1
        a = random.choice(md["goal_candidates"])["center"]
        ang = random.uniform(0, 2 * math.pi)
        d = random.uniform(3.0, 20.0)
        tx2, ty2 = a[0] + math.cos(ang) * d, a[1] + math.sin(ang) * d
        if not (-100 <= tx2 <= 100 and -50 <= ty2 <= 50):
            continue
        if hf.height_at_xy(a[0], a[1]) > 0 or hf.height_at_xy(tx2, ty2) > 0:
            continue                       # 端点本身在建筑里，跳过（豁免逻辑不同）
        pz = random.uniform(3.5, 6.0)      # 真实飞行高度区间
        tot += 1
        v_dda = los_real.visible(a[0], a[1], tx2, ty2, pz, TZ)
        v_brute = brute_visible(a[0], a[1], pz, tx2, ty2, TZ)
        if v_dda == v_brute:
            agree += 1
        elif len(disagree_ex) < 3:
            disagree_ex.append((a, (round(tx2, 1), round(ty2, 1)), round(pz, 2),
                                v_dda, v_brute))
    rate = 100.0 * agree / tot if tot else 0.0
    print("  一致 %d/%d = %.2f%%" % (agree, tot, rate))
    if disagree_ex:
        print("  不一致样例（DDA, 暴力）:")
        for e in disagree_ex:
            print("    %s" % (e,))
    check("DDA 与暴力采样一致率 > 97%%（实测 %.2f%%）" % rate, rate > 97.0)

    # ---- 用 metadata 自带的膨胀栅格做真正的 2D 对照 ----
    print("\n[4] 用 metadata 膨胀栅格做 2D 对照（飞机 5.5m）")
    g = md["grid"]
    W, H, RES = g["width"], g["height"], g["resolution_m"]
    OX, OY = g["origin"]
    # 还原 rle_pairs
    occ = bytearray(W * H)
    if g.get("encoding") == "rle_pairs":
        data = g["data"]
        idx = 0
        for pair in data:                      # 形如 [[1,802],[0,396],...]
            val, cnt = int(pair[0]), int(pair[1])
            for _ in range(cnt):
                if idx < W * H:
                    occ[idx] = val
                    idx += 1
    def blocked_2d(ix, iy):
        if not (0 <= ix < W and 0 <= iy < H):
            return True
        return occ[iy * W + ix] == 1

    los_2d = LineOfSight(blocked_2d, cell_size=RES, origin=(OX, OY))
    los_3d = LineOfSight(blocked_2d, cell_size=RES, origin=(OX, OY),
                         height_at=hf.height_at_cell)
    random.seed(7)
    same = d3_vis = d3_blk = 0
    n = 0
    tries = 0
    while n < 20000 and tries < 400000:
        tries += 1
        a = random.choice(md["goal_candidates"])["center"]
        ang = random.uniform(0, 2 * math.pi)
        d = random.uniform(2.0, 20.0)
        tx2, ty2 = a[0] + math.cos(ang) * d, a[1] + math.sin(ang) * d
        if not (-100 <= tx2 <= 100 and -50 <= ty2 <= 50):
            continue
        n += 1
        v2 = los_2d.visible(a[0], a[1], tx2, ty2)
        v3 = los_3d.visible(a[0], a[1], tx2, ty2, 5.5, TZ)
        if v2 == v3:
            same += 1
        elif v3:
            d3_vis += 1
        else:
            d3_blk += 1
    vis2 = same + d3_blk
    print("  样本 %d（2D 判可见 %d 组）" % (n, vis2))
    print("  两者一致        %6d" % same)
    print("  2D 判死 / 3D 判可见 %6d   ← 修正过度遮挡（矮物、绕行空间）" % d3_vis)
    print("  2D 判可见 / 3D 判死 %6d   ← 补上漏判（视线从楼里穿过）" % d3_blk)
    if vis2:
        print("  在 2D 认为可见的样本里，3D 否掉了 %.1f%%" % (100.0 * d3_blk / vis2))

print("\n==== 断言汇总 ====")
bad = [n for n, ok in OK if not ok]
print("%d/%d 通过" % (len(OK) - len(bad), len(OK)))
if bad:
    for n in bad:
        print("  FAILED: " + n)
    sys.exit(1)
print("全部通过")
