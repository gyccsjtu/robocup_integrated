# -*- coding: utf-8 -*-
"""量化：真实地图下，不同飞行高度 / 不同观察距离的目标可见率。

回答「5.5m 高度到底能不能看见地面目标」。
"""
import json
import math
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.environ.get("SWARM_SRC", os.path.join(HERE, "..", "core"))
sys.path.insert(0, SRC)
from swarm_task import LineOfSight, DETECT_RADIUS                   # noqa: E402
from swarm_heightfield import HeightField, TARGET_KEYPOINT_Z         # noqa: E402

META = os.environ.get(
    "ROBOCUP_METADATA",
    os.path.expanduser("~/team_ws/robocup/src/robocup_training_worlds"
                       "/worlds/generated/training_city_full_s7.json"))
md = json.load(open(META))
hf = HeightField.from_metadata(md)
los = LineOfSight(lambda ix, iy: False, cell_size=hf.cell_size,
                  origin=hf.origin, height_at=hf.height_at_cell)

# 自由区域采样点（高度场为 0 = 没有建筑）
free = []
random.seed(3)
tries = 0
while len(free) < 3000 and tries < 200000:
    tries += 1
    x = random.uniform(-95, 95)
    y = random.uniform(-45, 45)
    if hf.height_at_xy(x, y) == 0.0:
        free.append((x, y))

print("自由采样点 %d，DETECT_RADIUS=%.0fm，目标关键点高度 %.2fm"
      % (len(free), DETECT_RADIUS, TARGET_KEYPOINT_Z))
print()
print("飞机高度 | 可见率(距<20m) | 可见率(距<10m) | 可见率(距<5m)")
print("---------+---------------+---------------+-------------")
for alt in (2.0, 3.5, 4.5, 5.0, 5.5, 6.0, 8.0, 12.0):
    random.seed(5)
    cnt = {20.0: [0, 0], 10.0: [0, 0], 5.0: [0, 0]}
    n = 0
    t = 0
    while n < 12000 and t < 400000:
        t += 1
        a = random.choice(free)
        ang = random.uniform(0, 2 * math.pi)
        d = random.uniform(1.5, 20.0)
        b = (a[0] + math.cos(ang) * d, a[1] + math.sin(ang) * d)
        if not (-100 <= b[0] <= 100 and -50 <= b[1] <= 50):
            continue
        if hf.height_at_xy(b[0], b[1]) > 0.0:
            continue          # 目标不会站在楼里
        n += 1
        vis = los.visible(a[0], a[1], b[0], b[1], alt, TARGET_KEYPOINT_Z)
        for lim in cnt:
            if d <= lim:
                cnt[lim][1] += 1
                if vis:
                    cnt[lim][0] += 1
    row = ["%5.1fm  " % alt]
    for lim in (20.0, 10.0, 5.0):
        ok, tot = cnt[lim]
        row.append("   %5.1f%% (%d)" % (100.0 * ok / tot if tot else 0, tot))
    print(" | ".join(row))

print()
print("对照：旧二维判定（膨胀栅格，不看高度）的可见率")
g = md["grid"]
W, H = g["width"], g["height"]
OX, OY = g["origin"]
occ = bytearray(W * H)
idx = 0
for pair in g["data"]:
    v, c = int(pair[0]), int(pair[1])
    for _ in range(c):
        if idx < W * H:
            occ[idx] = v
            idx += 1


def blocked_2d(ix, iy):
    if not (0 <= ix < W and 0 <= iy < H):
        return True
    return occ[iy * W + ix] == 1


los2 = LineOfSight(blocked_2d, cell_size=g["resolution_m"], origin=(OX, OY))
random.seed(5)
cnt = {20.0: [0, 0], 10.0: [0, 0], 5.0: [0, 0]}
n = 0
t = 0
while n < 12000 and t < 400000:
    t += 1
    a = random.choice(free)
    ang = random.uniform(0, 2 * math.pi)
    d = random.uniform(1.5, 20.0)
    b = (a[0] + math.cos(ang) * d, a[1] + math.sin(ang) * d)
    if not (-100 <= b[0] <= 100 and -50 <= b[1] <= 50):
        continue
    if hf.height_at_xy(b[0], b[1]) > 0.0:
        continue
    n += 1
    vis = los2.visible(a[0], a[1], b[0], b[1])
    for lim in cnt:
        if d <= lim:
            cnt[lim][1] += 1
            if vis:
                cnt[lim][0] += 1
row = ["  2D   "]
for lim in (20.0, 10.0, 5.0):
    ok, tot = cnt[lim]
    row.append("   %5.1f%% (%d)" % (100.0 * ok / tot if tot else 0, tot))
print(" | ".join(row))
