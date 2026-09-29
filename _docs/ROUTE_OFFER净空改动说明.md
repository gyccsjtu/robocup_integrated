# ROUTE_OFFER 净空改动说明（2026-09-28）

对应 `docs/项目文档/问题.md` 与 `message_to_teammate.md` 里的第二个「三维降维 / 常量顶替」问题：
`_build_offer()` 的硬编码直角折线 + `clearance` 恒等于常量 25m。

**约束：地图生成器、actor（恐怖分子）、裁判系统一律未改。**

---

## 一、先说结论

| 场景 | clearance 来源 | 可飞率（1200 组随机端点） |
|---|---|---|
| 空世界（原配置） | 常量 `empty_world_clearance_m = 25.0` | —（恒真，掩盖问题） |
| 真实城市 / 直角折线 | 真实测量 | **84.9%**（最小净空 0.00 m） |
| 真实城市 / A\* 绕障（修复后） | 真实测量 | **99.4%**（最小净空 1.55 m） |

地图：`training_city_full_s7`，38 栋建筑 + 184 根灯柱，高度 2.4 m，要求净空 0.8 m。
A\* 路线 10.9 万个点逐点复核：**0% 穿墙**。

---

## 二、发现：补丁其实早就打过了，而且打了两次

`coordination_executor.py` 里 `_route_points()` / `_obstacles()` / `route_planner` 配置
都已经在了，**不是没做**。但同一份补丁被应用了两次：

```
44: try:                                   ← 英文版
      from ...coordination.route_planner import (load_obstacles, plan_route)
50: try:                                   ← 中文版（重复）
      from ...coordination.route_planner import (load_obstacles, plan_route)
```

配置块同样重复（128-135 英文 / 136-142 中文）。

值相同所以不影响运行，但根因是 **`patch_executor.py` 不幂等**——
`CFG_NEW` 里包含 `CFG_OLD` 那行，第二次运行时仍能匹配上，于是又插一份。

### 改动 1：清理重复块

删掉重复的 import 块与配置块各一份，保留英文版（与文件其余部分一致）。
602 行，`py_compile` 通过。

### 改动 2：`patch_executor.py` 幂等化

每步替换前先查「已应用标记」，命中就跳过；全部命中则直接报告"无需重复执行"。

```python
MARKS = {
    "import":  "from robocup_navigation.coordination.route_planner import",
    "config":  'self.metadata_path = self.p("metadata_path", "")',
    "route":   "points = self._route_points(start, goal)",
    "helpers": "def _route_points(self, start, goal)",
}
```

---

## 三、实测才暴露的真 bug：`GOAL_OCCUPIED` 没人管

`astar.plan()` 的失败原因里 `START_OCCUPIED` 和 `GOAL_OCCUPIED` 是分开的，
但 `plan_route()` 只处理了前者：

```python
if not route.success and route.reason == "START_OCCUPIED":   # ← 漏了 GOAL_OCCUPIED
```

首轮实测 1200 组：A\* 有 **93 次 NO_PATH**，其中 **86 次是终点落在膨胀栅格内**。

这里的关键认知：**膨胀区是安全裕度，不是墙**。目标点合法地离楼很近时，
膨胀会把它盖住，直接判 NO_PATH 等于白白放弃任务（fail-closed 但不必要）。

### 改动 3：`plan_route()` 同时救起点和终点

```python
if not route.success and route.reason in ("START_OCCUPIED", "GOAL_OCCUPIED"):
    ps, pg = (sx, sy), (gx, gy)
    if route.reason == "START_OCCUPIED":
        ps = _nearest_free(grid, (sx, sy)) or return None
    else:
        pg = _nearest_free(grid, (gx, gy)) or return None
    route = plan(grid, ps, pg, connectivity=8)
```

只挪**规划用的端点**，返回路线的首尾仍是对外承诺的真实起终点——
否则核心会用 `ROUTE_GRANT` 的 points 去比对上报位置，偏移超过
`tracking_bound_m` 就报 `AUTHORIZATION_CONFLICT`。

效果：NO_PATH **93 → 7**，可飞率 **92.2% → 99.4%**。
剩下的 7 次是 `_nearest_free` 在 6 m 搜索半径内找不到自由格（端点被楼围死），属真实不可达。

---

## 四、判定口径（与 executor 完全一致）

```python
clearance   = _route_clearance(points, obstacles)     # 最小「线段-障碍中心距 − 半径」
static_safe = clearance >= required_clearance_m        # 默认 0.8
```

`load_obstacles()` 把每个障碍近似成竖直圆柱，半径取**内切圆**（短边一半）而非外接圆——
否则 100 m 长的边界墙会变成半径 50 m 的巨型圆盘，任何路线净空都是 0。
边界墙默认跳过（在世界边界外），灯柱保留（184 根，`r=0.15 m`）。

---

## 五、怎么开

ROS 参数（默认仍是 `straight`，行为不变）：

```yaml
route_planner: astar
metadata_path: .../worlds/generated/training_city_full_s7.json
clearance_source: map        # 不能再用 empty_world，否则又回到常量 25
route_inflate_m: 0.5
```

注意：`clearance_source=empty_world` 且无障碍时仍走常量分支，这是为兼容空世界测试保留的，
真实地图必须切到 `map`。

---

## 六、验证脚本

`src/verify/verify_route_offer.py`，只依赖标准库 + `robocup_navigation.astar`：

```bash
python3 src/verify/verify_route_offer.py
ROBOCUP_META=/path/city.json ROBOCUP_SAMPLES=3000 python3 src/verify/verify_route_offer.py
```

四项检查：空世界常量复现 / spawn→goal 六条 / 随机大样本 / A\* 穿墙复核。
统计口径上 `NO_PATH` 计入分母——只统计"规划成功的路线"会让 A\* 显示虚高的 100%。

---

## 七、还没做的

- `plan_route()` 每次调用都重读 metadata 并重做膨胀（O(格数×r²)），1200 组跑约 2 分钟。
  真机上每架每次续租都调一次，建议加缓存。
- `_nearest_free` 只做 6 m 局部搜索，被楼围死的端点仍会失败。
- `route_inflate_m=0.5` 与 `required_clearance_m=0.8` 的匹配关系没有联合扫参，
  两者是独立的：膨胀影响**能不能规划出路线**，净空阈值影响**要不要放行**。
