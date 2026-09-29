# 训练地图生成器交付记录（2026-09-05）

对应任务：把《训练地图生成说明与 AI 提示词》落成可复现的 Gazebo Classic 随机城市生成器。
本文件记录新增文件、启动命令、随机化设计、实测结果、训练假设边界与仍缺资源。

## 1. 新增文件（未改动任何第三方与历史数据）

新增 ROS 包 `src/robocup_training_worlds/`：

| 文件 | 作用 |
|---|---|
| `package.xml` / `CMakeLists.txt` | catkin 包声明，只依赖 `python3-yaml` |
| `config/training_city.yaml` | 三个预设、安全间距、unit 类别契约、材质与物理参数 |
| `scripts/training_city_lib.py` | 几何/栅格/BFS/SDF 写出/校验核心，纯标准库 + PyYAML |
| `scripts/generate_training_city.py` | CLI：按 seed 生成 `.world` + `.json` |
| `scripts/validate_training_city.py` | CLI：起飞前门禁，检查 SDF 与 metadata，无 GUI |
| `scripts/rasterize_metadata.py` | CLI：metadata → PGM + YAML（OccupancyGrid 约定） |
| `launch/training_city_single_uav.launch` | 只把 world 注入现有 `px4/mavros_posix_sitl.launch` |
| `worlds/generated/README.txt` | 生成物目录说明 |
| `tests/test_training_city_generator.py` | 38 条离线测试 |
| `README.md` | 包内说明 |

新增工程级脚本与编排：

| 文件 | 作用 |
|---|---|
| `scripts/container/generate_training_city.sh` | 容器内生成 + 落盘即校验 |
| `scripts/container/start_training_city.sh` | 校验通过后才 `roslaunch`，失败即退出 |
| `scripts/container/run_training_city_tests.sh` | 容器内跑离线测试 |
| `docker-compose.training-city.yml` | 只覆盖 `sim.command`，复用同一容器/镜像/卷/环境 |
| `scripts/generate_training_city.ps1` | 主机入口：生成单元/小图/全图 |
| `scripts/start_training_city.ps1` | 主机入口：生成 → 校验 → 启动 → 等待 MAVROS → 健康检查 |
| `scripts/run_training_city_tests.ps1` | 主机入口：跑测试 |

修改：`README.md` 增加「训练地图（实验）」一节（仅追加文档，不改行为）。

**未触碰**：`third_party/PX4-Autopilot`、`third_party/XTDrone`、`../pre.data`、
`robocup_navigation` 的唯一控制节点、`start_single_uav.sh` 与
`docker-compose.single-uav.yml`。空场启动命令行为完全不变。

## 2. 启动命令（Windows PowerShell）

```powershell
Set-Location D:\a\.robocup\robocup_ws

# 1) 生成并启动 unit 场景（默认 single_wall，seed 42）后自动做健康检查
.\scripts\start_training_city.ps1 -Preset unit -Seed 42 -Category single_wall

# 2) 启动 small 场景（30x20 m 随机城市；起飞前必须先放宽并复验 5 m 半径围栏）
.\scripts\start_training_city.ps1 -Preset small -Seed 2026

# 3) 只生成 full（200x100 m）不飞行：当前 max_radius_m=5 的控制器禁止飞全图
.\scripts\generate_training_city.ps1 -Preset full -Seed 7

# 只重新生成、不启动
.\scripts\generate_training_city.ps1 -Preset unit -Seed 3 -Category u_shape

# 离线测试（容器内，不开 Gazebo）
.\scripts\run_training_city_tests.ps1
```

图形界面仍沿用现有 WSLg 独立 `gzclient` 方式；后台继续 `interactive:=false`、
`gui` 由 `ROBOCUP_GAZEBO_GUI` 控制，默认 `false`。落盘路径：
`src\robocup_training_worlds\worlds\generated\training_city_<preset>[_<category>]_s<seed>.world`。

## 3. 随机化设计

- 随机源是单一 `random.Random(seed)`，所有采样顺序固定；同一 seed 逐字节复现
  `.world` 与 `.json`（已用 `cmp` 实测）。metadata 默认不含时间戳。
- `small` / `full`：先把边界按「路—街区—路」交替切分成街区与道路条带，再在街区
  内做拒绝采样放建筑（尺寸、位置、间距随机），最后沿道路中心线两侧按间距放灯杆。
  建筑与灯杆互斥保留 `lamp_clearance_m` 间距。
- `unit`：**固定类别、不固定坐标**。墙的位置、缺口宽度、U 形开口尺寸、走廊宽度
  都随 seed 变化；`empty` 无内部障碍。
- 出生区：unit 用配置固定点；`small` / `full` 在「道路上的自由栅格」中按 seed 选取，
  再要求离所有障碍 ≥ `spawn_clearance_m`。目标候选区同样只在室外自由空间采样，
  并强制彼此间距与离出生点最小距离。
- 出生/目标位置只是训练产物，**不是**官方比赛出生点；目标候选数量（full 为 6）
  对齐规则中「6 名目标」的数量，但坐标纯属训练假设。

## 4. 实测结果（本机，Windows 宿主 Python 3.13 + PyYAML 6.0.3）

- 离线测试：`python -m unittest discover -s tests` → **Ran 38 tests, OK**（约 32 s）。
- 随机压力：8 个 unit 类别 × 10 个 seed + small × 10 + full × 6 = **105 张地图，
  0 条校验失败**。
- 字节级确定性：unit / small / full 同 seed 两次生成 `cmp` 完全一致；换 seed 必变。
- 样例（seed 42）：unit 5 个障碍、free ratio 0.574；small 29 个障碍（3 建筑 + 22 灯杆）、
  0.703；full 214 个障碍（28 建筑 + 182 灯杆）、0.788；调优后 full 稳定在
  38–43 个建筑、约 0.8 s 生成。
- 负例：10 个 seed 的 `blocked` / `no_path` 全部 BFS 判为不可达；`goal_in_obstacle`
  全部被标记为 `outdoors=false`、`valid=false`、`reachable=false`。
- 门禁反例测试：篡改目标点到建筑内部、篡改出生点到建筑内部、越界障碍、删除
  collision、重名模型、不可能满足的 `spawn_clearance_m`，均被校验器或生成器拒绝
  （生成器在校验失败时**不写文件**）。

**尚未实测**：真机 Gazebo 加载（撰写时 Docker Desktop 未运行）。启动脚本会在
`roslaunch` 之前完成落盘校验；下一步验收应执行
`.\scripts\start_training_city.ps1 -Preset unit -Seed 42 -Category single_wall`
并确认 `gzserver` 无 SDF 解析错误。

## 5. 属于训练假设的参数（不可对外称官方）

道路宽度、街区尺寸与目标值、建筑数量/尺寸/高度分布、建筑间距、灯杆半径/高度/间距
与离路缘偏移、边界墙高度与厚度、出生区模式与半径、目标候选数量与间距、安全膨胀
半径（0.6 m）、起飞净空（1.2 m）、规划高度裕度（1.0 m）、栅格分辨率、
`full` 的坐标原点 `x[-100,100]` / `y[-50,50]`、材质与光照。

规则明确但未公布的：随机脚本接口、随机种子来源、正式 world 名称、裁判话题、
六机出生点间距、行人模型与可行走区、正式场景是完整 world / 模型包 / 运行时生成器。

## 6. 仍缺少的正式比赛资源

1. 官方随机脚本接口与种子约定（当前只能自建 seed）。
2. 正式 world 名称、模型包、裁判程序与话题契约。
3. 六机 launch、飞机间最小间距与出生区约束。
4. 行人/目标模型名、可行走区、出生安全距离与瞬移规则的实现细节。
5. 官方场景的碰撞体、材质与高度标准（当前用 box/cylinder 基础几何体替代）。

收到正式资源后必须重新适配与验收：出生点、高度带、栅格分辨率、目标区语义都要
按官方口径重算，不能沿用本生成器的训练约定。
