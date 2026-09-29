# 训练地图 A* 接入与验证（2026-09-05）

## 本阶段交付

`robocup_navigation` 内新增独立于 ROS 的 A* 库，直接读取训练地图 metadata v1 的 RLE 栅格。使用已膨胀的占用格，支持 4/8 邻接；默认 8 邻接且禁止斜穿障碍角。搜索有扩展节点上限，输出确定性的路径或明确失败原因。

当前结果是二维格子中心折线，JSON 标记 `executable: false`。不会启动控制节点、解锁或发布 MAVROS 指令。原有单机控制节点未在本阶段修改。

## 操作命令

在 Windows PowerShell 中进入实际工程目录：

```powershell
Set-Location 'D:\a\.robocup\robocup_ws'
```

Docker 已运行、当前训练地图已记录时，一条命令生成路径：

```powershell
.\scripts\plan_training_route.ps1 -Output /workspace/build/training_city/current_route.json -AsciiOutput /workspace/build/training_city/current_route.txt
```

查看路径示意（`S` 起点、`G` 终点、`*` 路径、`#` 膨胀后的障碍，上方为 +y）：

```powershell
Get-Content .\build\training_city\current_route.txt
```

若尚未生成/启动训练场景，在无人机已落地、未解锁且控制任务已退出时执行：

```powershell
.\scripts\start_training_city.ps1 -Preset unit -Seed 42 -Category single_wall
```

该现有启动脚本会重建仿真容器，不能在飞行中切图。A* 命令本身只读 metadata，不要求 Gazebo 正在执行同一地图，也不会验证地图与当前场景一致；当前验收已另行核实场景模型。

测试与 ROS 包增量编译：

```powershell
.\scripts\run_astar_tests.ps1
docker exec robocup-single-uav bash /workspace/scripts/container/check_astar_package.sh
```

PowerShell 的 `-Metadata`、`-Output`、`-AsciiOutput` 使用容器路径 `/workspace/...`，对应本机工程目录。`-GoalId goal_0000` 可选择候选点。不指定 metadata 时读取 `build/training_city/current_metadata.txt`。

Python CLI 还提供成对的 `--start-x/--start-y`、`--goal-x/--goal-y`，供后续规划目标接口使用。坐标覆盖仍检查占用和越界；室外标记来自 metadata，不是感知验证。输入/输出文件路径不能重合。

## 契约与错误处理

- schema 为 `robocup_training_worlds/metadata/v1`，frame 必须 map/ENU/米；RLE 严格限制整数 0/1、长度和尺寸。
- metadata 的 `valid` 包含生成器 BFS 结果，不能用于跳过 A*。候选点先检查 `outdoors` 和 `inside_obstacle`，连通性由 A* 自行判断。
- `config/astar_planner.yaml` 控制邻接方式、禁止穿角、扩展上限及室外目标筛选。CLI 拒绝开启穿角。
- Python 退出码：0 成功，4 搜索失败（如 NO_PATH、端点占用/越界、EXPANSION_LIMIT），2 输入或输出错误。PowerShell 对非零退出码报错。
- JSON 包含输入 SHA-256、请求坐标、格子路径、二维世界坐标、路径长度、扩展数量、膨胀量和地图规划高度。路径端点为格子中心，可能与请求点相差半格；长度不包含请求点到格子中心的接线。
- 解析失败时尽可能覆盖结果为失败且清空路径，避免误用旧成功文件；无法写入时返回错误。任何消费者都必须检查本次退出码、success、地图摘要与时效，不能仅因文件存在就执行。
- JSON 原子替换；ASCII 为辅助显示文件。大于 120×120 的地图不支持 ASCII，请省略该参数。

## 实际验证记录（Asia/Shanghai）

1. 本次前段已在 Ubuntu 20.04 / Python 3.8 容器中运行训练生成器原有 38 项测试：31.530 秒，全部通过；约 19:30 启动 single_wall、`gz sdf --check` 返回 Check complete。存在 SDF version 属性兼容警告，未阻止场景加载。
2. 21:39 续作时 Engine 管道不存在，Docker 启动再次因 sailor-ingest.sock 失败。备份运行目录后恢复，详见 Docker 故障记录。
3. 21:41:48，经 Windows PowerShell 5.1 的 `run_astar_tests.ps1` 在现有容器中运行：14 项测试，2.759 秒，全部通过。覆盖 single_wall、wall_with_gap、u_shape、narrow_corridor、blocked、no_path、障碍内目标，以及禁止穿角、越界、占用、RLE、坐标约定、扩展预算、确定性、已知最短距离和 CLI 成功/失败输出。
4. `check_astar_package.sh`：独立 navigation catkin workspace 中 1 个包成功，编译总计 12.5 秒，无警告/失败；`rosrun robocup_navigation plan_training_route.py --help` 成功。未重建 PX4。
5. PowerShell 路径命令退出 0：`success=true, reason=SUCCEEDED, goal_id=goal_0000, path_length_m=6.165685, expanded_nodes=31`。
6. 输入 `training_city_unit_single_wall_s42.json`，SHA-256：`7d9f5503770954225f050716b3e1543d83635684d65d45735986c13b9f64a2b9`。50×50 栅格，分辨率 0.2 m，起点 [-3,0]、终点 [3,0]。路径绕过墙北端，未穿占用格或切角。不能把这一栅格结论视作机体连续碰撞验证。
7. 21:42:36 五项健康检查通过：ROS master、iris 模型、clock、MAVROS connected、local pose。
8. 随后 `/gazebo/get_world_properties` 返回 training_ground、四边界、unit_wall_0000、iris，success=True；MAVROS connected=True、armed=False、mode=AUTO.LOITER；节点仅 /gazebo、/mavros、/rosout。本次检查 headless 场景，没有验收 GUI 显示或绕障飞行。

Windows 附带的 Python 缺少 PyYAML，本地尝试未运行测试；有效的最终测试结果来自项目 Ubuntu 容器，不需要为此重装环境。

## 文件清单

- `src/robocup_navigation/src/robocup_navigation/{__init__,astar}.py`：可复用规划库。
- `src/robocup_navigation/setup.py`、`CMakeLists.txt`：catkin Python 模块与 CLI 注册。
- `src/robocup_navigation/scripts/plan_training_route.py`：离线命令与 JSON/ASCII 输出。
- `src/robocup_navigation/config/astar_planner.yaml`：配置。
- `tests/test_astar_planner.py`：14 项回归测试。
- `scripts/plan_training_route.ps1`、`scripts/run_astar_tests.ps1`：Windows 入口。
- `scripts/container/run_astar_tests.sh`、`check_astar_package.sh`：容器测试和无飞行编译检查。
- `README.md`、本记录、Docker 故障记录：使用说明与证据。
- 生成物：`build/training_city/current_route.json` 和 `current_route.txt`；早期调试结果 `debug_route.json/txt` 不作为最终结果。

## 下一阶段尚需完成

将路径接入已有唯一控制节点前，要明确 Gazebo world 与 MAVROS local 的原点转换（同名 map 不代表同原点），并校验整条路径的围栏、飞行高度及连续机体净空。当前出生点到目标为 6 m，超过现有相对起点 5 m 半径围栏；地图规划高度 2.5 m 与原航点演示 1.5 m 也需统一。不能直接把二维折线填入原航点 YAML 就起飞。

需要补充路径跟踪、取消/失败受控降落的集成验证，以及机体/定位/跟踪误差安全余量；原始栅格以格心采样，不能自动保证所有连续线段的足够净空。动态避障、YOLO 目标接入和六机均未在本阶段实现。生成器的官方地图/裁判接口缺项仍按其交付文档处理。
