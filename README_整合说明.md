# RoboCup 协同仿真代码整合包

生成时间：2026-09-28 23:35
面向：同步到本地虚拟机（ros-virtual-machine）

---

## 一、这个包是怎么来的

代码原本散在三处，版本还不一致。整合时先做了逐文件 md5 比对，结论是：

| 来源 | 角色 | 状态 |
|---|---|---|
| 云服务器 `/root/team_ws/robocup` | **权威基线** | 最新版，能实跑，含 ROUTE_OFFER 净空修复 |
| 本地桌面 `协同仿真代码_2026-09-28.zip` | 归档 | **比服务器旧** swarm_agent 49.8K vs 67.5K |
| 本地 `.workbuddy` 散件 | 工具/验证脚本 | 本轮新增的分析与验证工具 |

所以本包**以服务器版本为基线**，而不是以桌面 zip 为基线 —— 否则会用旧代码盖掉新代码。

桌面 zip 里独有的东西（LOS3D 相关、验证脚本）单独抽出来放进 `_extra/`，
**不直接覆盖**服务器新版的核心文件。

---

## 二、目录结构

```
robocup_integrated/
├── robocup/                     ← 直接覆盖到 VM 的 catkin 工作区
│   ├── src/
│   │   ├── robocup_swarm/        核心多机协同（agent/manager/task/规划）
│   │   ├── robocup_navigation/   含 coordination_executor.py（ROUTE_OFFER）
│   │   ├── robocup_training_worlds/
│   │   ├── robocup_environment_baseline/
│   │   └── px4pkg/
│   ├── scripts/                  vm/cloud/container 部署与工具脚本
│   ├── config/ docs/ tests/ launch/ examples/
│   └── mission_time.py observer_assign.py README.md
├── _extra/                      ← 本地独有，不进 ROS 路径
│   ├── los3d/swarm_heightfield.py    三维遮挡高度场
│   ├── verify/                       test_los3d / scan_visibility / verify_route_offer
│   └── tools/                        patch_executor / make_situ
├── _docs/                       ← 本轮全部文档
├── _ops/                        ← start_swarm_layer.sh / actor_watchdog.py 等
├── _symlinks.txt                符号链接清单（落地时重建）
├── MANIFEST.tsv                 327 个文件的 path/size/md5
├── sync_to_vm.sh                在虚拟机上执行的落地脚本
└── README_整合说明.md
```

---

## 三、两份关键差异（必读）

### 1. `coordination_executor.py` 在 navigation 包，不在 swarm 包

路径：`src/robocup_navigation/scripts/coordination_executor.py`（610 行）
之前本地整合时漏了它 —— 因为它不在 `robocup_swarm` 里。本包已包含。

### 2. 三维遮挡（LOS3D）**没有**落到主线

`swarm_heightfield.py` 只在本地旧版上做过，服务器新版 `swarm_task.py`
的 `visible()` 仍然是纯二维：

```python
def visible(self, ox, oy, tx, ty):   # 服务器新版 —— 无 oz/tz
```

特征串 `HeightField` / `ROBOCUP_LOS_3D` 在服务器 `/root/team_ws/robocup/src`
下**零命中**。

所以本包把 `swarm_heightfield.py` 放在 `_extra/los3d/` **备用**，
没有改主线 `swarm_task.py`。原因：本地那套 LOS3D 是基于**旧版**
swarm_task.py（37.8K）做的，直接盖到新版（42.2K）上会丢功能。

要真正启用三维判定，需要把改动重新移植到新版，涉及：
- `swarm_task.py`：`visible()` 加 `oz/tz` 参数 + `_visible_3d()`
- `swarm_agent.py`：三处调用点传入 `local_z` / 目标高度
- 新增 `swarm_heightfield.py` 到 scripts 目录

---

## 四、怎么落地到虚拟机

```bash
# 1. 把整个 robocup_integrated/ 目录放到虚拟机上（scp / 共享文件夹 / U 盘）
# 2. 在虚拟机里执行
bash sync_to_vm.sh                     # 默认落到 $HOME/team_ws/robocup
# 或指定路径
bash sync_to_vm.sh /home/ros/team_ws/robocup
```

脚本会：
1. 把现有工作区整体备份成 `<目标>_bak_时间戳`
2. 覆盖 `src/ scripts/ config/ docs/ tests/` 等
3. 重建 catkin 顶层 `CMakeLists.txt` 符号链接
4. 校验 6 个关键文件并打印结果

回滚：`rm -rf <目标> && mv <目标>_bak_时间戳 <目标>`

---

## 五、落地后

```bash
cd ~/team_ws/robocup/.. && catkin_make
source devel/setup.bash
```

跑之前注意（本轮实机验证得到的结论）：
- **gzclient 吃掉约 10.5 / 12 核**，是 RTF 掉到 0.40 的主因。只要数据就别开 GUI。
- 非交互 SSH 不 source `~/.bashrc`，`ROBOCUP_WS` 要显式 export。
- 起协同层：`bash /root/start_swarm_layer.sh false`

---

## 六、已知待修问题（本轮实机暴露，均未改线上代码）

| 编号 | 问题 | 影响 |
|---|---|---|
| P0-1 | ORCA 可证明无解：`sep_vel ≥ 6.0` 但候选速度上限 `MAX_SPEED = 5.0` | uav_3 坠机直接起因，共 358 次 |
| P0-2 | 3D 距离 4.0–10 m 无转向（避让真空带） | 接近时只降速不避让 |
| P1-1 | `ALT_TARGET_CAP 4.5 > ALT_HARD_CEIL 4.2` 且顶层目标 4.6 | uav_4 高度极限环 |
| P1-2 | 三处高度护栏静默覆写 `cmd.twist.linear.z`，无日志 | 出事只能靠抓包反推 |
| P1-3 | 无姿态/坠机检测，坠机后 `landed_state` 仍报 IN_AIR | 系统完全不知情 |

详见 `_docs/实机复盘_ORCA与高度护栏.md`。
