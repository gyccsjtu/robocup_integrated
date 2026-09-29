# 实机复盘：uav_3 坠机、ORCA 求解恒无解、高度护栏自相矛盾

抓取时间：2026-09-28 22:39–23:15　|　环境：云服务器 `region-42.seetacloud.com:25316`
世界：`robocup_training_city_full_s7`（200 m × 100 m，38 栋楼 9.8–17.9 m + 184 灯柱 7.5 m）
栈：PX4 SITL ×6 + Gazebo 11 + ROS Noetic，`swarm_manager` ×1 + `swarm_agent` ×6
CPU 配额：12 核（`cfs_quota 1200000 / period 100000`），内存 40 G

---

## 0. 本轮总体状态

| 项 | 值 |
|---|---|
| 仿真时钟 | 1089 s（RTF 0.40） |
| 协同层 | 1 个 manager + 6 个 agent 全活；A* 每架已重规划 15–23 次 |
| 有效在飞 | **5 / 6** |
| ORCA 无解告警 | **358 次**（agent_2 142、agent_3 116、agent_4 87、agent_6 13、agent_1/5 = 0） |
| 高度层 | 2.8 / 3.4 / 4.0 / 4.6（实测落位 2.47–4.08，uav_4 被硬顶压在 4.2） |

协同层确实是活的：管理器按格发租约、飞机散开到全场、A* 正常绕障。
**唯一的硬伤是 uav_3 已经坠机停摆，而系统对此毫无察觉。**

---

## 1. 结论一：uav_3 已坠机，且被"冻"在 0.24 m

### 证据链

| 观测 | 数值 | 判读 |
|---|---|---|
| pitch | **−75.0°**（全程恒定） | 机头朝下摔在地上（其余 5 架在 −0.1°…+5.0°） |
| roll | −3.1° | 正常 |
| 世界坐标 | (−41.490, −29.932, 0.241) | 与 20 min 前**完全一致** |
| 20 s 水平位移 | 0.10 m | 完全静止 |
| 所在位置 | 空地，最近建筑 6.5 m | **不是撞楼** |
| 下发 `vz` | **+1.00 m/s 恒定（25 s 内 209 帧一次没变）** | 爬升指令已饱和 |
| 实测 `z` | 0.23 → 0.30 m | 指令无效，升不起来 |
| `armed` / `mode` | `True` / `OFFBOARD` | 仍处于受控状态 |
| `landed_state` | 2（IN_AIR，15 s 内恒定） | PX4 认为它在空中 |
| `MPC_Z_VEL_MAX_UP` | 3.0（uav_1 同为 3.0） | 参数无异常 |
| `cmd_vel` 发布者 | 仅 `/swarm_agent_3` 一个 | 无第二发布者抢控制权 |

**判读**：机体已经摔在地上、机头插地，所以任何爬升指令都无效；
而 agent 侧完全不知情，仍在以 20 Hz 发 +1.0 m/s 爬升、仍在跑 A*（最近几次规划 26–129 航点），
管理器也仍在给它派格子。**算力全打在一条死链路上。**

### 时间线（这是最关键的一条）

```
22:39:57  uav_3 进入 OFFBOARD（高度层 4.0 m），此后从未重入（仅一次 arming）
22:42:16  ┐
   ...    │ 连续 116 条「ORCA 无解，强制排斥」（约 20 s 内）
22:42:36  ┘
22:42:40  uav_3 高度 4.06 m → 0.13 m（一个 20 s 采样窗内掉 4 m）
此后     一直在 0.17–0.30 m，pitch −75°，再未恢复
```

坠机与 ORCA 爆发在时间上咬合，而 ORCA 的失效是**可证明必然**的（见下节）。

---

## 2. 结论二：ORCA 的半平面求解是死代码（可证明）

`swarm_agent._apply_friend_avoidance()`：

```python
dz  = abs(my_alt - friend_alt)
d3  = math.hypot(dist, dz)
if d3 > SAFE_3D:            # SAFE_3D = 4.0
    continue                # ← 只在 d3 ≤ 4.0 时才建半平面
...
sep_vel = max(0.0, (FRIEND_SAFE_DIST - d3)) / _div   # FRIEND_SAFE_DIST = 10.0, _div = SEP_GAIN = 1.0
orca_halfplanes.append((nx, ny, sep_vel))
...
for angle in _angles:
    for speed in [MIN_SPEED, MAX_SPEED*0.5, MAX_SPEED*0.7, MAX_SPEED]:   # MAX_SPEED = 5.0
        valid = all(cand·n >= min_vel - 0.1 for ...)
```

约束要求候选速度沿分离法向的**投影** ≥ `sep_vel − 0.1`。
投影的数学上限就是候选速度的模长，即 `MAX_SPEED = 5.0`。而

```
进入求解的条件：d3 ≤ 4.0
此时       sep_vel = 10 − d3 ≥ 6.0 > 5.0 − 0.1
```

| d₃ (m) | sep_vel | 存在可行候选？ |
|---|---|---|
| 0.5 | 9.50 | 否 |
| 2.0 | 8.00 | 否 |
| 3.0 | 7.00 | 否 |
| 4.0 | 6.00 | 否 |
| 5.0 | 5.00 | 是（但 d₃ > SAFE_3D，根本不进这段） |

**只要进入求解分支，必然无解。** 全向采样、16 个方向、4 档速度——全部白写。
每次近距离遭遇 100% 退化到下面的裸排斥：

```python
if best_score == -inf:
    rospy.logwarn("ORCA 无解，强制排斥")
    for ...:
        force = FRIEND_K * (FRIEND_SAFE_DIST - dist) / dist   # FRIEND_K = 2.0
        best_vx += (wx - fx) * force      # 纯径向、XY 平面、最高 5 m/s、无姿态耦合
```

这解释了 6 架里 4 架中招、累计 358 次的分布——它不是偶发，是设计上的必然。

### 附带发现：另一处区间无人避让

`has_conflict` 用**水平**距离（0.5 < dist < 10）判断，
但真正生效的半平面只在 `d3 ≤ 4.0` 时才建。
于是 **3D 距离落在 4.0–10 m 的友机：不产生任何避让转向**，
唯一效果是 `_adaptive_speed` 把巡航速度从 5.0 压到 `SLOW_SPEED = 3.0`
（门限 `SLOWDOWN_DIST = 14.0`）。这是个 6 m 宽的"避让真空带"。

---

## 3. 结论三：高度常量三处自相矛盾

```python
ALT_BASE    = 2.8    ALT_STEP = 0.6    ALT_NLAYER = 4    # → 层 2.8/3.4/4.0/4.6
ALT_CEILING     = 4.6
ALT_HARD_CEIL   = 4.2     # z > 4.2 → 无条件 cmd.z = -1.0
ALT_PANIC       = 5.0     # z > 5.0 → cmd.z = -1.6
ALT_EMERG_CEIL  = 5.0     # z > 5.0 → cmd.z = -3.0（无条件覆盖上一档）
ALT_TARGET_CAP  = 4.5     # target_alt = max(2.0, min(4.5, base_alt + offset))
```

1. **目标 4.5 > 硬顶 4.2，而最高层是 4.6**：第 4 层的 UAV 被永久夹在
   "P 控制想上 4.5" 和 "硬护栏只要 > 4.2 就压 −1.0 m/s" 之间互搏。
   实测 uav_4 被钉在 **4.03–4.22 m**，1138 个采样里 195 个越过 4.2 —— 典型限环。
2. **`ALT_PANIC == ALT_EMERG_CEIL == 5.0`**：两档阈值相等，
   后面那档无条件覆盖前面，所以 **PANIC 那一档永远执行不到**（死代码）。
3. **`_div = 2.0 if d3 > SAFE_3D else SEP_GAIN` 是死分支**：
   上方已经 `if d3 > SAFE_3D: continue`，这里 `d3 > SAFE_3D` 永假。

---

## 4. 结论四：可观测性缺口（这是掉高时日志一片干净的原因）

三道高度护栏都是**直接覆写** `cmd.twist.linear.z`，一行日志都不打：

```python
if self.local_z > ALT_HARD_CEIL:   cmd.twist.linear.z = ALT_HARD_DESCENT
if self.local_z > ALT_PANIC:       cmd.twist.linear.z = ALT_PANIC_DESCENT
if self.local_z > ALT_EMERG_CEIL:  cmd.twist.linear.z = ALT_EMERG_DESCENT
```

所以事后查 agent 日志，掉高那一分钟**只有 A* 规划记录，没有任何告警**。
同时也没有"姿态异常 / 已坠机"的判定，导致一个已经摔死的机体
可以带着 20 Hz 的指令流和 A* 规划器一直空转到任务结束。

---

## 5. 建议补丁（未改动线上运行，等你确认）

### P0-1　让 ORCA 求解变可行（一行）

```python
# 原
sep_vel = max(0.0, (FRIEND_SAFE_DIST - d3)) / _div
# 改：把要求的分离速度夹进候选速度能达到的范围
sep_vel = min(MAX_SPEED * 0.85, max(0.0, FRIEND_SAFE_DIST - d3) / _div)
```

`sep_vel ≤ 4.25`，沿法向的 5.0 m/s 候选即可满足，LP 立刻有解，
全向采样那 16×4 个候选才真正开始起作用。

### P0-2　消除避让真空带

把建半平面的门限从 `SAFE_3D` 放宽到与 `has_conflict` 一致，
或显式增加一档"远距预警"（只降速、不转向）：

```python
if d3 > SAFE_3D:
    if d3 < FRIEND_SAFE_DIST:            # 4.0–10 m：降速但不减速为零
        orca_halfplanes.append((nx, ny, min(MAX_SPEED*0.5, ...)))
    continue
```

### P1-1　高度常量对齐

```python
ALT_TARGET_CAP = 4.2     # 不得超过 ALT_HARD_CEIL
ALT_PANIC      = 4.8     # 与 ALT_EMERG_CEIL 拉开档次
ALT_EMERG_CEIL = 5.4
# 或保留阈值、把层数改 3：ALT_NLAYER = 3  → 2.8/3.4/4.0，顶层不再高于硬顶
```

### P1-2　护栏加日志 + 加下界保护

```python
if self.local_z > ALT_HARD_CEIL:
    cmd.twist.linear.z = ALT_HARD_DESCENT
    rospy.logwarn_throttle(2.0, "[%s] 触发硬顶降高 z=%.2f", self.uav_id, self.local_z)
...
# 新增：低空异常告警（掉出搜索层）
if self.local_z is not None and self.local_z < MIN_CRUISE_ALT * 0.5:
    rospy.logwarn_throttle(5.0, "[%s] 高度异常偏低 z=%.2f", self.uav_id, self.local_z)
```

### P1-3　坠机检测，并把死掉的机体从任务里摘出去

```python
# 姿态异常 → 判失能，停止规划、停止上报有效状态
if abs(self._pitch_deg) > 40 or abs(self._roll_deg) > 40:
    if not self._disabled:
        rospy.logerr("[%s] 姿态异常（pitch=%.0f°），判定失能", self.uav_id, self._pitch_deg)
        self._disabled = True
    self._send_vel(0.0, 0.0, vz=0.0)
    return
```

同时在 `swarm_manager` 侧：收到 `disabled` 的机体，立刻释放其租约、
把已分配的格子重新投放。否则就是本轮的情况——
管理器给一台躺在地上的飞机派格子，派到任务结束。

---

## 6. 复现 / 验证方式

本轮所有结论都由只读观测得到，未修改任何官方文件（地图、actor、裁判）。

```bash
# 1) ORCA 恒无解的静态证明
python3 - <<'PY'
FRIEND_SAFE_DIST, MAX_SPEED, SAFE_3D, SEP_GAIN = 10.0, 5.0, 4.0, 1.0
print([ (d3, max(0,(FRIEND_SAFE_DIST-d3))/SEP_GAIN, MAX_SPEED >= max(0,(FRIEND_SAFE_DIST-d3))/SEP_GAIN - 0.1)
        for d3 in (0.5,1,2,3,4.0,4.5,5.0) ])
PY

# 2) 逐机姿态 + 高度快照（判坠机）
#    /gazebo/model_states 取真值位置
#    /uav_N/mavros/local_position/pose 取姿态四元数 → roll/pitch
#    |pitch| > 40° 视为坠机

# 3) 指令是否与实际一致（判"拉不起来"）
rostopic echo -n 1 /uav_3/mavros/setpoint_velocity/cmd_vel    # 看线性 z
rostopic echo -n 1 /uav_3/mavros/local_position/pose          # 看实际 z
rostopic echo -n 1 /uav_3/mavros/state                        # armed / mode
rosservice call /uav_3/mavros/param/get "param_id: 'MPC_Z_VEL_MAX_UP'"

# 4) 静态常量体检
grep -nE 'ALT_(TARGET_CAP|HARD_CEIL|PANIC|EMERG_CEIL|BASE|STEP|NLAYER)|FRIEND_SAFE_DIST|MAX_SPEED|SAFE_3D' \
  swarm_agent.py
```

---

## 7. 与本项目主线的关系

前面几轮修的是**判定层**（LOS 补高度维、ROUTE_OFFER 膨胀栅格）——
那些是"算错了"。这一轮暴露的是**执行层**：
避让求解器的量纲/幅值不自洽，高度护栏之间互相打架，
而且护栏动作完全静默、失能机体的信息不回流到调度器。

共同点还是那个老模式：**局部看都对，接起来不闭合。**

---

*本报告只做只读观测，未改动服务器上任何运行中的代码。*
