# 在云服务器上看 Gazebo — 实操指南（2026-09-28 实测通过）

机器：`region-42.seetacloud.com:25316`，用户 `root`，容器名 `autodl-container-70724a9771-175ef45d`

> 这份是我**实际连上去跑通之后**写的，命令、报错、坑都来自真机。截图见 `gazebo_城市全景.png`。

---

## 一、最简流程：三条命令 + 一个浏览器

### 1) 登录
```bash
ssh -p 25316 root@region-42.seetacloud.com
```

### 2) 起仿真（无头，别加 gui:=true）
```bash
setsid nohup bash /root/run_swarm_6uav.sh > /root/_demo_sim.log 2>&1 &
```

### 3) 等 6 架飞机就位
```bash
watch -n2 'grep -c "Successfully spawned entity" /root/_demo_sim.log'
```
计数到 **6** 就可以下一步（软渲染下我实测 15 秒～4 分钟都有，看机器当时忙不忙）。

### 4) 挂 Gazebo 窗口
```bash
bash /root/start_gzclient.sh
```

### 5) 看画面 —— 用 noVNC，不用装 VNC 客户端
在你 **Windows 上另开一个终端**（不要关，关了就断）：
```bash
ssh -p 25316 -L 6080:localhost:6080 -N root@region-42.seetacloud.com
```
然后浏览器打开 **http://localhost:6080/vnc.html** → 点 Connect。密码为空。

服务器上本来就有 `wsproxy.py` 在 6080 提供 noVNC 网页（`/usr/share/novnc`），5900 是 x11vnc 的原始端口。

> 想用 VNC 客户端也行：把上面换成 `-L 5900:localhost:5900`，客户端连 `localhost:5900`。

---

## 二、三个必须做对的点（做错就是"窗口一闪就没了"）

### 1. 必须先有 gzserver，再起 gzclient

gzclient 只是个"看客"，它连不上 gzserver（`11345`）会**等 30 秒然后自己退出，退出码 255**：

```
[Err] [ConnectionManager.cc:121] Failed to connect to master in 30 seconds.
[Err] [gazebo_shared.cc:106]  Unable to initialize transport.
[Err] [gazebo_client.cc:56]    Unable to setup Gazebo
```

我一开始就是先起 gzclient 后起仿真，连着死了两次。**顺序反了就是这个报错。**

### 2. `GAZEBO_MODEL_DATABASE_URI` 必须指向本地

`/root/.bashrc` 里已经有一行修好了，作者自己写了注释：

```bash
# 禁止 Gazebo 联网拉模型（避免 splash 卡 Preparing your world）
export GAZEBO_MODEL_DATABASE_URI="file:///root/.gazebo/models"
```

**为什么你会踩到：** 非交互式 SSH（比如脚本、自动化工具执行命令）**不读 `.bashrc`**，于是这个变量是空的 → gzclient 去 `models.gazebosim.org` 拉模型 → 外网不通 → **卡在橙色 "Preparing your world..." 启动画面不动**，最后崩掉。

**所以：不要手敲 `gzclient`，用 `/root/start_gzclient.sh`**，里面把模型路径、本地模型库、软渲染都设好了。

### 3. `ROBOCUP_WS` 必须设

`multi_uav_sitl.launch` 第 31 行：

```xml
<arg name="ws" default="$(optenv ROBOCUP_WS /home/ros/team_ws/robocup)"/>
```

不设 `ROBOCUP_WS` 就 fallback 到 **`/home/ros/...`**——那是本地 VM 的旧路径，云端不存在，于是：

```
gen_iris_sdf.sh: SDF: /home/ros/team_ws/robocup/...     ← 路径错了
Invalid <param> tag: Cannot load command parameter [sdf_iris_1]
```

`run_swarm_6uav.sh` 里有 `export ROBOCUP_WS=/root/team_ws/robocup`，所以用它启动没事；**你要是手敲 roslaunch，一定要先 export。**

---

## 三、为什么画面那么卡 —— 不是 bug，是 CPU 配额

```bash
cat /sys/fs/cgroup/cpu/cpu.cfs_quota_us   # 1200000
cat /sys/fs/cgroup/cpu/cpu.cfs_period_us  # 100000
```

**配额 = 12 核**（虽然 `nproc` 报 96，那是宿主机的）。实测同时跑 gzserver + 6×PX4 + 6×MAVROS + gzclient：

| 指标 | 实测值 |
|---|---|
| FPS | **1.5 ～ 2.2** |
| Real Time Factor | **0.42 ～ 0.48** |
| gzclient 单进程 CPU | ~1000%（约 10 核） |

gzclient 纯软渲染（`LIBGL_ALWAYS_SOFTWARE=1`，没有 GPU 直通）一个人就要吃掉 10 核，剩下 2 核喂 gzserver + 6 个 PX4。

**由此衍生两个现象，提前知道就不会慌：**

- **鼠标操作会被丢。** 我第一轮猛滚 40 格，3 秒全被丢掉，画面纹丝不动。要**一格一格来，每格间隔 0.9 秒**。
- **`spawn_sdf_model returned no response`**：gzserver 被饿死，spawn 服务超时。日志里会看到 `[spawn_retry] iris_x 3/5`。**这时候别急着敲命令，等 spawn 计数爬到 6**，重试机制会自己搞定。

想让画面顺一点：少起几架机，或者先把 gzclient 关掉让仿真自己跑一轮。

---

## 四、Gazebo 窗口怎么操作

| 操作 | 鼠标 |
|---|---|
| 旋转视角（轨道） | 左键拖动 |
| 平移 | 中键拖动 |
| 缩放 | 右键拖动，或滚轮 |

窗口管理器 `xfwm4` 是跑着的，所以窗口能拖、能最大化。

**注意 xfwm4 不在四件套守护里**，容器重启后它就没了（窗口不能动但画面正常）。补一句：
```bash
DISPLAY=:99 setsid nohup xfwm4 --replace > /root/xfwm4.log 2>&1 &
```

命令行改视角（FPS 只有 2，用 xdotool 更可控）：
```bash
DISPLAY=:99 xdotool search --name "^gazebo$" windowactivate
DISPLAY=:99 xdotool mousemove 700 400 click --repeat 25 --delay 900 5   # 滚轮拉远
```

---

## 五、让窗口一直挂着（推荐）

`/root/guard_vnc_stack.sh` 是**四件套守护**：Xvfb + x11vnc + wsproxy + gzclient，任意一个掉了 10～20 秒内自动拉起，还带 gzclient 卡死检测（运行 >15 分钟且 CPU 持续 >300% 就重启）。

```bash
setsid nohup bash /root/guard_vnc_stack.sh > /root/vnc_guard.out 2>&1 &
tail -f /root/vnc_guard.log     # 看它干了什么
```

**一个副作用要知道**：它会把 `websockify` 杀掉、改用作者自写的 `wsproxy.py`（注释写着"替代有断连缺陷的 websockify"）。两个都监听 6080，**别同时跑**。

---

## 六、看画面的另外两条路（不依赖 VNC）

```bash
# 1) 直接截屏
DISPLAY=:99 scrot -z /root/shot.png

# 2) 从自己电脑拉下来
scp -P 25316 root@region-42.seetacloud.com:/root/shot.png .
```

---

## 七、整栈突然全没了怎么办

我实测遇上过一次：roslaunch 日志刷出一整套 `killing on exit`，gzserver / px4 / mavros / rosmaster 全没，但 Xvfb / x11vnc 活着。**排查顺序：**

```bash
# ① 容器是不是重启过？看 PID 1 的年龄
ps -o pid,etimes,args -p 1
#   etimes 很小（几百秒）→ 容器刚重启
#   注意：`uptime` 显示的是宿主机内核的 uptime（653 天），不代表你的容器

# ② 端口和进程现状
pgrep -c -x gzserver; pgrep -c -x gzclient
python3 -c "import socket;s=socket.socket();print('11345',s.connect_ex(('127.0.0.1',11345)))"

# ③ 清残留锁（不清下一轮起不来，报 PX4 daemon already running）
rm -f /tmp/px4_lock-* /tmp/px4-sock-*
```

重启整条链：`bash /root/clean_view.sh`（清仿真但保留 Xvfb/x11vnc/websockify），然后回到第一节。

彻底停：
```bash
bash /root/stop_all.sh
# 或者
pkill -9 -f gzclient; pkill -9 -f gzserver; pkill -9 -f 'px4_sitl_default/bin/px4'; pkill -9 -f mavros_node
```

---

## 八、想看完整协同（不止 Gazebo 空场景）

按顺序叠：

```bash
bash /root/run_swarm_6uav.sh                 # ① gzserver + 6×PX4 + MAVROS
bash /root/start_official_robocup.sh         # ② 官方地图/actor/裁判 + 两个桥接节点
bash /root/start_swarm_layer.sh              # ③ swarm_manager + 6×swarm_agent
bash /root/start_gzclient.sh                 # ④ Gazebo 窗口
roslaunch robocup_swarm robocup_view.launch  # ⑤ RViz（可选）
```

或者一条命令跑完整一轮（含清场、收摊、日志归档）：
```bash
bash /root/run_full_round.sh
```
> 它会在结束/裁判退出后自动 `stop_all.sh` 收摊 —— **所以 gzclient 也会被一起关掉**，正常现象。

---

## 九、本次实测记录

| 时间 | 事件 |
|---|---|
| 22:14:33 | 容器重启（PID 1 年龄从 0 开始） |
| 22:15:42 | 起仿真 → gzserver=2 / px4=6 正常 |
| 22:16:31 | `guard_vnc_stack.sh` 拉起四件套，端口 5900/6080/11311/11345 全 LISTEN |
| 22:17:00 | **6 架 iris 全部 spawn 成功** |
| 22:19 | 截图：Gazebo 出画面，RTF 0.46 |
| 22:21 | 截图：城市几何（人行道铺装、建筑墙体） |
| ≈22:22 | 整栈被 SIGTERM（`killing on exit`），显示栈存活 |
| 22:27 | 重新起仿真，**15 秒** 6 架就位 |
| 22:31 | 全景截图 `gazebo_城市全景.png` |

**世界名**：`robocup_training_city_full_s7` —— 就是 `training_city_full_s7.world`（38 栋建筑 9.2–17.9m + 184 根 7.5m 灯柱 + 围墙），和我之前做三维视线判定分析用的是同一份地图。
