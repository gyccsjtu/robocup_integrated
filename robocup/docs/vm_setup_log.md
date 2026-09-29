# VMware Ubuntu 20.04 环境搭建记录 (RoboCup 2026)

> 本文件按项目要求逐步记录：实际命令、版本、验证结果、失败原因。
> 工程根目录：`D:\a\.robocup\robocup_ws`
> 目标 VM：`D:\VM\RoboCup-Ubuntu20`（RoboCup-Ubuntu20）

## 0. 环境基线（2026-09-07 检查）

| 项 | 值 | 来源/命令 |
|---|---|---|
| 宿主 OS | Windows 11 家庭中文版 10.0.26200 | version_manifest.md |
| CPU | i7-13650HX | 用户提供 |
| 内存 | 16 GB | 用户提供 |
| GPU | RTX 4060 Laptop GPU 8 GiB | version_manifest.md |
| D 盘 | 475 GB 总 / 195 GB 已用 / **280 GB 可用** | `df -h /d` |
| VMware | VMware Workstation Pro（`vmware.exe` 2026-04-24，26H1） | `C:\Program Files\VMware\VMware Workstation\vmware.exe` |
| VMware CLI | `vmrun.exe`、`vmware-vdiskmanager.exe` 均存在 | `ls "...\VMware Workstation\"` |
| D:\VM | **不存在**（待创建） | `ls /d/VM` |

## 1. Ubuntu 20.04.6 Desktop ISO 完整性核验

- 路径：`D:\a\.robocup\installers\ubuntu-20.04.6-desktop-amd64.iso`
- 本地大小：`stat` = **4,351,463,424 字节**
- 官方总量：对 `https://releases.ubuntu.com/20.04.6/...iso` 发 `Range: bytes 0-0` 请求，
  返回 `Content-Range: bytes 0-0/4351463424` → 官方总大小同为 **4,351,463,424 字节**，**大小一致**。
- 本地 SHA-256（`certutil -hashfile ... SHA256`）：
  `510ce77afcb9537f198bc7daa0e5b503b6e67aaed68146943c231baeaab94df1`
- 官方 SHA-256（releases.ubuntu.com 与 mirrors.tuna.tsinghua.edu.cn 两份 SHA256SUMS 一致）：
  `510ce77afcb9537f198bc7daa0e5b503b6e67aaed68146943c231baeaab94df1`
- **结论：ISO 完整且来源真实，无需续传。** ✅

## 2. 创建虚拟机（RoboCup-Ubuntu20）

- 位置：`D:\VM\RoboCup-Ubuntu20`
- 配置：Ubuntu 20.04 64 位；6 vCPU；8 GB RAM；80 GB 动态分配磁盘；NAT；3D 加速开启
- 实现脚本：`scripts/vm/create_vm.sh`（调用 `vmware-vdiskmanager.exe` 建盘 + 写入 `.vmx`）
- 产物：`D:\VM\RoboCup-Ubuntu20\RoboCup-Ubuntu20.vmx` + `.vmdk`（11 MB 动态盘，上限 80 GB）
- 配置核验：memsize=8192 / numvcpus=6 / guestOS=ubuntu-64 / NAT / mks.enable3d=TRUE / ISO 已挂载
- 状态：✅ 已创建（仅磁盘+配置，未安装系统）

## 3. 安装 Ubuntu 20.04（半自动 preseed，已确认方式）

- preseed 文件：`scripts/vm/preseed_ubuntu20.cfg`（用户 `robocup`，自动装 `openssh-server` + `open-vm-tools-desktop`）
- 分发方式：宿主机起本地 HTTP 服务，安装器通过网络拉取（避免改官方 ISO）
  - VMware NAT 主机IP（VMnet8）：**192.168.x.x**
  - 服务目录：`D:\a\.robocup\robocup_ws\scripts\vm\`
  - preseed URL：`http://192.168.x.x:8080/preseed_ubuntu20.cfg`（已验证 HTTP 200）
  - 启动命令：`cd scripts/vm && python -m http.server 8080 --bind 0.0.0.0`（后台运行，会话存活期间有效）
- **首次启动 GRUB 编辑**（在 VMware 控制台对 "Install Ubuntu" 条目的 `linux` 行，于结尾 `---` 之前追加）：
  ```
  automatic-ubiquity url=http://192.168.x.x:8080/preseed_ubuntu20.cfg
  ```
  然后 Ctrl+X / F10 启动。之后全程无人值守安装 + 自动建账户 + late_command 装 SSH/VMware Tools。
- ⚠️ 已知风险：Windows 防火墙可能拦截 VMnet8 入站 8080；若安装器拉不到 preseed 而卡在交互，需放行（见下方「待办/确认」）。
- 状态：preseed 服务已起；等待你在 VMware 控制台完成首次 GRUB 编辑并让安装跑完。

## 4. 安装 open-vm-tools-desktop

- 已并入 preseed `late_command`；安装后另行 `vmware-toolbox-cmd -v` 验证。

## 5. 原生安装 ROS/Gazebo/PX4/MAVROS/XTDrone

- 脚本：`scripts/vm/bootstrap_ubuntu20.sh`（已存在，pin 了 PX4 `46a12a09…`、XTDrone `62339a81…`）
- 参考：`docker/Dockerfile.single-uav` 的 apt 包清单 + `deps/requirements-px4-1.13.txt`
- 状态：待步骤 3 完成。

## 6. 工程克隆进 Linux 文件系统

- 目标：`~/robocup/robocup_ws`（VM 内 ext4，**不**在 /mnt/d 或共享目录编译）
- 状态：待步骤 3 完成。

## 7. 最小目标验证

- `roscore` / `gazebo` / `px4` SITL / `mavros` connected / iris 单机起飞
- 启动脚本：`scripts/vm/start_single_uav_native.sh`（已存在）
- 状态：待。

## 8. 快照

- `01-ubuntu-clean` / `02-ros-px4-ready` / `03-robocup-baseline`（关机或无人机在地面时拍摄）
- 状态：待。

---
## 执行记录（按时间追加）

- 2026-09-07 15:1x  ISO 核验通过（大小 + SHA-256 与官方一致）。
- 2026-09-07 15:17  执行 `scripts/vm/create_vm.sh`：创建 `D:\VM\RoboCup-Ubuntu20\`（80GB 动态盘 + .vmx，6 vCPU/8GB/NAT/3D/挂载 ISO）。VM 定义完成。
- 2026-09-07 15:25  用户首次启动 VM；GRUB 编辑（preseed 自动安装）对用户不友好（反馈"没看懂"），同时 WSL2 被沙箱安全策略拦截、无法用 xorriso 改造 ISO，故放弃半自动 preseed 路径，**回退到 Ubuntu 桌面 GUI 手动安装**。
  - 保留产物：preseed 文件 + HTTP 服务（`http://192.168.x.x:8080/p.cfg`，含短名 `p.cfg` 别名）暂不清理，作为可复用资源。
  - 改用引导用户点击「Install Ubuntu」向导：用户名 `robocup`、密码 `******`、主机名 `robocup-ubuntu-20`、取消勾选"下载更新/第三方软件"以加速。
  - 装完提示「Please remove the installation medium」时，需先由我通过 `vmrun` 断开 CDROM，再让用户按 Enter 进系统。

- 2026-09-07 15:44  安装完成验证：VMDK 9.3GB；`vmrun getGuestIPAddress` = 192.168.x.x；VMware Tools 回报 `Ubuntu 20.04.6 LTS` / `kernel 5.15.0-139-generic` → 确认已从虚拟硬盘启动装好的系统（非安装盘）。
- `vmrun runProgramInGuest` 用 robocup/****** 认证失败（未登录桌面或凭据不匹配）→ 放弃 vmrun 直驱，改走 SSH 密钥方案。
- 已在主机生成 ed25519 密钥 `D:/a/.robocup/tmp/host_ssh/id_ed25519`（公钥 robocup-host）；并把 .vmx 光驱改为空（`ide1:0.startConnected=FALSE`，下次重启不再从 ISO 启动）。VM 当前仍正常运行。
- 待：用户在 VM 终端粘贴一次性命令（切 tuna 源 + 装 openssh-server/open-vm-tools-desktop + 写入主机公钥 + 打印 IP），然后我 SSH 进去做 ROS/Gazebo/PX4/MAVROS/XTDrone 安装与验证。

## 2026-09-07 16:0x  SSH 接管 + 工程入 VM + 修正 bootstrap 并启动安装
- 用户提供 VM 密码授权操作；主机无 sshpass，改用 Python paramiko 以密码登入，把主机 ed25519 公钥写入 VM `linfen@192.168.x.x:~/.ssh/authorized_keys` → 之后免密 SSH 成功。
- 在 VM 为 `linfen` 开启 NOPASSWD sudo（仅本 VM，便于无人值守安装）；apt 源替换为清华镜像并 `apt-get update` 成功。
- 将 `robocup_ws` 打包（444K，排除 `third_party`/`build`/`logs`）scp 进 VM，解包到 `~/robocup/robocup_ws`（不在 /mnt/d 编译）。
- 首次运行 `scripts/vm/bootstrap_ubuntu20.sh` 暴露两处脚本 bug：① 在添加 ROS 软件源前就 `apt-get install gazebo11`（源里没有）；② 用 curl 取 ROS 密钥时 curl 尚未安装。已修正脚本：先装 `curl`/`ca-certificates`、先把 ROS 源加入再安装依赖。重跑已进入安装阶段。
- 当前：`bootstrap_ubuntu20.sh` 在 VM 后台执行（ROS Noetic + Gazebo11 + MAVROS + 拉 PX4 `46a12a09`/XTDrone `62339a81` + 编译），由 `tmp/monitor_bootstrap.sh` 每 60s 探活，结束自动通知。
- 待：安装完成后验证最小目标（roscore / Gazebo / PX4 SITL / MAVROS connected / iris 单机起飞），再拍快照 01-ubuntu-clean / 02-ros-px4-ready / 03-robocup-baseline（快照需再确认）。

## 2026-09-07 16:5x  绕开 VM 网络封锁：直接复用主机已有的 PX4 / XTDrone 源码
- 诊断 VM 出网能力：`github.com` / `raw.githubusercontent.com` / `pypi.org` **全部不通**；可达的只有 `mirrors.tuna.tsinghua.edu.cn`（含 ros/ubuntu、rosdistro、pypi）、`gitee.com`、`sourceforge.net`。原脚本在 VM 内 `git clone` PX4 的方案不可行。
- 改为**主机直传**：`D:/a/.robocup/robocup_ws/third_party` 下已有 PX4 `46a12a09`（19 个子模块全部已初始化）与 XTDrone `62339a81`，即项目固定版本。打包（排除 PX4 的 1G build 产物）得 `px4.tar` 1.49 GB + `xtdrone.tar` 2.45 GB，经 scp 传入 VM 并解包到 `~/robocup/robocup_ws/third_party/`。
- `scripts/vm/bootstrap_ubuntu20.sh` 改造为离线友好版：ROS 源指向清华、签名密钥优先使用本地文件（`ROS_KEY_FILE`，由主机下载后传入）、pip 走清华 pypi、rosdep 走清华 rosdistro 索引、PX4/XTDrone 在 HEAD 已匹配固定 commit 时跳过 fetch/checkout/子模块更新、GeographicLib 与 rosdep 失败仅告警不中断。
- 两处实施坑：① Git Bash 的 `tar` 目标路径写成 `D:/...` 会被当作远程主机（`Cannot connect to D: resolve failed`），须用 `/d/...`；② 上一轮失败的 `curl | gpg` 遗留了 0 字节的 `ros-archive-keyring.gpg`，使脚本误判"密钥已存在"而跳过，已改为按文件非空判断并先删除空文件。
- 当前：`bootstrap_ubuntu20.sh` 在 VM 后台执行（正从清华源下载 ROS 相关包），由 `tmp/monitor_bootstrap.sh` 每 30 秒探活，最长 90 分钟。

## 2026-09-07 17:1x-17:4x  排障记录
1. **卡在 GeographicLib 数据集**：MAVROS 的 `install_geographiclib_datasets.sh` 从 sourceforge 下载，实测 680 B/s（wget 空转 14 分钟）。改为在**主机经代理**下载 `geoids/egm96-5`、`gravity/egm96`、`magnetic/emm2015` 三个数据包，scp 进 VM 手工解压至 `/usr/share/GeographicLib/`。脚本已加判断：数据集已存在则跳过。
2. **pip 在 VM 内始终 SSL 失败**：`SSLEOFError: EOF occurred in violation of protocol`。已验证与网络、证书、TLS 版本、pip 版本（25.0.1 与 24.0 均失败）、是否走代理无关——系统自带 `urllib3 1.25.8` 可正常访问，唯独 pip 内置的 urllib3 无法完成握手。
3. **改为离线 wheelhouse**：在主机用 `pip download --python-version 3.8 --abi cp38 --platform manylinux*` 取齐 `deps/requirements-px4-1.13.txt` 及其依赖；`empy==3.3.4` 无 wheel，在主机编译成 `py3-none-any`；因主机按 Python 3.13 解析，需手工补 `python_version < "3.9"` 的 `pkgutil-resolve-name` 等依赖。共 55 个文件置于 VM 的 `~/robocup/wheels`，脚本优先用 `--no-index --find-links` 离线安装。
4. 附带：主机侧 `tmp/proxy_relay.py` 把 `192.168.x.x:7890` 转发到宿主 Clash 的 `127.0.0.1:7890`（仅绑定 VMware 网卡），使客户机在需要时可经代理出网。

## 2026-09-07 18:5x  ✅ 最小目标全部验证通过 + 快照完成

### OFFBOARD 起飞失败根因排查（完整链路）
前一阶段 `connected: True` 但解锁后立即 failsafe / OFFBOARD 无法进入，逐层排查：

1. **解锁即失败（RC 丢失 failsafe）**：ulog 显示 `Failsafe enabled: no RC and no datalink` →
   - `NAV_RCL_ACT=0`（禁用 RC 丢失反应）、`NAV_DLL_ACT=0`（已默认）。
   - 注意：`rosservice call /mavros/param/set` 嵌套 YAML 会超时，**用 `rosrun mavros mavparam set` 才有效**。
2. **OFFBOARD 请求被 ACK 但 nav_state 不变**：逐层验证——
   - 设定值包确实到达 PX4（tcpdump 解码 MSGID=84 / z=-2.5m / mask=3576 / target 1:1，20Hz）；
   - mavlink 接收处理函数在跑（故意发非法坐标系的设定值，ulog 出现 25 条
     `SET_POSITION_TARGET_LOCAL_NED coordinate frame 0 unsupported` —— 反向证明守卫通过）；
   - `MAV_FWDEXTSP=1`（mavlink 外部设定值转发开关，精确名验证）。
   - **真正根因**：`set_nav_state()` 的 OFFBOARD 分支在 `rc_signal_lost=true` 且
     `COM_RCL_EXCEPT` 无 OFFBOARD 豁免位时进 failsafe 拒绝切换。
     枚举定义：`RCL_EXCEPT_MISSION=1, RCL_EXCEPT_HOLD=2, RCL_EXCEPT_OFFBOARD=4`。
     **`COM_RCL_EXCEPT` 必须设 4**（XTDrone 官方 communication 脚本即设 4）。
3. `COM_RCL_EXCEPT=4` 设置后：OFFBOARD 立即生效。

### 最终验证结果（scripts/vm/offboard_takeoff_test.py）
- OFFBOARD 进入 ✓，解锁 ✓，6s 爬升至 1.89m，9s 达 2.38m，**精确悬停 2.5m（±0.03m，持续 20s+）** ✓
- AUTO.LAND 降落 ✓，着地自动上锁 ✓
- `VERDICT: PASS (peak z = 2.52 m)`
- 环境参数跨重启持久化已验证（COM_RCL_EXCEPT / NAV_RCL_ACT / NAV_DLL_ACT）

### 最小目标清单（原需求第 7 条）
| 项 | 状态 |
|---|---|
| roscore | ✅ |
| Gazebo 11.15.1（无头 gzserver + iris） | ✅ |
| PX4 v1.13.2 SITL（46a12a09） | ✅ |
| MAVROS connected | ✅ |
| iris 单机 OFFBOARD 起飞/悬停/降落 | ✅ PASS |

### 快照（原需求第 9 条）
- `02-ros-px4-ready`：关机态快照（ROS/PX4/XTDrone 构建完成 + 起飞验证通过）✅
- `03-robocup-baseline`：运行态热恢复点 ✅
- `01-ubuntu-clean`：无法补拍（纯净安装时刻已过去，系统已完成配置）；如需可重装后立即拍。

### 复用要点（供后续开发）
- 关键参数（已持久化）：`COM_RCL_EXCEPT=4`、`NAV_RCL_ACT=0`、`NAV_DLL_ACT=0`、`COM_RC_IN_MODE=1`
- VM 内 GitHub/pypi 直连依赖宿主 Clash TUN 模式；离线 wheelhouse 在 `~/robocup/wheels`
- 一次性 SSH 公钥已写入 `linfen@VM:~/.ssh/authorized_keys`（主机密钥 `tmp/host_ssh/id_ed25519`）
