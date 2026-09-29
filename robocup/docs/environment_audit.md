# 主机环境审计

审计与安装时间：2026-09-03；复核时间：2026-09-04（Asia/Shanghai）。`../pre.data` 未被改动、运行、解压、覆盖或删除。

## 结论

推荐路线是 **Docker Desktop + WSL2 + Ubuntu 20.04**。赛事要求比赛工作站为 Ubuntu 20.04 且使用 Docker；本机是 Windows 11、具备 NVIDIA RTX 4060，因此 Docker Desktop 的 WSL2 后端最接近赛场隔离方式。该路线现已实际安装并验证：Ubuntu 20.04 为 WSL2 发行版，Docker Desktop 使用 Linux containers，容器 GPU 可用。公共 XTDrone/PX4 单机环境也已编译并运行，不需要现在重装为原生 Ubuntu。

原生 Ubuntu 可作为备用的比赛镜像验收机，但不应在本机直接替换 Windows；必须在赛项技术群确认其是否允许与比赛工作站一致的原生部署。现阶段不建议用历史工程替代官方环境。

## 主机结果

| 项目 | 实测结果 | 判定 |
|---|---|---|
| Windows | Windows 11 家庭中文版，10.0.26200，64 位 | 可作为 Docker Desktop 宿主机 |
| CPU | Intel Core i7-13650HX，20 逻辑处理器 | 算力基础可用 |
| 内存 | 15.63 GiB，审计时空闲约 5.15 GiB | 六机 Gazebo 需关闭后台程序并以官方工作站实测为准 |
| 磁盘 | C: 空闲 124.40 GiB；D: 空闲 105.23 GiB | 可存放镜像与日志；须避免把大模型提交到源码目录 |
| GPU | NVIDIA GeForce RTX 4060 Laptop GPU，8 GiB | 主机可见 |
| NVIDIA 驱动 | 592.00；`nvidia-smi` 报告 CUDA compatibility 13.1 | Docker Desktop 已提供 `nvidia` runtime；CUDA 12.4 Ubuntu 20.04 验证容器实际可见 RTX 4060。官方镜像的 GPU 要求仍待赛项确认 |
| CUDA 编译器 | `nvcc` 未安装 | 不作为当前阻塞项；不要为此安装系统 CUDA |
| Docker | Docker Desktop 4.89.0；Engine/CLI 29.7.2 | 已验证 Linux containers 与 `hello-world` |
| WSL | WSL 2.7.11.0、Linux kernel 6.18.33.2 | 已确认 Ubuntu-20.04 和既有 Ubuntu-26.04 均为 WSL2 |
| Ubuntu 20.04 | Canonical Ubuntu 20.04.6 LTS，WSL2；Python 3.8.10 | 已安装并验证 |
| 虚拟化 | Docker Desktop WSL2 后端已启动并运行 Linux containers | 当前实际能力证明 WSL2 虚拟化链路可用；早期 WMI 结果不采用 |

说明：非提升会话无法读取部分 CIM 和 Windows 功能状态；一次提升审计补充了 CPU、内存和显卡。Docker Desktop 后端的实际运行结果优先于早期受限 WMI 查询。

## Docker Desktop 故障恢复记录

Docker Desktop 曾因 secrets engine 的 stale socket 路径报“unexpected error”。未执行 factory reset；保留原数据并将冲突目录重命名为：

- `C:\Users\pc\AppData\Local\Docker\run.codex-backup-20260904`
- `C:\Users\pc\AppData\Local\Docker\run.codex-backup-20260904-2`
- `C:\Users\pc\AppData\Local\docker-secrets-engine.codex-backup-20260904`

重启后 Engine 恢复，现有镜像未被覆盖。备份目录尚未删除。

## 尚未发现的赛项专用资源

没有发现赛项专用 Docker tar、最终裁判程序、2026 最终场景包或接口契约。规范给出的公共 XTDrone 仓库已经成功获取并用于训练环境；历史材料未被运行、解压、修改或覆盖。

## 图形界面说明

Ubuntu-20.04 的 WSLg 环境本身可见 `DISPLAY=:0` 和 `/mnt/wslg/.X11-unix`，但 Docker Desktop 当前不能挂载该发行版的 distro service（`ubuntu-20-04.sock` 不存在）。因此当前验收采用 headless `gzserver`；ROS 时钟、物理模型、PX4、MAVROS 与位姿链路均真实运行。若需要 Gazebo 窗口，应后续启用兼容的 Docker Desktop WSL integration 或安装独立 X server，再单独验证，不影响现有环境。

## 需要用户明确批准后才可执行的主机动作

已完成 Ubuntu 20.04 与 Docker Desktop 安装。Docker Desktop 已提供 NVIDIA runtime，且 Ubuntu 20.04 CUDA 容器测试通过。由于 Docker Desktop 当前版本对 Ubuntu 20.04 的 WSL 内 CLI 集成存在上游兼容性风险，本工程使用 Windows Docker CLI / PowerShell 脚本驱动容器，不依赖在 Ubuntu 20.04 内执行 `docker`。
