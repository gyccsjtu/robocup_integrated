# 版本、来源与完整性清单

冻结时间：2026-09-04（Asia/Shanghai）。`已实测` 表示当前机器实际安装、编译或运行；赛项专用资源未发布时不猜测。

| 组件 | 精确版本/标识 | 状态 | 来源/许可证 |
|---|---|---|---|
| 2026 正式赛事规范 | SHA-256 `AA141D4D76E2905449FE4351B01049C62C3EFBB633A49C5615A3467DB1FD2CF2`；5,589,088 B；17 页 | 已核验 | `C:\Users\pc\Downloads\1786409923668794.pdf`；赛事规则发布页 |
| 宿主 OS | Windows 11 家庭中文版 10.0.26200 | 已实测 | Microsoft |
| 开发 Linux | Ubuntu 20.04.6 LTS WSL2；AppX 2004.6.16.0；SHA-256 `4621D13A4AFD45694FD891B8CC350DAD18B53A07378A8474D2CE4B09E05B1298` | 已实测 | Canonical |
| WSL | 2.7.11.0；kernel 6.18.33.2 | 已实测 | Microsoft |
| Docker Desktop | 4.89.0；Engine/CLI 29.7.2 | 已实测 | Docker Desktop 许可 |
| NVIDIA GPU | RTX 4060 Laptop GPU 8 GiB；驱动 592.00 | 主机及容器已实测 | NVIDIA |
| 基础镜像 | `ros:noetic-ros-base-focal@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d` | 摘要已固定 | Docker Official Image；镜像内各组件许可 |
| 单机开发镜像 | `robocup-2026-single-uav:local`；image ID `sha256:8578593c614df1bf875f1da5a7354441ce8c6af33a8958c4cde5f1fe73efd1a7`；1,289,626,329 B | 已构建/运行 | 本工程 `docker/Dockerfile.single-uav` |
| XTDrone | branch `1_13_2`；commit `62339a816ef815113a0366a62e8aca4be3000f80` | 已固定/overlay 已应用 | `https://gitee.com/robin_shaun/XTDrone.git`；MIT |
| PX4-Autopilot | tag `v1.13.2`；commit `46a12a09bf11c8cbafc5ad905996645b4fe1a9df` | 已固定/已编译 | `https://github.com/PX4/PX4-Autopilot.git`；BSD-3-Clause |
| PX4 SITL Gazebo submodule | `48440d7b5c78a21182415266334981f1163f4b2c` | 已固定/已编译 | PX4 submodule |
| MAVLink submodule | `3b52eac09c2e37325e4bc49cd2667ea37bf1d7d2` | 已固定/已编译 | PX4 submodule |
| pymavlink submodule | `df6e4d7c37edda3f687bb31db1b3e12a67eab831` | 已固定/已编译 | MAVLink submodule |
| OpticalFlow submodule | `28ef45c1fcb532b2bfd54c16b059c6a545143b2f` | 已固定/已编译 | sitl_gazebo submodule |
| NuttX version source | `91bece51afbe7da9db12e3695cdbb4f4bba4bc83` | 已固定；仅供 PX4 版本生成/SITL 源树 | Apache-2.0/BSD 类，见上游文件 |
| ROS | Noetic；`ros-noetic-ros-base=1.5.0-1focal.20250521.010531`；desktop-full `1.5.0-1focal.20250521.014741` | 已实测 | ROS apt；各包许可 |
| Gazebo Classic | 11.15.1；apt `11.15.1-1~focal` | 已实测 | OSRF/Ubuntu；Apache-2.0 |
| MAVROS | `1.20.1-1focal.20250520.005419`；extras `1.20.1-1focal.20250520.011525` | 已实测 | ROS apt；BSD-3-Clause |
| Python | 3.8.10；仅 Python 3 | 已实测 | Ubuntu Focal |
| 编译工具 | GCC 9.4.0；CMake 3.16.3；catkin-tools 0.9.4-1 | 已实测 | Ubuntu/ROS apt |
| 最终比赛镜像/场景/裁判 | 尚无可冻结版本 | 待赛项方发布（若提供） | 收到后必须记录 URL、digest/SHA-256、许可证 |

## 冻结与离线备份

开发镜像使用本地 tag，因此 image ID 证明当前内容；若导出 tar，还应记录 tar 的 SHA-256。最终赛项镜像必须优先记录 RepoDigest，不能只记录可变 tag。

```powershell
docker image inspect robocup-2026-single-uav:local --format '{{.Id}} {{.Size}} {{.Created}}'
docker save robocup-2026-single-uav:local -o <源码目录外路径>\robocup-2026-single-uav.tar
Get-FileHash <tar路径> -Algorithm SHA256
```

```bash
git -C third_party/XTDrone rev-parse HEAD
git -C third_party/PX4-Autopilot rev-parse HEAD
git -C third_party/PX4-Autopilot submodule status
```
