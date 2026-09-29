# 看得到无人机的 Gazebo 演示

适用：当前 Windows 11 / Docker Desktop WSL2 / 已构建单机镜像。这里的飞机是 **iris 四旋翼**，不是传统单旋翼直升机；本次没有重新制作飞机模型。

## 最简单的运行方式

1. 打开已安装的 Docker Desktop，等待 Engine running；不用在 Docker 界面点容器的启动按钮。
2. 打开 Windows PowerShell，复制下面整行（不要复制 `PS ...>` 提示符）：

```powershell
& 'D:\a\.robocup\robocup_ws\scripts\run_single_uav_visual_demo.ps1'
```

脚本会检查/启动后台仿真，打开 **Gazebo (docker-desktop)** 窗口，自动跟随 iris，然后运行既有低速起飞 → A → 悬停 → B → 悬停 → 降落/上锁流程。连接与画面准备需要数十秒，之后飞行约一分钟。窗口若被终端遮住，点击 Windows 任务栏中的 Gazebo。

只想先打开画面、不自动起飞：

```powershell
& 'D:\a\.robocup\robocup_ws\scripts\show_single_uav.ps1'
```

画面打开后，随时可在 PowerShell 执行原有 `run_single_uav_demo.ps1` 再飞一遍。每轮航点相对当轮起始位置计算，所以重复飞行后飞机不会自动回到世界原点；镜头会跟随它。

## 观看与结束

- 鼠标放在三维区域，滚轮调整距离。软件渲染时缩小窗口通常更流畅。
- 地面上静止不代表失败：只打开画面不会解锁起飞；完整演示也会先做连接与落地检查。
- 飞行时不要点 Gazebo 暂停、Reset Time、移动/删除模型，也不要关闭 Docker。暂停仿真会触发控制器的时钟看门狗。
- 关闭 Gazebo 窗口只关闭显示，**不会取消正在进行的飞行**。
- 要提前取消，在另一个 PowerShell 执行下面命令，并等原控制终端出现 `LANDED` / `RESULT`：

```powershell
& 'D:\a\.robocup\robocup_ws\scripts\cancel_single_uav_demo.ps1'
```

正常或取消后确认已经落地/上锁，再关闭窗口；需要节省资源时再运行 `stop_single_uav.ps1` 停止后台仿真。

## 本次环境配置做了什么

复用现有 `robocup-2026-single-uav:local` 镜像和源码，无新下载安装、无 PX4 重编译、无 Windows 防火墙修改。保持原 Compose 的 headless 服务不变，新增仅显示的 `robocup-single-uav-gui` 容器运行 `gzclient`：

- 与 `robocup-single-uav` 共享网络命名空间，连接同一个 Gazebo master。
- 复用仿真容器挂载，使模型、纹理、插件路径一致；使用仿真容器的实际镜像 ID，不拉取新镜像。
- 只读挂载 Docker Desktop 后端的 `/mnt/host/wslg/.X11-unix`，`DISPLAY=:0`，将窗口显示到 Windows。
- `QT_X11_NO_MITSHM=1`、`QT_QPA_PLATFORM=xcb`、`LIBGL_ALWAYS_SOFTWARE=1`。本轮采用软件渲染，没有宣称验证 Gazebo 的 RTX 硬件加速。
- 有界等待 GUI 相机出现，再通过 Gazebo 相机命令跟随 `iris`；显示失败时包装脚本不启动飞行。
- 显示容器不运行第二个无人机控制节点、不发 MAVROS 指令。原 `robocup_navigation` 仍是唯一飞控出口。

显示容器退出/仿真容器被重建后，脚本只重建带有 `robocup.role=gazebo-viewer` 标签的显示容器；原仿真和挂载的项目文件不删除。不自动接管同名但没有该标签的其他容器。这里的 WSLg 后端路径是当前机器已验证的 Docker Desktop 实现路径，并非所有 Docker 平台通用约定。

PowerShell 只是 Windows 上的启动入口。真正的 ROS Noetic、PX4 和 Gazebo 仍运行在 Docker 内 Ubuntu 20.04，不是把 ROS 改为 Windows 版本。

## 常见问题

- `Docker Engine is not ready`：等待 Docker Desktop 启动。如果仍弹出此前异常对话框，不要点恢复出厂设置，参考 `docker_recovery_2026-09-05.md`。
- `WSLg socket is unavailable`：当前后端没有可用的 WSLg 接口。此脚本仅报错，不重装环境、不盲目重启正在飞行的仿真。先确认已落地，再检查 Docker Desktop/WSL 配置。
- `viewer did not become ready`：运行 `docker logs --tail 50 robocup-single-uav-gui` 查看图形日志；不要因为显示失败就反复起飞。
- 窗口标题带 `(docker-desktop)` 是正常的，表示由 Docker 后端的 WSLg 显示。
- 顶部 Gazebo Classic end-of-life 提示是版本生命周期提醒，不代表仿真故障。本项目本轮仍遵循固定 Gazebo 11 基线，没有进行版本迁移。

后续寻路、避障、YOLO 和六机不在本次图形配置范围内。详细运动配置及安全限制见 `uav_motion_demo.md`。

本机实际测试结果见 [2026-09-05 图形与飞行验证记录](gazebo_visual_validation_2026-09-05.md)。

WSLg 容器图形机制参考微软官方 [容器说明](https://github.com/microsoft/wslg/blob/main/samples/container/Containers.md)；本机的具体后端挂载路径以本轮实测为准。
