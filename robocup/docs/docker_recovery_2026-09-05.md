# Docker Desktop 启动故障记录（2026-09-05）

## 现场证据

本次执行单机健康检查时，Docker Linux Engine 管道不存在。用户截图和本机 `C:\Users\pc\AppData\Local\Docker\log\host\com.docker.backend.exe.log` 一致：Ingest 服务无法将 `sailor-ingest.sock` 重命名为 `.stale`，报 `The file cannot be accessed by the system`。

`Docker\run` 为普通目录；其中 socket 是 ReparsePoint，原条目时间为 2026-09-04。第一次恢复后启动推进到 Secrets Engine，并在 `docker-secrets-engine\engine.sock` 遇到相同错误。该目录现场仅含 socket。

## 已执行的可恢复操作

关闭启动失败的 Docker Desktop / com.docker.backend 进程，将运行时目录改名备份，然后重启已安装的 Docker Desktop。未执行 factory reset，未删除镜像、容器、卷或虚拟磁盘。

备份路径：

- `C:\Users\pc\AppData\Local\Docker\run.codex-backup-20260905-120818`
- `C:\Users\pc\AppData\Local\docker-secrets-engine.codex-backup-20260905-120910`
- `C:\Users\pc\AppData\Local\Docker\run.codex-backup-20260905-120954`

第三个备份保留了第一次失败重启新生成的 socket。处理多个服务的残留路径时，需要在 Docker 完全退出后统一处理，否则每次失败启动可能再次生成残留。

## 验证状态

恢复后 `docker info` 返回 Engine 29.7.2，原容器 `robocup-single-uav` 自动启动。2026-09-05 12:11（Asia/Shanghai）复用 `health_check_single_uav.ps1`，ROS master、Gazebo iris、clock、MAVROS 连接和本地位姿五项均通过。未重装 Docker 或镜像。

后续单机航点已完成真实 headless 验证，见 [运动验证记录](uav_motion_validation_2026-09-05.md)。恢复后的运行状态与 9 月 4 日的历史验收分开记录。本次备份仍可恢复，未删除。

## 晚间复发与恢复

19 点训练地图接入时同类问题复发，保留以下运行目录备份后恢复：

- `C:\Users\pc\AppData\Local\Docker\run.codex-backup-20260905-192053`
- `C:\Users\pc\AppData\Local\docker-secrets-engine.codex-backup-20260905-192305`
- `C:\Users\pc\AppData\Local\Docker\run.codex-backup-20260905-192421`

21:39 续作复核再次发现 Engine 管道缺失，启动日志仍为 Ingest socket rename 失败。停止 Docker Desktop/backend/build 进程，核实绝对路径后统一备份两个运行目录，隐藏启动已有 Docker Desktop：

- `C:\Users\pc\AppData\Local\Docker\run.codex-backup-20260905-214052`
- `C:\Users\pc\AppData\Local\docker-secrets-engine.codex-backup-20260905-214052`

此次没有删除文件、镜像、卷或容器磁盘。21:41 单机容器自动恢复，A* 14 项测试通过；21:42:36 五项健康检查全部通过。场景包含 unit_wall_0000 与 iris，飞控 connected=True、armed=False。这里只确认本次恢复成功；socket 反复失效的长期根因尚未解决，不能保证下次启动不复发。不要使用 factory reset 作为常规启动步骤。
