# full 三维城市 Gazebo 加载记录

验证时间：2026-09-05 22:02–22:07，Asia/Shanghai。

## 加载的地图

复用另一位成员的 `robocup_training_worlds` 生成器。其新加的 `render_preview.py` 使用 full/seed=7 生成网页预览；开始本次验证时 generated 目录只有 preview.html 和原 unit 场景，没有 full 的落盘 world。因此按同一 full/seed=7 导出，不改变生成器或布局参数。

- world：`src/robocup_training_worlds/worlds/generated/training_city_full_s7.world`
- metadata：同目录 `training_city_full_s7.json`
- world SHA-256：`7b6e7775d8a4bd5c589510ce19011a84325833f6e42ccd711167e77c074ef28d`
- 边界：x[-100,100]、y[-50,50]，即 200×100 m。
- 38 栋建筑、184 根灯杆、4 面边界墙；出生点 [-4.75,7.75]。

## 真实运行结果

加载前 MAVROS connected=True、armed=False、landed_state=1，节点仅 gazebo/mavros/rosout。复用现有容器 robocup-single-uav，其启动命令已是 start_training_city.sh；导出验证通过后重启该容器读取新的 current_world.txt。

`gz sdf --check` 返回 Check complete，生成器独立校验通过。首次打开 GUI 时完整仿真仍在启动，/mavros/state 尚未出现，因此健康检查失败；随后连接检查成功，重试 GUI 入口通过。22:04:32 ROS master、iris、clock、MAVROS connected、local pose 五项健康检查全部通过，22:04:41 gzclient ready。

通过桌面窗口实看 Gazebo (docker-desktop)：有立体建筑方块、圆柱灯杆、地面和 iris。左侧模型树包含建筑。界面底部一次观察的 real time factor 约 0.99，FPS 约 7.92，仅是当时快照，不能当作长期性能测试。窗口初始相机跟随 iris，显示街道局部；用户可在视口中用滚轮拉远。

通过 /gazebo/get_world_properties 独立核对实际加载模型：building_* 38 个、lamp_post_* 184 个，并有四面边界、training_ground 和 iris，success=True。加载后飞控仍 connected=True、armed=False、landed_state=1，AUTO.LOITER。没有启动运动控制节点，没有执行起飞或绕障。

日志中仍存在 PX4 模型加载时的 SDF version 属性兼容警告，未阻止加载。不是零警告运行。

## 当前使用

Gazebo 窗口已打开。关闭显示窗口后，可在 PowerShell 中重新打开：

```powershell
Set-Location 'D:\a\.robocup\robocup_ws'
.\scripts\show_single_uav.ps1
```

当前仿真容器使用训练地图启动器；完整城市文件路径已记录在 build/training_city。此状态下如需重新生成并加载相同城市，确保已落地、未解锁且没有飞行任务，执行：

```powershell
.\scripts\generate_training_city.ps1 -Preset full -Seed 7
docker restart --timeout 10 robocup-single-uav
.\scripts\start_training_city.ps1 -Preset full -NoGenerate
.\scripts\show_single_uav.ps1
```

这里显式 restart 是必要的：地图由文件指针选择，compose 配置相同的情况下 up 不一定重启旧容器。第三条命令复用已有就绪等待逻辑，避免在 MAVROS 初始化完成前立即检查 GUI。

如果之前切回了空场启动器，先用 start_training_city.ps1 -Preset full -Seed 7 切换到训练启动器，再显示。不要在仿真未运行时把 show_single_uav.ps1 当作 full 地图启动器，它的默认回退行为是启动空场。

## 本次改动与边界

新增 full world、metadata 和本记录；生成脚本更新 build/training_city 下的 current_world.txt、current_metadata.txt、spawn.env。重启仿真和重建现有带 gazebo-viewer 标签的显示容器；旧显示容器只承载临时窗口，不涉及工程文件。本次未修改另一位成员的生成器、预览或飞控代码。

本次确认三维城市可以被 Gazebo 11 加载、显示，并与 PX4/MAVROS 单机共存。建筑/灯杆是基础几何模型，尺寸、数量和位置属于训练假设；6 个候选区域并非已实现的移动人物。没有完成官方地图一致性验收、六机、人物行为、碰撞实飞或寻路控制集成。现有 5 m 半径控制器不能直接执行 full 全图任务。
