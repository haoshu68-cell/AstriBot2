# 导航仓库真实渲染 RGB-D 与 CAD 姿态验证

日期：2026-09-21。复用源码 `warehouse_sim.launch.py`、同仓库 small_warehouse 世界与 Astribot S1 机器人。只运行物理、机器人状态、控制器与传感器；未发底盘、机械臂、夹爪或抓取执行命令，未做真机/VLA 验收。

## 已取得证据

- `ROS_DOMAIN_ID=87`，`IGN_PARTITION=GZ_PARTITION=astribot_grasp_pose_20260921`，discovery ports 18174/18175；运行中的 robot_state_publisher 环境已反读确认，见 `process_environment.json`。
- 物理实测 `/stats` iterations 7170、sim time 7.17 s、RTF 1.0001；基线 8 秒采集收到 8002 个时钟样本，时间从107.401 s推进至115.402 s。不是用进程/话题发布者存在性替代就绪。
- 原始 torso RGB-D 实际分辨率640×360，原生 CameraInfo K 的 fx=fy=345.694、cx=320、cy=180、D=0；禁用畸变后处理，按该实测 K 投影。
- `scene.xyz` 是 Nx6：相机光学坐标 xyz（米）与面向相机的单位法向。颜色掩码只用渲染 RGB 的 HSV 洋红范围、有限深度和1像素腐蚀；法向来自实际组织化深度差分并剔除深度边缘。此分割仅是验证夹具，不代表 YOLO 实例分割验收。
- 最终每份快照的 RGB、Depth、CameraInfo、CameraHealth.capture_stamp 完全相同，health.valid=true；相机 TF 使用图像采集时刻查询，导出时新鲜度均≤250ms。帧是 `astribot_s1/astribot_torso_base/torso_rgbd_sensor`，实际旧安装健康节点 source_epoch=`gazebo_camera`、calibration_revision=1；独立进程身份另见 session_identity.json。
- `truth.json` 只由离线记录器读取独立 Gazebo 位姿消息后生成。估计器命令只接收 CAD、scene.xyz 和输出文件。目标是静态夹具；动态相机 TF在采集时刻查询，world/odom锚点由独立机器人真值核验。真值消息与图像不同时间的间隔及锚点误差均逐帧记录。

## 保留的七个场景

| 场景 | 有效表面点 | 初版固定全CAD覆盖门槛结果 |
|---|---:|---|
| near | 2008 | 拒绝，无pose |
| tilted | 2389 | 拒绝，无pose |
| yawed | 2186 | 拒绝，无pose |
| far | 1688 | 拒绝，无pose |
| occluded | 1186 | 拒绝，无pose；额外绿色面板遮挡 |
| close | 5844 | 拒绝，无pose；近乎单平面 |
| clear | 2556 | 通过，位置2.093mm、旋转0.435° |

初版固定评测门槛为位置20mm、旋转10°；模型内部全CAD覆盖≥35%、scene覆盖≥65%、RMSE≤5mm。没有按场景改阈值。clear的全CAD覆盖36.94%、scene覆盖99.96%、RMSE2.427mm，耗时5.47s。拒绝样本的位姿及位置/角度误差均为null，不能算零误差。

初版结果、二进制/库SHA及输入SHA在 `pose_results_full_cad_baseline/summary.json`。后续若增加CAD可见性模型，必须在独立结果目录统一复测并区分这轮基线。

## 相机视野问题与边界

head相机收到了时间推进的RGB-D，但RGB恒灰、深度全Inf。`head_rgbd_invalid/`保留原始观测。对实际STL和原始标定外参做153条视野射线，153条均在5.39–5.74mm处与自身头部parent mesh相交；torso有57条与底座parent mesh相交，距离143–216mm，见 `parent_mesh_camera_audit.json`。这给出了刚体自遮挡的直接几何证据；转动相机与同一parent mesh不改变两者相对位置。本轮未修改外参、mesh、碰撞或遮挡规则。厂家 `torso` 坐标语义到 `astribot_torso_base` 的映射仍需单独核验。

clear在现有无遮挡视野中展示了非对称三维表面，单位法向二阶矩特征值约[0.181,0.216,0.603]；close约[0.00015,0.0046,0.995]，后者缺少充分方向约束。仿真姿态通过不等于真实相机标定准确，也不等于完整机械臂碰撞/IK或物理抓取成功。

## 复现与原始记录

- 验证脚本：`tools/vision/sim_pose_session.py`、`sim_pose_scene.py`、`sim_pose_capture.py`、`sim_pose_truth.py`、`sim_pose_mesh_audit.py`、`sim_pose_evaluate.py`。
- `snapshots/<scene>/`含原始RGB、掩码、无损压缩的实际float深度、点云、实测相机元数据和隔离真值；`dataset_manifest.json`记录SHA。
- 有效会话日志：`/home/yjh/WorkSpace/astribot_sdk_ros2/runs/grasp_pose_sim_20260921/session/session.log`。`query_env.sh`固定这次隔离查询环境。所有本次进程以supervisor/launch PID和start ticks记录；不允许按通用进程名清理，也未清理共享内存。
- 当前会话保留clear场景，供后续真实 Action 快照与 GraspNet 推理。磁盘旧快照必须按离线回放处理，不能修改时间戳冒充新鲜传感器数据。

## 健康进程升级后的实时接口探针

在保留以上原始基线后，仅替换本session拥有的两只camera_health_node，旧PID退出后才启动新PID，未重启Gazebo或TF。新可执行文件来自 `/tmp/grasp_pose_build/install/astribot_s1_perception_components`；旧/新PID、start ticks、参数、二进制SHA和日志都在 `health_replacements.json`。它们独立于原launch进程组，最终关闭session时须按所记录身份一并关闭。

`camera_health_v03_capture.json`记录升级后的clear实采：2556点、图像/深度/标定/health同stamp、health.valid=true、19ms新鲜度，实际source_epoch含唯一启动身份 `gazebo_camera:1950770428236:2100578422`。`torso_health_publishers_v03.txt`确认仅一个health发布者；实际收到有效消息仍为就绪依据。原七场景快照保持原始epoch，未改时间戳或伪造来源。
