# AstriBot SLAM

统一仿真和真机的 Voxel-SLAM 后端，负责双雷达/IMU 估计、回环、关键帧和三维会话保存。算法源自原 SLAM 目录中的 Voxel-SLAM，保留其 GPL-2.0 许可证；目录和依赖整理不改变算法参数。

使用 `astribot_s1_perception/voxel_slam.launch.py` 启动，唯一导航栅格由 `astribot_s1_mapping` 提供，消息契约由 `astribot_slam_msgs` 定义。数值依赖为项目统一 Eigen 与 GTSAM。原外部工作空间脚本及专用示例启动入口已删除。

完整链路、存图/载图命令与验收范围见 [统一 SLAM 参考](../../../docs/SLAM_INTEGRATION_REFERENCE_20260918.md)。

`initial_chassis_pose:=x,y,z,qx,qy,qz,qw` 是估计器启动时实测的
`map<-astribot_torso_base` 位姿。仿真静态地图模式须由启动方在机器人已静止时
采集实际世界位姿并显式提供，不能使用尚未落地的 spawn 指令值。它只初始化一次
IMU 估计状态；之后 `/slam/pose`、世界点云和关键帧均来自同一 LiDAR/IMU 估计，
不持续读取 Gazebo、odom 或 TF 来替代 SLAM。重力初始化保留底盘原点和航向。
留空仍新建以起始底盘为原点的局部地图；已有地图的 `previous_map` 定位不能叠加
此参数。该入口不是自动标定，提供方仍须记录地图身份、采样时刻和实际位姿来源。

运行日志统一使用 ROS logger，经公共 launch/supervisor 汇入 spdlog。通过统一仿真入口启动时，
SLAM、建图和 TF 适配日志进入同一 `session.log`，默认 INFO、10 MiB 轮转、5 个备份；
Release 也可开启 DEBUG。轨迹采样单独写入 `artifacts/highrate_tf_<PID>_<时间>.txt`，
保持 TUM 数据格式，不参与日志轮转。独立启动、保存工具及数据保留边界见
[统一日志说明](../../../docs/LOGGING.md)。
