# AstriBot SLAM

统一仿真和真机的 Voxel-SLAM 后端，负责双雷达/IMU 估计、回环、关键帧和三维会话保存。算法源自原 SLAM 目录中的 Voxel-SLAM，保留其 GPL-2.0 许可证；目录和依赖整理不改变算法参数。

使用 `astribot_s1_perception/voxel_slam.launch.py` 启动，唯一导航栅格由 `astribot_s1_mapping` 提供，消息契约由 `astribot_slam_msgs` 定义。数值依赖为项目统一 Eigen 与 GTSAM。原外部工作空间脚本及专用示例启动入口已删除。

完整链路、存图/载图命令与验收范围见 [统一 SLAM 参考](../../../docs/SLAM_INTEGRATION_REFERENCE_20260918.md)。

运行日志统一使用 ROS logger，经公共 launch/supervisor 汇入 spdlog。通过统一仿真入口启动时，
SLAM、建图和 TF 适配日志进入同一 `session.log`，默认 INFO、10 MiB 轮转、5 个备份；
Release 也可开启 DEBUG。轨迹采样单独写入 `artifacts/highrate_tf_<PID>_<时间>.txt`，
保持 TUM 数据格式，不参与日志轮转。独立启动、保存工具及数据保留边界见
[统一日志说明](../../../docs/LOGGING.md)。
