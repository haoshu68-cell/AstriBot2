# astribot_s1_perception_components

本包提供标准 `sensor_msgs/PointCloud2` 的自滤、二维投影和可选代价图清除适配：

`Voxel-SLAM /map_scan_filtered -> pointcloud_slice_scan_node -> /scan`

点云转换、双雷达时间同步和旧 `livox_ros_driver2/CustomMsg` 适配已移除。硬件与仿真
必须提供相同的 PointCloud2/IMU 接口，SLAM 负责多雷达融合和位姿估计。

`astribot_s1_autonomy::ObservedRayObstacleLayer` 继承 Nav2 ObstacleLayer，补充实际有限
清除射线的连续栅格遍历，处理视点移动后整数射线遗漏边缘格的问题。保留原观测缓冲、
当前障碍物标记、量程和静态层；没有回波的角度桶不作为额外清除依据，不按时间擦除障碍。
普通导航仍使用原 ObstacleLayer；仿真启动器仅在指定 `--social-scenario` 时通过
`obstacle_layer_plugin` 同时为局部/全局代价图启用此实现。

[架构、迁移及质量验收](../../../docs/AUTONOMY_ARCHITECTURE_INTEGRATION.md)
