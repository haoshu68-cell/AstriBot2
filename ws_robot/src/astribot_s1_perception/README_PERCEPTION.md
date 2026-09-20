# astribot_s1_perception

感知入口把仿真和真机收敛到同一条数据契约：两路标准
`sensor_msgs/PointCloud2`、一路 `sensor_msgs/Imu`，由 Voxel-SLAM 完成点云融合、
里程计和地图后端，输出 `map -> aft_mapped`、`/map_scan_filtered` 和导航所需
的地图接口。点云预处理、同步融合和 `CustomMsg` 转换中间层不再存在。

## 包内入口

- `launch/hardware_livox.launch.py`：真机驱动，固定 `xfer_format=0`，输出标准点云和 IMU。
- `launch/voxel_slam_sim.launch.py`：Gazebo 点云/IMU 到 Voxel-SLAM 的仿真入口。
- `launch/sim_perception.launch.py`：接收 SLAM 的融合点云并生成 `/scan_from_cloud`。
- `launch/hardware_perception.launch.py`：真机使用同一套自滤和二维投影链。
- `launch/perception_slam_bringup.launch.py`：顶层组合入口；默认使用 Voxel-SLAM；静态地图仅用于仿真基线。

## 构建

```bash
cd /home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot
source /opt/ros/humble/setup.bash
bash src/astribot_s1_perception/scripts/prepare_livox_driver2.sh
colcon list --base-paths src | rg 'astribot_s1_slam|astribot_s1_mapping|astribot_slam_msgs|astribot_eigen_vendor'
bash ../tools/robot/build_slam_dependencies.sh
export CMAKE_PREFIX_PATH="$PWD/deps/gtsam:${CMAKE_PREFIX_PATH:-}"
colcon build --base-paths src --symlink-install --packages-up-to \
  astribot_s1_slam astribot_slam_msgs astribot_s1_mapping \
  astribot_s1_perception_components astribot_s1_perception \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DGTSAM_DIR="$PWD/deps/gtsam/lib/cmake/GTSAM"
source install/setup.bash
```

Voxel-SLAM 需要仓库约定的 GTSAM 安装前缀；若本机没有该前缀，构建会在 CMake 配置阶段
明确失败，不能用旧转换节点绕过依赖。

## 统一接口

| 数据 | 仿真 | 真机 |
|---|---|---|
| 左/前点云 | `/livox/lidar_left` | `/livox/lidar_left` |
| 右/后点云 | `/livox/lidar_right` | `/livox/lidar_right` |
| IMU | `/livox/imu` | `/livox/imu`（由部署配置映射） |
| SLAM 融合点云 | `/map_scan_filtered` | `/map_scan_filtered` |
| 导航扫描 | `/scan_from_cloud` | `/scan_from_cloud` |

硬件部署前须确认驱动参数中的设备 IP、网卡地址和 `frame_id`，但不再改变消息类型或
启动另一套兼容链。

存图、载图和本轮验收边界见 [统一 SLAM 参考](../../../docs/SLAM_INTEGRATION_REFERENCE_20260918.md)。
