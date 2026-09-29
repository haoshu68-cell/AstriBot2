# astribot_s1_perception_components

双腕按需感知新增 `wrist_camera_session.launch.py`：通过 C++ 相机会话服务管理
订阅、健康和私有点云，默认标定版本为 0，未提供有效版本时不放行健康。
`dds_profile` 显式设置整个腕部处理链的传输配置，留空则继承调用环境。
任务服务、时间边界、实际验证范围及未完成接入见
[W1 实施与接续说明](../../../docs/evidence/task_chain_20260922/w1_handoff.md)。
会话只授权感知资源，不授予运动权限；双腕点云尚不直接进入底盘代价图。

本包提供标准 `sensor_msgs/PointCloud2` 的自滤、二维投影和可选代价图清除适配：

`Voxel-SLAM /map_scan_filtered -> pointcloud_slice_scan_node -> /scan`

点云转换、双雷达时间同步和旧 `livox_ros_driver2/CustomMsg` 适配已移除。硬件与仿真
必须提供相同的 PointCloud2/IMU 接口，SLAM 负责多雷达融合和位姿估计。

`astribot_s1_autonomy::ObservedRayObstacleLayer` 继承 Nav2 ObstacleLayer，补充实际有限
清除射线的连续栅格遍历，处理视点移动后整数射线遗漏边缘格的问题。保留原观测缓冲、
当前障碍物标记、量程和静态层；没有回波的角度桶不作为额外清除依据，不按时间擦除障碍。
普通导航仍使用原 ObstacleLayer；仿真启动器仅在指定 `--social-scenario` 时通过
`obstacle_layer_plugin` 同时为局部/全局代价图启用此实现。

## RGB-D 点云

`rgbd_pointcloud_node` 将深度图按最新 `CameraInfo` 投影为 optical-frame 的
`PointCloud2`，支持 `16UC1`（毫米）和 `32FC1`（米），默认二倍抽样、0.2--5 m
深度门限。仿真启动时只为头部和躯干相机启动，输出为：

* `/camera/head_rgbd/points`
* `/camera/torso_rgbd/points`

这两个话题经过 `pointcloud_slice_scan_node` 的 TF 连杆胶囊体和底盘足迹过滤；
TF 不可用或点云超时时停止输出，避免陈旧点云被 Nav2 当成实时障碍。
腕部相机暂不送入底盘代价地图，避免手臂运动把自身误报成导航障碍；它们保留给
手眼定位和局部抓取感知。MoveIt 的 `PointCloudOctomapUpdater` 会对进入
Planning Scene 的点云执行机器人自身过滤。

[架构、迁移及质量验收](../../../docs/AUTONOMY_ARCHITECTURE_INTEGRATION.md)

## 相机健康、检测和位姿

`camera_health_node` 为每路 RGB-D 同时检查颜色图、深度图和 `CameraInfo` 的时间、帧、
分辨率、同步偏差、频率和标定版本，发布 `astribot_perception_msgs/CameraHealth`。
健康状态不是“有话题发布者”的替代品；健康无效时，后续识别和位姿节点必须拒绝输出。

`yolo_detector_node` 使用 C++ OpenCV DNN 加载本地 ONNX，接收 RGB/BGR 图像，发布
`Detection2D`。支持显式 v5 `[1,N,5+C]` / v8 `[1,4+C,N]` 检测张量；不接受分割原型、
end-to-end NMS 或未知输出布局。当前真实权重兼容性仅验证固定 640、单输出的 YOLOv5n v6.0。
接收与推理分属回调组；只保留最新待处理图像，推理完成后重新检查时间、健康、source epoch
和标定。超过预算丢弃结果；OpenCV 推理调用本身不能中途取消。

外部检测器仍可发布相同契约。`detection_gate_node` 检查身份、帧、模型/标定版本和时效，
`rgbd_object_pose_node` 从有界历史按采集时刻匹配深度和内参，不使用任意最新帧。
可选有效掩码限制支持区域；无掩码时框内背景也可能影响结果，不能直接当成抓取目标。
`ObjectPoseObservation` 的位置是可见有效深度的表面质心，`position_valid=true`、
`orientation_valid=false`。协方差包含支持点散布和保守下限，未经测量误差标定。
完整物体 6D 姿态需要独立算法；GraspNet 输出的是夹爪抓取 6D 候选，不能代替物体姿态估计。

健康、检测门控、位置观测分别检查接收时效，ROS 时钟暂停不会永久保留旧缓存。
健康源每次启动生成不同 epoch；同一版本的内参改变会拒绝，需要显式更新标定版本并重启。
本轮消息接口为 0.2.0（增加 source_epoch 和位置/朝向有效标志），必须一起重建消息与消费者。

`ComputeGrasps` 目前是消息/动作契约，没有 GraspNet 服务实现。
`grasp_candidate_gate_node` 只对候选进行静态上下文筛选，三项 `required_*` 版本缺失即拒绝，
版本参数只读，换上下文需重启。几何和碰撞布尔值是上游声明，不能代替独立 MoveIt 场景检查。
`/manipulation/valid_grasp_candidates` 保留原话题名，但输出原因明确要求 MTC 再验证；
节点没有运动执行权，也未接入自动抓取。

导航仓库 launch 默认关闭 YOLO 和位置观测，保留原导航配置。可选参数：

- `enable_rgbd_pose_estimator:=true`：头部/腹部检测门控和位置观测。
- `enable_yolo_detector:=true yolo_camera_id:=head_rgbd`：选一台相机推理（支持 `torso_rgbd`）。
- `yolo_model_path`、`yolo_labels_path`、`yolo_model_revision`：必须提供本地模型、每行一类的标签和版本。
- `use_camera_postprocess:=false` 时健康、检测和位置节点统一读取对应 raw 话题及原生 frame。

独立入口 `rgbd_vision_pipeline.launch.py` 提供同一链路，可用 `enable_health:=false` 复用仓库健康源，
避免重复发布；独立入口使用 `enable_yolo` 和无 `yolo_` 前缀的模型参数。

本轮为真实模型离线推理及隔离 ROS 消息验证，未接管共享 Gazebo 或验证真机标定。
详见 [实施、复现和证据](../../../docs/VISION_CPP_CONTINUATION_20260921.md)。
