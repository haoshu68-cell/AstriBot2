# 唯一导航仓库仿真相机基线

本目录固化 2026-09-23 六相机验证配置，不再依赖 runs 临时文件。头/腹 RGB-D：640×360、20 Hz；双腕 RGB-D：640×320、20 Hz；头部双目左右：400×300、5 Hz。

这些是仿真缩放内参及采样设置，不是新真机标定。安装位置统一使用上级目录 `camera_mounts_reference_sim.yaml` 的暂定参考外参；各文件中的固件 mount 字段仅保留来源，参考安装配置具有优先级。原固件标定文件不覆盖。

共同启动参数：`use_camera=true`、`use_wrist_cameras=true`、`use_stereo_cameras=true`、`use_camera_pointcloud=true`；`camera_profile` 和 `torso_camera_profile` 分别指向本目录对应 YAML，`camera_calibration_dir` 指向本目录。不要把仅有两路点云处理误记为六路物体感知均已接通。

当前已验证的是六路图像和头/腹点云的静止输入；双腕/双目的完整后处理与三维覆盖仍需相应任务验收。运行目录和 ROS domain 可以隔离，但模型、配置与源码使用同一主线。
