# 相机模型加载与基线核验 — 2026-09-23

## 结论与修复

六路相机物理安装、外壳碰撞体和固定TF已统一。原先采集开关包住整个相机宏，会同时删除物理组件；现在仅关闭Gazebo传感器采集，物理质量、碰撞体和固定安装始终存在。六路全开配置固化在 description/config/simulation_navigation_full，后续共同启动入口使用此配置。

参考外参仍是照片/文档推定的仿真安装参数，不是新真机标定。头/腹640×360@20Hz，双腕640×320@20Hz，头部双目左右400×300@5Hz。关闭采集不改变机械臂和导航所需的物理包络。

## 验证

- description全量32项检查通过，含全关/部分开/全开时物理模型相同、参考安装与夹爪关节检查。原始输出 description_tests.log。
- 导航仓库运行时 robot_state_publisher 与 MoveIt 的六路相机link/joint树完全相同，camera_models.json；源数据 runs/joint_acceptance_20260923/kinematic_matrix_05/{robot_reference,moveit_reference}.urdf。
- domain94、payload_live_20260923导航03，5.00秒只读观察：六路共16个原始图像/深度/CameraInfo话题全部收到多帧，采样时间推进。six_camera_raw_streams.json保存每路帧数、帧名、尺寸和时间范围。该短窗口验证数据已加载，不用于性能/长期新鲜度放行。
- 首次查询后处理话题为零，核查确认本基线关闭后处理，实际输出在 /camera/raw/<camera>/ 下；改查实际桥接话题后通过。未将错误查询记为传感器故障。

## 范围

本次Git记录只覆盖description模型/配置资产及测试，不代表双腕/双目完整感知、物体识别、三维覆盖或硬件标定验收。MoveIt参数转发、共同启动preset和UDP修复由后续同一分支提交关联；不创建第二套开发基线。运行时完整链仍按任务分阶段验证。

三条历史对照分支现已先保存 annotated tags 与完整 bundle，再删除分支引用；见 `docs/ARCHIVED_COMPARISON_BRANCHES_20260923.md`。不再维护第二条开发基线。
