# 仿真搬运 MTC 规划与执行保护

`mtc_planner` 提供 `/transport/plan_manipulation`，只返回经校验的分阶段计划；它不发送关节命令。执行沿用搬运执行器和 MoveIt。`execution_guard` 按本次执行 context 检查新鲜控制器反馈、关节跟踪误差与底盘位姿，不负责规划或放宽控制器限制。

## 关节限位余量

`joint_limit_margin_rad` 默认 0.1 rad，允许有限数值 `(0, 0.2]`。对左臂所有有位置限位的关节，规划可用区间为 `[URDF下限+余量, URDF上限-余量]`。它不修改 URDF、真实设备限位、跟踪保护的 0.05 rad 阈值或底盘漂移阈值。

搬运入口参数为 `ros2 launch astribot_s1_transport transport_skills.launch.py mtc_joint_limit_margin_rad:=0.1`，省略时使用相同默认值。

初始状态不在余量区间时返回 `MTC_START_OUTSIDE_PLANNING_MARGIN:<joint>`，不自动移动关节。PREGRASP/PREPLACE、接近、抬升、退出和收臂规划均带路径约束；输出时间参数化轨迹再次校验各个路点，拒绝 `MTC_JOINT_PLANNING_MARGIN:<joint>`。余量可能使某些目标不可达，应调整工位/姿态或重新规划，不能自动去掉约束。

抓取和放置都先通过 ComputeIK 枚举满足余量与碰撞约束的预作业姿态，再由 Connect 规划到该关节解；不能直接用全关节路径约束加笛卡尔目标代替这一步，否则当前 OMPL 配置可能选择无法求解位姿目标的关节采样器。

此策略针对实测的硬限位分支：仿真关节3停在3.1 rad，目标反向到3.04885 rad时实际仍未离开限位，跟踪误差0.05115 rad触发保护。具体 Gazebo 限位动力学仍与真实设备行为分开解释。余量避免选择该分支，不声称修复 Gazebo 物理引擎。

`joint_limit_margin_rad`、`max_velocity_scaling`、`max_acceleration_scaling` 是启动参数，运行时变更拒绝并返回 `PLANNING_PARAMETERS_REQUIRE_RESTART`，避免参数服务显示新值而规划器仍用缓存旧值。关键参数由 diagnostics_recorder 白名单记录。

`joint_planning_margin` 回归覆盖实测失败边界、区间内目标及非法余量；`execution_guard_boundaries` 保留原保护测试。实际仿真验证与剩余任务见仓库 `docs/TRACKING_FIX_AND_TASK_STATUS_20260919.md`。
