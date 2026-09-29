# 仿真搬运 MTC 规划与执行保护

`mtc_planner` 提供 `/transport/plan_manipulation`，只返回经校验的分阶段计划；它不发送关节命令。执行沿用搬运执行器和 MoveIt。`execution_guard` 按本次执行 context 检查新鲜控制器反馈、关节跟踪误差与底盘位姿，不负责规划或放宽控制器限制。

## 关节限位余量

`joint_limit_margin_rad` 默认 0.1 rad，允许有限数值 `(0, 0.2]`。对左臂所有有位置限位的关节，规划可用区间为 `[URDF下限+余量, URDF上限-余量]`。它不修改 URDF、真实设备限位、跟踪保护的 0.05 rad 阈值或底盘漂移阈值。

搬运入口参数为 `ros2 launch astribot_s1_transport transport_skills.launch.py mtc_joint_limit_margin_rad:=0.1`，省略时使用相同默认值。

初始状态不在余量区间时返回 `MTC_START_OUTSIDE_PLANNING_MARGIN:<joint>`，不自动移动关节。PREGRASP/PREPLACE、接近、抬升、退出和收臂规划均带路径约束；输出时间参数化轨迹再次校验各个路点，拒绝 `MTC_JOINT_PLANNING_MARGIN:<joint>`。余量可能使某些目标不可达，应调整工位/姿态或重新规划，不能自动去掉约束。

抓取和放置都先通过 ComputeIK 枚举满足余量与碰撞约束的预作业姿态，再由 Connect 规划到该关节解；不能直接用全关节路径约束加笛卡尔目标代替这一步，否则当前 OMPL 配置可能选择无法求解位姿目标的关节采样器。

此策略针对实测的硬限位分支：仿真关节3停在3.1 rad，目标反向到3.04885 rad时实际仍未离开限位，跟踪误差0.05115 rad触发保护。具体 Gazebo 限位动力学仍与真实设备行为分开解释。余量避免选择该分支，不声称修复 Gazebo 物理引擎。

`joint_limit_margin_rad`、`max_velocity_scaling`、`max_acceleration_scaling` 是启动参数，运行时变更拒绝并返回 `PLANNING_PARAMETERS_REQUIRE_RESTART`，避免参数服务显示新值而规划器仍用缓存旧值。关键参数由 diagnostics_recorder 白名单记录。

## 占据图场景一致性

`_transport_scene_native` 使用 C++ / liboctomap 将占据图转换为碰撞语义签名，供兼容任务的执行前场景检查使用。重复观测只改变 log-odds、没有改变 FCL 使用的占据/空闲分类时，不使计划失效。所有已分配节点的位置、深度、叶节点标记和分类仍参与签名；外层继续校验 frame、origin、resolution、碰撞体及允许碰撞矩阵。

机械臂运动改变视野后，占据图几何可能发生有效变化。兼容执行器等待占据语义稳定 1 秒（最多 15 秒），确认其他场景内容未变，再调用 `/transport/revalidate_manipulation`。C++ 服务仅保留最近一次成功规划的上下文（120 秒有效），在各阶段原有 MTC 场景副本中替换占据图，重验所有剩余运动轨迹；原 ACM、预测附着/解除附着和轨迹保持不变。碰撞、限位、奇异点检查失败则拒绝，服务本身不执行动作。执行器还会再次读取场景确认未变，并重新检查 HOLD、底盘和标定条件；不会因复核成功放宽执行门槛。

模块随本包安装并设置 Python 导入路径；缺模块、坏数据或不支持的树类型均拒绝，不退回忽略占据图。独立构建入口为 `native_scene/CMakeLists.txt`，用于只验证此核心而不启动 ROS。它没有启用 fixed_v2 抓放执行，也不替代轨迹碰撞检查。

`joint_planning_margin` 回归覆盖实测失败边界、区间内目标及非法余量；`execution_guard_boundaries` 保留原保护测试。实际仿真验证与剩余任务见仓库 `docs/TRACKING_FIX_AND_TASK_STATUS_20260919.md`。

## 实际附着后的受限收臂重规划

`/transport/revalidate_payload_transition` 保留实际附件、完整场景、context 和 transaction 校验。首次 PICK 事务中，只有实际载荷版本的原 LIFT 全路径已通过，随后未执行的 `TRANSPORT_POSTURE` 因 `EXTERNAL_COLLISION` 被拒绝时，才允许对这一段调用一次既有 RRTConnect。起点为已验证的原 LIFT 末态，目标为原收臂末态；使用该阶段原 ACM、当前占据图和实际附件，不改变阶段数或工位。其他阶段、PLACE、限位／奇异性失败，以及已绑定事务的后续碰撞均直接拒绝。

新收臂路径的首末状态绑定原边界，然后使用原速度／加速度系数、时间缩放、全路径几何／奇异性与关节余量检查。非规划组关节不得改变。未重规划的轨迹不重新定时，LIFT／RETREAT 等原消息时标和导数保持一致。单次验证仍限于 10 秒，并受原上下文 120 秒有效期限制；返回后缀序列化完成后再次检查同一期限，原缓存创建时间不刷新。

成功响应必须显式返回 `start_index=4` 起的两段 `remaining_stages` 以及 `transport_replanned`；只有完整候选和响应准备成功才原子提交缓存及事务绑定。执行器必须在独立场景读回、所有权和原期限仍有效后明确采纳后缀，服务不会自行发运动指令。同一事务重放保留最初的 `transport_replanned` 值并返回已缓存轨迹，不再规划或重新定时；场景／占据变化仍需重验，失败不提交候选。

定向核心用例使用真实 MoveIt 场景／碰撞检查和可注入的离线规划回调，覆盖实际体绕障、原轨迹保留、重复事务、失败不提交、边界关节、其他关节不可变及原期限。它们不替代真实 OMPL、控制器或完整搬运验收；构建及现场验证状态以相应证据记录为准。
