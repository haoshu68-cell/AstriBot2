# 夹爪抓取的两种技术路线

以本项目已核对的感知/搬运设计划分为两条路线；架构目标包含待接通部分，不表示已经形成模型驱动的完整抓取闭环。

## 路线 A：GraspNet 直接生成夹爪抓取候选

RGB-D + 目标分割 → 目标点云 → ComputeGrasps / C++ GraspNet → 多个夹爪 6D 候选、宽度与排序分 → 实际夹爪/TCP 适配 → MTC/MoveIt → 受保护的执行与抓取确认。

重点是从观察到的目标形状产生候选。该路线的候选生成不要求先求出完整物体 6D 位姿；仍需明确目标身份、采集时刻变换、完整障碍与下游执行条件。

现有服务有历史单场景实际推理记录。候选保持 collision_checked=false / collision_free=false，排序值不是概率；必须经过下游碰撞、IK 和任务准入。通用实例分割、夹爪适配及模型候选驱动的完整执行仍须贯通验收。

## 路线 B：物体 6D 位姿 + 物体坐标系抓取模板

RGB-D / mask + 注册 CAD → 物体 6D 姿态估计 → 身份、几何、可观测性和歧义检查 → 选择物体系抓取模板 → 变换为机器人 TCP 目标 → MTC/MoveIt → 同一执行与确认链。

已知 CAD 6D 当前基线是 C++ PPF/ICP；FoundationPose C++ 是拟接入后端，不能把现有 6D 服务标成 FoundationPose。所查 P0 记录仍保留 engine、实际推理和 tracking 待验收状态。物体系抓取模板还需定义允许抓取面、接近/退出方向与夹爪开口，并验证转换和实际消费。

## 两条路线的区别与共用边界

| 项目 | 路线 A | 路线 B |
|---|---|---|
| 首要输出 | 夹爪抓取候选位姿 | 物体自身位姿 |
| 项目算法入口 | ComputeGrasps / GraspNet C++ | EstimateObjectPose / PPF-ICP；拟引入 FoundationPose |
| 抓取目标来源 | 网络提出多组候选，再筛选 | 物体坐标系下已定义的抓取模板 |
| CAD 的作用 | 非当前候选推理输入的必需项；碰撞/业务可独立使用 | 对象模型、尺度、坐标及模板基准 |
| 典型目标 | 对目标点云生成并筛选抓取姿态 | 对已知零件指定抓取面和工具方向 |
| 主要待验收点 | 分割到候选到真实夹爪执行的完整接线 | 模型后端、歧义/跟踪、模板变换到执行 |

两条路线共用相机时效/版本、PlanningScene、MTC/MoveIt、任务所有权、底盘保持和执行保护。路线 B 的物体姿态还可以为路线 A 提供对象身份、CAD 碰撞模型和语义抓取面约束，两者可以组合。

## 坐标转换约定

`T_A_B` 将 B 系的点变换到 A 系；`base` 指规划使用的机器人参考系，`camera` 为相机光学系。

- 路线 A：`T_base_tcp = T_base_camera(t_capture) × T_camera_grasp × T_grasp_tcp`。GraspNet 的 grasp frame 到实际夹爪 TCP 需要独立核对，不能直接把网络位姿作为控制器目标。
- 路线 B：`T_base_object = T_base_camera(t_capture) × T_camera_object`；`T_base_tcp = T_base_object × T_object_tcp_template`。

模板与候选都保留采集时间、标定、对象和场景版本。感知推理用目标点云，碰撞检查必须保留完整障碍；目标物体位姿、可见表面中心和抓取位姿不能相互替代。

## 共同执行流程

候选与场景一致性 → IK / 整机与夹爪碰撞 → 预抓取 / 接近 / 抬升 / 退出轨迹 → 任务准入与资源持有 → 机械臂执行 → 夹爪闭合 → 独立抓取确认。

夹爪关节闭合、Action 成功、物体被检测到，均不能单独证明真实抓牢。当前普通仿真演示使用运动学附着，夹持力、摩擦、滑落及真实抓持反馈仍单独验收。失败收尾由任务与执行保护负责，不由感知模型自动发动作。

## 依据

- [docs/GRASP_POSE_SIMULATION_20260921.md](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/GRASP_POSE_SIMULATION_20260921.md)
- [docs/FOUNDATIONPOSE_INTEGRATION_DESIGN_20260923.md](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/FOUNDATIONPOSE_INTEGRATION_DESIGN_20260923.md)
- [docs/FOUNDATIONPOSE_P0_PROGRESS_20260923.md](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/FOUNDATIONPOSE_P0_PROGRESS_20260923.md)
- [docs/MAINLINE_AND_EXTENSION_REVIEW_20260924.md](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/MAINLINE_AND_EXTENSION_REVIEW_20260924.md)
- [ws_robot/src/astribot_perception_msgs/action/ComputeGrasps.action](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_perception_msgs/action/ComputeGrasps.action)
- [ws_robot/src/astribot_perception_msgs/action/EstimateObjectPose.action](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_perception_msgs/action/EstimateObjectPose.action)
- [ws_robot/src/astribot_s1_transport/README.md](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport/README.md)
