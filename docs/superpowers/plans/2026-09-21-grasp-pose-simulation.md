# GraspNet 与物体 6D 仿真实施计划

> For agentic workers: use superpowers:subagent-driven-development for independent model/pose tasks and requesting-code-review for final review.

用户已授权实现 GraspNet 服务、完整物体 6D，先在仿真验收；不做真机和 VLA 验证。延续既有相机链路与导航仓库环境。

## 设计与约束

- GraspNet 与物体姿态为两个独立估计器。GraspNet 使用官方 checkpoint，禁止以随机/几何候选冒充模型推理。
- 物体姿态本阶段是已知 CAD/几何模型的 C++ 点云配准（OpenCV PPF/ICP）；不宣称未知物体泛化或 FoundationPose 已接入。
- 真值仅由验证记录器读取，不作为估计器输入。对称物体报告等价朝向；不可观测旋转拒绝，不能身份四元数冒充。
- 运行时 ROS、预后处理、输入检查、服务生命周期优先 C++；模型框架若暂需独立适配依赖，记录清楚，优先导出 C++ 可加载产物。
- 保留时间、来源、标定、模型、场景、包络版本。服务仅推理，不取得机械臂/夹爪控制权；碰撞布尔值不能自证。
- 保留已有工作区变更。构建/install、模型依赖、日志、ROS domain、Gazebo partition 均独立，不清理其他会话。
- 现有场景复用导航仓库；测试物体可在该环境中生成，不另建平行机器人场景架构。

## 任务

- [x] A：取得 GraspNet baseline 权重并验证真实推理；导出/适配 C++ 可用接口；保留 checkpoint/source 哈希、镜像下载来源及兼容差异。最终使用采样语义修复后的 v2 模型。
- [x] B：实现可独立测试的 C++ 模型 6D 配准核心，返回 object-to-camera 变换、匹配误差/覆盖率、明确失败原因；测试旋转、遮挡、噪声、退化输入。
- [x] C：补齐显式采集快照的 ComputeGrasps / EstimateObjectPose Action 与 C++ 服务，包含帧、时效、版本、取消、过期结果、有限数值和候选上限。
- [x] D：完成隔离导航仓库 RGB-D 采集、真实 GraspNet 和姿态验证，真值独立评分。**验证已执行但全场景未放行**：v2 离线 2/7 成功，5/7 质量拒绝；clear 实时 Grasp 20/20、pose 10/10 成功，不能推及被遮挡视角。
- [x] E：原视觉与门控回归通过，独立代码审查完成，构建与证据已保存。双 RGB-D 传输容量及异步配对已修复；未改变新鲜度/碰撞门槛。仍失败项见主报告。

## 验收矩阵

正常目标、多姿态/距离、部分遮挡、稀疏/空点云、退化平面、对称几何、错 frame、过期输入、标定不符、取消/超时。
位置与旋转误差阈值作为明确测试配置记录（目标 20 mm / 10°，对称物体按已声明对称群评价），不能逐场景放宽；不以单帧和固定输入代替持续回归。
模型推理、点云碰撞过滤、MoveIt 全机器人碰撞/IK、物理抓取成功分别记录；本阶段不自动将候选交给执行器。

## 执行记录

- 初检：GPU RTX4090 24GiB 可用；系统 Python 无 torch，PATH 无 nvcc；OpenCV 4.5.4 有 surface_matching。隔离工具已在当前源码提供 instance/domain/partition，仍需按实际进程核验。
- 用户附件作技术参考，不将文档中另建 Isaac 仿真的建议视为用户命令。
- 主记录：[实现与仿真验证](../../GRASP_POSE_SIMULATION_20260921.md)。任务清单勾选表示该项工作及其验证已执行，不代表所有输入条件或后续机械臂执行已验收。
