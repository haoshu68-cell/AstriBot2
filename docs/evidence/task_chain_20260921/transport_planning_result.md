# 搬运规划服务诊断与修复

## 结果

三次独立启动，每次三次 `PlanSkill(named, arm_right, transport_compact)`，9/9 成功，每条轨迹 12 点；三次四个子节点均正常退出。`allow_trajectory_execution=false` 已由参数服务核验，未发送执行请求。

每次实际读取 move_group / transport_skill_planner / transport_mtc_planner 的 URDF、SRDF，三者字节哈希一致。离线 reference 有/无 lidar、original 三种配置也一致。

## 根因与改动

原 `MoveGroupInterface(node, group)` 使用无限连接等待；MoveIt 2.5.9 构造函数同时等待 MoveGroup 和 ExecuteTrajectory action。规划专用服务关闭执行能力后，后者不存在，导致构造停住，60 秒 PlanSkill 请求超时。改为每项 action 连接等待最多 2 秒；未开启执行能力，未降低碰撞和状态时效阈值。后续规划/执行各自检查服务可用性。

红证据：`transport_ready_dds/probe.log` 和 `skills.log`。绿证据：`transport_bounded_client`、`transport_bounded_client_repeat2`、`transport_bounded_client_repeat3`。包构建：`bounded_client_build.log`。现有 manipulation CTest 1/1 通过，该单测不是本次超时修复的替代证据；本次回归为实际服务重复请求。

## 范围

本结果包含当前静态仓库传感器和真实反馈起点；不包含原任务工位对象、载荷、多起点、MTC 全任务、轨迹执行或硬件验收。原先的 READY_RIGHT 规划无解不能据此一概关闭。MoveIt 退出仍有 class_loader 未卸载警告，不宣称内存无泄漏或全功能生命周期通过。

## 复现

在任务拥有的 domain 89 相机会话中，source 该 session/query_env.sh，再 source tools/setup_mtc_humble.sh；使用仓库 camera_bridge_fastdds.xml 的 loopback DDS profile，并设置 ROS_LOCALHOST_ONLY=0。运行 tools/vision/run_transport_diagnostic.py --output 一个全新证据目录。脚本拒绝重复服务、只启动规划服务，最多 150 秒探测，然后清理自己创建的服务。
