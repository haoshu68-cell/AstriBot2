# 项目源码入口索引

最近核对：2026-09-22；范围是当日工作区可读入口，不证明安装、接线、仿真或硬件验收。使用时按受影响领域核对源码和 launch；目录变更时更新该行，不能由旧文件失踪推断能力消失。

| 领域 | 源码/入口 | 状态与使用边界 |
|---|---|---|
| 导航任务仲裁 | [C++ TaskArbiter](../../../../ws_robot/src/astribot_s1_task_arbiter_native/src/task_arbiter_node.cpp) | 导航 Action 所有权；不代表整机双臂资源协调已实现 |
| 导航前置约束与速度适配 | [NavigationConstraint](../../../../ws_robot/src/astribot_s1_navigation_policy_native/src/navigation_constraint_node.cpp)、[速度转换](../../../../ws_robot/src/astribot_s1_navigation_policy_native/src/cmd_vel_body_to_world_node.cpp) | 前者只发布 Nav2 约束，不输出速度；上肢状态不进入底盘末级门禁。实际 profile 与唯一速度写入者须查所用 launch |
| 抓放/运输 | [transport](../../../../ws_robot/src/astribot_s1_transport/README.md)、[MTC](../../../../ws_robot/src/astribot_s1_transport_mtc/README.md) | 按 README 定位规划、执行、账本和场景；规划候选/附着假设不证明真实抓持 |
| GraspNet/物体姿态 | [感知服务](../../../../ws_robot/src/astribot_s1_manipulation_perception/src/manipulation_perception_server.cpp) | 抓取候选、物体 6D、可见中心分开；仍需 IK/碰撞及执行准入 |
| 地图/虚拟墙/禁区 | [zone server](../../../../ws_robot/src/astribot_navigation_zones/src/zone_server.cpp)、[领域手册](../../../../docs/manuals/VIRTUAL_WALLS_AND_KEEP_OUT.md) | 保存、消费者应用、当前版本准入是不同状态 |
| 相机/时序 | [sync core](../../../../ws_robot/src/astribot_sensor_sync/include/astribot_sensor_sync/sync_core.hpp)、[同步设计](../../../../docs/SENSOR_HARD_SYNC_DESIGN_20260922.md) | 软件配对/合成边沿与物理曝光证据分开 |
| 仿真启动/隔离 | [warehouse launch](../../../../ws_robot/src/astribot_s1_gazebo_bringup/launch/warehouse_sim.launch.py)、[supervisor](../../../../tools/sim_stack_supervisor.py)、[isolation](../../../../tools/sim_isolation.py) | 可配置 domain；还需 partition、端口、安装链及会话所有权；操作前使用 `ros2-stack-ops` |
| C++ 迁移/采用基线 | [阶段记录](../../../../docs/CPP_MIGRATION_STAGE_REPORT_20260921.md) | 历史采用/候选决策；不能据报告日期假定当前源码和 install 一致 |

需要更广的背景时查 [架构参考手册](../../../../docs/manuals/ARCHITECTURE_REFERENCE.md) 和有日期的审查文档。手册明确各部分核对范围；历史建议不自动等于当前实现。接口、参数、包数及语言实现都可能变化，现场从受影响源码验证，不在技能中复制固定总包数。

选择用例时读 [场景目录](../../robot-scenario-boundary-validation/references/project-scenarios.md)；它描述应测的命题，不是通过清单。
