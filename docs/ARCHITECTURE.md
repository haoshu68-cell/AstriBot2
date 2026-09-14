# 仓库结构与运行链路

当前架构由纯算法、感知适配、探索任务、任务仲裁、导航策略和路径执行组成。
跟踪控制公式与参数保持既有基线；本次修改集中在运行时事务、任务与世界状态契约。
七项修复的接口、验证和部署边界见 [架构修复记录](ARCHITECTURE_FIXES_20260914.md)。

## 模块边界

| 目录/包 | 职责 |
|---|---|
| `astribot_sdk` | 厂商 Python API、会话和运动接口；部分实现由同名 `.so` 提供 |
| `astribot_msgs` / `astribot_bridge_msgs` | 厂商接口与本项目桥接消息 |
| `astribot_config` / `astribot_s1_description` | 原始网格、URDF/xacro、关节和碰撞模型、控制配置 |
| `astribot_s1_gazebo_bringup` / `astribot_s1_chassis_effort_drive` | 仓库场景、传感器桥接、全向轮力矩闭环 |
| `astribot_s1_perception` | 雷达预处理与融合、SLAM 适配、地图来源、map→odom 分解 |
| `astribot_autonomy_core` | 不依赖 ROS 的感知几何与探索算法 |
| `astribot_s1_perception_components` | 点云切片、自身剔除、ROS/PCL 适配与配置事务 |
| `astribot_s1_exploration` | 可取消的候选搜索、探索状态机、导航任务提交 |
| `astribot_s1_autonomy` | 旧包入口兼容转发 |
| `astribot_s1_navigation_policy` | 任务仲裁、观测与健康、世界版本、让行/绕行及包络协调 |
| `astribot_navigation_msgs` | 跨模块任务、策略、健康、包络与候选接口 |
| `astribot_s1_navigation` | Nav2 参数、行为树、速度链路、姿态监控、运行指标 |
| `astribot_s1_path_tracking` | ThreePhase 跟踪、到位精调、目标检查器、Smac 精确终点适配 |
| `astribot_s1_dynamics_coupling` | 根据机械臂伸展与关节速度限制底盘速度 |
| `astribot_s1_manipulation` / `astribot_s1_moveit_config` | 双臂闭链规划、奇异性/碰撞检查、轨迹优化、夹爪控制 |
| `astribot_trajectory_bridge` | SDK 会话、状态/里程计、轨迹 action 和底盘命令桥接 |
| `third_party` / Livox / warehouse | 运行依赖、驱动与场景资源 |
| `tools` / `examples` | 部署、启停、遥控、运行诊断与 SDK 示例 |

## 数据与控制

```text
雷达 → 预处理/融合 → SLAM或地图提供者 → /map、TF
                └→ 自滤/切片 → /scan_from_cloud → Nav2 costmap
人工目标/探索协调器 → 任务仲裁 → Nav2 执行端 → Smac规划（精确末点）
    → ThreePhase起步对齐 → MPPI/RPP跟踪与接近限速 → 到位精调与停稳
    → velocity_smoother → 姿态监控/可选坐标旋转 → 机械臂动态限速
    → P2/P3 策略约束与独立最终防护 → /cmd_vel → 仿真力矩底盘或实机SDK桥接
```

当前基座为 `astribot_torso_base`。力矩底盘使用车体系速度，坐标旋转默认关闭。
规划器不产生速度；ThreePhase 不替代 MPPI/RPP 搜索；ArrivalGoalChecker 负责联合精度和停稳确认。
精确终点适配保留 Smac 搜索，只替换最后一个位姿，避免栅格中心误差污染用户目标精度。

探索主循环：IDLE → GEN_NEXT_POINT → VALIDATING → NAVIGATING → ARRIVED。
搜索、路径与 action 结果异步处理；异常进入 PAUSED。探索不直接发速度，不执行旧自举/脱困；移动能力统一由导航任务链路承担。
“没有前沿”和“前沿存在但均不可行”分别处理为完成与暂停。

双臂链路：规划请求 → IK/闭链投影 → 碰撞与奇异性检查 → 时间参数化 → SDK 执行。
轨迹优化不合法或不优于合法基线时回退，是有效运行逻辑，保留。

## 清理边界

- 删除自研单元测试目录、测试构建目标、测试依赖与无用实验脚本；同步移除其安装入口。
- 精简叙事性注释，保留版权、公开接口摘要、参数单位及内联约束。
- 原有起步对齐、终点旋转、惯性补偿、横移开关、接近限速、相位计时和重规划策略保持。
- 原有机械臂旧度量分支、姿态退化处理、门控、自动恢复及实机启停流程保持，避免改变既有行为。
- 第三方依赖、SDK 二进制、模型资源及仍使用的诊断工具保留。

构建与仿真入口见 [导航说明](../ws_robot/src/astribot_s1_navigation/README_NAVIGATION.md)。
核心参数见 [跟踪说明](../ws_robot/src/astribot_s1_path_tracking/README.md)。
发现但未修改的问题见 [逻辑问题清单](LOGIC_REVIEW.md)，验证见 [清理验证记录](CLEANUP_VALIDATION.md)。
