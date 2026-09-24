# 上肢状态前置导航决策：2026-09-24

用户明确要求：停车、限速和不可执行判断均前置于上层导航规划模块，取消独立的整机末级执行保护。此决定取代旧六消费者包络握手中的 `protection` 消费者，不撤销载荷、姿态和几何事实本身。

## 评估与职责

旧 `final_protection_cpp` 混合了包络接收与 ACK、传感器/相机时效、区域约束、扫掠判断、导航约束发布和 `/cmd_vel` 写入。直接删除会让 Nav2 失去其唯一的最终 `MotionConstraint` 来源；因此保留必要判断并前移为 `navigation_constraint_cpp`，取消其速度发布和包络 ACK 职责。

```text
附着物实测 / PlanningScene 读回 → 版本化载荷账本 → 全身/载荷几何
任务持有机械臂 HOLD + 限速预算 → 固定导航包络
  → global_costmap / local_costmap / planner / controller / policy 应用确认
  → 导航仲裁准入、BT 生命周期、路由姿态/通道策略
  → Nav2 规划与控制：停车、限速、不可执行及原因
  → 既有速度平滑 → 坐标适配 → 底盘执行

相机/scan/odom/区域及策略观测 → C++ NavigationConstraint → Nav2 MotionConstraint
臂展/关节活动 → C++ 百分比限速源 → NavigationConstraint 合并限速
```

- **账本与几何**继续提供显式、带版本的 EMPTY/ATTACHED、质量和保守包络；不得默认空载、home 或无附件。
- **五个 ACK**只证明导航消费者应用了同一包络。旧 `protection` ACK 既不能补足缺失的 controller，也不能撤销新契约的导航许可。
- **TaskArbiter**在接收目标、派发、等待和执行期间检查包络；失效或事务身份变化取消自己的后端目标，迟到受理也取消，保留后端终态屏障和明确失败原因。
- **BT**在最外层检查同一有效性；**Nav2 planner/controller**保留已有完整几何和实际 costmap 检查。控制器所有运动分支统一应用速度/禁轴约束，改变后的非零指令重新检查扫掠。
- **NavigationConstraint**只产生短时有效的停车/限速/规划约束及诊断。订阅 Nav2 的底盘坐标系速度意图 `/cmd_vel_nav_body_raw` 和 odom 实测速度，不把世界坐标系命令当作底盘坐标计算。
- **臂展耦合**取消末端 Twist 输入输出，已有臂展/关节活动算法仅提供上游百分比预算。专用 `/navigation_policy/arm_speed_limit` 使用 `nav2_msgs/SpeedLimit`，本契约明确 `0% = HOLD`；它不是原生 Nav2 `/speed_limit`，不得直接接入原生速度限值入口（原生消息零值意为不限速）。采集时刻与接收期限同时约束，定时发布不能刷新旧样本；缺失或无效在导航约束层停车。
- **平滑器与底盘输出端**不新增账本、几何、包络 ACK 或第二次上肢准入。不增加末级保护的替代节点。
- **恢复动作入口**：当前三棵主线 BT 均通过受约束的 FollowPath 执行运动；未适配本契约的原生 Spin/BackUp/DriveOnHeading 不再由主线启动加载，behavior_server 仅保留 Wait。避免这些独立 Action 直接写入平滑器输入、绕过 Nav2 控制器约束。日后需要这些动作须在上层完成适配后单独验收。
- **底盘本身**保留已有急停、驱动错误、设备限幅及命令失联处理；当前 body-to-world 适配已有 steady-clock 失联置零逻辑。仿真设备的 ROS 时间 watchdog 不能外推为真机硬实时保护。

## 必须明确的边界

上层下达停车、命令归零、平滑器减速结束、机器人实测停稳、资源释放仍是不同事件。删除末级速度截断后，普通停车保留已有 Ruckig 减速尾段；新的整栈回归须实测停止过程，旧末级截断记录不能作为新链证据。不可把取消请求当成已停稳。

此次不扩大到动态载荷加速度模型重构。原末级 CommandRestriction 只在受限/恢复阶段约束增速，并非全程动力学保证；当前 Nav2 平滑器主要使用启动时的加速度/jerk 参数。质量账本存在不能证明所有载荷下的动态稳定性已经验收。

臂展比例现用于收紧规划速度上限；旧节点则在每条最终速度上直接乘比例。二者作用位置和低速行为不同，不能把此改动写为运动学数值完全等价迁移。启用臂展约束时必须启用导航策略；`navigation_policy_stage=off` 的独立诊断须显式关闭 `enable_arm_chassis_coupling`，启动时拒绝无人消费的限速配置。

原控制链保存在改动前源文件/哈希或历史 Git 中；新链验证前不删除唯一可复现候选。当前完整 C++ PICK→NAV→PLACE 父任务在 14:30 留存未编译检查点，待本边界验证后续接，不重置其历史排查计时。

## 验证范围

本轮只检查真实受影响边界：五 ACK 契约、公开导航入口拒绝与失效取消、BT 中止、Nav2 各分支速度限制、约束节点无速度写入、启动单写入者。随后使用原固定工位单箱场验证上层停车与正常搬运。不得把编译、离线或隔离 ROS 协议通过写成整栈搬运验收。

证据与改动前快照：`docs/evidence/mainline_20260924/nav_boundary_refactor/`；执行结果随实际验证追加。

2026-09-24 15:20 收尾状态：修改已构建并安装到独立候选目录。最终启动装配 17 项、接线/ACK/采集静态检查 28 项通过；path_tracking 6 个 CTest、策略核心 4 个及 ownership 1 个通过。导航仲裁 13 个新协议用例均有通过记录（首轮 12/13，修正测试发现同步后仅重跑失败 1 例），兼容协议 7 例通过。导航约束消费最终 5 例和臂展生产端最终 1 例隔离 ROS 通过。臂展纯测试 3 个条目均有通过记录（首轮 2/3，修正 1 ULP 浮点断言后仅重跑失败 1 项）。不重复累计历史复跑。

搬运导航 helper 最终 3 例通过，原卡点是测试端 ACK 消费不足。完整父任务仍在原 15:10 一小时检查点暂停，尚未编译/验证其连续 PICK→NAV→PLACE 组合版本；首次 client 数量和父/子反馈 context 的工装契约问题已登记。此轮没有新 Gazebo 运动、完整录屏或真机验收。

新候选 overlay：`runs/mainline_20260924/nav_boundary/overlay.bash`；源码/安装绑定见同目录 `installed_binding_final.json`。恢复主线时必须叠加新版 dynamics、navigation policy、path tracking、arbiter 和 navigation launch 的对应安装前缀，不能只换 launch 后继续加载旧末端耦合程序。外部仿真基线仍须保留已有相机/Gazebo修正；该 overlay 不启动任何进程。

官方架构参考：[Nav2 Humble Collision Monitor](https://docs.ros.org/en/humble/p/nav2_collision_monitor/) 将独立末级监控描述为附加碰撞保护；本项目按用户明确选择取消该层，因此不宣称保有独立于导航链的快速碰撞制动能力。
