# P0—P5 异常场景与分阶段验收

更新：2026-09-19。用户确认“完善多场景验证”，不是安排夜间定时任务。本轮不创建自动化。

最新总览、施工顺序及当前阻塞修复见 [任务状态与关节跟踪问题修复](TRACKING_FIX_AND_TASK_STATUS_20260919.md)。下文按实施批次保留历史验收，不把后续仿真通过等同于全部阶段完成。

## 阶段顺序与放行条件

| 阶段 | 本阶段交付 | 放行条件与剩余工作 |
|---|---|---|
| P0 | 接口、控制租约、幂等、错误码与假后端 | 拒绝旧 boot/旧命令、重复不重发、失联结果可查询；持续回归 |
| P1 | 导航/循环路线、探索建图、记录与只读回放 | 假后端与 UI 自动测试；真实仿真建图、资源预算与现场表现另验 |
| P2 | 地图资产/工位/换图事务、Voxel 激活适配 | 假生命周期与地图证据测试；真实双地图定位及人工换层事务仍需验收 |
| P3-a | C++ 规划预览 | 已接入网关与 RViz；真实模型的 MoveIt 显示仍待验 |
| P3-b | 执行事务核心与故障测试 | 本轮实现；尚未接入 ROS 执行 action，不开放生产执行按钮 |
| P3-c | 资源锁、持久化执行意图、证据采集、执行适配 | 下一施工项；沿用 transport domain 资源锁和 hold/包络事务，接入 fake action 后才进入隔离仿真 |
| P3-d | 抓放和载荷、MTC 阶段反馈 | 完成仿真，再做授权的受控单臂/双臂验收；未知持物状态禁止自动松爪 |
| P4 | 单房间队列/WCS/检查点恢复 | P3 执行和实物状态证据闭环后接入，不提前用假结果宣称闭环 |
| P5 | 动态工位、跨楼层、载荷与故障现场验收 | 独立现场测试记录，不能由单测、SLAM 内部误差代替 |

## 本轮执行事务核心

代码：`ws_robot/src/astribot_operator_backend/include/astribot_operator_backend/arm_execution_transaction.hpp`。

纯 C++ 状态机绑定 plan_id、scene_revision、map_version、calibration_revision、payload_revision、lease_id 与有效期。可信证据由适配器采集，不能由 UI 填写布尔值来授权。`begin()` 返回 true 只表示允许适配器持久化一次执行意图；只有持久化成功后才能发送 action。核心不发布任何硬件命令。

READY → SUBMITTED → RUNNING → VERIFYING → SUCCEEDED / FAILED / CANCELED。取消进入 CANCELING，必须等待对应 action 终态及终态之后的新停稳、载荷和结果证据；缺失或超时进入 UNKNOWN，禁止自动重试。取消与成功竞争时保留设备真实成功结果，不把成功改写为取消。迟到消息不能让 UNKNOWN 自动恢复。

当前单次消费约束只覆盖一个内存事务，跨进程/重启的去重和恢复必须由下一步持久化台账实现。`blocksRelease()` 是适配器必须遵守的约束，不等于本轮已持有真实资源锁。UNKNOWN 只提供人工核对入口，核心不实现“强制恢复成功”。执行前证据年龄 < 1 秒；ACK/取消/终态验证超时 5 秒，执行上限 120 秒。这些值当前是核心固定契约，生产接入前需形成版本化配置与日志快照。

## 异常矩阵

“自动”表示已有对应自动测试入口，不代表真机通过。每条验收均要求：操作 ID、boot、版本、预期/实际状态、命令次数、失败原因、证据等级与日志位置。

| 编号 | 触发/注入 | 期望行为 | 入口/层级 |
|---|---|---|---|
| A01 | 两个客户端争夺控制；租约超时 | 单一控制者；失去租约请求取消，终态前不交接 | test_backend，自动假 ROS |
| A02 | 同 ID 重复请求/同 ID 不同参数 | 重复不执行；内容变化拒绝 | test_backend / test_loop_route，自动 |
| A03 | 后端重启、旧 boot/路线 ID | UI 控制权失效；旧请求不能重启任务 | test_workstation / test_loop_route，自动 |
| N01 | 多点顺序、首尾循环、停留期间取消 | 逐点串行；取消后不进入下一点 | test_loop_route，自动 |
| N02 | 目标拒绝/失败/被抢占/超时 | 停止路线，不悄悄跳点 | test_loop_route，自动 |
| N03 | 取消发生在 goal ACK 前；服务不可用 | 迟到 goal 被定向取消；不发新 goal | test_loop_route，自动 |
| E01 | 暂停探索与取消探索 | 暂停不保存；取消只发一次结束/保存请求 | test_exploration_end，自动 |
| E02 | 无停稳 odom；SLAM 只发 final 信号但资产缺失 | 不提前保存成功；超时/文件不完整明确失败 | test_mapping_session，自动假服务 |
| E03 | 存图禁用、重试、重复结束事件 | 禁用不调用 finish；重试校验资产与幂等 | test_mapping_session，自动 |
| E04 | 开阔/窄通道/孤立区域/无前沿/动态障碍 | 区分探索完成、暂时无目标和失败预算；保存与地图质量分别验收 | 待隔离 Gazebo 场景验收 |
| M01 | 地图文件损坏、路径越界、源目录删除 | 拒绝篡改/越界；已归档副本可用 | test_catalog / test_voxel_activation，自动 |
| M02 | 换图中断/重启/验证超时/恢复 | 保留旧地图事实；进入恢复状态，不盲目重放 | test_map_manager / test_catalog，自动 |
| M03 | 外部 SLAM 已运行、错误 map_topic、缺少激活证据 | 不抢占外部进程；所有证据满足才 READY | test_voxel_adapter，自动假生命周期 |
| M04 | 真正两张地图的重定位、跨层人工确认、载荷未知 | 地图与定位版本一致才导航；载荷未知禁止换层放行 | 待隔离仿真/现场 |
| R01 | 参数事件丢失、端点变化、删参、半行记录 | 不沿用旧值；显示未知区间 | test_evidence，自动 |
| R02 | 记录启动/停止、缺失参数、事件循环 | bag 与参数证据可读取，缺失有明确标记 | test_recorder，自动 |
| R03 | 回放运动话题、伪造类型、seek | 白名单拒绝运动/伪造类型；seek 重建只读流 | test_workstation / test_replay_stream，自动 |
| R04 | 磁盘将满/写失败/进程中断/长时间资源负载 | 已落盘证据保留，结果失败可诊断；受限资源仍可取消控制任务 | 待故障注入，禁止在共享盘上填满磁盘 |
| P01 | 规划 NaN、关节偏移、地图变化、计划过期 | 计划无效，不显示为可执行 | test_arm_preview，自动假规划 |
| P02 | 取消后规划迟到、控制权丢失 | 迟到结果不能恢复 READY | test_arm_preview，自动 |
| P03 | 非法时间戳、尝试动态修改固定安全参数 | 输入拒绝；参数修改拒绝，实际值一致 | test_arm_preview，自动 |
| X01 | 前置证据：多来源、租约、资源锁、base hold、控制器、起点、载荷缺失 | begin 不授权派发，不消费计划 | test_arm_execution_transaction，自动纯 C++ |
| X02 | 场景/地图/标定/载荷/租约/计划版本变化，证据过期或未来时间 | 不授权派发 | 同上，16 项参数化 admission 场景 |
| X03 | 重复执行、错误目标 ID、重复终态 | 单次消费；忽略其他操作反馈 | 同上 |
| X04 | ACK 前取消、成功与取消竞态、ACK 超时 | 迟到 handle 需取消；保留真实结果；超时 UNKNOWN | 同上 |
| X05 | action 成功但未停稳/无夹持证据/资源锁丢失 | 保持 VERIFYING，超时 UNKNOWN，不允许释放事务 | 同上 |
| X06 | 取消无回包、持久化失败、时钟回退、迟到结果 | UNKNOWN，禁止重新派发/自动恢复 | 同上；磁盘落盘失败仍需适配器实测 |
| X07 | 实际控制器断连、负载滑落、接触碰撞、双臂互扰 | 停止/保持由硬件适配器确认，不自动松爪或归位 | 待 fake action、仿真和受控现场 |
| W01 | WCS 重复任务、ACK 丢失、任意阶段重启 | 去重与事实恢复；禁止重复抓放 | P4 实现后自动化 |

## 执行方式与证据保存

使用现有功能包内 C++ gtest/CTest，不增加采集脚本。隔离构建目录 `/tmp/astribot-route-build`；测试各自使用源码 CMake 定义的 localhost domain（214、215/216、220/221、223、228/229/230 等），按包串行。不启动仓库默认 domain 25 的仿真，不清理共享栈。

当前构建与单包测试日志：`/tmp/astribot-p3-scenarios-build.log`、`/tmp/astribot-p3-scenarios-test.log`。
跨模块回归日志：`/tmp/astribot-multiscenario-regression.log`。机器可读结果在 `/tmp/astribot-route-build/build/<package>/test_results/`。

```bash
source /opt/ros/humble/setup.bash
source /tmp/astribot-route-build/install/setup.bash
colcon --log-base /tmp/astribot-route-build/log test \
  --base-paths ws_robot/src/astribot_operator_backend \
    ws_robot/src/astribot_operator_station ws_robot/src/astribot_map_manager \
    ws_robot/src/astribot_route_executor ws_robot/src/astribot_s1_exploration \
  --build-base /tmp/astribot-route-build/build \
  --install-base /tmp/astribot-route-build/install --executor sequential
```

进入 Gazebo 验收前另建独立启动配置，验证 ROS domain 与 Gazebo partition 的实际隔离。不能只 export ROS_DOMAIN_ID 后启动仓库默认仿真来宣称隔离。真机阶段需设备状态、场地、人员和授权明确后另行验收。

## 前一轮实际结果（纯核心和假后端）

最终构建成功，`git diff --check` 通过。以下为本轮重新运行的 gtest 用例数，非 CTest 包装层计数：

| 功能包 | 用例数 | 失败/错误/跳过 |
|---|---:|---|
| astribot_operator_backend | 35（含新增执行事务 32 项） | 0 / 0 / 0 |
| astribot_operator_station | 13 | 0 / 0 / 0 |
| astribot_map_manager | 10 | 0 / 0 / 0 |
| astribot_route_executor | 11 | 0 / 0 / 0 |
| astribot_s1_exploration | 6 | 0 / 0 / 0 |
| 合计 | 75 | 0 / 0 / 0 |

构建提示当前隔离安装目录同时作为 underlay，未出现编译错误或 C++ 编译器警告。所有结果仅为纯 C++ 核心、隔离假 ROS 后端及离屏 Qt 验证；没有启动真实机械臂执行、真机运动或新 Gazebo 实验。生产执行入口仍保持拒绝，P3-c/P3-d 与 P4/P5 未宣称完成。

## 后续隔离 Gazebo 验收增量

用户确认先仿真、后真机。本轮新增 C++ `astribot_sim_validation`，domain 213 + 独立 Gazebo partition 和地图目录。具体实现、实跑结果与证据索引见 [仿真链路记录](SIMULATION_OPERATOR_CHAIN_20260919.md)。

| 编号 | 实跑结果 | 尚未通过的验收 |
|---|---|---|
| S01 | 时钟/关节/里程计/扫描推进，8 控制器 active | 不等于完整功能就绪 |
| S02 | 上位机租约 → 抓取 → 附着 → 行走姿态 → 导航故障回传 | 原工位导航阻塞，放置未完成 |
| S03 | 两客户端控制权冲突被拒绝，多网关来源时拒绝派发 | 租约丢失后各运动阶段的取消停稳仍待实跑 |
| S04 | 目标栅格对齐仍导航失败；增加侧向净空则预抓取规划失败 | 路径预测与起步朝向关系待定位 |
| S05 | 失败保留物体 ATTACHED/WORLD 的账本事实，会话进入恢复状态 | 现场恢复事务尚未完成 |

本轮不是 P3 完成验收，也未将 P4/P5 标记完成。继续按成功闭环、取消与失联、探索建图、多图/WCS 的顺序推进。

### 仿真入口修正后的增量

| 编号 | 最新证据 | 边界 |
|---|---|---|
| S06 | 仿真感知/导航/执行统一 fixed_v2 + 自动通道策略 p5；几何从高度过滤拒绝恢复 GEOMETRY_CURRENT | 导航策略名不等于交付阶段 P5 |
| S07 | v2c 原工位抓取、附着、首个离台目标成功；第二目标 POLICY_LEASE_EXPIRED | 闭环仍失败；新增实际约束录包和双时钟租约诊断继续定位 |
| S08 | 5 秒时主动取消，6.178 秒终态；账本 CANCELED，WORLD，无附着、stop_error 空 | 仅覆盖抓取前阶段，不代替附着后/导航中/放置中取消 |
| S09 | C++ 派发前拒绝缺失/不完整/未确认/未来/过期几何；4 项核心 gtest 通过 | 几何参数与真实传感器可见范围分别验收 |

路径跟踪插件装载、通道到位、通道路线共 3 项 CTest 通过。新增内容和证据目录继续维护在上述仿真链路记录中。

### 时钟修复后的最新验收

- S07 的下一轮直接定位到约束源时间快 1 ms；改为有界零速度等待时钟追上，真实过期仍失败。新增 policy_clock 回归后，路径跟踪共 4 项 CTest 通过。
- S10：`operator_clockfix` 完成抓取、附着、两个导航目标、放置、收臂和回传；214.703 秒，SUCCEEDED / PLACED / 无 attachment。录包正常关闭，关键参数和实际约束/几何消息可读。
- 正常闭环目前成功 1 轮；S08 只覆盖抓取前取消。附着后取消、运行中失联、重复成功率、探索/地图/回放组合仍待后续独立验证。


### 阶段触发故障注入增量

| 编号 | 结果 | 验收边界 |
|---|---|---|
| S11 | C++ 阶段触发异常验收、8 项仿真 gtest 通过 | 测试工具编译和单测，不等于所有现场用例通过 |
| S12 | 带载行走时停止续租，实测 0.020289 m/s；控制权失效、CANCELED、停稳、零命令、载荷保留及恢复门禁全部通过 | 1 轮 Gazebo 运动学附着；8.336 s 为停止续租到验收完成，不是制动时间 |
| S13 | 收臂 TRANSPORT_POSTURE 取消：前两次取消终态成立但验收时序错误；修复后第三次出现 FAULT/WAIT_TIMEOUT | 取消及时中断机械臂仍未验收，保留失败证据 |
| S14 | 带载导航主动取消首轮在注入前 PATH_QUALITY_UNSAFE 退出 | 未发生故障注入，不能计入取消通过率；需验证姿态/包络/离台净空稳定性 |
| S15 | 录包增加载荷附着状态和异常用例关键参数，SQLite 与参数快照实查可读 | 参数回读仍标记 effective_confirmed=false |

下一步先解决 S13 的动作取消传播和 S14 的起步净空重复性，再继续时钟暂停、进程重启、探索存图、循环导航及回放联合验收。保留已有安全拒绝和恢复门禁，不推进真机。

### 取消执行与静止保持增量

详见 [逐轮证据与控制边界](CANCEL_AND_STATIONARY_HOLD_20260919.md)。

| 编号 | 最新结果 | 验收边界 |
|---|---|---|
| S13 | 新 C++ MoveIt capability 在 r6 活动收臂速度 0.052621 rad/s 时接受取消，TEM 返回 PREEMPTED，CANCELED/USER_CANCEL、载荷保留、恢复门禁及独立停稳全部通过 | 取消受理至 TEM 终态约 33 ms；独立验收 2.778 s，均不等于物理制动时间。单轮仿真通过 |
| S14 | 旧失败包导航前底盘已漂移约 0.451 m，cmd_vel 全为零；仿真新增默认关闭的轮位保持选项，5 项回归通过 | r9 在收臂时触发原有 20 mm 漂移保护，未进入导航；不算起步碰撞问题已修复 |
| S16 | recorder 增加轮端力矩、静止保持、MoveIt capability 和 MTC 缩放参数记录 | r9 录包未正常关闭，保留 database is locked 异常；仅正常停止的 incident 可作为完整回放交付 |

真机桥接仍按现有位置目标写入逻辑工作，本次未修改或操作真机。

后续同轮验证：S14 的 r10 在显式保持增益10、MTC缩放0.03下完成完整搬运（140.308 s，SUCCEEDED/PLACED）；r12 在实测0.020111 m/s带载行走时主动取消，通过停稳、载荷保留和恢复门禁，取消到验收完成2.725 s。r11同配置收臂关节3跟踪超限，取消尚未注入，因此重复稳定性仍未通过。r10/r12录包正常关闭且关键参数/控制器/保护消息已实查；详见上述逐轮记录。
