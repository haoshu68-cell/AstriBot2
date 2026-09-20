# 探索、存图及多地图验收续作

前置：夹爪 mimic 修复后，r23/r24 两次完整搬运通过，见
[关节与接触诊断](GRIPPER_MIMIC_AND_CONTACT_REPAIR_20260919.md)。本页仅记录其后的仿真验收，不涉及真机。

## r25：真实 Voxel 取消存图

新增功能包入口 `astribot_sim_validation/launch/mapping_validation.launch.xml`，使用自有 domain 213/专用 Gazebo partition，替代静态地图世界，不同时启动两个世界。真实 Voxel、概率栅格、探索与存图节点一起启动；探索初始暂停，Nav2 单独启动。存图目录默认为 `/tmp/astribot_validation_213/slam_sessions`，地图名必须唯一。

首次启动因隔离安装未包含 `astribot_s1_mapping` 失败；补构建该既有 C++ 包后重启。此为构建闭包缺失，不计探索通过。实际进程环境已反读；仿真probe测得8个控制器active，clock/joint_states/odom/scan均推进，Nav2全部active。

暂停阶段 `/mapping_session/status` 为 IDLE、finish=false。恢复后前沿搜索得到候选，但规划器均返回 `ENVELOPE_V2_NOT_READY`，没有探索导航目标下发。再次暂停后finish仍为false，没有误将失败候选当作建图完成。

正常取消后实际链路：

| 阶段 | 墙钟 Unix 秒 | 证据 |
|---|---:|---|
| WAIT_STOP | 1789827709.842 | 冻结探索，等待新鲜停稳观测 |
| READ_CONFIG | 1789827710.418 | 确认保存已启用、会话名/目录 |
| FINALIZING | 1789827710.518 | 一次设置真实 Voxel finish |
| 栅格写完 | 1789827710.571 | 278×416，PGM/YAML |
| SAVED | 1789827710.717 | 文件检查通过、原子提交 manifest |

实际成果目录：`/tmp/astribot_validation_213/slam_sessions/mapping_cancel_r25/`。
manifest 标记 `CANCELED_PARTIAL`，1349条扫描位姿、1个关键帧；PGM、YAML、alidarState和PCD的SHA256已由现有session读取器再次校验一致。取消后resume被拒绝，重复cancel受理但没有第二次finish/保存流程。

此结果只证明**未下发导航时的暂停/取消/真实存图**。没有覆盖运动中取消后制动到停稳，也没有证明自主遍历、地图质量或自然完成。Voxel正常结束后数据和TF陈旧是已结束会话的事实，不能继续用旧会话派发导航。

录包正常关闭：`/tmp/astribot-joint6-fix/incidents_r25/incident_1789827605758029717_442445`。保存后按PID/启动时间清理39个自有进程，remaining为空，其他实验栈保留。

## 找到的探索接入缺口与本轮修复

`fixed_v2` 的保持确认/包络握手目前由搬运执行器完成，探索入口没有对应的任务所有者。不能通过伪造 ArmHoldStatus、仅观察零速度或关闭包络保护来声称具备移动准入。

本轮先修复错误的就绪展示与选点入口：探索 C++ 节点新增只读 `require_fixed_envelope`；启用时要求唯一发布源、合法且新鲜的时戳/有效期、固定姿态模式、hold/session/epoch和导航允许标志，之后才做地图/位姿/里程计/校验图检查。上位机显示具体ENVELOPE_V2原因，而不是反复消耗候选规划失败预算。`nav2_full_bringup` 在 fixed_v2 下自动传入，独立入口默认false以保持既有legacy接口；参数加入回放白名单。该检查不发布保持确认、不授予任何新的机械臂控制权。

## 下一门槛，按顺序处理

1. 在上层补通用 C++ 导航姿态会话所有者：与搬运共用资源互斥、确认控制器占用及稳定参考姿态、执行 fixed_v2 握手；臂任务接管前撤销，所有者丢失/版本变化必须停止放行。不得让探索节点自行伪造保持事实。
2. 验收探索实际选点与导航、运动中pause/resume、运动中cancel后停稳存图；用已知封闭场景验证自然无前沿完成，未知区域/不可达前沿不能算完成。
3. 之后再用两份真实会话走导入、激活、重定位、代价地图刷新、人工跨层中断/恢复；当前只有单份局部地图，**多地图实跑未验收**。

既有自动回归在本轮重跑：探索2个CTest套件、地图管理4个CTest套件全部通过（suite数，不冒充6个用例）。其假依赖/事务边界结果不能替代上述真实运动和重定位门槛。

包络检查新增2个C++用例通过：纯边界检查覆盖缺失、重复来源、未来/过期/非法时间和缺失保持确认；节点级检查确认未收到包络时ready=false、没有在途导航，并拒绝运行中关闭检查。探索2个CTest套件在最终构建后再次全部通过。正常关闭的r25包中8485帧odom最大平移速度为0，未见goal_in_flight，存图状态转换完整；进一步限定了本次静止取消的证据范围。

构建使用私有 `/tmp/astribot-route-build`，曾因CMake缓存仍指向旧消息安装缺少NavigationEnvelopeV2而失败；清理本包CMake缓存后重新解析正确消息前缀。最终探索、验证包、导航launch和操作台4包构建通过，launch语法和git diff --check通过。未改SLAM源码及其注释。
