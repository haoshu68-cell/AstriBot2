# 导航边界修改最终验证索引

记录时间：2026-09-24T15:20:52.221000+08:00。只读取原始日志/XML/result，没有重新构建、启动ROS或改动产品。**当前离线与隔离协议证据已取得；尚未完成整栈仿真验收。**

| 范围 | 证据层级 | 当前采用的记录 | 历史记录处理 |
|---|---|---|---|
| 启动与上肢限速装配 | 静态 pytest | 最终17用例通过 | 前两轮16/14通过保留，不加总 |
| 约束/ACK接线 | 静态 pytest | 最终28用例通过 | 初版28保留，不加总 |
| path_tracking | C++ 离线 CTest | 6 条目通过 | 内部断言不重复加总 |
| 策略核心 | C++ 离线 CTest | 4 条目通过 | 内部GTest不重复加总 |
| ownership | C++ 离线 CTest | 1 条目通过 | 不是Action通信闭环 |
| 上肢限速核心/边界 | 离线 CTest | 首轮3项中2通过、1失败；失败单项重跑通过 | 不写成完整3项重跑通过 |
| navigation_constraint | 隔离 ROS GTest | 最终5用例通过 | 初版4通过保留，不加总 |
| fixed_v2 arbiter | 隔离 ROS pytest | 首轮12通过、1失败；失败单例重跑通过 | 不写成完整13例重跑 |
| legacy arbiter | 隔离 ROS pytest | 7用例通过 | 单列兼容模式 |
| navigation helper | 隔离 ROS GTest | `navigation_helper_drained` 3/3通过 | 此前4轮失败全部保留 |
| 上肢限速生产者 | 隔离 ROS pytest | 警告修复后的final 1例通过 | 初版同1例通过保留，不加总 |

不将不同层级和包含关系中的数字相加成总测试数。6个path_tracking条目是policy_recovery、envelope_evidence、policy_execution、policy_clock、policy_velocity_restriction、corner_contract；其中分别可见7、8、73项内部断言。4个策略条目是publication_lease_test、cmd_vel_body_to_world_core、fixed_hold_flow_test、fixed_envelope_authority；前者14个、fixed_hold_flow_test 32个内部GTest均是包含关系。

最终启动检查还确认：behavior_server只加载Wait，未经MotionConstraint适配的Spin/BackUp/DriveOnHeading在主线装配中停用；三棵当前BT不使用这些动作。这是上层运动入口收敛，没有新增最终命令门控。

## 原始失败与后续记录

- fixed_v2首轮失败发生在 `test_fixed_v2_revoke_then_immediate_recovery_keeps_terminal_barrier` 初始目标等待backend handle阶段，尚未进入预期撤销段。之后仅该例通过；不从目录名推断唯一根因。
- 上肢限速首轮 `arm_navigation_limit_core` 在第21行严格浮点相等断言失败。修正测试允许 `1 - (1 - .15)` 相对 `.15` 的1ULP舍入，采用 `std::nextafter(.15,1.)`；runtime阈值未改。其后仅该core条目重跑通过，另外两个原通过条目不重复计数。
- policy初次构建因Humble不提供所用Node图查询API失败，修复构建记录保留；arbiter初次误用旧overlay头文件，缺 `navigationReason/sameExecution`，新头文件构建另存。
- navigation helper首轮3例中1通过、2失败；diagnostics两例失败；detailed轮缺 `ASTRIBOT_FIXED_STATION_TEST_DOMAIN`，属于测试环境失败；detailed_bound两例缺有效local_costmap ACK。最终drained轮3例通过，旧失败不覆盖、不改写为通过。
- 固定工位scene是相关主线证据，**不计入本次导航边界验收**：初次编译遇到std::apply ADL错误；初次运行因canonicalScene动态库符号缺失exit127。绑定正确库后1个CTest内4个GTest通过，旧失败完整保留。

## 进程收尾与验收边界

12个已结束ROS运行的result均记录 `still_alive=[]`，保留PID/start_ticks、boot_id、domain36、localhost-only、RMW、命令、起止时间及返回码。这只支持“各runner观察到的自有进程身份在收尾时均不再存活”；100ms采样不保证捕获全部短命进程，本次未做全机在线检查。

最终constraint的5例和producer的1例是独立fixture验证；导航助手3例使用fake Nav2/odom/ACK。不能升级为同一Gazebo实际链路，更不代表完整抓取→载荷导航→放置→EMPTY→资源释放及录屏已验收。平滑器正常减速尾段不等于立即物理停车；动态载荷加速度完整贯通和真机验收仍未完成。

## 证据位置

- [机器可读索引](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/mainline_20260924/nav_boundary_refactor/validation_index.json)：逐项用例、失败文本、原始证据绝对路径、大小与SHA-256；`latest_evidence_by_scope`明确当前采用哪轮，`runs`保留历史。
- [原始离线/静态日志](/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/nav_boundary/logs)。
- [原始隔离协议记录](/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/nav_boundary/protocol)。

未复制构建目录。根任务另产出的 `installed_binding_final.json` 是独立安装绑定manifest，不作为本历史验证索引的输入。后续证据变化须重新核SHA，不能沿用本次索引宣称覆盖新版本。
