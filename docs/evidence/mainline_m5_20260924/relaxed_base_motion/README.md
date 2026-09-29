# 私有验证脚本：显式仿真底盘放宽条件

2026-09-25，按主任务转达的用户明确授权修改。M5 仅修改 `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring/` 下两个私有脚本；不修改 runner、launch、native、MTC、产品、夹具或历史场景副本。

| 文件 | 冻结 SHA256 |
|---|---|
| verify_full_transfer.py | `1333136611ff06fbf31d285858c61833893c6997f11a8937ec936b9ac3081184` |
| read_full_executor_parameters.py | `6db3a2214f222c66f02eff73d9deac3b8d6cbc698321bb4b4f8a653ea655240c` |

`--relax-base-motion` 默认 false。false 保持非 NAV 实际角速度阈值 0.03 rad/s；true 使用 0.10 rad/s。实际线速度阈值仍为 0.02 m/s，非零命令拒绝阈值仍为 1e-6，比较运算的原有边界语义不变。最终停车检查未放宽。

父 Goal 提交前，分别读取 `/task_trajectory_executor/get_parameters` 和 `/manipulation_execution_guard/get_parameters` 的 `simulation_relaxed_base_motion`。两端均要求恰好一个 bool 类型值且与 CLI flag 相同；缺失、错误类型或不一致均快速失败，并保存实际读回内容。仿真旋转偏移容差由 native/guard 实现，本脚本通过该明确开关绑定运行条件，不新增旋转检查实现。

`base_motion_conditions` 报告同时记录原 rotation=0.02 rad/angular=0.03 rad/s 与当前 rotation=0.05 rad/angular=0.10 rad/s（放宽时），并标记 `relaxed_simulation_not_original_precision_acceptance`。放宽实验通过不能记成原精度验收通过。

注册读取器要求实际 chassis `idle_position_hold` 为 bool true，否则以 `CHASSIS_IDLE_POSITION_HOLD_REQUIRED` 终止。它只校验，不设置底盘参数；永久默认值和私有启动层由主任务负责。

离线验证执行真实生产代码片段：15/15 定向用例通过，覆盖严格/放宽运动接受与拒绝、线速和非零命令拒绝不变、NAV 分支不变、两节点配置匹配/不匹配/未声明，以及 idle true/false/类型错误。CLI 默认值、条件报告和语法另行检查通过。stop 与后缀交接块和修改前逐字相同，只读同行复核无脚本阻断。

复现入口：`python3 docs/evidence/mainline_m5_20260924/relaxed_base_motion/check_offline.py`。证据包含 before/after、最小 patch、离线结果与哈希清单。本次没有 ROS/build/GPU 操作；参数服务使用离线 mock，不代表实际节点已经安装、读回成功或整场仿真通过。guard/native 候选及现场读回由主任务负责。
