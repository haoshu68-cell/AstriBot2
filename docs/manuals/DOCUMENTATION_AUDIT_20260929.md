# 架构与手册核验记录

日期：2026-09-29。基点 `2c5354c3fe2d4df46103684182c1ab852a1cf64e` / `chassis-effort-drive`。本轮只修改当前手册、根 README 入口和统一仿真 launch 的参数转发；不更新第三方子模块，不覆盖原有 `docs/evidence/mainline_20260929/`，不构建/替换正在使用的 ROS 库。

## 1. 交付与成功标准

| 标准 | 本轮结果 |
|---|---|
| 总体功能/控制权/数据/部署/故障/证据视图 | 总架构文档完成，包含 4 图及关键接口表 |
| 每个职责模块有细化图、代码包、功能状态与缺口 | MODULE_ARCHITECTURE 的 M01–M16 共 16 个模块，逐项覆盖 |
| 代码包没有遗漏/重计 | 直接扫描 ws_robot/src/*/package.xml，49/49 映射；47 AstriBot、2 第三方；根 SDK/消息另列 |
| 仿真与真机操作 | 各有构建/部署、环境、启动、就绪、任务、保存、关停、证据/排障；阻塞步骤明确标注 |
| 图可解析、链接可定位、命令语法有效 | 21 张 Mermaid 实际渲染；文档本地链接/模块锚点和 bash 代码块静态检查；总览 PNG 人工查看后导出 SVG |
| 明显配置遗漏最小修复 | sim_stack.launch.py 补齐 6 项已有 supervisor 参数的声明及转发，默认行为和保护不变 |

图渲染使用临时目录中的 Mermaid CLI 与本机 Chrome，没有给仓库添加 npm 依赖；仅总览 SVG 作为文档产物保存。图的可渲染性不等于架构运行正确。

## 2. 配置修复

文件：[sim_stack.launch.py](../../ws_robot/src/astribot_s1_navigation/launch/sim_stack.launch.py)。原 CLI 主管已有下列参数，但 ROS launch 风格入口未声明/转发，导致两种入口功能不一致：

| 新补齐的 launch 参数 | 对应主管参数 | 用途 |
|---|---|---|
| save_session | --save-session | 保存完整 Voxel 会话的目标目录 |
| match_threshold | --match-threshold | 已保存地图的定位匹配门槛 |
| payload_source_id | --payload-source-id | 隔离 fixed_v2 中显式库存来源 |
| social_policy | --social-policy | 显式 H2 社交策略选择 |
| obstacle_layer_plugin | --obstacle-layer-plugin | 选择已有受支持 obstacle/voxel layer |
| exclusive_performance | --exclusive-performance | 已有性能独占要求，false 时不发开关 |

复用既有 supervisor 的选择、范围和前置条件检查，没有新增运行时算法或兼容层。地图路径、source ID、硬件证据、碰撞/租约门槛没有自动填造或放宽。启动文件属 Python 允许范围，按 robot-runtime-cpp 技能执行。

## 3. 已执行验证

1. **真实 launch 替换 → supervisor dry-run：4 组通过。** 使用源码 generate_launch_description 声明的默认值、真实 LaunchContext 和 ExecuteProcess 命令替换，将所得命令交给现有主管，仅带 dry-run。覆盖：mapping+fixed_v2 的保存/库存/独占/VoxelLayer；localize 的 0.42 阈值；H2 社交选项；手册 p3 mapping。检查预期最终子命令与“不创建会话目录”。定位组只使用参数校验所需的临时文件，不是有效 SLAM 地图或定位验证。
2. **非法布尔值：1 组通过。** exclusive_performance 非 true/false 在启动命令生成处被明确拒绝。
3. **既有 SLAM 启动所有权回归：6/6 通过。** 在加载 ROS Humble 与现有工作区后执行 `python3 -m unittest discover -s tools/test -p 'test_slam_launch_ownership.py' -v`。此测试解析真实 launch 和 dry-run，不启动传感器/控制器。
4. **真机启动计划只读核查。** `hardware_exploration.py --navigation-only --dry-run` 确认 off 与 coupling=true 的冲突仍存在；没有执行计划。
5. **文档静态检查。** 本轮 7 份手册的 386 处本地链接、50 处模块锚点引用、49 包集合、16 模块图覆盖、22 段 bash 代码块语法、launch Python AST、git diff --check 均通过；21 图完成渲染，总览重新布局后复渲染并目视检查，无文字重叠/裁切。

首次参数验证把 H2 场景文件定位到错误包目录，被主管按“文件不存在”拒绝；改用实际 gazebo_bringup/config/social/h2_empty_observed.yaml 后通过。该失败属于本轮验证路径错误，不作为产品缺陷或一次成功运行。

## 4. 已核对的实现与证据差异

| 旧描述或易误读处 | 当前核对 |
|---|---|
| 19 个包、9 月 17 日为“当前手册” | 已替换为 9 月 29 日逐包清单与版本范围 |
| 独立 FinalProtection 末级速度链 | 当前 launch 为上游约束，适配后直接进入 cmd_vel |
| 六方包络 ACK | FixedEnvelopeCore 当前列五方；自滤确认属于几何证据链 |
| SlipMonitor/LinearSlip 和 slip.mode | 当前控制实现已删除，旧说明不继续使用 |
| 原生搬运还完全未接通 | 当前已有完整原生 FixedStationTransfer 与 scene96 限定成功证据 |
| 一个成功视频意味着真机/严格精度通过 | scene96 acceptance.strict_2mm_passed=false，仅 Gazebo 运动学附着，临时 3 cm / 0.1° |
| 默认 precision 或 baseline 命令可直接照抄 | 实际存在策略/上肢限速冲突、静态地图 SLAM 实测配准要求 |
| 停车脚本反馈为 true 就完成新契约 | 当前真机旧停止器仍用 odom，不能证明 SLAM 口径停稳 |
| UI 搬运按钮就是 native 完整主线 | transport_session 仍调用旧 Python 路径，臂预览 can_execute=false |

scene96 的[验收说明](../evidence/mainline_20260929/PLACE_ACCEPTANCE.md)及其[原始 acceptance.json](../../runs/mainline_20260928/workstation_alignment/full_transfer_runner/front_transfer_scene96_transfer_budget_place_3cm_world44_20260928/acceptance.json)本轮只读核对：TRANSFER_COMPLETE、resources_released、最终 EMPTY、0.693 s SLAM 停稳、bag 返回 0、2527 帧核验。没有重跑，没有将该记录升级为当前源码全栈回归。

## 5. 未验证与恢复入口

- 未运行新的 ROS 图、Gazebo、GPU 推理或机器人运动；没有硬件验收、编译全部包或更新安装空间。
- 真机策略 profile/接线、SLAM 停止器迁移、静态基线自动实测配准不是简单配置拼写修正，保留原安全拒绝。分别从 navigation.launch.py、hardware_exploration.py、robot_task_control.py、perception_slam_bringup.launch.py 恢复。
- 原生固定工位成功场景仍依赖本地 runs 内的专用运行器、配置和私有产物清单；没有假装提供跨机器通用一键搬运命令。运行原始证据可能不随 Git/部署传输，需单独保留。
- FoundationPose、物理硬同步、接触/力控、真机载荷、自动回充、全系统强制资源仲裁等缺口逐模块明示；未因文档完成将其记为通过。
