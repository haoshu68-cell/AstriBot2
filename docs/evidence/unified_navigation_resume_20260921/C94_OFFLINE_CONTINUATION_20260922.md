# C94：等待计时与转角状态证据的离线收尾

本轮先审查每项的数据流、状态切换、边界与恢复逻辑，再实施和验证。用户最后一次明确重新计时为 **2026-09-22 22:06:27 +08:00**，90 分钟上限为 **23:36:27**。本轮在上限内完成下述离线范围。

**状态：源码已写回，离线验证通过；C/I0.1 仍未通过。** 没有启动 ROS 节点、Gazebo、RViz 或真机，没有替换共享 install，也没有部署到机器人。独立安装目录仅用于对象测试与插件构造。运行参数中的速度、增益、到位阈值和停车阈值均未调整。

## 逐项逻辑检查与修改

| 顺序 | 先检查的逻辑 | 修复及验证 |
|---|---|---|
| 1 | 换路发生在 HOLD 计时之前；新阶段和继承阶段的预算语义不同 | 只扣除等待区间与当前阶段生存区间的交集，避免阶段开始时间被推到未来。覆盖 0.1/60 s 间隔、等价刷新、活动角点换路及停稳窗口复用 |
| 1 | 行人静止与循环路线也必须进入静态净空检查 | 无目标时检查初始位置一次；循环路线补最后目标到首个目标，不能接回出生位置。占据、地图外、自由区、单目标/非循环反例均通过 |
| 2 | 控制状态变化与诊断记录是否保持一致 | 日志补执行代次、事件序号、控制时间和位姿可用性；等价刷新输出保留状态；复位、退出、清理显式输出 IDLE。清理记录仍绑定被结束的执行 |
| 3 | 解析记录、匹配阶段和派发记录之间是否会串任务或使用过期凭据 | 显式绑定执行代次；无位姿的复位也能撤销旧记录；旧执行、重复事件、乱序、版本变化和失效时间不能生成新 READY |

第 1 项候选原先留在 C93 隔离目录，本轮重新构建和验证后才写回。详细先行审查分别保存在 [任务 1](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/task1_logic_review.md)、[任务 2](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/task2_logic_review.md)、[任务 3](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/task3_logic_review.md)。

等待计时反例：HOLD 上一拍在 200 s，260 s 换路创建新阶段，旧逻辑会把新阶段起点从 260 推到 320 s。修复后仍为 260 s。若该阶段在等待之前已有 1 s 有效运行，则刷新/继承后仍保留这 1 s 的预算使用量。

## 数据流与日志契约

```text
C++ 控制器的显式子状态/复位/路径刷新
  → CORNER_STATE schema=1
  → 离线解析 + 录制器提供的会话/发布者/时钟代次
  → 执行编号、路径修订、角点序号、时间与独立高层相位共同匹配
  → READY_FOR_INJECTION（仅时序证据）
  → 派发前再次检查全部时间条件
  → ACK 与实际物理效果另行验收
```

日志保留旧的四字段前缀，并追加：

```text
CORNER_STATE state=TURNING cursor=0 revision=1 pose_s=200.000000000 schema=1 execution=1 event=3 ros_s=200.000000000 pose_valid=1 active=1 reason=transition
```

- `execution` 是进度检查器提供的任务执行代次；`event` 在控制器对象存活期间递增；进程重启还须由会话/发布者和时钟代次区分。
- `ros_s` 是控制器输出记录的 ROS 时间，`pose_s` 是被观察位姿的源时间，接收 ROS/单调墙钟由录制器另存，三者不能互相替代。
- IDLE/复位允许没有位姿，此时不能满足阶段命中，却必须撤销旧候选。
- 旧日志可在明确的 `LEGACY_CONTEXT_ONLY` 模式下读取；不能通过外部补字段伪装成 `EXECUTION_BOUND`。
- 日志仍按状态事件输出，没有增加逐拍心跳。长 HOLD 后若没有新鲜匹配事件，结果继续为 NOT_READY；没有扩大 0.3 s 的观测窗口来凑通过。
- 当前工具是离线验证脚本，没有向机器人发指令。在线采集/注入适配仍为 **NOT_CONNECTED**；发布者归属与外部元数据真实性还需在线链路核验。

## 独立复核与反例

新上下文审查发现 3 个 P2 问题：上下文失效后旧派发凭据未清除、外部字段能把旧日志升级为强身份记录、派发前未复核控制事件年龄。均先复现失败再修复；同时修复接收 ROS 时间的对应缺口，并拒绝 reset 与活动 TURNING 相矛盾的输入。

[复核处置记录](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/review_resolution.md)；[修复前反例](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/review_all_red.log)中 16 项测试含 5 个失败案例，修复后 16/16 通过。原来的 22 项判据回归保持通过。未将收到 ACK 写成物理效果通过。

## 本轮验证结果

| 验证层与范围 | 结果 | 证据 |
|---|---:|---|
| 隔离 C++ 构建与 CTest | 13/13 通过 | [ctest_final.log](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/ctest_final.log) |
| 上表中的控制契约测试 | 73 个断言通过 | [contract_verified.log](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/contract_verified.log) |
| 上表中的日志生命周期测试 | 14 个断言通过 | [observation_final.log](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/observation_final.log) |
| 上表中的接近段简化动力学 | 48 个组合通过 | [plant_verified.log](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/plant_verified.log) |
| 静态路线覆盖 / 既有指标工具 | 6/6、8/8 通过 | [fixture_final.log](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/fixture_final.log)、[metrics_final.log](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/metrics_final.log) |
| 原阶段判据 / 版本化判据 | 22/22、16/16 通过 | [phase_final.log](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/phase_final.log)、[versioned_final.log](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/versioned_final.log) |
| 实际 C++ 对象日志到 Python 工具 | 6/6 通过 | [integration_final.log](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/integration_final.log) |
| Gazebo/RViz、真实动作服务、物理故障效果 | NOT_RUN | 遵守不启动仿真的要求 |
| 真机与外部真值 | NOT_RUN | 未连接或部署 |

73/14/48 是 13 个 CTest 目标内的细项，不能再相加冒充独立场景数。48 组仅是接近段简化模型，没有真实 MPPI、Gazebo 接触/滑移或旋转闭环。日志衔接使用真实对象输出，但会话、接收时间与高层相位来自明确的离线测试夹具，不是在线录制证据。构建仍有既有消息显式构造告警，没有构建错误。

## 版本与复现

本轮证据根目录：`/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441`。

本轮验证加载的控制库：`/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/install/lib/libastribot_s1_path_tracking.so`。

SHA-256：`930a1621003f58ce9e73aa24741ff55218091bd3ca397ac6b5d86b3f7dbc4fc8`。

[动态加载记录](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/loader_verified.log)确认使用隔离 install；[完整改动](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/changes.patch)和[写回清单](/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/final_copy_manifest.json)记录 11 个选定源码/验证文件。每个写回文件先与本轮基线或本轮上一阶段的已写回哈希核对，未覆盖其他工作内容。

在仓库根目录运行离线脚本：

```bash
python3 tools/social_navigation/test_corner_fixture_coverage.py
python3 tools/social_navigation/test_corner_phase_evidence.py
python3 tools/social_navigation/test_corner_versioned_evidence.py
python3 tools/social_navigation/test_corner_log_integration.py \
  --log /home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/observation_final.log
```

C++ 测试复现（只运行对象测试，不启动 ROS/Gazebo）：

```bash
source /home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/setup_frozen.bash
export AMENT_PREFIX_PATH="/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/install:$AMENT_PREFIX_PATH"
export LD_LIBRARY_PATH="/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/install/lib:/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/build:$LD_LIBRARY_PATH"
ctest --test-dir /home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C94_20260922_182441/build --output-on-failure
```

## 任务链位置

C 的授权离线修复与本轮工具衔接已完成。阶段通过仍需：当前候选的左右 45°/90°/135° 重复 A/B、直线/弧线对照、连续角点和重规划；真实阶段触发的横穿/取消/定位跳变/包络失效及恢复；完整轨迹对应的 Gazebo/RViz 视觉证据。已有 24 张故障场景卡仍是准备态，不能作为已执行样本。

因此 C/I0.1 = **NOT_PASSED**；D、I0.2、I1–I6 和真机验收继续 **NOT_OPENED**。本轮暂停原因是不启动仿真的既有要求，而非达到 90 分钟上限。
