# 离线复盘界面与逻辑评审

日期：2026-09-23。范围：`astribot_operator_station` 的采集入口、事件/参数查看器、独立回放控制窗口及其 RViz 配置、C++ 回放进程。结论：**基础只读边界与参数缺口语义合理，但当前界面不能作为“画面、事件、参数严格对齐”的完整故障复盘工具验收。**

本次只读评审生产实现，新增文档和验证探针；未修改生产源码或配置，未启动 Gazebo、真实 RViz、真机或发送速度。Qt 测试使用实际窗口源码和模拟子进程，不能冒充真实 3D 验收。

操作步骤见 [操作与多场景验证手册](manuals/OFFLINE_REPLAY_OPERATION_VALIDATION_20260923.md)。

## 1. 基线与证据

- Git HEAD：`a7026bcd57c3a992f6b25d8dd38397a0d1f30168`，审查的是**含既有未提交修改的工作树**，不是纯提交版本。
- 8 个关键文件的 SHA256：[baseline.json](evidence/offline_replay_review_20260923/baseline.json)。审查前后全部一致：[source_unchanged.json](evidence/offline_replay_review_20260923/source_unchanged.json)。
- 从当前源码单独编译测试目标到 `/tmp/astribot_replay_review_20260923/build`；没有覆盖 `ws_robot/install`。已安装播放器是否与源码完全一致不在本次证明范围内。
- ROS 隔离测试使用 domain 93、localhost 和每次唯一话题前缀，测试前未发现显式使用该 domain 的本机进程。仅停止本次创建的子进程。这不证明其他容器或未显式设置环境的参与者绝对不存在。
- 本次由同一评审者完成源码审查和实验，不声称独立双人评审。

| 证据 | 类型与结果 | 可以证明什么 |
|---|---|---|
| E1 | [参数证据测试](evidence/offline_replay_review_20260923/test_evidence.xml)：5/5 PASS | 快照、删除、缺口、身份变更、追溯不确定区间的参数重建逻辑 |
| E2 | [回放白名单测试](evidence/offline_replay_review_20260923/test_replay_stream.xml)：1/1 PASS | 合成 bag 内地图被发布到前缀话题；原始及前缀 `/cmd_vel` 均未被播放器发布 |
| E3 | [实际回放探针](evidence/offline_replay_review_20260923/probe_results.json) | 1 倍速、相邻目标样本理论间隔 200 ms：从头播放实测 199.138 ms，跳转后实测 1.132 ms；本次单次观测，不是延迟分位数或性能基准 |
| E4 | 同一探针、[时钟回退 bag 元数据](evidence/offline_replay_review_20260923/rollback_metadata.yaml) | 按 20、21、1、2 秒写入，SQLite Reader 返回 1、2、20、21；读取顺序不能代表原始到达顺序 |
| E5 | [Qt 实验结果](evidence/offline_replay_review_20260923/ui_probe_results.json)、[截图](evidence/offline_replay_review_20260923/ui_event_selection.png) | 实际窗口在“当前 6 秒”下显示选中“第 1 秒”参数；跳回 0 秒后旧参数仍留在详情框 |
| E6 | 源码检查 | 默认显示项、错误处理、域选择、入口行为等；没有对应真实 RViz 操作验收 |

E3 使用 1200 个历史地图样本，每 2 ms 一个，再放置相隔 200 ms 的两个识别样本；跳转位置 2.4 s，速率 1x。观测使用订阅回调的 steady clock。地图单元中的 120/121 仅为探针识别标记，**不是可导航地图**。预实验采用 1 ms 高密度样本，普通播放本身出现追赶，因此将控制组间隔调整为 2 ms 后重新采集上述结果；不将预实验当作正常对照。

## 2. 现有设计合理的部分

1. 主工作站入口清楚地区分“记录”与“离线回放”。离线回放另起进程，播放器采用名称与消息类型双重白名单，并将数据放到唯一前缀下；默认回放 RViz 只有查看工具。
2. 参数事件有接收序号、ROS 时间、steady 时间和 UTC 时间；按序号重建时会处理删除、节点身份变化和缺口，不默认沿用旧值。
3. `observed` / `event_only` 与 `effective_confirmed=false` 保留了“读取到参数”与“设备执行周期已生效”的区别。
4. 旧版事件查看器明确注明每 0.5 秒一条，属于顺序查看，不伪装成按原始时间播放。
5. 记录器保存清单和数据质量，停止后落盘；这些信息有基础，但独立回放窗口尚未充分使用。

## 3. 发现与修复建议

P1 表示会显著误导故障归因、应优先修复；不等于已经发现机器人误动作。P2 表示交互、完整性或隔离可观测性不足。源码风险与实际复现分别标注。

| 编号 / 要求 | 触发、当前行为与后果 | 严重度 / 证据 | 锚点 | 修改方向与关闭条件 |
|---|---|---|---|---|
| F1 / R1 时间一致 | 仿真时钟回退后，bag 按时间排序，事件仍按接收序号保存；界面用 ROS 时间推进序号，可能把两个时钟段的数据混用。播放器的递减检查只能检查 Reader 输出，不能恢复被排序抹去的到达顺序 | P1；排序行为 E4 实测，跨段关联错误为源码推导 | [recorder.cpp:86](../ws_robot/src/astribot_operator_station/src/recorder.cpp#L86)、[recorder.cpp:240](../ws_robot/src/astribot_operator_station/src/recorder.cpp#L240)、[replay_stream.cpp:41](../ws_robot/src/astribot_operator_station/src/replay_stream.cpp#L41)、[replay_viewer.cpp:81](../ws_robot/src/astribot_operator_station/src/replay_viewer.cpp#L81) | 采集时识别 clock epoch，明确录制时间、消息头时间与单调序号的映射；回放按 epoch 分段，不能仅按 ros_ns 排序。SC08 中不跨段拼接；不支持的旧包明确拒绝或降级顺序查看 |
| F2 / R2 倍速一致 | 跳转先重放历史，但倍速计时锚点设在历史重建之前；重建耗时进入播放预算，目标后的消息立即追赶 | P1；E3 复现：200 ms 压缩到约 1.13 ms | [replay_stream.cpp:29](../ws_robot/src/astribot_operator_station/src/replay_stream.cpp#L29)、[replay_stream.cpp:43](../ws_robot/src/astribot_operator_station/src/replay_stream.cpp#L43)、[replay_stream.cpp:51](../ws_robot/src/astribot_operator_station/src/replay_stream.cpp#L51) | 历史重建和实时播放分别计时；在目标边界重置 wall/ROS 锚点。SC04 的 1x 200 ms 间隔应满足预设容差，不能突发追赶 |
| F3 / R3 界面一致 | 选中事件只改详情与目标输入，不暂停/跳转；下一事件到来又覆盖详情。向前跳至首次事件之前时没有清空旧详情，出现“当前 0 秒、参数来自 1 秒” | P1；E5 复现两种错位，自动覆盖为源码确认 | [replay_viewer.cpp:66](../ws_robot/src/astribot_operator_station/src/replay_viewer.cpp#L66)、[replay_viewer.cpp:80](../ws_robot/src/astribot_operator_station/src/replay_viewer.cpp#L80)、[replay_viewer.cpp:97](../ws_robot/src/astribot_operator_station/src/replay_viewer.cpp#L97) | 将“选中事件”和“播放游标”分区显示，提供“定位到此事件”；重建先清空旧状态，目标前未知值显式标 unknown；暂停状态有 ACK，定位后默认暂停。SC03/SC05 不允许未标注的跨时刻参数展示 |
| F4 / R4 业务复盘完整 | 虚拟墙 MarkerArray 已允许回放，但默认 RViz 只有地图、路径、里程计和 TF；禁区事件虽记录在 bag，却未转换为事件列表条目。代价地图、关节等同样不等于默认可见；当前事件详情框也不是全量日志搜索器 | P2；E6 源码确认，真实渲染 NOT_RUN | [replay_viewer.cpp:70](../ws_robot/src/astribot_operator_station/src/replay_viewer.cpp#L70)、[replay_policy.hpp](../ws_robot/src/astribot_operator_station/include/astribot_operator_station/replay_policy.hpp)、[recorder.cpp:232](../ws_robot/src/astribot_operator_station/src/recorder.cpp#L232) | 明确采集、解码、展示三份能力清单；增默认禁区显示及按地图/区域版本关联的事件。未录数据标不可用。SC10 同时能核对区域图形和应用/失败事件；不能用图形存在代替消费者 ACK |
| F5 / R5 故障可见 | 打开时不检查 manifest 的 closed/partial/recording；坏包可能在窗口显示前退出。主面板只检查是否成功创建进程，播放器正常退出但返回非零也没有 finished 状态处理 | P2；E6 源码确认，故障弹窗交互 NOT_RUN | [replay_viewer.cpp:32](../ws_robot/src/astribot_operator_station/src/replay_viewer.cpp#L32)、[replay_viewer.cpp:91](../ws_robot/src/astribot_operator_station/src/replay_viewer.cpp#L91)、[workstation_panel.cpp:400](../ws_robot/src/astribot_operator_station/src/workstation_panel.cpp#L400) | 显示清单、包质量和预检错误；定义 OPENING/READY/REBUILDING/PAUSED/PLAYING/ENDED/ERROR。SC07 对坏包/子进程异常给明确失败原因，不能保留成功假象 |
| F6 / R3 控件语义 | 秒数输入是“跳转目标”，不是当前进度；播放结束后仍可点播放/暂停，但进程 EOF 循环不再读命令。定位会自动播放，也不保持暂停状态 | P2；E6 源码确认，完整手动操作 NOT_RUN | [replay_viewer.cpp:49](../ws_robot/src/astribot_operator_station/src/replay_viewer.cpp#L49)、[replay_viewer.cpp:85](../ws_robot/src/astribot_operator_station/src/replay_viewer.cpp#L85)、[replay_stream.cpp:28](../ws_robot/src/astribot_operator_station/src/replay_stream.cpp#L28)、[replay_stream.cpp:54](../ws_robot/src/astribot_operator_station/src/replay_stream.cpp#L54) | 给输入加“跳转目标”标签、独立当前时间和状态；EOF 禁用无效操作并提供“从头回放”；定位暂停契约固定。SC02/SC06 验证状态与按钮匹配 |
| F7 / R6 隔离可观测 | domain 固定取 219（父进程为 219 时取 218），不检查占用；唯一前缀降低混流风险，但不能称为每实例独占隔离域 | P2 加固项；E6 源码确认，未发现控制命令泄漏 | [replay_viewer.cpp:56](../ws_robot/src/astribot_operator_station/src/replay_viewer.cpp#L56) | 可配置回放域并显示域/前缀/子进程归属，发现冲突给明确提示；保持严格白名单。SC09 和 SC12 验证多个窗口及已占用域；不扩大为真机安全认证 |

额外待测项：启动前最多同步扫描 20000 条消息；`parameters_at()` 随事件数量线性扫描；详情最多保留 1000 个文本块。大包首屏耗时、界面卡顿、长参数截断、两窗口在小屏上的布局还未测量，不能标为已修复或已验收。原始参数时刻以记录器接收/读回时刻为主，也不能从显示同步直接推导传感器采样时间或控制器生效时间。

## 4. 建议交互结构与实施顺序

建议保留统一工作站的“日志与回放”入口，将离线窗口明确命名为“OFFLINE + 诊断包 ID”，顶部始终展示机器人、版本、地图会话、数据质量与当前模式。控制区区分“当前时间”“定位目标”“选中事件时间”，中间为 3D 和事件列表，详情区分别展示参数、事件和错误，避免共用文本框互相覆盖。

目标状态流（设计，尚未实现）：

`打开包 → 完整性预检 → 选择时钟段 → 重建 → 暂停 → 播放 ↔ 暂停 → 结束`

`选择事件 → 查看详情；点击定位 → 清空旧状态 → 重建到事件 → 暂停`

`任意阶段异常 → ERROR + 原因；关闭 → 仅回收当前窗口拥有的进程`

按以下顺序整改，不需要新增运行时采集脚本：

1. **时间与参数可信性**：先 F1/F2/F3，C++ 实现 epoch、重建计时及详情上下文，增加回退、重复时间戳、向前/向后跳转、暂停定位测试。
2. **可操作与可诊断**：F5/F6/F7，C++ 完整性预检、状态机、可见错误与隔离身份；再接真实 RViz 做人工验收。
3. **业务场景补齐**：F4，联合地图/SLAM 会话版本，显示禁区/虚拟墙、路线、探索结束/取消/存图/载图及消费者应用结果；原始日志按 session/task ID 关联，不把 recorder 的 session.log 当成所有节点日志。
4. **容量与现场复核**：大包、长事件、窗口缩放、多个回放、缺失 TF/地图、仿真重启等。达到手册判据后，才能报告完整离线复盘通过。

当前可用范围：单时钟段的事件顺序与参数证据查询、白名单地图轨迹查看。临时操作限制见手册；它们是建议，不代表用户已接受这些缺陷。
