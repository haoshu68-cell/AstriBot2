# M3 静止入口修改交接

2026-09-24。本目录是 `4eac3656` 冻结 probe 之后的独立增量记录。原 manifest、binary 和 source/model 未运行状态不覆盖。接口问题计时仍为约 12:56 → 13:56；原 11:52 → 12:52 场仍为 BLOCKED_PREREQUISITE / NOT_RUN。

## 当前状态

- 实现：4 文件已完成本轮静止入口修订。第一候选版本通过 19 项后，独立审查发现真实 envelope 同 stamp 状态转换缺口；修正后重新单线程构建并通过全部 **20 项纯 C++ GTest**（CTest 为 1 个测试目标）。最终日志 `build_envelope_revision.log` 无编译警告，`unit_envelope_revision.xml` 为 20 项 / 0 失败。
- 静态/离线：envelope 同 stamp 合法状态变化保留原 receipt 与有效期上界（新值只可收紧）；CDR 保存原始真实消息。geometry 继续严格判重。新增实采 `/data/envelopes[0:11]` 投影字段回归通过；这是回调输入回归，不是完整真实场重放。
- ROS 协议、实际 source/模型仿真：仍 NOT_RUN。
- 资源：两次明确分配的构建窗口均已释放；没有启动 ROS、GPU 或模型工作进程。本轮不再增加采集设计或 ROS 矩阵。

## 修改与边界

1. `probe.cpp` 新增 5 条只读订阅；配置新增必填 `hold_owner_id`（真实 HoldResources 持有者）；在首次 Scene 请求之前完成静止门控并冻结底盘基准。等待 Scene/模型时也持续检查，结束前再次检查，入口及结束写 CDR / receipt / metrics 证据。
2. `stationary_gate.hpp` 为 probe 私有判据，运行时与测试共用。接收真实 typed Hold、六当前 ACK、odom、最终 cmd_vel 及导航状态事件，沿用已批准阈值。相同源戳不续期/增样本；坏→好、源戳冲突/回退、超时和活动导航锁存失效。envelope 发布者允许同 stamp 状态变化，单独保留原 receipt/effective deadline，登记后负向变化仍锁存；其他带源戳消息保持严格冲突判据。零 Twist 是无源戳真实心跳，允许相同数值续 receipt。
3. `stationary_gate_test.cpp` 是不初始化 ROS、不连接 graph 的 GTest 固定输入测试。覆盖正常/缺 ACK/保持身份、已绑定上下文、活动导航、重复/过期/回退、慢漂移及窗口数值边界。
4. CMake 显式添加 nav_msgs / geometry_msgs 和上述测试目标。

原 capture+5 秒、单端 4 秒、登记 30 秒、总 60 秒、Scene 1 秒、取消收尾 1 秒保持。仅取消本次两端推理 Goal；不写导航命令、不更改 Hold / envelope / Scene。没有修改生产包。

`execution_status` 无 IDLE 心跳/全量任务快照，probe 不把无消息当空闲证明，也不声称解析了 owner_evidence。M2 仍须独立证明冷启动、导航入口清空、发布者身份、实际最终命令路径及任务期间排他占有。

## 复现入口（后续运行仍须资源归属）

先核实未有 M1/M2 协议或仿真运行；本轮使用独立 build，与旧冻结产物分离。以下命令已在明确窗口执行，日志与最终 hash 位于本目录；重跑不自动获得资源许可：

```bash
source docs/evidence/m3_source_model_probe_20260924/runtime_overlay.bash
cmake -S tools/vision/m3_source_probe \
  -B runs/m3_source_model_probe_20260924/stationary_gate_build \
  -Dastribot_s1_manipulation_perception_DIR=/home/yjh/WorkSpace/astribot_sdk_ros2/runs/m3_source_model_probe_20260924/clock0_install/astribot_s1_manipulation_perception/share/astribot_s1_manipulation_perception/cmake \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build runs/m3_source_model_probe_20260924/stationary_gate_build --parallel 1
ctest --test-dir runs/m3_source_model_probe_20260924/stationary_gate_build \
  --output-on-failure -R '^stationary_gate_test$'
```

configure/build/ctest 日志、最终源码/产物 hash 已保留，实际编译命令位于新 build 的 compile_commands.json；初版 help/ldd 是上一候选版本证据，不冒称最终修订的运行结果。遵循主线优先，本轮不追加旧 72 项矩阵、命令行矩阵或采集门控。原 M2 measured_stop 同输入对照尚未运行。合成 ROS 的失效→两端取消（SG7）以及真实 source/双模型（SG8）需总调度另定时段；本次单元通过不能记 SG7/SG8 通过。

## 覆盖和待验收

|设计条目|当前实现映射|当前执行|
|---|---|---|
|SG1 / SG2|Ready / MissingAck / BadThenGood / FrozenContext / Scene03 同 stamp 实采序列|已实现用例 PASS|
|SG3|MeasuredStop 数值、跨度、样本数、间隔、年龄、拟合、角度解缠|已实现 C++ 用例 PASS；M2 同输入 Python 对照 NOT_RUN|
|SG4|DuplicateSources / SameStampConflict / LateDelivery / Renewal / CommandLoss / geometry-envelope 过期锁存|已实现用例 PASS|
|SG5|FixedReference / QuaternionSign / InvalidDeviceNumbers|已实现用例 PASS|
|SG6|BadThenGood / ActiveNavigation / ExistingObservedTask|已实现用例 PASS|
|SG7|本次静止失效导致两个推理 Goal 的实际取消接线|BLOCKED + NOT_RUN|
|SG8|M2 独占新场与同 capture raw 证据|BLOCKED + NOT_RUN|

未执行用例不计为通过；测试粒度不换算成机器人成功次数。四文件之外不扩大生产接口或权限协议。

## 最终产物和剩余入口

- `runs/m3_source_model_probe_20260924/stationary_gate_build/m3_source_probe`：SHA256 `2626ed5e2b66d00b62d8183c9d30ab566266b76c0b5cc45530a10243bdb424fc`。
- `stationary_gate_test`：SHA256 `fcb8d503ccbaf1e96d2947597ef5e32244f3a39c3942a00a19005d58338ebd8a`。
- `implementation_manifest.json` 和 `final_source.sha256` 绑定最终四文件；`git_paths.txt` 给总调度精确交接范围，本任务没有提交或清理他人文件。
- 采集仍有已知顺序缺口：server 配置依赖 probe 登记，现有 probe 在 server 就绪后立即 capture；有限 raw 窗口与实际 capture 的确定绑定尚未完成。因此新版编译/单元通过并不授予 SG8 启动许可。
- M5 已在 `head_metadata_topics.json` 增补 5 个控制状态话题，共 14 项；20 秒 metadata 不替代 9 个头部 raw / 3 秒及同 capture 核验。
- 旧版本归档/删除由总调度按用户要求统一进行，本任务保留最小交接证据，不自行删除冻结产物。
