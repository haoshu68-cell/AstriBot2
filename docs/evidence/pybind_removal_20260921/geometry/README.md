# Geometry C++ 入口与完成延迟复核（2026-09-21）

本轮在独立 Release 安装中保留唯一 `geometry_state` C++ ELF 入口，删除未安装的旧 Python 脚本及模块 `main()`，并修复已复现的载荷发布延迟退化。验证范围是隔离 ROS2 fixture 与纯核心回归；没有启动 Gazebo、导航闭环或真机，也没有修改共享 `ws_robot/install`。

## 结果

同一批最终 A/B 的三轮合并结果，延迟单位 ms，RSS 单位 MiB：

| 场景 / 实现 | 原始采集→发布 P50 | P95 | P99 | CPU（单核百分比） | RSS P50 |
|---|---:|---:|---:|---:|---:|
| 空载 Python | 33.621 | 34.297 | 34.324 | 8.666% | 76.996 |
| 空载 C++ | 3.495 | 7.272 | 7.335 | 1.167% | 27.125 |
| 偏置载荷 Python | 37.660 | 39.139 | 39.171 | 8.555% | 77.879 |
| 偏置载荷 C++ | 13.272 | 20.105 | 20.300 | 1.167% | 26.688 |

最终 C++ 325 帧、Python 323 帧全部通过检查：无不完整测量帧、无输出序号缺口、原始源时间逐帧精确匹配、float32 physical/reserved 线值与哈希一致、保守包含成立、载荷身份与过滤确认成立、原始截止时间未延长。采集→接收的 P50/P95/P99 也全部优于本批 Python。数据见 `final_wall_ab/aggregate.json`、`result.json`；用 `python3 summarize.py` 重算精简结论。

## 已复现的退化与定位

修复前的原始数据保留在 `wall_ab/`：偏置载荷 Python P50/P95 为 30.381/31.566 ms，旧 C++ 为 33.270/36.421 ms。该候选未通过端到端不退化条件，未把差异当作噪声删掉。

`phase_sweep/` 对两种实现使用完全相同的整数纳秒 JointState 与 `/clock` 序列，将采样时源年龄固定为 0、5、10、15 ms。旧 C++ 的载荷计算 P50 为 0.118–0.129 ms，但计算后等待为 19.774–19.805 ms；两种实现的 ROS 采集→发布延迟均为 20、25、30、35 ms。原有 20 ms 完成轮询把计算优势消耗在等待上，启动后不同的采样相位又改变了非配对墙钟测量的延迟分布。

修复仅增加任务在途期间启用的 1 ms wall timer。完成回调调用 `tick(false)`，复用 `observeClock`、输入刷新及原来的 generation/model/revision/epoch/source lease/height/filter 检查；该回调不能采样或创建任务。原来的 20 ms ROS 采样定时器、100 ms wall 节流、单个 `std::async` 所有权和所有时间戳/期限不变。结果消费及完成检查异常后都会关闭短定时器，避免错误时变成 1 kHz 不完整消息循环。两个定时器保留同一互斥回调组。

`final_phase_sweep/` 复用了四个相位和相同序列：C++ 计算 P50 为 0.102–0.125 ms，计算后等待为 0.943–0.961 ms；ROS 采集→发布为 0、5、10、15 ms，Python 仍为 20、25、30、35 ms。虚拟时钟在这约 1 ms 墙钟完成阶段没有推进，所以相位 0 的 ROS 延迟为 0；这不表示计算没有耗时。修复前后各 16 轮均没有测量帧校验失败或序号缺口。

旧 C++ 的 CPU 为 0.944% 单核，最终为 1.167%，增加约 0.222 个百分点；最终仍低于 Python 的 8.555–8.666%。这只是本 fixture 的前后观测，CPU 计数粒度为 0.01 s，不能作为所有模型上的固定开销或实时保证。

## 行为与失效证据

- `completion_schedule_red.log`：新增回归在旧 ELF 上失败，计算结果不能在下一次 ROS 采样 tick 前发布。
- `final_ctest.log`：5/5 CTest 通过，包含 O2/O3 几何/模型差分、核心边界及两个 ROS 协议测试。新增协议在最后一个采样 tick 后固定 `/clock`，要求结果只发布一次，且不能因此开始另一轮计算。
- `completion_faults/summary.json`：仅在独立诊断副本给计算增加 120 ms 延迟；源期限到达、时钟回退及 malformed joint 三例均确实完成计算，但不得发布原任务的 complete 结果，分别返回 `SOURCE_LEASE` 或 `INPUT_CHANGED`。每例保留绝对 `session.log` 路径。该注入副本不是性能候选。
- `geometry_state_core_test.cpp` 还覆盖精确截止时刻、已完成 future 遇到 epoch/input-generation 改变、模型/载荷版本改变、过滤确认和高度覆盖拒绝。
- `serializedPolygon` 的 `volatile float` 存储保留，O3 浮点序列化修复没有倒退。

## 测量口径与限制

所有子进程都由本脚本启动并回收；运行环境从各子进程 `/proc/PID/environ` 反读确认 domain 115、localhost。C++ maps 无 `libpython` 或 `_geometry_native`。输入是 fixture URDF 的一个 `arm=0.2` 关节，50 Hz；frame 为 `base`，误差界 0.003 rad，源 TTL 上限 300 ms。偏置 box 载荷尺寸 `[0.3, 0.25, 0.2]`，相对 `arm_link` 偏置 `[0.8, -0.3, 0.2]`。模型不是完整机器人。

每个墙钟轮次预热 2 s、计量 6 s；三个组交错 Python/C++ 顺序，空载与载荷分别计量。发布延迟是 `published_at - 原始 JointState stamp`，接收延迟包含 DDS 和 fixture 调度；输入时间戳必须逐帧匹配，未改写 stamp。CPU/RSS 仅取被测 producer PID，CPU 包含其线程并归一化到单核。约 9 Hz 的输出保留原来的 10 Hz 节流与 20 ms 调度量化；未选中的 50 Hz 输入不是丢失输出。序号无缺口不能独自证明任意 DDS 场景都无丢失。

`contended_aborted/` 为另一测试尚在结束阶段时启动的部分轮次。发现后只中断自己的 benchmark，由其 finally 回收子进程；整批排除并保留原始记录，最终统计完全来自后续协调空闲窗口中的新批次。

Python 清理限制单独记录在 `shutdown_audit.json`：前后相位实验各 8 个 Python 子进程在请求 SIGINT 后，验证驱动 `finally` 重复调用 `rclpy.shutdown()`，因 `rcl_shutdown already called on the given context` 退出 1；不能称为正常退出。各次测量期间子进程持续存活，完整回放结束后才发关闭信号；父驱动均完成并退出 0。相位实验 C++ 均正常退出 0。旧墙钟脚本没有记录子进程退出码，因此该字段记为未采集；保留的 Python 日志也显示同类清理异常。

测量完成后仅将 test oracle 改为 `rclpy.try_shutdown()`，独立 `oracle_shutdown_smoke.json` 验证收到 SIGINT 后退出 0；没有重跑或替换性能数据。各基准目录中的脚本快照/hash 是测量时的旧 driver，当前源 hash 晚于基准且仅清理逻辑不同。墙钟脚本也补充了未来运行的子进程退出码记录。运行时 C++ 候选未因此改变。

## 来源与复现

- 最终 C++ 源 SHA256：`e77fcb77e3e86592bc2acf1a483938b4f902d586d38c8fce7e5450615bd0ca83`。
- 最终 ELF SHA256：`3ca61f738bc8fd11d4254bca6eec87535ea8d3fb105895deea0a34cf449fa721`。
- 几何 kernels 头 SHA256（前后不变）：`e36f9fa5fb5505c097caa32c54f601a2fbff7f7c493fd597a85b087e6ef69f17`。
- 最终隔离安装：`/tmp/codex_geometry_validation_20260921/final_install`；ELF 位于其 `lib/astribot_s1_robot_geometry/geometry_state`。这是本轮临时构建路径，不承诺跨会话持续存在。
- 构建为 `Release`，`-O3 -DNDEBUG`，`ASTRIBOT_GEOMETRY_BUILD_PYTHON_COMPAT=OFF`，没有 fast-math。缓存、编译/链接命令和依赖记录在 `build_provenance/`、`build_provenance.json`、`final_cpp_ldd.txt`。
- 冻结 Python class/extension 用作对照；基准 manifest 保存加载路径、binary hash 和源快照，extension 的原始 C++/headers 另保存在 `baseline_sources/`。当前工程的 Python class/helpers 和 compat 构建选项仍供未迁移消费者及 oracle 使用，不能据此宣称整个几何包已完全无 Python 依赖。
- 从仓库根执行 `bash docs/evidence/pybind_removal_20260921/geometry/reproduce.sh`；它在新的独立目录重建、测试并重放当前候选。性能部分需要协调安静窗口，不停止共享栈。完整源码清单为 `source_manifest.json`。

逐轮 raw JSON、maps、日志使用 gzip 保留，summary/aggregate/source 快照保持可读。`compressed_artifacts.json` 记录压缩前后 SHA256 和字节数；每个文件压缩后都验证解压字节完全一致。更新后的 `test/analyze_geometry_state_ab.py` 可读 `.json` 或 `.json.gz`；`compressed_replay_check.json` 确认从压缩 raw 数据重算的两个 A/B aggregate 与原结果完全相同。

这些证据支持本受控 fixture 内的入口切换、语义保持与已测性能退化修复，不扩展为完整整机几何、多传感器、闭环导航或真机性能验收。
