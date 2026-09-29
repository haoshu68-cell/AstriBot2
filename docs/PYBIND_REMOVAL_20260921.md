# 全项目去 pybind：当前迁移与验收记录

**状态：未完成全项目去 pybind。** 本轮把剩余绑定中的主要几何/控制状态机抽成可独立链接的 C++，新增直接 ROS C++ 生产者与保护节点；3 个自有绑定仍有未迁移的生产消费者，厂家 SDK 还存在独立二进制依赖。关闭兼容构建选项不等于完成删除。

本文只记录 2026-09-21 本轮证据。先前包络 legacy / costmap scan 实验保存在 `docs/evidence/cpp_migration_20260921/`，不将其数字冒充本轮新节点结果。

## 最新策略观测候选

最新字符串修复记录为 `f4feccdf`：独立无绑定构建安装 **14/14 CTest、108/108 成对 ROS 用例**通过，1,600 帧配对测量业务输出一致。8/64目标 tick P95 分别由 **1.729/3.963 ms** 降至 **0.188/0.573 ms**，整进程 CPU 每帧由 **6.75/14.15 ms** 降至 **0.70/1.95 ms**。原 `1b51e14e` 和整数修复 `b7a2240b` 的历史结果另存，不替换其源码/产物身份。[字符串阶段证据](evidence/pybind_removal_20260921/policy_json_strings/README.md)。

JSON大整数 epoch、图像尺寸、精确像素和孤立surrogate/NUL字符串边界已补齐；Stamp域及失败时机、完整controller仍待完成。已确认超大有限超时和图像观测时间戳在原生实现中会更早失败，[差异与后续验证](evidence/pybind_removal_20260921/policy_stamp_boundary_review/README.md)单独留档。生产入口尚未切换，绑定不能删除。最新盘查仍为79个自有运行时文件，另1个旧标定入口，23个直接生产绑定消费者；这是源码可达性而非进程观测。[盘查证据](evidence/pybind_removal_20260921/runtime_audit_stage6b_strings/README.md)。

## 参考模块生产安装清理

17 个已替换的 Python 参考模块已逐字节移到 `tools/migration/python_reference/`，生产源码和全新安装均不可导入这些旧模块。验证目录没有生产入口或环境钩子。关联回归 83 项、ROS 对照 226 项、动态参数 52 项通过（另 12 项沿用已说明的旧接口跳过）；新几何 Release CTest 5/5。详见 [清理证据及失败记录](evidence/pybind_removal_20260921/reference_retirement/README.md)。这些结果不替代剩余生产消费者的迁移，也不代表共享安装已更新。

## 后续完成的两项原生运行时

- **臂底盘 Twist 耦合**：包改为 ament_cmake，同名 C++ ELF 唯一入口，生产 Python 3 文件及 setup/resource 删除。10,027 核心输入、31 pytest / 2 CTest、安装后 3 项 ROS 用例通过；旧实现仅冻结于测试目录。四组交替实验共 8,000 条输出匹配，CPU 中位 5.40%→0.85%、RSS 65.90→24.11 MiB、p95 0.715→0.357 ms。最大单条延迟 3.784→3.933 ms，保留尾延迟例外。[原始证据](evidence/pybind_removal_20260921/dynamics_coupling/README.md)
- **导航任务仲裁**：新增纯 C++ task_arbiter_cpp，launch 仅选该入口，删除旧 Python module/console。安装后 77 项测试、ownership CTest 1/1；两 action 共用唯一所有者，取消 ACK 不释放任务。五组配对每实现 500 个假 Nav2 任务全成功；submit→terminal 中位 23.583→3.957 ms、p95 24.886→6.245 ms、CPU 5.28→0.40 ms/goal、RSS 64.12→26.62 MiB。[原始证据](evidence/pybind_removal_20260921/task_arbiter/README.md)

这些数值是隔离节点实验，未升级为真实导航/驱动、物理停止、Gazebo、长期稳定性或真机结论。两包默认 Release，显式 Debug 保留。原参数类型拒绝和消费时机保持；耦合的非有限参数、空 legacy joint、负时间龄期等继承行为另列契约，不包装成新的安全保证。

上述仲裁/耦合阶段盘查剩余 **82 个**自有运行时 Python 文件、1 个已有 C++ 的旧标定入口；20 个历史模块仅验证使用。原始 85 文件快照留存，更新清单见 [剩余 Python 盘查](PYTHON_CPP_REMAINING_20260921.md)。依赖审计仍明确失败：3 个自有绑定和厂家 SDK 链未清零，不能因新节点无 pybind 链接就宣称全项目完成。

## 剩余依赖链

| 绑定 | 尚存生产链 | 本轮已切出的直接 C++ 能力 | 真正删除绑定前仍需完成 |
|---|---|---|---|
| `_chassis_math_native` | Python bridge container → chassis/arm/gripper façade → SessionPort → 厂家 SDK | `bridge_runtime`：夹爪、机械臂流式执行/waypoint、底盘状态机及类型明确的端口；已有直接 arm speed limiter | 生产 rclcpp 桥接、单会话所有权/控制权心跳/写准入、厂家原生端口；把剩余 Python 运行时消费者迁出，再删除兼容代码/依赖 |
| `_navigation_math_native` | PolicyObserver/PolicyNode 的时钟、融合/风险、路径、候选、规划会话等 | `final_protection_cpp`（legacy/fixed_v2）、`fixed_envelope_cpp`；已有 legacy envelope、cmd_vel、scan 直接节点 | 完整 observer/policy controller 的所有阶段与传感器适配，不得只保留简单场景；迁完消费者及验证入口后删除绑定 |
| `_geometry_native` | Python polygon、observer scan、fusion snapshot；transport `SourceInbox` | 公共 polygon/scan/model C++ API、`geometry_state` ELF；保护/固定包络可直接调用几何 | 搬运 ROS 后端和 source inbox、observer/fusion 的原生消费者；保留载荷、原始采集期限、版本/取消契约 |
| 厂家 `astribot_rclcpp_py_ext_pybind11` | SDK Python proxy 硬导入；仓库有 CPython 3.10 和 3.8 两份 `.so` | 尚无可验证的原生控制适配 | 需要厂家 C++ SDK 头文件/库或官方等价协议。导出符号本身不能证明类 ABI、单位、控制权、filter/direct/stop/hold 等契约 |

`rclpy._rclpy_pybind11` 也会被验证/记录脚本使用；允许保留 Python 启动和验证脚本，不把 ROS 发行版内部依赖混称为自有 C++ 运行时已删除。第三方仓库中存在 pybind 头/构建模板；仅凭这些文件存在不能判断实际运行链，不能批量删除第三方目录。

## 本轮实际实现

- 几何包：公共无 Python 类型接口，URDF/FK/mimic、primitive/mesh、偏置载荷、保守关节误差、异步计算与来源期限；`geometry_state` 安装入口改为同名 C++ ELF。
- 桥接包：夹爪、机械臂/waypoint、底盘状态机从 `py::object` 回调实现移入 `bridge_runtime`。旧绑定只转换配置/结果和端口，转调同一核心。C++ 构造器保留原先 Python 入口的安全配置校验。
- 最终保护：20ms steady watchdog、ROS 时钟停滞/回退、原始 TTL、TF 等待队列、指令及实测速度双扫掠、clear hold、防重放、alignment/centering、fixed_v2 非矩形与 hash/ACK。
- fixed_v2 协调器：六消费者 ACK、未来正 ACK 不续期/负 ACK 即时撤销、机械臂保持所有权与原始租约、几何/载荷版本检查、关节与物理包络越界锁存、legacy 只准撤销、未实现的预留运动明确拒绝。
- 已通过隔离验证的速度转换、臂展开限速、轮驱动、scan adapter、包络协调器、最终保护与几何状态入口收敛为 C++。移除 Python/cpp 选择开关、旧 Python console entry/main 和几何脚本；legacy/fixed_v2 仅选择对应 C++ 实现。已退役节点参考类仅供验证脚本显式构造；尚未迁出的桥接/策略/几何生产接口另有独立依赖链。共享安装和在运行的栈未替换。

代码级兼容修复：Release O3 下发现 float→double 的中间舍入未按预期落地，导致多边形 canonical 顺序变化；已以实际 float32 存储后再取凸包，固定原失败输入并在 O2/O3 逐元素验证。未降低几何容差或放宽授权门槛。坏 V2 撤销授权时保留旧碰撞多边形，避免异常路径退回矩形。

## 已运行的验证

| 范围 | 证据 |
|---|---|
| fixed_v2 核心/服务/话题 | 96 个独立进程事务差分 + 18 个 Python/C++ ROS 协议用例，共 114 通过；完整比较每一步 output、ACK、fault 与 epoch |
| 最终保护 | 22 个 Python/C++ 协议用例；追加保留旧 polygon 的红绿回归 2/2；312 组几何扫掠对应 3 个参数组通过 |
| 几何 | 整包 82/82；纯 C++ Release CTest 5/5；三项 C++ 核心显式 `-UNDEBUG` 重验通过；实际进程无 Python/geometry binding 映射 |
| 桥接 | 新旧完整包 68/68，其中 29 个无绑定进程 oracle 场景；纯 C++ CTest 6/6；外部 CMake consumer 链接运行成功 |
| 导航基础 | 干净 Release CTest 3/3，包括无 Python 的 fixed envelope 授权/来源期限不变量；极端时间/NaN 载荷边界另经 UBSan 检查通过 |
| 启动与扫描门禁 | 入口唯一性/无旧 main/scan 参数等 11/11；已有非 Home 参数测试 3/3；依赖审计工具 10/10（合计 24） |

本轮重新构建后的 fixed_v2 96 差分 + 18 协议、最终保护 22 协议与 3 组扫掠合计 **139/139 通过**（`navigation_latest_protocol.log`）；导航 CTest **3/3**。从迁移 Git 提交树解出的入口测试另 **16/16**，检验选择性收录的 launch 仍可执行。

测试数存在包含关系，不将上表简单相加。ROS 测试仅启动自己拥有的进程，使用 localhost 的 114/115/116 域；没有 Gazebo、整栈闭环或真机验收。

证据入口：[`docs/evidence/pybind_removal_20260921/`](evidence/pybind_removal_20260921/)。`direct_cpp_dependencies.json` 是早期 3 个生产节点及 3 个桥接 probe 的 ELF 哈希与依赖记录；最新清洁安装入口、ROS 包索引解析与 ELF 检查分别保存在 `entry_cleanup/fresh_*entries.json`、`fresh_ament_entry_resolution.json`。这些只覆盖对应候选，**不能证明全项目的依赖链已清零**。

## 入口清理后的补充修复

独立复查找到速度转换节点的六项运行参数被启动缓存吞掉。现在合法动态参数在后续控制回调生效，保持既有里程计、姿态样本窗、来源时间和止损锁存；原子参数批次校验失败不会应用其中其他参数。测试先复现旧 C++ 的差异，再验证修复，最终 **30 通过/12 跳过**。12 跳过仅表示旧 Python 不具备新增加的非法值拒绝和启动只读契约；不能写成两实现对非法请求完全等价。构造失败改为非零退出，正常关闭保持零退出。完整 native 包重新构建及 CTest **3/3** 通过。详见 [cmd 参数证据](evidence/pybind_removal_20260921/entry_cleanup/CMD_PARAMETERS_README.md)。

臂展开限速同样补齐七项原有动态参数，并修复非法启动成功退出的问题。独立审查发现 NaN 阈值可被误判为收拢而解除限速，现对非法原子更新明确拒绝并保留原参数及展开限速。24 次非法批次覆盖 NaN/Inf、负数、零百分比及迟滞越界；合法动态行为、参考向量原长度语义、TF 陈旧/缺失与边界场景共 **22/22** 通过，完整桥接纯 C++ Release CTest **7/7**。新拒绝契约比旧 Python 更严格，详见 [臂限速证据](evidence/pybind_removal_20260921/arm_parameters/README.md)。

三个 Python 启动/剩余业务包安装到全新隔离前缀后，旧的六个 console 可执行均不存在；ROS 包索引也确实解析到该前缀。对应八个 C++ ELF 已安装、依赖全部解析，未链接 libpython。既有共享 `ws_robot/install` 与根 `install` 均未覆盖，里面的历史入口另列在盘查报告。Python 参考类没有运行入口，但仍随部分 Python 包安装；不把这一状态称为旧参考模块已全部删除。

## 性能

fixed_v2 服务实验采用 3 组交错顺序，每实现每组 1000 次同输入提案，暂停 ROS 时间维持同一合法几何/hold/odom 来源。每次核对提案结果/epoch，批末核对实际输出与来源 deadline。服务往返不包含六消费者安装确认、运动或真机 SDK。

| 指标（3 组中位数） | Python | 直接 C++ | 变化 |
|---|---:|---:|---:|
| 1000 次提案总耗时 | 1.821 s | 0.183 s | -89.9% |
| 节点 CPU | 1.830 s | 0.100 s | -94.5% |
| 采样 RSS | 80,644 KiB | 27,412 KiB | -66.0% |
| 服务往返 P50 | 1.721 ms | 0.167 ms | -90.3% |
| 服务往返 P95 | 2.499 ms | 0.296 ms | -88.1% |
| 服务往返 P99 | 3.079 ms | 0.328 ms | -89.3% |

此表为本轮入口清理后重新构建的 Release 候选；早期测量另存 `fixed_benchmark_before_entry_cleanup.json`。原始 6000 个延迟样本见 [`fixed_benchmark.json`](evidence/pybind_removal_20260921/fixed_benchmark.json)。

桥接纯核心实验采用同一 CPU、5 组交错、每组每核心 50,000 操作，预热 5,000；计时排除进程启动、JSON/IPC 和配置创建。完整写轨迹预检以及每轮结果快照通过。

| 核心操作 | Python | 直接 C++ | Python/C++ |
|---|---:|---:|---:|
| 夹爪请求 | 4.018 µs | 0.734 µs | 5.48× |
| 机械臂 tick | 4.157 µs | 0.219 µs | 18.96× |
| 底盘 tick | 29.220 µs | 0.385 µs | 75.95× |

这只覆盖 fake SDK 端口。底盘比值包含原 Python 诊断/锁等开销，不能当作孤立算术或实际机器人性能收益。精确方法及全部样本见[桥接报告](../ws_robot/src/astribot_trajectory_bridge_native/test/benchmark_results/standalone_fake_cores_20260921.md)。

最终保护以 1080 射线、50Hz、每轮 400 帧、每模式 3 组交错配对运行，12 轮共 4800/4800 帧收到。legacy 的 8 秒节点 CPU 中位数为 2.57→0.16 秒（-93.8%），fixed_v2 为 2.82→0.20 秒（-92.9%）；RSS 分别为 61.49→26.25 MiB 和 61.89→28.39 MiB。逐输出检查包括速度限制、有限数值、HOLD 与稳定障碍拒绝；输入 Twist 没有关联序号，不报告端到端延迟。见[最终 Release 保护基准](evidence/pybind_removal_20260921/protection/README.md)。

几何节点最初的带偏置载荷试验出现了实际延迟退化，失败数据保留。排查确认计算已完成后仍等待下一次 20ms ROS 定时回调；现在仅有未完成任务时增加 1ms 完成检查，沿用原来的版本、时钟、来源期限与过滤确认校验，原采样调度不变。代价是 C++ CPU 从约 0.944% 增至 1.167% 单核（增加 0.223 个百分点）。

重新进行每场景 3 组交错配对：50Hz 原始关节输入、2 秒预热 + 6 秒计时。下表为各场景合并有效输出帧的采集到发布延迟；不包含机器人运动或 SDK。

| 场景 | Python P50 / P95 | C++ P50 / P95 | Python / C++ CPU（单核） |
|---|---:|---:|---:|
| 空载 | 33.621 / 34.297 ms | 3.495 / 7.272 ms | 8.666% / 1.167% |
| 偏置载荷 | 37.660 / 39.139 ms | 13.272 / 20.105 ms | 8.555% / 1.167% |

12 轮所有完整输出均保留原采集时间、float32 顶点与 hash，完整输出无校验失败、无序号缺口；50Hz 输入按原约 10Hz 作业策略抽取最新状态，未选中的输入不能算作输出丢帧。另 16 轮受控相位实验和计算中失效用例覆盖调度与有效性边界。Python 旧实验 runner 在请求关闭后重复 shutdown 导致部分退出码 1，计时过程完整且存活；该清理问题独立记录与修正，不伪称正常关闭通过。见[几何原始与复现实验](evidence/pybind_removal_20260921/geometry/README.md)。

## 删除门禁与下一步

`tools/migration/audit_pybind.py --assert-clean` 当前应失败。它检查自有源文件/构建/清单/显式及可选导入、厂家命名二进制，以及可选指定的安装目录残留。注释、历史文档不算实现；Python/包清单语法错误、错误仓库根目录或不存在的显式安装路径不能静默放行。对不透明二进制和动态构造导入的能力限制写入 JSON。

```bash
python3 tools/migration/audit_pybind.py --root "$PWD" \
  --install-root "$PWD/ws_robot/install" --assert-clean
```

真正收尾顺序：完成 observer/policy 全阶段消费者与 transport 后端 → 完成桥接 ROS/厂家原生端口和所有权契约 → 将生产入口切至验证后的 C++ → 把旧 Python 留作测试 oracle 并移除绑定引用/环境开关 → 删除 3 个绑定源码、目标、manifest 依赖 → 使用全新安装目录验证不存在旧 `.so` 遮蔽，再做隔离闭环与硬件范围内验收。任何一步未完成，都不把 `PYBIND=OFF` 或当前单节点性能称为“全项目完成”。

## Git 与继续盘查

源码和证据分批记录在本地 `codex/pybind-migration-20260921`，具体检查点和对照方法见 [Git 记录](CPP_MIGRATION_GIT_RECORD_20260921.md)。共享工作分支与共享安装未切换。

[剩余 Python 逐文件清单](PYTHON_CPP_REMAINING_20260921.md) 排除启动/配置和验证脚本，记录生产可达调用链、消息/服务/action/SDK 接口及迁移验证边界。安装目录旧文件另列，不能把源码入口清理等同于旧安装已清理。


## 定位 / 地图运行时迁移追加记录

本地 Git `80a70bde` 新增 `astribot_s1_perception_native`，包含同名
`map_odom_tf_node`、`map_domain_relay` C++ ELF。原两 console 和三生产 Python 模块
删除；两 launch 只选择 native 包。冻结参考不安装，逐字匹配前序 Git `1b2c33a4`。

最终整包 4/4 CTest 通过，含 34 纯函数、64 TF ROS、46 中继 ROS 用例与核心探针；
两个安装 ELF 分别再通过 4 项 ROS smoke，干净 Python 安装无三旧模块。
[集成证据与复现](evidence/pybind_removal_20260921/perception_delivery/README.md)。

- TF：每实现800/800测量输入匹配；100Hz发布、50Hz输入时，P50 5.735→5.271ms，
  CPU/input 1.575→0.213ms，RSS57.25→23.80MiB。原定时周期仍主导延迟。
- 地图：最终1120/1120测量输出匹配；256² P50 4.710→0.562ms，1024² P50
  65.959→1.973ms。小地图最大单条延迟6.161→10.436ms，保留异常样本，不称尾延迟全部改善。
- 保留旧Python忙碌SIGINT异常、漏加载测试日志库的失败、原生SIGINT退出码差异的red/green；
  不修改oracle或吞错误来获得通过。仅验证隔离ROS，未完成整栈/跨机/长期/硬件验收。

最新盘查为79个自有运行时文件、23个验证参考、20个Python console；SDK和剩余绑定仍在。

## 策略核心后续阶段

新增 C++ contracts、持续融合、健康/标定、多边形扫掠和风险组合库，整包关闭绑定构建的
8/8 CTest 通过，包含126个差分测试及1,783个额外风险回放；sanitizer与独立审查发现的
范数门槛、负零及派生溢出差异均有失败样本与修复记录。风险核心8/64障碍场景 P50
分别从1.337/2.786ms降至0.044/0.300ms，四组AB/BA、每实现每场景600样本。
详见[证据与范围](evidence/pybind_removal_20260921/policy_risk/README.md)。

本阶段未切换策略ROS入口，未降低79文件计数，也未退役其在用绑定。下一步是包络、观测
适配和完整observer/controller状态机；仅在角色完整验证后删除旧入口。已有验证通过角色
继续只保留C++生产入口。微基准不代表整机性能或长期稳定性验收。
下一阶段需完成策略observer/controller的完整P2/P3/P4/P5/H2闭包，详见
[策略施工边界](CPP_POLICY_MIGRATION_NEXT_20260921.md)。单独一个observer不能删除绑定；
geometry绑定还需完成transport消费者。全项目目标继续保持未完成。
