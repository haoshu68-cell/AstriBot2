# 运行时 C++ 迁移：包络与清障扫描复核及性能证据

本轮交付是无 pybind 的 **legacy 包络协调器、清障扫描候选及其验证**。默认启动仍为 Python；
没有完成全项目去 pybind，也没有进行 Nav2/Gazebo 闭环或真机验收。

## 修复与证据

- 补齐 C++ `Profile` 解析、默认值、单层继承、停止参考、数值/联合预算和硬件证据
  准入。包括 JSON 尾文拒绝、停止参考来源类型及极小数分母下溢。
- 心跳改用 ROS 时钟；暂停仿真时不再用墙钟持续刷新包络。
- 非有限轮廓在排序前拒绝并撤销该侧确认；修复已有确认被 NaN 保留的问题。
- TF 监听绑定协调器节点，保留节点限定的重映射；异常时间字段不再退出整个进程。
- 无绑定构建不再安装空 Python 兼容包，Python Development/pybind 查找位于显式开关内。
  ROS 的构建工具仍使用 Python 解释器；两个包的清洁安装中没有 Python 文件。

先在旧候选上复现，再修复：消息测试 **4 failed / 8 passed**，配置解析追加测试
**7 failed / 428 passed**，停止分母下溢测试也先失败。原始失败输出保存在本目录。

最终包络专项 **517 passed**，见 [envelope_tests.log](envelope_tests.log)：

| 范围 | 数量 | 实际断言 |
| --- | ---: | --- |
| 配置差分 | 436 | 与 Python 比较接受/拒绝，以及全部解析后字段；含仓库配置、继承、停止参考、类型、数值和预算 |
| C++ 启动 | 11 | 对照 Profile 准入；有效配置存活、退出正常，非法配置拒绝 |
| ROS 协议 | 70 | 68 项共同契约检查；另 2 项明确记录非有限 odometry 差异 |

ROS 协议覆盖：ROS 暂停/回退、零/单/双确认、新 epoch 和旧时间戳、所有包络字段、
主动保持、请求拒绝后状态不变、odom/footprint 新鲜度边界、平移和旋转 TF、轻微非单位
四元数、TF 缺失恢复、padding/缩小边界、NaN 轮廓撤销、异常时间戳。
探针使用独立 ROS domain 108/109、localhost、受控 `/clock`，仅管理自己创建的进程。

另外，相邻迁移与运输/策略测试 **189 passed**，见 [adjacent_regression.log](adjacent_regression.log)。
这批包含现有过渡 binding 的差分，不能作为“pybind 已删除”的证据。

## CTest 证据更正

复核发现先前导航/轨迹 CTest 在 RelWithDebInfo 的 `-DNDEBUG` 下关闭了 `assert`。
先前仅据退出码报告的“通过”不足以证明断言成立，本轮以以下重跑替代：

- 两包测试显式添加 `-UNDEBUG`，轮控原已启用。
- 按实际 Python oracle 核对并修正旧测试的错误期望：50 ms 恢复积分上限、未覆盖扫描
  角域、清障端点偏移、重叠矩形距离界，以及局部位移和含 SDK 初始值的 anchor。
- 没有为这些测试修改运行时数学算法，也没有放宽比较容差。
- 清洁无绑定构建：导航 **2/2**、轨迹 **1/1**，轮控 **1/1** 通过；各自日志已保存。

## 成对性能实验

同一主机、同一配置、同一服务/消息负载，按 Python/C++、C++/Python、Python/C++
顺序运行三对；每进程预热 30 次，再测 10,000 次，总计 60,000 次测量。
每次检查接受结果、连续 epoch、输出尺寸和 pending 状态。ROS 时间暂停，以相同的
有效静止样本隔离提案处理开销；不是正常 10 Hz 空闲负载或整机导航负载。

| 指标（三次运行中位数） | Python | C++ | 变化 |
| --- | ---: | ---: | ---: |
| 10,000 请求总时间 | 3.7710 s | 1.8946 s | -49.76% |
| 被测进程 CPU 时间 | 3.23 s | 0.48 s | -85.14% |
| 采样 RSS | 61,336 KiB | 26,008 KiB | -57.60% |
| 请求至包络可见 P50 | 0.3454 ms | 0.1638 ms | -52.56% |
| 请求至包络可见 P95 | 0.5737 ms | 0.2985 ms | -47.98% |
| 请求至包络可见 P99 | 0.6784 ms | 0.3626 ms | -46.56% |
| 完成请求吞吐 | 2,651.8 /s | 5,278.1 /s | +99.04% |

延迟包含测试端发起服务、收到响应并观察对应 epoch 包络的全过程；进程 CPU 不含测试端，
从 `/proc` 取 user+system 时间，采样分辨率 10 ms。RSS 每 100 请求采样；非系统整体内存。
三次中位数不等于置信区间，也不证明 Nav2/Gazebo 或 SDK 性能收益。

[envelope_benchmark.json](envelope_benchmark.json) 包含全部延迟样本、运行顺序、源码/
二进制 hash、CPU/系统/ROS 信息和消息契约 hash。1000 请求预实验独立保留，最终表使用
10,000 请求版本，以降低 CPU 计数分辨率的影响。

## 清障扫描后续迁移验证

继续检查直接 C++ `costmap_scan_cpp`，复现 **3 failed / 5 passed** 后修复：

- 每次收到扫描时读取最新 `max_marking_range_m`，保留 Python 的动态参数行为。
  原候选缓存启动值，运行中降低范围后仍丢弃原本应发布的扫描，改成非法值后反而仍发布。
- 保留原实现对零值的处理；非法范围在对应扫描到达时拒绝，不擅自增加启动准入条件。
- 启动参数类型错误时返回非零退出码，避免记录 fatal 后却向启动脚本报告成功。

消息专项 **20 passed**，见 [costmap_scan_tests.log](costmap_scan_tests.log)。覆盖自定义输入
话题、空数组、全部元数据、1080 射线、NaN/正负 infinity/no-return、float32 边界、
动态参数、非法/非有限范围、拒绝后的恢复和启动失败。
每个字段按实际 float32 位值比较，不放宽数值容差；不比较 CDR 的非字段对齐填充字节。
大帧预实验曾因为这类填充字节失败，检查确认字段一致后修正测试比较器。
域 110 的连接探针可重发，正式测试帧仅发一次。

扫描性能采用相同三对交替顺序，每进程预热后测 5000 帧、每帧 1080 射线，合计 30,000
帧，每帧核对所有输出字段。延迟样本在正确性比较前截取；总时间包含比较成本，吞吐
受单个 Python 测试客户端限制，不能称作节点最大吞吐能力。

| 指标（三次运行中位数） | Python | C++ | 变化 |
| --- | ---: | ---: | ---: |
| 5000 帧总时间 | 2.5728 s | 1.4414 s | -43.97% |
| 被测进程 CPU 时间 | 1.62 s | 0.18 s | -88.89% |
| 采样 RSS | 56,100 KiB | 22,808 KiB | -59.34% |
| 输入至输出可见 P50 | 0.4961 ms | 0.2558 ms | -48.44% |
| 输入至输出可见 P95 | 0.7108 ms | 0.4448 ms | -37.42% |
| 输入至输出可见 P99 | 0.8029 ms | 0.5032 ms | -37.32% |
| 完成帧吞吐 | 1,943.4 /s | 3,468.8 /s | +78.49% |

原始样本见 [costmap_scan_benchmark.json](costmap_scan_benchmark.json)。这是适配器消息负载，
不包含真正 costmap 的障碍标记/清除效果，仍须在 Nav2 闭环验证场景行为。

## 仍然存在的边界与后续工作

- Python 的 `speed > limit` 会放过 NaN odometry；C++ 保持拒绝。两项专项测试明确断言
  此差异，未修改 Python oracle，也未把安全漏洞复制给 C++。
- C++ 的 ROS 时钟 TF Buffer 会在时间跳变时清理动态缓存；Python 当前 Buffer 不与
  同一时钟绑定。已验证 odom 回退拒绝，尚未验收动态 TF 缓存回退后的场景等价性。
- C++ 直接启动需要显式 `profile`；Python 默认 simulation profile。现有 launch 显式
  传入，不受影响。`fixed_v2` 仍明确拒绝使用该候选。
- 包络确认保留原语义：服务要求新鲜且静止的 odom；首次 footprint 确认要求已知静止
  样本，已有确认按形状检查保持。不能将其描述成每次确认都检查 odom 新鲜度。
- 仍需真实 Nav2 costmap 闭环、动态 TF 回退、重启后的上下游恢复验收，之后才能评估默认切换。
- 轨迹 `_chassis_math_native`、导航 `_navigation_math_native`、几何 `_geometry_native`
  三个绑定目标仍有消费者。下一步是把完整 ROS/SDK 端口、保护/感知/任务节点及几何消费者
  移到直接 C++ 接口，再逐个删除 facade 和依赖；只关闭构建开关不算完成删除。

## 复现

在仓库根目录，使用独立构建目录，先载入 ROS 与现有消息安装：

```bash
source /opt/ros/humble/setup.bash
source ws_robot/install/local_setup.bash
cmake -S ws_robot/src/astribot_s1_navigation_policy_native \
  -B /tmp/astribot_navigation_cpp_validation \
  -DASTRIBOT_BUILD_PYBIND=OFF -DBUILD_TESTING=ON \
  -DCMAKE_DISABLE_FIND_PACKAGE_pybind11=TRUE
cmake --build /tmp/astribot_navigation_cpp_validation -j2
ctest --test-dir /tmp/astribot_navigation_cpp_validation --output-on-failure
ENVELOPE_CPP_BINARY=/tmp/astribot_navigation_cpp_validation/envelope_coordinator_cpp \
POLICY_PROFILE_PROBE=/tmp/astribot_navigation_cpp_validation/policy_profile_probe \
python3 -m pytest -q \
  ws_robot/src/astribot_s1_navigation_policy_native/test/test_envelope_protocol.py \
  ws_robot/src/astribot_s1_navigation_policy_native/test/test_envelope_profile_startup.py \
  ws_robot/src/astribot_s1_navigation_policy_native/test/test_policy_profile_differential.py
python3 ws_robot/src/astribot_s1_navigation_policy_native/test/benchmark_envelope_protocol.py \
  --binary /tmp/astribot_navigation_cpp_validation/envelope_coordinator_cpp \
  --requests 10000 --pairs 3 --output /tmp/envelope_benchmark.json
COSTMAP_SCAN_CPP_BINARY=/tmp/astribot_navigation_cpp_validation/costmap_scan_cpp \
python3 -m pytest -q \
  ws_robot/src/astribot_s1_navigation_policy_native/test/test_costmap_scan_protocol.py
python3 ws_robot/src/astribot_s1_navigation_policy_native/test/benchmark_costmap_scan_protocol.py \
  --binary /tmp/astribot_navigation_cpp_validation/costmap_scan_cpp \
  --frames 5000 --output /tmp/costmap_scan_benchmark.json
```

域 108/109/110 必须留给本探针；性能实验单独运行，不与编译或回归并行。
源码清单见 [source_manifest.json](source_manifest.json)。默认启动与共享仿真安装未切换。
