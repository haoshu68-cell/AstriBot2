# 臂—底盘连续耦合节点 C++ 迁移证据

范围：`astribot_s1_dynamics_coupling/arm_chassis_speed_coupling_node`，不是二值
`arm_speed_limiter_cpp`。本次按现有逻辑迁移，接口/时钟/参数/已知缺陷详见
[CONTRACT.md](CONTRACT.md)。源码快照为当前共享工作树，其他任务的 Git 提交状态
不能用于替代本目录的源文件 SHA256。

## 交付边界

生产包已改为 ament_cmake；同名 executable 是纯 C++ ELF，launch/config 不需要改。
旧 Python node、metric、__init__、setup.py/setup.cfg 和手写 ament_python resource
已从生产位置删除。三文件逐字节冻结于 `test/reference`，只由测试显式加载且不安装。
另调整 bridge_native 的一个数学对照测试导入路径，未改其 CMake 或生产代码。

为保持独立部署，包内小型纯 C++ header 承载耦合所需公式与 zip/map 规则，无可选绑定、
没有从其它 Python 包导入内核；无全局生产回退。安装仅包含本 ELF、launch/config/
README 和 ament 元数据。必须使用干净前缀：共享旧 install 完全未改、未激活，其
历史脚本不会因本次独立安装自动消失。

已验证层级：Release 编译；纯核心固定种子相同输入；真实 ROS 2 Humble 隔离协议、
TF、参数服务、仿真时钟；独立进程的性能/短时连续消息测试。没有启动 Gazebo、Nav2、
SDK、硬件控制器，也没有整机闭环或真机安全验收。

## 回归与失败记录

- 初始 `red.txt`/`red_junit.xml`：10个先写测试全部失败，原因是 native executable/core
  尚未实现；这是缺失实现的 red，不是声称已有旧 C++ 行为回归。
- `first_green.txt`：初版10/10通过，其中10,007条核心输入与9个ROS用例。
- `mutation_red.txt`：仅 `/tmp/.../mutation` 的 core 副本将 activity 系数乘0.5，
  原核心测试立即失败，证明公式差异会被检测；从未安装或替换正式源码。
- `green.txt`：扩展后19通过1失败，失败是测试把DDS discovery的未报告depth=0误当
  实际队列深度；源码两侧均KeepLast10。修正测试后检查已公布reliability/durability。
- `log_clock_red.txt`：独立审查发现陈旧告警限流时钟不同。真实ROS暂停/回退用例
  Python输出3条、旧Cpp仅1条；改为独立system clock，最终用例通过。
- `types_probe/results.json`：6种startup类型覆盖×两实现，12子进程均非零退出。
  int/bool/string不是double的兼容override，整数数组也不是double-array；未放宽类型。
- 最终 `final_ctest.txt` / `final_ctest_junit.xml` / `final_LastTest.log`：2/2 CTest，
  包含31个pytest用例；核心10,027条输入与30个真实ROS用例。正常子进程必须exit0，
  非法startup必须非0；每例JSON记录自有PID及回收结果。

新增边界包括：动态参数与原子错误类型拒绝、topic remap、QoS、joint消息重复/缺失/
空/截断/非有限、速度数组长度不符、±π原始角差、EMA与无定时输出；静态zero stamp、
TF未来/陈旧/任一路缺失/无效xyz/恢复、cache精确边界；超时等于边界和+1ns、时钟
回退；启动fallback/invalidtimeout。极大有限坐标的平方溢出与和溢出分别对照。

NaN/Inf阈值与minimum按Python min/max参数顺序对照；NaN/Inf alpha或NaN minimum
污染EMA且重设有限参数不能清除NaN，原实现同样如此。测试明确记录这个继承缺陷，
未把它包装为安全通过。freshness超时仍可切到有限degraded_scale；更改这些控制策略
需独立授权和新的安全契约。中间件抛异常、进程崩溃期间cmd链中断、长时内存泄漏等
未验证，不得从短时测试推断。

## 性能定义

`test/benchmark_ros.py` 运行4组配对，顺序 P/C、C/P、P/C、C/P；每次只起一个被测
实现，节点全部线程固定CPU26、驱动CPU27（实际亲和掩码随每次JSON记录）。每次预热
2秒、采样10秒：14关节50Hz、4路TF批次20Hz、Twist100Hz；水平reach固定0.65m，
默认EMA0.25与阈值。CPU = 被测进程 `/proc/PID/stat` 的 user+system CPU时间增量 /
采样墙钟×100%，按单核100%计；RSS为被测进程statm每0.2秒采样，不包含驱动。

延迟是驱动publish调用前 monotonic_ns 到输出回调 monotonic_ns，包含双向DDS、节点
处理和驱动调度；不是纯算法执行时间。Twist无header，以vx/vy比例编码序号，缩放
前后比值不变；每条输出可关联输入，记录CSV。固定workload的scale/其它轴均有断言，
统计缺失序号、输出数量与RSS短期增长。该规模不代表全导航栈负载或硬实时上界。

采样前与其他agent协调性能空窗，没有修改或清理共享仿真。所有query环境从自有
child `/proc/PID/environ` 反读核验：domain140–142、localhost；C++ maps无libpython
或_chassis_math_native。只对Popen保存的PID发SIGINT并wait；必要升级信号会使正常
cleanup断言失败；未运行任何按名字pkill/daemon/shm全局清理。

性能数值见 `performance/benchmark_summary.json` 与本README末尾的结果表。

## 复现

从仓库根目录，仅加载 `/opt/ros/humble/setup.bash`，不加载共享工作区overlay：

```bash
source /opt/ros/humble/setup.bash
cmake -S ws_robot/src/astribot_s1_dynamics_coupling \
  -B /tmp/codex_dynamics_coupling_20260921/build \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_INSTALL_PREFIX=/tmp/codex_dynamics_coupling_20260921/install
cmake --build /tmp/codex_dynamics_coupling_20260921/build -j2
DYNAMICS_EVIDENCE="$PWD/docs/evidence/pybind_removal_20260921/dynamics_coupling/final_green" \
  ctest --test-dir /tmp/codex_dynamics_coupling_20260921/build --output-on-failure
cmake --install /tmp/codex_dynamics_coupling_20260921/build
DYNAMICS_CPP=/tmp/codex_dynamics_coupling_20260921/build/arm_chassis_speed_coupling_node \
DYNAMICS_EVIDENCE="$PWD/docs/evidence/pybind_removal_20260921/dynamics_coupling/performance" \
  /usr/bin/python3 ws_robot/src/astribot_s1_dynamics_coupling/test/benchmark_ros.py --duration 10 --pairs 4
```

复现性能前应另行协调空窗。上面测试依赖package.xml所列rclpy、tf2_ros_py、
rosgraph_msgs、rcl_interfaces、pytest、yaml；生产仅依赖C++ ROS组件与launch/logging。
真实运行日志是每例目录的 `python.log`/`cpp.log`（完整路径在result.json命令行）；
未创建整栈session.log或latest_sim。原始CSV、JSON与node日志各自归档，不混作物理轨迹。

## 实测结果（4次/实现：中位数，括号为各次范围）

| 指标 | Python | C++ |
|---|---:|---:|
| 单核CPU % | 5.400 (5.300–5.599) | 0.850 (0.800–0.900) |
| RSS MiB | 65.901 (65.829–65.963) | 24.109 (24.009–24.154) |
| 端到端p50 ms | 0.367 (0.332–0.392) | 0.187 (0.185–0.193) |
| 端到端p95 ms | 0.715 (0.688–0.777) | 0.357 (0.343–0.360) |
| 端到端p99 ms | 0.855 (0.783–1.765) | 0.450 (0.423–0.585) |

8次各1000条，合计8000/8000输出关联成功，无丢失/重复，有限正常负载的scale全符。
Python最大单条延迟3.784ms，Cpp3.933ms；C++最坏尾延迟没有改善，不能宣称所有延迟指标更优。
每次10秒RSS增长：Python [0.08984375, 0.09375, 0.09375, 0.09765625] MiB，C++ [0.07421875, 0.07421875, 0.07421875, 0.07421875] MiB。这是短时采样，不能据此证明无长期内存泄漏。

最终构建ELF SHA256：`201fca0b97f42cd6fea13d1886fd7f03daaffa4a5ade8555158368666d9aec4d`。
安装ELF SHA256：`9dce7eb86190c8fc1dd01b2ce7495f1725850f0072e7f1ba0aa1f73dc2459b11`；CMake去除build RUNPATH后哈希变化，安装产物另外3/3 ROS用例通过。
安装枚举仅1个运行入口（installed_executables.txt），colcon识别ros.ament_cmake；完整安装与清理清单见final_manifest.json。

安装探针首次误按colcon workspace布局source prefix/local_setup.bash（direct CMake install没有这个聚合脚本），没有激活；改source share/astribot_s1_dynamics_coupling/local_setup.bash后ros2 pkg executables成功。这是探针路径错误，不是构建/运行失败。

默认构建类型已设为Release（单配置且用户未指定时）；两个新目录配置验证：无选项=Release，显式Debug仍为Debug，证据default_configure.txt/debug_configure.txt。性能与回归使用显式Release，ELF未变。
