# Arm speed limiter 在线参数回归

2026-09-21。本轮修改 C++ 限速适配器读取参数的时机、无效在线参数的原子拒绝，以及构造异常的退出码；不是整栈或真机验收。未修改共享 install、未运行 Gazebo、未建立厂家 SDK 会话。

## 问题与修复

旧 Python 在关节回调、TF 查询或重发时读取参数。C++ 迁移曾在构造时缓存七项：`extended_speed_limit_pct`、`chassis_base_frame`、`monitored_links`、`extended_reach_m`、`extended_reach_hysteresis_m`、`folded_reference_rad`、`extended_threshold_rad`。ROS 参数服务接受更新后，C++ 的实际输出仍使用旧值。

现在在原消费位置读取这些参数。`extension_metric` 仍只在启动解析；topic 与 timer 周期仍只在构造时决定。未改变 `last_extended`、迟滞公式、TF 新鲜度/有限值/缺失判据、无新 JointState 时的重发机制。

构造时 `reach_tf_timeout_sec` 或 `check_period` 非法，原 Python 退出码为 1，旧 C++ catch 后返回 0。C++ 现在记录异常、shutdown 并返回 1；正常 SIGINT 收尾仍返回 0。

边界审查进一步发现：直接恢复 getter 后，NaN 阈值会令比较恒 false，出现解除限速。新增 on-set callback **只校验合并后的候选参数，不写参数也不修改 last_extended**；任何无效项拒绝整个原子批次。相较旧 Python，这是明确更严格的无效在线参数契约：

| 在线参数 | 接受范围 |
|---|---|
| extended_reach_m | 有限，> 0 |
| extended_reach_hysteresis_m | 有限，0 <= hysteresis < 同批候选 reach |
| extended_threshold_rad | 有限，>= 0 |
| folded_reference_rad | 各值有限；不强制长度，保留原 zip 语义 |
| extended_speed_limit_pct | 有限，(0, 100] |
| reach_tf_timeout_sec | 有限，> 0 |

动态 `extended_speed_limit_pct=0` 被拒绝：本机 `/opt/ros/humble/share/nav2_msgs/msg/SpeedLimit.msg` 明确 0 表示 no-limit，仓库 ArrivalController 同样将 0 解释为解除。解除限速继续由“收拢”分类输出 0；未找到已有生产配置以动态 0 表达另一种任务语义。100% 与小于 1 的正百分比仍接受。topic、metric、check_period 不在此校验扩展的动态功能范围内；startup metric fallback 保持原样。

## 已验证

| 阶段 | 结果 |
|---|---|
| 修复前真实 ROS A/B | 14 项：10 failed / 4 passed；失败分别为七项参数缓存、更新为缺失 TF link 后未保护、两项非法参数仍返回成功 |
| 首次参数/退出码修复后真实 ROS A/B | 14/14 passed；包含上述差异、TF stale 保守限速、保留启动 metric/timer 与 hysteresis last state |
| 无效更新门禁加入前 red | 新增 8 场景中 6 failed / 2 passed；六组无效更新被 C++ 接受，合法合并批次/短参考边界通过 |
| 最终真实 ROS 回归 | 22/22 passed；6 组共 24 个无效原子更新全部拒绝，回读参数不变，已展开限速仍为 50% |
| 最终完整纯 C++ 构建 CTest | 7/7 passed；六项既有核心/独立进程回放，加一项包含上述 22 用例的 ROS 回归，不与 22 简单相加 |

Python reference 与 C++ 是独立子进程。测试驱动通过真实 ROS 参数服务更新，用同一 JointState 与 TF 数据校验两端 SpeedLimit；期望值为独立字面量（50%、27%、解除限速 0%），未调用被测数学函数生成期望。

域固定 118、localhost；各节点使用唯一 namespace/TF frame。测试读取每个自有 child 的 `/proc/PID/environ` 核实域和 localhost；C++ 运行映射中断言无 libpython / `_chassis_math_native`。最终 40 个自有子进程均按其 Popen PID 回收（36 个正常结束码 0，4 个非法启动码 1）。未按名称扫描/杀死外部进程。

首轮测试工具在短命进程 exec 期间读到瞬时空 `/proc/PID/environ`，导致 4 项工具断言失败。已改为有界等待环境可见，然后在**未修改生产 C++**前重跑，得到上表有效 red 结果。最初日志保留为 `red_harness_initial_*`，不把该轮当作启动退出码证据。

## 证据与版本

- `red_manifest.json`：修复前源码、测试与 ELF SHA256、构建开关、测试命令。
- `red_pytest.txt` / `red_junit.xml`：有效 red 结果。
- `green_manifest.json`：修复后逐源码、oracle、测试和 ELF SHA256，以及结果/边界。
- `green_ctest.txt` / `green_ctest_junit.xml` / `green_LastTest.log`：完整 CTest 与内层 14 用例结果。
- `red/` 与 `green/`：逐场景参数接受结果、实测 SpeedLimit 序列、节点日志、命令/PID/回收退出码。
- `configure.log` / `build_red.log` / `build_green.log`：Release、`ASTRIBOT_BUILD_PYBIND=OFF` 构建记录。
- `green_dependencies.txt`：实际候选 ELF 的解析动态依赖；无 Python/pybind 依赖。
- `validation_red_*`：新增原子参数门禁前的独立红测（6 failed / 2 passed）。
- `validation_green_manifest.json` / `validation_green_ctest*` / `validation_green_LastTest.log` / `validation_green_dependencies.txt`：**最终版本**的 SHA256、完整 7/7 CTest、内层 22/22 ROS 测试与依赖。
- `validation_red/` / `validation_green/`：第二轮逐场景原始记录；最终包含 24 个拒绝的批次及保持限速记录。

最终候选：`/tmp/codex_arm_parameters_20260921/build/arm_speed_limiter_cpp`。
最终 SHA256 见 `validation_green_manifest.json`；`green_manifest.json` 是加入原子校验前的中间版本，不能冒充最终二进制。

## 复现

从仓库根目录运行；以下安装前缀为独立临时目录，未执行 install。

```bash
source /opt/ros/humble/setup.bash
cmake -S ws_robot/src/astribot_trajectory_bridge_native \
  -B /tmp/codex_arm_parameters_20260921/build \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DASTRIBOT_BUILD_PYBIND=OFF -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_INSTALL_PREFIX=/tmp/codex_arm_parameters_20260921/install
cmake --build /tmp/codex_arm_parameters_20260921/build -j2
ASTRIBOT_ARM_PARAMETER_EVIDENCE="$PWD/docs/evidence/pybind_removal_20260921/arm_parameters/validation_green" \
  ctest --test-dir /tmp/codex_arm_parameters_20260921/build --output-on-failure
```

新测试已注册为 `arm_dynamic_parameters`，`TIMEOUT=120`、`RUN_SERIAL=TRUE`。CMake 显式给出候选路径、domain 118 和 localhost。旧 Python 仅由测试文件的 oracle launcher 导入类；没有恢复任何生产 Python main/console/launch 入口。

构建/运行显式增加 `rcl_interfaces`；测试清单显式列出 `python3-pytest`、`rclpy`、`geometry_msgs`、`tf2_ros_py`，并使用包已有的 `nav2_msgs`、`sensor_msgs`、`tf2_ros` 依赖。oracle 来自本仓库相邻 navigation 包的参考源码，路径与哈希存于 manifest；不会把这一测试 launcher 安装成生产入口。

剩余边界：未对本轮改动做新的性能测量，未验证真实机器人在线调参、真实 TF 延迟或整栈运动效果。其他尚未迁出的 Python/pybind 消费者不在本次修复范围。
