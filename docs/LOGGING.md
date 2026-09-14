# 统一日志：spdlog

仓库使用 spdlog 作为运行日志后端。公共包位于 `ws_robot/src/astribot_logging`。

| 来源 | 接口 | 文件后端 |
| --- | --- | --- |
| ROS2 C++ 节点、Nav2 插件 | `RCLCPP_INFO/WARN/ERROR` 等现有宏 | 由 launch 捕获控制台输出，写入 spdlog 轮转文件 |
| ROS2 Python 节点 | `node.get_logger()` | 同上，保留 `/rosout` |
| Gazebo、外部子进程和启动器 | stdout/stderr、launch logger | spdlog 文件 handler / 管道收集器 |
| SDK、启动诊断、独立 Python 工具 | `astribot_logging.get_logger()` | C ABI 调用系统 spdlog |
| SDK 内的厂商 Python logger | 显式替换其 stdlib handler | 同上 |

保留 ROS2 的 `/rosout`、节点名称、命名 logger、once/throttle 和运行时级别控制。
不重新实现 ROS 日志后端，不修改预编译厂商包。SDK logger 不初始化 ROS、DDS 或节点，
不更改 root logger，也不调用全局 `spdlog::shutdown`，ROS context 关闭后仍能记录 SDK 的退出日志。

## 构建

依赖由 `package.xml` 声明。Ubuntu 22.04 / ROS2 Humble 对应
`libspdlog-dev`、`ros-humble-rcl-logging-spdlog`、`ros-humble-ament-cmake-python`；
测试还需 `ros-humble-ament-cmake-pytest`。

```bash
source /opt/ros/humble/setup.bash
cd ws_robot
colcon build --symlink-install --packages-up-to astribot_s1_navigation astribot_s1_gazebo_bringup astribot_trajectory_bridge
source install/setup.bash
```

所有自有运行包均声明了公共日志包依赖；全量构建或 `--packages-up-to` 会自动处理顺序。
首次更新已有工作空间时，必须同时重建修改过 launch 文件的包。只构建日志包不能更新其它包的安装副本；
`ros2 launch` 读取的是 `install/<包>/share/<包>/launch`，不是 `src`。即使 Python 模块可见，旧 launch 仍可能绕过收集器。
真机必须在 aarch64 上构建，不复制开发机的 `.so`。
SDK 的 `env.sh` 会加载已构建的日志包；未构建时首次使用默认 logger 会明确报错。

## 统一配置

### 默认启动参数

不额外设置参数即可启动：

```bash
bash tools/launch_sim_stack.sh
```

上述脚本现在默认进入 `astribot_s1_navigation/sim_stack.launch.py`，统一 `session.log` 自动开启。
也可以直接使用 ROS2 launch 参数：

```bash
ros2 launch astribot_s1_navigation sim_stack.launch.py
bash tools/launch_sim_stack.sh log_level:=debug log_max_bytes:=20971520
# 只验证配置，不启动机器人：
bash tools/launch_sim_stack.sh dry_run:=true
```

`log_dir`、`log_level`、`log_max_bytes`、`log_backup_count` 均有默认值；
统一文件固定为会话中的 `session.log`。原有 `--log-level`、`--dry-run` 等 CLI 仍兼容。
`sim_stack.launch.py` 依赖本仓库 tools 目录；安装位置无法自动定位时传 `repo_dir:=<仓库根目录>`。

默认日志级别为 `info`，写入 stderr；纳管的每个日志文件达到 10 MiB 时轮转，
保留 5 个备份。仿真会话目录为 `~/.ros/log/astribot/sim_<时间>_<PID>`。
硬件入口自动加载相同级别与轮转默认值。
默认值集中在 `ws_robot/src/astribot_logging/env_hook/astribot_logging.sh`，
已设置的环境变量不会被默认值覆盖。

仿真启动也支持命令行覆盖，优先级为：命令行 > 环境变量 > 默认值。

```bash
bash tools/launch_sim_stack.sh --log-level debug --log-max-bytes 20971520 --log-backup-count 10
# 只查看最终配置，不启动进程：
bash tools/launch_sim_stack.sh --dry-run
```

这组参数只管理日志，不改变导航模式、控制器或速度等机器人运行参数。

启动前设置环境变量，然后 source 工作空间。配置应在进程第一次记录日志前完成。

```bash
export ASTRIBOT_LOG_DIR="$HOME/.ros/log/astribot/session_001"
export ASTRIBOT_LOG_LEVEL=debug
source ws_robot/install/setup.bash
bash tools/launch_sim_stack.sh --log-dir "$ASTRIBOT_LOG_DIR/simulation"
```

| 变量 | 默认值 | 范围 |
| --- | --- | --- |
| `ASTRIBOT_LOG_DIR` | 已设置的 `ROS_LOG_DIR`，否则 `${ROS_HOME:-$HOME/.ros}/log/astribot` | 全部运行日志目录 |
| `ROS_LOG_DIR` | 环境 hook 设置为 `ASTRIBOT_LOG_DIR` | ROS2、launch |
| `ASTRIBOT_LOG_LEVEL` | `info` | SDK、工具及仓库 launch 中的节点默认级别 |
| `ASTRIBOT_LOG_MAX_BYTES` | `10485760`（10 MiB） | SDK、launch 和进程输出文件的轮转阈值 |
| `ASTRIBOT_LOG_BACKUP_COUNT` | `5` | 每个纳管文件的备份数量 |

级别支持 `debug/info/warn/error/fatal`；`warning/critical` 是别名。
无效级别和非正轮转参数会报错。SDK 独立运行时每进程独立文件，多个 logger 共享该进程的 sink，
防止多进程对同一文件轮转。写到 stderr，stdout 保留给 JSON、报表、SRDF 等命令输出。
SDK 同步写入并逐条 flush；高频控制循环继续使用 ROS 原有节流日志，避免磁盘 I/O 干扰控制。
在 Python 多进程程序中使用 `spawn`，不要在日志初始化后 `fork`。

仓库 launch 通过公共 `Node` / `ComposableNodeContainer` 注入默认级别。
已有 `log_level:=error` 或节点显式 `--log-level` 优先，不会被环境默认值覆盖。
直接使用 `ros2 run` 时按 ROS2 标准传参：

```bash
ros2 run <package> <executable> --ros-args --log-level debug
```

`ASTRIBOT_LOG_LEVEL` 不是 ROS2 原生环境变量，不能替代上述直接启动参数。
环境 hook 使用 stock Humble 支持的 `{time}` 时间戳；字段统一为级别、epoch 时间、logger 名、消息。
ROS 与原生 spdlog 的级别大小写和小数位数可能不同，均使用系统时间，不依赖仿真 `/clock`。
`RCUTILS_CONSOLE_OUTPUT_FORMAT` 的显式设置仍被保留。

## 目录与保留策略

仿真 supervisor 默认在统一根目录下创建 `sim_<时间>_<PID>` 会话目录，
`--log-dir` 可指定新目录。每次启动创建新会话；同一会话的导航重试共用日志，通过来源字段区分。

```text
sim_<时间>_<PID>/
  session.log                    # 仿真、导航、ROS2 节点、SDK、探针和会话状态汇总
  session.1.log                  # 达到大小上限后的轮转备份（按需生成）
  session.json                   # 最新机器可读会话状态，unified_log 给出统一日志绝对路径
  nav_udp.xml                    # 可选的 DDS 传输配置，不是日志
```

新会话默认使用统一模式：supervisor 是 `session.log` 的唯一写入进程，子进程 stdout/stderr
通过管道汇入同一个 spdlog 轮转 sink。导航重试仍在同一文件中，以 `navigation_1`、
`navigation_2` 区分。每行增加 UTC 收集时间和来源，例如：

```text
[2026-09-14T08:00:00.123+00:00] [simulation pid=1234] [节点原始输出]
[2026-09-14T08:00:00.124+00:00] [navigation_1 pid=5678] [节点原始输出]
```

此 PID 是被收集的 launch 进程 PID；具体节点名、原始时间及级别保留在原始输出中。
收集时间表示到达 supervisor 的时间，不保证不同进程事件的真实发生顺序。

查看当前日志：`tail -F <会话目录>/session.log`。轮转默认仍为 10 MiB、5 个备份，
因此是一个当前日志文件加有界历史备份，不是无限增大的单文件。
`session.json` 继续保留供脚本读取，状态保存时也将完整快照写入 `session.log`。
新会话不再生成 `simulation.log`、`navigation_*.log`、`probe_*.log` 或重复的 SDK 日志。
旧会话不会自动转换或删除。

supervisor 为纳管进程设置内部变量 `ASTRIBOT_LOG_CAPTURE=1`：SDK 仅写 stderr，
launch 仅输出到控制台，再由父进程落盘。专用 `astribot_logging.launch_entry` 在 ros2cli
创建首个文件 handler 前安装此配置，避免重复的 launch.log。launch 可能仍创建空的时间目录。
**不要在没有父进程收集的独立终端手工设置该变量，否则只会有控制台日志。**
多进程不能直接把自己的 spdlog 轮转 sink 指向同一个文件；本实现由一个进程串行管理轮转。

独立运行的 SDK/launch（不经 supervisor）仍保留原有逐进程/launch 文件策略。

仓库 launch 的公共 `Node` / `ComposableNodeContainer` 会传入 `--disable-external-lib-logs`，
关闭 ROS 原生的额外文件输出，保留控制台和 `/rosout`。通过 launch 的公共 handler factory
将文件写入交给 spdlog。独立运行使用 `OVERRIDE_LAUNCH_PROCESS_OUTPUT=both`；
supervisor 纳管模式使用 `screen` 并由父进程同时收集 stdout/stderr。
这也覆盖被包含的 Gazebo `ExecuteProcess`。原先 `output='screen'` 的进程也会写入文件。
过滤级别仍在 ROS 节点内生效；收集器原样保存进程已经输出的内容，不按 stdout/stderr 猜测严重级别。

Humble 原生 `rcl_logging_spdlog` 不支持配置轮转，因此纳管 launch 使用上述控制台收集路径。
**直接 `ros2 run` 或第三方自带入口绕过公共 Node 时，仍走原生 spdlog 后端，其原生文件不受轮转变量控制。**
第三方内部自行创建的文件、colcon 构建日志、rosbag、CSV/JSON 数据产物不属于运行日志收集器。
硬件入口中的 ROS launch 文件同样接入 spdlog；shell 自身的重定向副本仍由 shell 管理。

收集器逐条 flush、持续排空子进程管道并检查写入错误，退出时等待读完剩余输出；
写盘失败会反映在会话状态中，不能报告正常完成。若纳管的进程输出/launch 文件被外部删除，
下一条输出会重建文件并写入恢复提示，已丢失的历史无法自动恢复。
每个文件有大小/备份上限，但跨会话目录总量没有上限，结束的会话仍需定期归档。
单条消息可能超过轮转阈值，不应记录完整点云、图像或大型数组。

## 本次缺失日志的定位记录

- 原会话 `sim_20260914_113358_3406517` 只保留 `session.json` 和两个 launch 生命周期日志。
  预期的仿真、导航汇总文件及节点文件已经不在目录中，无法从现有证据确认它们缺失的具体原因。
- 源码中 `output='screen'` 不会把子进程内容写入 `launch.log`；原 supervisor 的汇总文件
  是普通 `open(..., 'w')` 重定向，不经过 spdlog。此前“已经统一”的范围不完整。
- 本机 `install/astribot_s1_navigation/.../navigation.launch.py` 实际仍导入
  `launch_ros.actions.Node`，与源码不同。感知、动力学、底盘和桥接的安装副本也未更新。
- 本次已将收集路径接入 spdlog，并重新构建日志包和上述五个 Python 包，核对安装后的 launch 与源码一致。
- 回归验证使用真实子进程与 ROS 日志 context，验证 stdout/stderr、ROS 日志、退出尾行、轮转及文件删除后恢复；
  不把这些测试描述为整套仿真运行验证。

## 新代码写法

```python
from astribot_logging import get_logger
log = get_logger('astribot.my_tool')
log.info('connected to %s', endpoint)
try:
    perform_work()
except Exception:
    log.exception('operation failed')
    raise
```

ROS 节点继续用原生日志 API；新 launch 使用 `from astribot_logging.launch import Node`。
不要将供脚本消费的 JSON、报表和生成文件内容替换成日志。
Shell 命令回显、Gazebo 内部输出及第三方库内部诊断保留原协议，由会话文件收集。

## 验证

```bash
source /opt/ros/humble/setup.bash
source ws_robot/install/setup.bash
python3 ws_robot/src/astribot_logging/test/test_logging.py -v
python3 ws_robot/src/astribot_logging/test/test_output.py -v
```

测试覆盖真实 spdlog 文件、中文与堆栈、级别过滤、轮转、进程文件隔离、无效配置、
厂商 handler 去重、launch 参数优先级、ROS context 与 SDK 共存。只初始化测试 context，
不启动 Gazebo、不创建控制节点、不向机器人发送消息。

参考：[ROS2 Humble 日志架构](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Logging.html)、
[Humble spdlog 后端源码](https://github.com/ros2/rcl_logging/blob/humble/rcl_logging_spdlog/src/rcl_logging_spdlog.cpp)、
[spdlog 官方仓库](https://github.com/gabime/spdlog)。

统一模式额外验证：两个并发子进程的 ROS 控制台、SDK、普通 stdout/stderr 汇总且不重复；单写入者轮转保留来源与消息。测试不启动机器人或发布运动指令。

## 自定义日志目录的查找入口

supervisor 启动时立即打印 `session.log` 的绝对路径，不再需要等 READY。
默认日志根目录下的 `latest_sim` 符号链接指向最近一次建立日志的仿真会话，即使使用
`--log-dir /tmp/...` 也会登记。它表示最近创建的会话，不是存活或就绪证明；状态应查看
该目录的 `session.json`。轮转不改变 `latest_sim/session.log` 的查找方式。
若自定义目录被清理，链接会失效；索引不会复制日志或改变其保留周期。
若 `latest_sim` 已被普通文件或目录占用，程序保留它并告警，实际日志仍按打印路径写入。

2026-09-14 本轮定位：运行中的 supervisor 显式使用 `--log-dir /tmp/astribot_door_fix`，
进程映射包含 `libastribot_spdlog.so`，文件描述符指向该目录的 `session.log`。
因此本轮是自定义路径缺少默认目录索引，不是 spdlog 未加载。此次添加索引和即时路径提示。
