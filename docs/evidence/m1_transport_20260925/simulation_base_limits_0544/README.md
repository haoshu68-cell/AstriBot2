# 仿真底盘门槛：MTC 最小变更归档

用户经主线明确授权仿真底盘姿态门槛放宽。本任务仅拥有本目录列出的 MTC 四个文件；主线拥有 Git、构建/安装、native、idle hold 默认配置及现场运行。

## 契约与范围

共享头 `base_motion_limits.hpp` 提供 `BaseMotionLimits` 和 `baseMotionLimits(bool relaxed, bool simulated)`。默认平移 0.02 m、旋转 0.02 rad、线速度 0.02 m/s、角速度 0.03 rad/s；仅显式仿真放宽时旋转改为 0.05 rad、角速度改为 0.10 rad/s。平移和线速度不变。`relaxed && !simulated` 抛出 `std::invalid_argument`，错误为 `SIMULATION_RELAXED_BASE_MOTION_REQUIRES_USE_SIM_TIME`。

`execution_guard` 新增只读布尔参数 `simulation_relaxed_base_motion`，默认 false；启动时读取 ROS `use_sim_time` 传入共享函数。仅替换其平移/旋转硬编码阈值；跟踪误差、300 ms 新鲜度、时钟回退、上下文隔离、锁存故障等原逻辑未修改。速度阈值由主线 native 使用同一共享头消费，本任务不修改 native。`use_sim_time` 是配置门槛，不能独立证明连接的是仿真设备。

CMake 增加共享头安装。现有 `execution_guard_test` 覆盖默认的仿真/非仿真配置、放宽配置四个字段、非仿真放宽拒绝；原有姿态、跟踪误差和新鲜度断言保留。

## 验证边界

本任务仅完成文件比对和限定文件的 `git diff --check`；未运行编译、测试、ROS 或机器人操作。构建与 `execution_guard_boundaries` 测试等待主线结果，不把实现完成视为验证通过。目标：`execution_guard`、`execution_guard_test`；还须由主线安装共享头后验证 native 对接。`before/` 保存修改前的工作区原始字节，`after/` 和清单保存封板字节，`changes.patch` 包含新增头文件。

## 主线验证回填（2026-09-25 05:52 后）

上述待验证状态保留为封板时历史记录。现已只读核对主线 `runs/mainline_20260925/base_motion_user_0542/` 的构建、安装及测试日志，MTC、native、底盘构建安装均有成功记录。原始日志、CTest LastTest 和 native XML 已复制至 `verified_coordinator/raw/`；最新机器可读结论见 `verified_coordinator/verification_summary.json`。

| 验证项 | 原始记录核对结果 |
| --- | --- |
| execution_guard_boundaries | 1/1 CTest 目标通过 |
| native 四组 | 4/4 CTest 目标；trajectory_digest 6、payload_scene 6、mtc_plan 15、scene_binding 5，共 32 个 GTest 用例通过，失败/错误/禁用均为 0 |
| wheel_math / wheel_reversal | 2/2 CTest 目标通过 |
| MTC 四个封板文件 | 当前源码哈希全部与封板清单一致 |
| 共享头安装 | 安装头与封板头 SHA-256 一致 |
| idle_position_hold / idle_position_kp | 主线源码默认值与源配置、安装配置分别为 true / 3.0；AGENTS 已记录用户固定要求 |

主线配置及 AGENTS 的三个源码哈希与 `user_decision.json` 一致。源配置和安装配置有一行注释差异，所有非注释有效行一致；首次归档的整文件相等断言因此失败，核对差异后按真实结果记录，未修改任何配置。归档保留了构建警告和测试原始诊断输出，不以摘要替代原文。

以上为构建、离线测试及静态配置证据。7 个 CTest 目标与 32 个 GTest 用例属于不同计数层级，不相加为同类用例数；本轮也未重新执行此前 MTC 的 24 项载荷重规划测试。安装候选文件哈希不能证明 scene38 实际加载的二进制或运行时参数。scene38/domain53 的进程身份、参数读回、完整任务结果及视频仍由主线独占验证，本任务未启动 ROS、未运行构建或测试、未操作机器人。
