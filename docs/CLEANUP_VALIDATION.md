# 保持逻辑的清理验证（2026-09-13）

## 最终范围

按照最后确认的要求，撤回了本轮所有行为修复。清理仅移除冗余注释、54 个单元测试文件及
6 个无用实验脚本，并同步删除测试构建/依赖和调试脚本安装入口。
ThreePhaseController 是用户明确要求的恢复项：核心实现及 MPPI/RPP 配置来自清理前备份。
既有精确终点规划、3 cm / 1.5° 到位精调和不可达处理保留。

删除的实验入口：`watch_nav_live.sh`、`run_five_round_exploration.sh`、
`summarize_five_rounds.py`、`measure_heading_vs_tangent.py`、`measure_path_clearance.py`、
`scan_slice_debug.py`。仍在使用的启动、实机部署、诊断和 SDK 示例保留。

## 等价性检查

| 检查 | 结果 |
|---|---|
| Python AST | 159 个保留文件与本轮开始时一致 |
| C++ 词法内容 | 62 个文件排除注释/空白后相同；恢复的跟踪文件与清理前备份比较 |
| Shell 执行文本 | 16 个保留文件排除注释行后相同 |
| 运行 YAML | 32 份配置相同；恢复 ThreePhase 的参数单独与旧配置逐项核对 |
| 构建元数据 | 5 个 setup.py 仅移除测试声明；CMake 移除测试块与已删调试入口 |
| 语法/结构 | 208 个 Python、Shell、XML/xacro、YAML 文件通过检查 |
| 空白检查 | git diff --check 通过 |

这些检查证明本轮清理未改变保留代码的计算流程，不等于原逻辑不存在漏洞。
问题与建议见 [逻辑问题清单](LOGIC_REVIEW.md)，未应用其中的行为修改。

## 构建及离线回归

12 个自研工作空间包构建成功。description 保留原有 CMake CMP0009 开发警告，不影响构建；
本轮未为消除该警告改变符号链接安装策略。

临时目录运行原有对齐/惯性/接近限速回归 54 项、到位精调回归 16 项，均通过。
规划器对照 1 项覆盖三个目标和障碍绕行：Smac 与精确终点适配的路径点数相同，
除最后一个位姿外逐点相同，末点为用户原始目标。

回归程序仅在 `/tmp/astribot_repo_cleanup` 及此前的临时验证目录中运行，没有重新加入仓库。

## 仿真验收

Gazebo 仓库场景，静态地图 `maps/warehouse_baseline.yaml`，真值定位，MPPI。
按已验证方式分开启动 Gazebo/感知/RViz 和 Nav2。整体冷启动问题本轮未修改。
误差相对原始下发目标，取 Gazebo `/odom`；成功时和两秒后各采样一次。

| 目标 | 目标位姿 (m, m, °) | 位置误差 cm | 角度误差 ° | 耗时 s |
|---|---|---:|---:|---:|
| 直行 | (1, 0, 0) | 1.849 | 0.917 | 16.11 |
| 横移 | (1, -1, 0) | 1.338 | 0.984 | 23.31 |
| 斜向 | (0, -2, -90) | 1.858 | 1.001 | 20.31 |
| 原地转向180° | (0, -2, 90) | 1.827 | 0.955 | 27.56 |
| 返回起点 | (0, 0, 0) | 0.500 | 0.990 | 21.26 |

五个 Action 均为 SUCCEEDED，停车两秒后的采样位姿均与成功时一致。最大位置误差 1.858 cm，
最大角度误差 1.002°。此结果是功能回归，不能据此保证随机 MPPI 的每一拍轨迹或耗时完全相同。

Gazebo 实测 21 个 transport 话题，iterations=41828，实时因子 0.9994。
六秒采样：clock 5838 帧，odom 297 帧，scan 63 帧。
Nav2 日志确认七个受管节点均已 active；Gazebo/RViz 保持运行，机器人已回到起点。

本轮运动仅验证 MPPI + 真值定位；RPP 参数已核对，未重跑其运动路线。SLAM、视觉和 Mark
接口有离线回归，未进行真实传感器或实机运动验收。

原始记录位于 `/tmp/astribot_repo_cleanup`：`equivalence.json`、`build_preserved.log`、
`arrival_preserved.log`、`align_preserved.log`、`planner_preserved.log`、
`navigation_preserved.json`、`nav_preserved.log`、`stack_preserved.log`。
备份 `before.tar.gz` 保存本轮开始时的工作区文件；临时目录可能被系统清理。
