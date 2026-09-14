# 仿真验证记录（2026-09-13）

这是前一轮验证记录；当前恢复 ThreePhase 后的结果见 [最新清理验证](../../../docs/CLEANUP_VALIDATION.md)。

## 环境与结果

Gazebo 仓库场景，ROS 2 Humble，MPPI，全向底盘；静态地图 `maps/warehouse_baseline.yaml`，
`ground_truth` 定位，精调来源 `nav2_pose`。误差相对于用户下发的连续目标位姿，
依据 Gazebo 真值 `/odom` 计算，在 Action 成功时及两秒后各采样一次。

五个目标均返回 SUCCEEDED（状态 4），停车两秒后的位姿均与成功时一致。

| 测试 | 目标 (x m, y m, yaw °) | 修正栅格终点前位置误差 cm | 最终位置误差 cm | 最终角度误差 ° | 耗时 s |
|---|---|---:|---:|---:|---:|
| 直行 | (1, 0, 0) | 3.190 | 1.777 | 0.857 | 16.11 |
| 横移 | (1, -1, 0) | 2.845 | 1.774 | 0.917 | 14.26 |
| 斜向并转向 | (0, -2, -90) | 1.149 | 1.572 | 0.853 | 14.36 |
| 原地转向 180° | (0, -2, 90) | 0.575 | 1.556 | 1.004 | 28.46 |
| 返程 | (0, 0, 0) | 4.441 | 1.962 | 1.215 | 23.31 |

最大位置误差 **1.962 cm**，最大角度误差 **1.215°**，满足 3 cm / 1.5° 验收条件。

## 实测发现与修正

Smac 在 tolerance=0 时仍把 `(1, 0)` 目标返回为约 `(0.995, -0.025)` 的栅格中心。
对这个偏移末点的精调成功不能保证对用户目标满足 3 cm；直行和返程分别测得 3.190 cm、
4.441 cm。增加 ExactGoalPlanner 保留原始目标后，以相同五个目标重新验证，结果见上表。

## 不可达验证

- 目标 `(3, -1, 0°)` 位于占用栅格。ComputePathToPose 返回 ABORTED（状态 6），无路径。
- 绕过规划器，直接通过 FollowPath 发送终点相同的稠密直线路径。控制器在障碍物前无法继续运动，
  日志出现 `PATH_TRACKING/NO_MOTION_PROGRESS`，44.38 秒后返回 ABORTED；未触发测试程序取消。
- 失败两秒后真值位置约 `(2.353, -0.954)`，平移速度与角速度均为 0。

失败后另发返回起点的 NavigateToPose 目标，返回 SUCCEEDED（状态 4），导航可继续使用。

## 构建与回归

- path_tracking、navigation 在工作空间构建成功。
- 临时目录运行到位控制器回归 16 项通过，覆盖误差、停稳、碰撞、定位源、过期数据、TF、重规划预算。
- 保留的运行指标记录模块测试 17 项通过；Python 语法、YAML、XML 与 diff 空白检查通过。
- 按清理要求，废弃控制器、航向门、验证/扫速脚本及相关测试从运行仓库移除；
  本次一次性验证程序与原文件备份留在 `/tmp/astribot_cleanup_validation`，没有安装进运行包。

## 运行状态与边界

Gazebo 实测 21 个 transport 话题，iterations=302179，实时因子 0.9995。
六秒采样收到 clock 5912 帧、odom 298 帧、scan 64 帧；TF 龄期 0.014 秒，
controller_server、planner_server、bt_navigator、velocity_smoother 均为 active。Gazebo 与 RViz GUI 已启动。

整体冷启动曾卡在 controller_server 配置阶段。分开启动仿真与 Nav2 后完成上述验证；
这是规避手段，根因未确认。复现命令见 [导航说明](../astribot_s1_navigation/README_NAVIGATION.md)。

本次运动实测仅覆盖 MPPI + nav2_pose/真值定位。SLAM、视觉、Mark 接口通过程序回归，
尚未接真实传感器验证；RPP 未进行本轮运动实测。实机误差仍取决于定位噪声、标定与底盘响应。

原始数据与日志：`/tmp/astribot_cleanup_validation/{navigation.json,blocked.json,probe.json,nav_restart.log,stack.log}`。
临时目录可能被系统清理，本文件保留关键验收数据。
