# 到位控制实施与复测记录（2026-09-15）

目标为完整 Nav2 导航链路下的原地旋转和四向往返。真机使用 SLAM 的 map→astribot_torso_base 位姿验收，位置欧式误差不超过 3 cm，角度误差不超过 1.5°；Gazebo 真值定位仍按 2 mm / 0.1° 验收。

## 已实施的控制链

`SLAM / 仿真真值 → ArrivalController / ThreePhase / MPPI → 唯一速度平滑器 → 原有坐标与姿态处理 → 底盘桥接 → SDK 位置接口`

- 有限前瞻桥接在每个新 SLAM 时间戳到达时，用相同时刻的 SDK 历史位置重设接口基准，再施加本帧积分与有限前瞻。三个轴同步；不会把连续的小横向指令与长期累积的定位偏差叠加。SDK 历史位置仅用于位置接口坐标基准，到位闭环仍以 SLAM 为准。SDK 查询时间不是厂家硬件采样时间，时间对齐仍有采样不确定性。
- 精度档位在原 `velocity_smoother` 的位置使用 Ruckig，保持唯一平滑环节。指令按加速度、jerk 连续改变，正常停车逐渐撤除前瞻；超时、失效、停用仍立即输出零。消除了数值舍入残留的极小非零速度，确保桥接能识别真正的零速边沿。[Ruckig 官方说明](https://docs.ruckig.com/index.html)。实现针对两端已安装的 0.9.2 编译和验证。
- 末端停车预测采用参考曲线停止距离，而非固定 `v × 1.5 s`。真机线性预测额外使用旧批次测得的最终残余位移包络；最大前冲与最终残余没有混用。角度使用参考曲线预测，线性经验表不套用到 yaw。
- `ArrivalCoast` 只在本轴主动运动后停车或换向时等待停稳。另一轴运动引起的漂移可以被闭环纠正；每次完成 XY 调整后，重新计入进入精细 yaw 调整的阶段进展。
- 保留独立 XY/yaw 迟滞、0.6 s 源时间停稳窗口及碰撞检查。Nav2 的无进展位移/角度阈值与精度档位匹配，避免毫米级微调被旧的 1 cm / 5.7° 判据中止。

未调整 ThreePhase 起始对齐参数、MPPI 权重和采样参数，也未增加桥接速度/加速度限幅。本次没有同步开发机的通用 `chassis_bridge.yaml`；真机仍使用原部署档位的有限前瞻边界和输入失效处理。

## 关键参数

| 参数 | 仿真、真机共同值 |
|---|---:|
| 导航入口线速度上限 | 0.35 m/s |
| 末端最大线速度 / 角速度 | 0.06 m/s / 0.15 rad/s |
| 末端最低非零线速度 / 角速度 | 0.008 m/s / 0.02 rad/s |
| 参考曲线线加速度 / jerk | 0.25 m/s² / 0.5 m/s³ |
| 参考曲线角加速度 / jerk | 0.6 rad/s² / 1.2 rad/s³ |
| 平滑器频率 | 50 Hz |
| 进入保持 / 重新纠偏 | 到位容差的 65% / 90% |
| 单次末端总超时 | 135 s |

上述加速度/jerk 是参考指令约束，不能直接宣称为机械运动的实测上限。正常行驶的速度上限未降低；低速静摩擦和停稳等待允许占用更多时间，运行时长不作为本次优劣指标。

真机线性最终残余表：实测速度节点 `[0, 0.053, 0.093, 0.161, 0.249] m/s`，残余包络 `[0, 4, 4, 8, 12] mm`。这是旧接口停车试验形成的候选包络；本轮验证了末端闭环效果，没有重新完成所有速度档位的直接停车标定。超出实测节点上限不外推。

## 验证方式

1. 离线桥接 43 项检查通过，包括时间对齐、重复/乱序 pose、微小横向指令、三轴复位、超时、失效和停车写入次序。参考曲线反解、变目标加速度/jerk 约束、源时间停滞和轴间漂移检查通过。
2. 独立 ROS 生命周期测试收集 270 帧：正常停止最终输出精确零；停止输入后的零速响应为 0.265 s，位于配置的 0.3 s 超时附近。该时间从最后一次发送循环退出处计量，不是机械停车延迟。
3. Gazebo 使用正常仿真速率、真值定位，七个 Nav2 生命周期节点 active。执行 ±60° 原地转向及各自回原点，随后 x+/x−/y+/y− 各 25 cm 及各自回原点，共 12 个目标。
4. 补充 45 cm 的 x+、y+ 往返，共 4 个目标，覆盖起始对齐、FOLLOW、REFINE、DONE。距离较短，FOLLOW 数据处于终点附近；不能用它代表长直线或复杂曲线的整体跟踪性能。
5. 真机保留 SLAM 和当前地图。启动缺失的厂家反馈状态桥及 robot_state_publisher 后，`/scan` 从 8 s 内 0 帧恢复至约 10 Hz，没有伪造扫描或绕过自滤。随后通过 `/navigate_to_pose` 完整任务入口执行相同 12 点序列，并在更新阶段进展计时后复测。

每组目标相对同一个原点定义；返回动作不重定义原点。测试脚本只发送导航目标，不直接发送速度。每个动作成功后继续采样 2 s，最后 0.6 s 检查漂移；所有记录按源时间戳去重。真机 SDK odom 只用于对照，不把空 twist 当作真实零速。

## 结果与原始记录

详细结果、图和逐目标表见 [验证目录](../runs/arrival_control_20260915_210852/)。目录内保留了失败迭代，不能把它们算作通过样本。

- `sim_precision_v4`：12/12 通过，最大 1.312 mm / 0.08754°。
- `sim_follow_v5`：4/4 通过，最大 1.502 mm / 0.07892°。
- `hardware_precision_v1`：12/12 通过，最大 20.143 mm / 0.84597°；原点半径内的最大实测移动为 0.23442 m。成功后约 2 s 的最大位姿变化为 1.836 mm / 0.03239°，末 0.6 s 漂移最大为 1.730 mm / 0.02725°。
- `hardware_precision_v2`：最终版本 12/12 通过，最大 21.822 mm / 0.81553°；原点半径内最大实测移动为 0.23381 m。成功后约 2 s 的最大位姿变化为 1.821 mm / 0.03315°，末 0.6 s 漂移最大为 1.965 mm / 0.02030°。
- 两轮真机合计 24/24 通过，最大位置误差 2.183 cm、最大角度误差 0.846°。两轮结束均收到桥接 `disabled` 返回；15 个实施源文件的本机与机器人散列全部一致。

[最终逐目标结果表](../runs/arrival_control_20260915_210852/results_table_final.md) · [到位误差图](../runs/arrival_control_20260915_210852/arrival_errors.png) · [四向轨迹与转向收敛图](../runs/arrival_control_20260915_210852/hardware_trajectory.png)

上述验证目录和图片链接位于开发机；机器人上的原始真机数据在 `/home/astribot/astribot_projects/current/deployment/arrival_control_20260915_210852/hardware_precision_v1` 和 `hardware_precision_v2`。

初始失败分别暴露了轴间漂移误触发停车等待、Nav2 无进展阈值不匹配，以及完成 XY 后精细 yaw 阶段进展未重计的问题。补充短路径的较大 FOLLOW 航向偏差位于终点转向区；日志显示起始对齐交接角速度已经很小，未据此修改 ThreePhase 核心对齐算法。

真机精度是相对于本次 SLAM 定位的闭环误差，尚未使用外部视觉、mark 或测量仪验证物理位置误差，也未完成多地面、多负载、动态障碍物下的重复性统计。本次约 25 cm 的六维任务主要验证末端行为，不能据此保证所有长路径的性能。

## 真机操作

当前项目目录：`/home/astribot/astribot_projects/current`。

收尾状态：最终真机测试于 22:05:08 完成并停用桥接；22:07 会话目录出现 `STOP` 文件，导航与桥接随后退出。`session.json` 记录 `state=stopped`、`remaining=[]`。这发生在测试全部完成之后，不属于导航目标失败；未自动重启或继续发送目标。退出记录的 `feedback_stopped=false` 不能当作机械仍在运动的证据，也不能替代现场停车确认。测试结束前的零指令、停稳窗口和 `disable_result=disabled` 分别记录在测试原始数据中。

已有完整导航链运行时，可逐次执行：

```bash
cd /home/astribot/astribot_projects/current
source tools/robot/env_deployed.sh

# 仅检查目标点是否可规划；不会使能底盘。
python3 tools/robot/navigation_precision_check.py \
  --output "$HOME/chassis_tests/nav_precision_preview_$(date +%Y%m%d_%H%M%S)"

# 在确认场地安全后执行，默认 ±60°、四向 25 cm，并逐次返回原点。
RUN_DIR="$HOME/chassis_tests/nav_precision_$(date +%Y%m%d_%H%M%S)"
python3 tools/robot/navigation_precision_check.py --execute --output "$RUN_DIR"
```

脚本运行时 Ctrl-C 或 `touch "$RUN_DIR/STOP"` 会取消本次导航并停用桥接，不会自动进行回原点运动。成功完成后也会停用桥接。若要在同一次 SLAM/主机启动下复测同一个原点，给 `--origin /上一轮结果目录/origin.json`。

若当前没有导航程序，用原部署入口启动完整感知和人工目标导航：

```bash
bash /home/astribot/astribot_projects/current/tools/robot/run_deployed.sh precision
```

该入口会按现有设计重启感知与导航；不要在测试进行中再次启动。全任务一键关停：

```bash
bash /home/astribot/astribot_projects/current/tools/robot/stop_robot_tasks.sh
```

本轮最终导航统一日志：`/home/astribot/astribot_projects/current/deployment/arrival_control_20260915_210852/full_navigation_v5/session.log`，索引：`/home/astribot/.ros/log/astribot/latest_hardware_precision/session.log`。

代码备份位于同一部署目录的 `before_deploy/` 和 `before_progress_fix/`，安装散列见 `deployment.json`、`progress_fix_install.json`。回退应先停车并关闭导航/桥接，再恢复对应源文件和 ARM 安装文件；不能在控制进程运行时覆盖动态库。
