# 统一 SLAM 最终布局仿真验证

日期：2026-09-18。验证对象是迁移后的 SLAM/栅格/消息包、项目 Eigen 3.4.0 与项目编译的 GTSAM。使用仓库根 `install`，Gazebo Fortress、ROS 2 Humble、Nav2 MPPI、策略 `off`，Gazebo headless 并启动 RViz。

## 验收结果

| 工况 | 目标结果 | 最大到点 XY | 最大到点 yaw | 无效采样 |
|---|---:|---:|---:|---:|
| run15 新建图 | 3/3 | 2.502 cm | 1.383° | 0 |
| run16 完全重启后载图 | 3/3 | 2.686 cm | 1.395° | 1 |
| run16 同会话复测 | 3/3 | 2.690 cm | 1.213° | 0 |

路线为 `(0.6,0,0) → (0.6,-0.5,-0.5) → (0,0,0)`，位置单位 m，角度 rad。位置来自 SLAM 建立的 `map → astribot_torso_base`，不是独立地面真值。目标阈值仍为 3 cm / 1.5°，跟踪耗时不计入性能判分。本轮没有修改控制器参数。

这是正常建图、存图、载图与短路线的功能回归；不证明完整历史路线、窄通道、长时稳定性或真机绝对精度。航向/横向误差、速度、加速度、jerk、路径版本等原始指标均保留于每组 `route/results.jsonl` 和 `samples.csv`。

## 分层证据

- Gazebo：21 个 transport 话题，`/stats` iterations 增长，启动检查 RTF 约 1。
- ROS：实测 `/clock` 约 1 kHz、IMU 200 Hz、左右 PC2 与导航扫描约 10 Hz、地图约 1 Hz；七个 Nav2 生命周期均 active。以源时间计算话题频率，详情见 observe.txt。
- 地图：run15 保存 44 个关键帧、1911 条扫描位姿；PGM/YAML、轨迹、关键帧与 manifest 完整性校验通过。
- 载图：run16 冷启动重新匹配已保存会话，ICP RMSE 约 0、最小特征值 32.1347；匹配与优化后才进入 TRACKING。该匹配是同一仿真出生位置，不代表任意位置重定位性能。
- 探索：run16 在载图会话启动探索协调器，连续完成 2 个自主候选目标的导航、到达和驻留校验；随后在校验下一候选期间调用暂停服务，成功冻结派发。探索驻留判据沿用协调器配置，不能替代路线工具的 3 cm / 1.5° 精度评价；未验证整仓库探索完成、活动目标取消或暂停后恢复。
- 接口：关键帧类型为 `astribot_slam_msgs/msg/KeyframeSubmap` 和 `KeyframePoseArray`；未发现 slam_toolbox 节点。`/slam/pose` 实测 10 Hz、frame 为 map、协方差数值有限。首次诊断订阅误用了 reliable，改用与发布端兼容的 SensorDataQoS 后收到数据；控制器本身已使用兼容 QoS。
- 数值库：五个直接消费包的 `.o.d` 未发现系统 Eigen 头文件，实际来自项目 vendor；运行中 `libgtsam.so.4.2.0` 和 METIS 均来自 `ws_robot/deps/gtsam`。Eigen 的 540 个源码文件 SHA-256 校验通过。
- 构建：本轮相关 10 个包编译成功。存在旧算法 signedness/未使用返回值警告，以及系统 OpenCV/PCL 的 TBB 依赖版本警告；未宣称构建零警告。

## 采样和时效异常

run16 第一轮第三段有 1 次无效样本，最长记录间隔约 109 ms；该时间附近出现自滤所需连杆 TF 陈旧。整段完成后还出现过一次约 0.35 s 扫描观测超时告警。源数据不足时自滤丢帧，未放宽 0.25 s TF 或 0.30 s 观测时效约束。

随后不并行运行额外探针，复跑三段无采样缺口、无路线告警。两轮起点存在到位容差内差异，不能据此给出统计性能优劣结论，也不能认定告警根因已修复。控制循环超期仍有日志记录，后续长时实时性验收应保留这一项。

## 重现与文件

启动、存图、载图命令见[统一参考](SLAM_INTEGRATION_REFERENCE_20260918.md)。本轮原始日志：

- `/tmp/astribot_slam_unified/run15/session.log`
- `/tmp/astribot_slam_unified/run16/session.log`

持久归档：[runs/slam_unification_20260918](../runs/slam_unification_20260918/)。其中 `sessions/sim_unified_15` 是完整可加载地图；`final_binary_sha256.json` 标识实际测试二进制，`eigen_consumers.json` 记录头文件来源，`summary.json` 汇总各轮路线结果。归档不包含临时测试脚本。

两次仿真主管均已正常停止，run16 的 `remaining_owned_pids` 为空；暂停后采集到的 29 帧里程计速度均为零。退出进程核对无本轮 ROS 栈残留。

## 后续验收范围

退化后的坐标连续性/恢复事务、长时跑机、完整历史路径和窄通道，以及真机传感器标定与独立真值验收仍未完成。按[架构评审](SLAM_ARCHITECTURE_REVIEW_20260918.md)分阶段推进；正常链路通过不能替代这些项目。
