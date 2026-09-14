# 真机参数逐项验证操作手册

本手册配套 `tools/robot/validate_hardware_params.py`。现有硬件模板全部仍为待测参数；本次只添加验证工具与说明，不修改控制器、启动链或默认控制参数。脚本只有订阅和文件写入，不发布速度、不获取控制权、不发送导航目标、不停止任何机器人进程。

## 1. 开始前准备

在**真机终端**进入 SDK 仓库根目录，加载真机已有环境。不要在仿真电脑上录一份数据就当成真机结果；两套启动脚本可能均使用 Domain 25。

```bash
source tools/robot/env_robot.sh
python3 tools/robot/validate_hardware_params.py init "$HOME/hardware_validation_case01"
```

`init` 拒绝覆盖已有目录。每个机器人版本、运输姿态、载荷、地面工况单独建目录，先填写：

- `conditions.json`：序列号、操作者、软件版本、双臂关节姿态、载荷/形状、地面/坡度、电量、定位来源、指令坐标系、时钟同步证据、外部测量设备及精度。
- `topics.json`：按现场实际接线修改话题名与 ROS 类型，`required` 表示此次采集必需。默认 `/odom`、`/scan`、`/tf` 为必需；视觉/策略节点尚未启用时可以没有数据，不能因此宣称这些功能验证通过。
- `measurements.json`：10 项逐项填写 `measured_values`、原始证据路径、人工判定、复核人。`parameter_review` 已逐字段列出模板值和待填建议值，包括规划预算。
- `hardware.candidate.json`：待审核副本，不被当前运行栈加载。暂时保持 `hardware_validated=false`，不要直接替换正式配置。

查询前按仓库 `ros2-stack-ops` 流程，从真机实际运行进程 `/proc/<PID>/environ` 对齐 DDS 环境；不要只凭自己的 shell 环境猜 Domain。采集结果保存实际查询环境及接收帧数。发现阶段可能需要 30–50 秒，首次建议录 90 秒。

静态包络/遮挡先测；进入运动项目时使用现场已确认的低速操作入口和独立急停，留足停车空间，由现场人员操作。起始速度须由现场确认，不能把下表模板上限作为首次试验速度。逐档扩大到拟支持的速度范围，每个方向/工况至少重复 5 次，尾部风险项目增加重复数；这只是最小工程采样建议，不代表统计安全认证。

## 2. 参数清单及如何得到建议值

下列数值是**当前模板值，均非真机已验证值**。几何以米、速度以 m/s、角度以 rad、时间以秒计。拟支持的所有载荷/姿态均需覆盖。

| 组 / 字段（模板值） | 真机验证动作和记录 | 取值及判定方式 |
|---|---|---|
| 01 `half_length_m=.31`, `half_width_m=.31`, `height_m=1.63` | 测 base 原点到前后左右及最高点，包含双臂、夹爪、线缆、余转；拍照、关节角、量具误差 | 对称矩形取前后距离最大值、左右距离最大值；非对称外伸不能用总长的一半代替。记录高度起算面与 base 的偏移。与实际碰撞包络一致 |
| 01 `payload_mass_kg=0`, `payload_extra_margin_m=0`, `clearance_margin_m=.08` | 称重并测抓持外伸、摆动、定位误差和跟踪误差；空载/典型载荷/最大拟支持载荷 | 额外包络按最坏姿态；净空覆盖测量、定位和跟踪误差。载荷质量元数据不能代替动力学试验 |
| 02 `base_frame`, `tracking_frame` | 静止 90 秒；人工前后、左右、正负旋转，核对 TF、速度单位、轴向、符号；量尺校验已知位移/角度 | 不跳变、不重复陈旧源时间，命令与反馈方向一致。真机桥接可能有 body→world 转换，必须沿最终驱动接线确认，不能把仿真 body Twist 原样假定为真机最终坐标 |
| 03 `max_speed_m_s=.35`, `max_angular_speed_rad_s=1.5` | 每档稳定运行再停车，记录实际线/角速度及误差；前后左右斜向、双向旋转 | 仅支持停车、感知和跟踪均验证过的范围；模板 .35 和仿真已验 .32 都不构成真机放行 |
| 03 `max_acceleration_m_s2=.5`, `max_angular_acceleration_rad_s2=3.2` | 分轴加减速、转弯出弯、接近段，采原始命令/反馈，查滑移、振荡、载荷稳定和电流限制 | 用可靠源时间计算各轴加速度/jerk；阈值取可稳定重复的范围，不把瞬时噪声导数当极限 |
| 03 `brake_deceleration_m_s2=.5`, `angular_brake_deceleration_rad_s2=3.2` | 各初速度下独立测命令停止事件、响应起点、实际停车点及余转；用视频/外部位姿复核 | 区分响应延迟和减速阶段；取最差工况的保守减速下界，保留测量误差。脚本等效值含延迟，不能直接当物理减速度 |
| 04 `command_timeout_s=.5`, `input_command_timeout_s=.3`, `constraint_lease_s=.3` | 分别由人工中断上游指令、策略约束、末级保护节点、驱动输入，记录最后有效消息到实际停止 | 分开确认软件失效处理和驱动独立 watchdog。不能把“持续发送零速度”的结果当断流测试；协议约束租期上限 .5 秒 |
| 05 `reaction_time_s=1`, `sensor_timeout_s=.3` | 同步采集端/计算端/驱动时钟；测 capture→推理→TF→融合→策略→实际动作；正常/高负载/网络抖动 | 保留 P95/P99/最大值、时钟误差及丢帧；反应预算覆盖真实最坏链路和失效检测。源时间至订阅接收的 age 只是一段延迟 |
| 06 `scan_min_valid_fraction=.9` | 已知标靶在前后左右、近场、低矮、悬空、反光/透明、臂后分别摆放，记录每处检出和最小净空 | +inf 无回波可计有效射线，但不证明没有障碍；不能只看 .9 的比例，逐格建立覆盖/盲区图；验证自滤不会删真实物体 |
| 06 视觉内参/外参/深度单位/标定版本（接口参数） | 标定板、多距离标靶、采集时 TF；加相机信息与视觉输出采集项；有深度、无深度、遮挡分别试 | 米制观测与像素/方位观测分开；保留 sensor_id、calibration_epoch、capture stamp、协方差、模型版本，不用推理完成时间冒充采集时间 |
| 07 `association_distance_m=.6`, `max_obstacle_speed_m_s=2`, `min_tracked_speed_m_s=.1` | 静止目标、已知速度横穿、迎面、交叉/遮挡再出现，以视频/标靶提供真值 | 统计身份切换、误关联、静止误判运动、速度误差及漏检；按实际帧间位移与噪声调关联距离 |
| 07 `velocity_confirmation_s=.4`, `velocity_fit_window_s=.6`, `velocity_fit_max_residual_m=.06`, `stationary_velocity_variance_m2_s2=.0004` | 比较不同速度/距离/遮挡下拟合残差、协方差和确认耗时 | 平衡抗噪声与确认延迟；窗口增大后必须重新检查反应/停车预算，不能只看曲线变平滑 |
| 07 `track_memory_s=1`, `prediction_horizon_s=2.8`, `prediction_step_s=.1` | 遮挡/重现及不同相对速度碰撞时距回放 | 遮挡保留不把旧轨迹当永远有效，预测覆盖反应+停车；步长需证明不会跨过小障碍物，验证整个扫掠包络 |
| 09 `clear_hold_s=.6`, `blocked_confirm_s=1.5`, `wait_budget_s=8` | 短遮挡、间歇通行、持续堵路分别测暂停/恢复、等待/绕行决策 | 不抖动、不因短暂清空抢行，等待结束必须有明确后续或失败；时间不作为跟踪性能排名 |
| 09 `planning_budget.request_timeout_s=2`, `episode_timeout_s=30`, `max_requests_per_goal=5` | 高负载规划超时、持续无解、新目标替换、旧响应迟到；保存请求身份/版本/提交事件 | 有限次数/时限、旧结果不提交；新目标或现路径风险才触发全局规划，无定时重规划。仅在对应阶段完整接线后做闭环 |
| 10 `narrow_speed_m_s=.15`, `narrow_heading_limit_rad=.05`（约 2.86°） | 测通道宽度、入口偏航、侧向误差、载荷包络；渐进缩小通道，检查入口会车及出口堵塞 | 通道阈值不是终点 1.5°。所需半宽至少含 `half_width*cos(theta)+half_length*abs(sin(theta))` 加净空/额外包络；过小就拒绝通行 |
| 08 到点 `.03 m / 1.5°`、过程性能（验收项） | 同路线、同起姿/载荷做基线和改动后配对重复；直线、转弯、出弯、接近段分别统计 | 到点欧式误差与角度误差均含测量不确定度达标；横偏/航向 P95 和最大值、速度波动、分轴加减速/jerk、误停/异常旋转不能劣化，耗时仅记录 |

停车预算可用 `v*reaction_time + v²/(2*brake_deceleration) + clearance_margin` 做初步核对，须再覆盖载荷、定位/估计误差和二维扫掠包络；它不是动态障碍的完整安全判据。角运动同样检查余转对全身外廓的影响。

## 3. 逐项采集

```bash
CASE="$HOME/hardware_validation_case01"
python3 tools/robot/validate_hardware_params.py record "$CASE" --item 02_frames --environment hardware --duration 90
```

将 `--item` 换成 `01_geometry`、`03_braking`、`04_watchdog`、`05_latency`、`06_coverage`、`07_tracking`、`08_navigation`、`09_avoidance`、`10_narrow` 即可逐项录制。每次自动建时间目录，重复同一项不会覆盖前一次。运行时显示目录，用另一个终端打操作标记：

```bash
RUN="$CASE/03_braking/实际输出的时间目录"
python3 tools/robot/validate_hardware_params.py mark "$RUN" brake_request --notes '前进/本档速度/空载/零命令停车'
```

按 Ctrl-C 只结束采集，不会替你停车。正常或中断结束均输出：

- `metadata.json`：工况、环境标签、DDS、脚本/参数哈希。标签由操作者指定，不能自动证明连接的是硬件。
- `messages.jsonl`：原始消息、源时间、接收墙钟时间和单调相对时间 `t_s`；NaN/Inf 编码成字符串，避免非法 JSON。
- `capture_summary.json`：实际帧数、发现等待、接收频率、最大间隔、最后数据年龄、非递增时间计数。缺必需数据返回 1；正常结束返回 0 只表示采集完成。
- `mark_*.json`：人工操作墙钟标记，有人的操作延迟，只供对齐线索。

订阅采用 BEST_EFFORT，`/tf_static` 采用 TRANSIENT_LOCAL；不依赖 ROS CLI 守护进程。采集不保证无丢帧，不替代高带宽原始传感器 bag。视觉可在 `topics.json` 增加实际 `/camera/.../camera_info`（`sensor_msgs/msg/CameraInfo`）和 `PoseStamped` 外部真值话题；图像、点云建议用现有 rosbag 工具单独录制并在 evidence 中关联，避免 JSON 采集大数据干扰被测时延。策略 JSON 内部 capture 时间需离线与同源消息关联，摘要不会把无 header 的 JSON 当端到端延迟。

## 4. 自动提取与制动分析

```bash
python3 tools/robot/validate_hardware_params.py inspect "$RUN" --output "$RUN/inspection.json"
```

提取速度范围、源时间导数、扫描比例以及各命令话题“非零→零”的接收时刻。先查看这些事件，选中**实际驱动输入链路上本次停车对应**的 `t_s`；以下 `12.34` 只是命令格式示例，要换成此次事件值：

```bash
python3 tools/robot/validate_hardware_params.py brake "$RUN" \
  --event-s 12.34 --event-source '本次最终指令非零转零的接收时间' \
  --output "$RUN/braking.json"
```

默认从事件后 10 秒内寻找线速度 ≤.01 m/s、角速度 ≤.02 rad/s 连续 1 秒的停止区间；阈值应结合**静止噪声及外部实际静止观测**调整，不能为得到通过结果随意放宽。输出路径累计长度、绝对余转、停止阈值到达时间、持续稳定确认时间。累计到确认结束，包含静止抖动；不是端点直线距离。

对断流试验没有零命令事件：从原始消息定位最后一次驱动输入的 `t_s`，注明 `event-source`；人工标记只是近似，不适合精确 watchdog 时延认证。事件前必须处于运动状态；坐标系变化、重复/倒退源时间、超过 .2 秒断档和未稳定停车均报失败。若定位有重定位跳变而帧名不变，仍需人工/外部基准识别，不能把定位跳变当实际运动。

`inspect` 的导数为**速度模长**变化，完整分轴加速度、jerk、路径横偏需从原始数据按源时间分析。比较路径时将采集时机器人位姿转换到对应路径 frame，按目标/路径版本和阶段切段，在交叉路径处沿连续进度匹配，不能把 map 路径直接与 odom 坐标相减。当前工具不自动出上述全路线质量通过结论。

## 5. 独立验证到点精度

当前真机 `tools/hardware_nodes/tf_to_odom_node.py` 从定位 TF 反推 `/odom`，速度也是位姿差分。用同一定位源同时当控制输入和真值，会掩盖误差；应使用测量定位标记、全站仪/动捕或已标定独立视觉，在相同坐标系中测目标与最终 base 位姿。

停止后持续观察稳定区间，并包含最差漂移样本。填写初始化生成的 `arrival.csv`，每行一个独立测量样本，角度用度：

```text
trial,reference,goal_x_m,goal_y_m,goal_yaw_deg,actual_x_m,actual_y_m,actual_yaw_deg,position_uncertainty_m,yaw_uncertainty_deg
```

`reference` 写设备/标定或测量文件路径；位置/角度不确定度填目标与实际测量合并后的保守误差界，不是标准差直接代入。终点采用欧式距离，航向做 ±180° 环绕。

```bash
python3 tools/robot/validate_hardware_params.py arrival "$CASE/arrival.csv" --output "$CASE/arrival_result.json"
```

程序逐样本要求 `距离+不确定度≤.03 m` 且 `角差+不确定度≤1.5°`。样本全部达标仅证明填写的样本通过，不代表全部真机功能放行。

## 6. 如何回填与进入后续验证

1. 完成每项后在 `measurements.json` 填测量值、工况、证据、复核人及 `status`（pending / measured / passed / failed）。缺数据保持 pending；失败保留原始记录。
2. 各字段在 `parameter_review` 给建议值和依据，然后人工回填 `hardware.candidate.json`；代码不会凭单次试验自动调参或将 `hardware_validated` 改为 true。
3. 将 01→`transport_envelope/payload`、03+04→`braking`、05→`latency`、06+07→`sensor_coverage` 证据关联到配置，并检查所有耦合预算。证据字段非空只满足配置结构要求，不能代替工程审核。
4. 先离线回放，再受控低速闭环，逐档扩大工况。当前 P0–P2 已有仿真证据；P3 及后续完整局部绕行/路径提交/窄通道接线仍需按阶段完成验证，不能把尚未实现的动作视为真机已支持。
5. 回传整个 CASE 目录及关联的原始 bag/标定/照片，即可按同工况比较并形成正式参数建议。所有脚本输出拒绝覆盖已有同名结果，重新分析需换输出文件名。
