# Navigation policy

独立领域模块通过 ROS 适配器接入现有导航。默认启动参数 `navigation_policy_stage=off` 保留原命令链路；阶段进度及仿真证据见 [实施记录](../../../docs/NAVIGATION_POLICY_IMPLEMENTATION.md)。P0–P2 基线仿真检查已通过，停车修订仍有航向质量对照待完成；P3 功能闭环已验证、质量待办保留；用户已授权继续 P4，P4 已通过指定配置仿真，P5 自动候选与包络准入专项通过，完整阶段仍有待办。P2 验证入口为 `--navigation-policy p2 --max-linear-speed .32`，原默认基线保留。

- `contracts.py` / `ports.py`：SI 单位、时间与 epoch、协方差、来源和健康信息，分离执行与规划意图。
- `profile.py` / `config/simulation.json`：参数来源、仿真假设及真机证据字段。仿真参数只允许配合 `use_sim_time=true`。
- `fusion.py` / `risk.py`：观测关联、相关来源去重、保守预测、未关联观测保留、路径与实际运动扫掠风险。
- `observer_node.py`：扫描、地图、里程计与视觉适配。扫描等待采集时刻对应的 TF，不使用最新 TF 冒充采集时刻变换。
- `behavior.py` / `policy_node.py`：减速、让行、确认恢复和阻塞 episode；P2 不发动态绕行请求。
- `path_evidence.py`：当前路径身份、碰撞段距离和证据有效期；远处占据允许谨慎接近，未知或过期占据不能获得慢行权限。
- `protection.py` / `protection_node.py`：独立扫描扫掠、输入及墙钟看门狗、约束汇总与末级速度输出。正常许可稳定后原样传递未受约束的指令；安全停车优先，恢复限制加速度。
- `planning_session.py`：P3 请求所有权、版本与预算管理，由 `route_coordinator.py` 接入 P3 异步联合决策。几何候选服务返回 `geometry_valid`，不能直接作为执行许可。

P3 仿真入口：`--navigation-policy p3 --max-linear-speed .32`。`candidate_safety.py` 复核停止与不同可达速度下的预测占据；`ResolveRoute` 把经过验证的候选交回 BT 提交，不能直接发布底盘指令。单目标执行的局部/全局候选共用有界请求，完整阶段回归通过前不开放 P4/P5。

P2 的命令链：现有跟踪器 → 现有速度平滑与姿态/双臂约束 → `/cmd_vel_policy_input` → `final_protection` → `/cmd_vel`。最终约束使用 `astribot_navigation_msgs/MotionConstraint`；ArrivalController 与 PoseProgressChecker 的适配层排除显式让行时间，保留原控制和检查逻辑。

## 视觉接口

话题 `/navigation_policy/vision_observations`，`std_msgs/String` JSON：

```json
{
  "schema_version": 1,
  "sensor_id": "camera_front",
  "stamp_ns": 1000000000,
  "frame_id": "camera_optical_frame",
  "calibration_epoch": 1,
  "observations": [{
    "kind": "metric_box",
    "measurement_id": "frame-1-object-1",
    "center_m": [0.0, 0.0, 2.0],
    "size_m": [0.4, 0.6, 1.2],
    "position_variance_m2": 0.01,
    "geometry_quality": 0.9,
    "classes": {"person": 0.9},
    "provenance": ["camera_front:frame-1"]
  }],
  "resolved_measurement_ids": []
}
```

`stamp_ns` 是与 ROS 仿真时钟一致的采集时间，不能填写推理完成时间。米制框需要有效深度、尺寸及各向同性位置方差；类别置信度不能替代位置方差。`image_box` 使用 `image_size_px` 和 `box_xyxy_px`；`bearing_cone` 使用单位 `direction` 和 `half_angle_rad`。无深度输入保持未知风险，不伪造距离。

未关联视觉观测不会因过期自动清空。同一来源在确认原检测区域无障碍后，可使用较新的帧携带 `resolved_measurement_ids`；不能把“检测器这次没输出目标”直接解释为自由空间。硬件相机适配器仍需接入标定读取、覆盖健康和可追溯的清空证据；本阶段验证输入为模拟视觉消息。

构建需包含 `astribot_navigation_msgs`。试验脚本、故障注入与结果均放外部验证目录，不安装进产品包。

扫描簇没有稳定的物体 ID，仅提供占据。速度预测使用带 `track_id` 的跟踪观测，或米制框的可选 `velocity_m_s`、`velocity_variance_m2_s2`（两项必须同时提供，速度在消息 `frame_id` 中，以采集时刻 TF 旋转到跟踪坐标系）。扫描簇分裂、可见面变化不能直接解释为物体运动。

启用策略时，Nav2 接收 `/navigation_policy/costmap_scan`：适配器将无回波转换为量程内的清除端点，端点必须超过 Nav2 障碍标记距离，采集时间戳不变。策略观测和末级防护仍使用原始扫描。

`/navigation_policy/path_risk` 提供全足迹检查结果及当前路径绑定的距离证据，有效期由 `path_risk_timeout_s` 配置。允许慢行前扣除时间延迟对应的运动预算和安全余量，同时保留局部预测与末级防护。`/navigation_policy/path_blocked` 作为没有有效匹配记录时的保守后备。等待预算仅计让行停车，不计慢行接近。策略状态包含 `path_risk_status` 和扣除预算后的 `path_distance_m`，便于排查提前停车。


## 人工窄通道（P4）

`--navigation-policy p4 --corridor-file <JSON>` 显式启用。JSON 示例：

```json
{"schema_version":1,"environment":"simulation","frame_id":"map","corridors":[{"corridor_id":"door_a","entry":[0,-2],"exit":[0,-4],"width_m":1.3,"postures":["simulation_transport"],"boundary_margin_m":0.025,"tracking_margin_m":0.05,"bidirectional":true}]}
```

坐标、宽度和边界必须与实际场地一致，此例仅说明格式。范围外的普通路径沿用现有策略。入口停稳、必要时对齐、持续确认后签发绑定世界版本的本地许可；通道内保留独立防护并限制换路，全身通过出口后释放。局部许可不代表其他机器人或行人已经让路。

`CorridorAlignment` 是现有跟踪任务的入口修正请求，不发布速度。准备距离使用完整旋转半径加制动距离，停得较近时仍需车体在入口外且完整旋转扫掠可行。入口横向偏差先由 `CENTER` 修正，再停稳、对齐航向和确认准入；居中扫掠覆盖当前位姿到目标中心线的全包络及预测障碍。

控制器要求短时租约、完整 active path、坐标系和位置锚点匹配。`MotionConstraint.alignment_required` 与 `centering_required` 互斥，缺失或无效请求保持停车；居中目标还受 30 cm 距离、锚点线段偏离和朝向漂移检查约束。最终防护仍为唯一 `/cmd_vel` 输出方，对齐禁止平移，居中禁止旋转。普通 P2/P3 的两个字段均为 false。

仿真入口居中参数：`narrow_centering_speed_m_s=0.05`、`narrow_centering_tolerance_m=0.01`、`narrow_centering_max_offset_m=0.3`。更窄通道还将释放偏差收紧到可用侧向余量的一半。超过修正范围、已进入通道或扫掠不可行时保留有界失败；此接口不执行未经验证的后退。

航向阈值用于入口准入；获得许可后的正常跟踪继续由原控制器收敛，持续检查实际朝向下的车体投影和独立扫掠风险。越过出口中心平面后，只有完整旋转扫掠可行才允许按开口几何处理转弯；仍须全身和余量通过出口、持续确认后才能释放通道限制。

车体仍在入口外且预计侧向包络超界时，可以撤销许可并重新居中；恢复须通过全包络扫掠，不能刷新当前通道的入口超时预算。下一个通道使用独立预算。

`corridor_tracking_required` 是第三种互斥模式，在入口准备完成、许可有效后启用，覆盖入口直线衔接和通道内部。匹配的 `CorridorAlignment.tracking_required` 请求提供直通道航向。控制器保留内层输出的平移速度，复用已配置的到点航向比例增益修正角速度，限制为不超过 0.2 rad/s 并复核修改后指令的 costmap 扫掠。准备前的曲线接近、越过出口后的转弯以及普通路径仍使用原控制逻辑；最终到点继续由原精确到位流程处理。

`narrow_speed_m_s`、`narrow_angular_speed_rad_s`、`narrow_heading_limit_rad` 来自策略配置，当前仿真值分别为 0.15 m/s、0.20 rad/s、0.05 rad；30 s 失败预算复用规划 episode 配置。未验证的后退和通道内转身不启用。


P5 仿真入口为 `--navigation-policy p5`，可不提供人工通道文件。自动识别只提出直通道候选，仍需通过 P4 的实时准入。`automatic_corridor_max_width_m` / `automatic_corridor_min_length_m` 默认 1.8 / 0.8 m；未知路径或单侧边界不确定时不签发候选通行许可。多模态障碍仍由统一融合接口提供，包络变更复用 `SetRobotEnvelope`。硬件、任意机械臂姿态、复杂三维载荷和曲折窄通道不在此入口的已验证范围。
