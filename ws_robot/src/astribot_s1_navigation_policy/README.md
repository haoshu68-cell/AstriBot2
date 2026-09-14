# Navigation policy

独立领域模块通过 ROS 适配器接入现有导航。默认启动参数 `navigation_policy_stage=off` 保留原命令链路；阶段进度及仿真证据见 [实施记录](../../../docs/NAVIGATION_POLICY_IMPLEMENTATION.md)。P0–P2 仿真检查已通过，P3 正在实施，P4/P5 尚未放行。P2 验证入口为 `--navigation-policy p2 --max-linear-speed .32`，原默认基线保留。

- `contracts.py` / `ports.py`：SI 单位、时间与 epoch、协方差、来源和健康信息，分离执行与规划意图。
- `profile.py` / `config/simulation.json`：参数来源、仿真假设及真机证据字段。仿真参数只允许配合 `use_sim_time=true`。
- `fusion.py` / `risk.py`：观测关联、相关来源去重、保守预测、未关联观测保留、路径与实际运动扫掠风险。
- `observer_node.py`：扫描、地图、里程计与视觉适配。扫描等待采集时刻对应的 TF，不使用最新 TF 冒充采集时刻变换。
- `behavior.py` / `policy_node.py`：减速、让行、确认恢复和阻塞 episode；P2 不发动态绕行请求。
- `protection.py` / `protection_node.py`：独立扫描扫掠、输入及墙钟看门狗、约束汇总与末级速度输出。正常许可稳定后原样传递未受约束的指令；安全停车优先，恢复限制加速度。
- `planning_session.py`：P3 请求所有权、版本与预算管理，已做隔离验证，尚未接入运动决策。几何候选服务返回 `geometry_valid`，不能直接作为执行许可。

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

启用策略时，Nav2 接收 `/navigation_policy/costmap_scan`：适配器将无回波转换为量程内的清除端点，端点必须超过 Nav2 障碍标记距离，采集时间戳不变。策略观测和末级防护仍使用原始扫描。`/navigation_policy/path_blocked` 是行为树提供的路径占据证据，统一策略处理让行；代价地图重新确认路径有效后解除。
