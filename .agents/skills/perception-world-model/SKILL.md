---
name: perception-world-model
description: 在本仓库新增或修改激光/点云/相机/标记/定位/SLAM 融合，或把感知结果交给 Nav2、MoveIt 和双臂任务时使用。建立带来源、时间、坐标、标定、质量和版本的世界快照，避免把 TF 可查或单帧检测当成可执行事实。
---

# 感知与整机世界快照

先按 [源码入口索引](../astribot-architecture-design/references/project-map.md) 定位本次数据源与消费者。按实际问题补充：SLAM 拆分查 [历史评审](../../../docs/SLAM_ARCHITECTURE_REVIEW_20260918.md)，导航投影查 [动态避障设计](../../../docs/DYNAMIC_AVOIDANCE_AND_NARROW_PASSAGE_DESIGN.md)，真实标定查 [硬件手册](../../../docs/manuals/HARDWARE_OPERATIONS.md)，改变整机控制权才读 `mobile-dual-arm-architecture`。
世界快照字段在本技能维护；通用模块/通信契约由 `robot-dataflow-module-design` 负责。

## 数据流与责任

```text
传感器/模型输出
  -> ObservationAdapter（设备、frame、时间、QoS、标定、来源）
  -> FusionEngine / 质量门控
  -> WorldSnapshot（对象、障碍、机器人状态、版本和健康）
  -> Nav2 二维投影 + MoveIt 三维 PlanningScene + 任务账本
```

- 算法核心只消费统一观测/快照接口，不直接订阅某个厂家视觉模型。
- `astribot_s1_slam`、`astribot_s1_mapping`、`astribot_s1_perception` 和 `perception_components` 按变化原因分层；不要为了减少包数合并估计、栅格和设备启动。
- 导航投影和操作场景可共享对象 ID、时间、坐标、标定和版本，但不共用一个无界全局场景锁。
- 传感器的“最后一帧”必须带采集时间、接收时间、frame、源 epoch、标定版本、质量和丢帧/时效状态。

## 世界快照最小契约

对每一条可影响运动的事实至少保留：

```text
source_id / source_epoch
sample_stamp / receive_stamp / clock_epoch
frame_id / transform_chain / calibration_revision
object_id（若有）/ geometry / covariance_or_quality
valid_until / drop_count / health_reason
map_revision / localization_revision / scene_revision
```

- 目标、障碍、载荷和附着分别建模；不要用“检测到了”覆盖 `ObjectHypothesis`、`TrackedObject`、`AttachmentState` 的不同语义。
- TF 查询成功只证明变换链可计算，不证明外参、定位质量、时间新鲜度或对象身份可信。
- 规划快照失效原因包括 frame/时间/标定/地图/定位/包络/场景版本不一致、时钟回退、传感器断流和质量不足。
- 传给导航的二维足迹必须是保守投影；传给 MoveIt 的三维几何必须保留碰撞和附着关系。不能由二维地图反推可抓取或可放置。

## QoS 与内部实现

- 传感器流优先使用与发布端匹配的 sensor-data QoS；状态、版本、确认和故障消息使用可靠且有界的 QoS。先检查兼容性，不靠默认 depth=10 猜测。
- 高频 callback 只复制/排队有界数据并更新诊断计数；融合、点云切片和几何计算放到可测的 C++ worker 或组件中。
- 实现和脚本的语言选择统一遵循 [robot-runtime-cpp](../robot-runtime-cpp/SKILL.md)，不另设感知薄适配例外。
- 记录器不能反向控制机器人；“观测过期”应产生结构化状态，具体停车/降级由任务或保护层决定。

## 验收边界

单元测试或离线回放只能证明公式、版本门控和消息转换；它们不能证明相机标定、实时 SLAM、真实遮挡、Gazebo 传输或真机定位精度。每个数据源说明其已实现、已验证和仍待真机验收的范围。

## 外部参考

- ROS 2 QoS 配置与兼容性：[Quality of Service](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- Nav2 感知到规划/控制的模块化链路：[Nav2 architecture](https://docs.nav2.org/)
- MoveIt PlanningScene 作为操作场景边界：[Planning Scene](https://moveit.picknik.ai/humble/doc/examples/planning_scene/planning_scene_tutorial.html)
