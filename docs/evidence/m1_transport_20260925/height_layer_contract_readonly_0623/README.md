# 高度分层契约与后续独立输出的最小接口边界

本记录仅核对已有源码与 scene39 离线记录。没有修改运行时代码、配置、门槛或 TF，没有构建或运行 ROS。当前顺序是先地图切片，再分层包络，最后规划避障耦合；Hold 回执已封存，运行验收未完成。

机器人 `RobotModel::geometry()` 当前以 `astribot_torso_base` 为坐标基准，从模型最小 z 向下取整、最大 z 向上取整，每 0.25 m 输出 `geometry.slices`。每个形体已包含模型 padding、关节保持误差及实际载荷；形体高度区间与切片相交时取整块形体的保守二维投影。现有 `geometry.slices`、总 footprint 和高度完整性校验继续保留。

地图旧四段是 base 坐标的 `[-0.03,0.17)`、`[0.17,0.6)`、`[0.6,1.1)`、`[1.1,1.55)`。用户地面边界恰好等于这些旧值加 0.08 m；policy 配置的来源说明仍保留该旧偏移及已迁移的包路径。当前实际感知配置位于 `astribot_s1_perception_components`。fixed_v2 启动另外把最后上界改到 base z=2.2 m，并保留稀疏障碍点。

scene39 离线同戳 base/world z 为 0.1292237616 m；已核对实际安装地面碰撞网格的厘米单位及单位矩阵，其顶面世界 z 为 0.03422339～0.03422592 m。相减得该平地近水平采样的底盘原点离地约 0.095 m。world z 不能直接当离地高度，旧 0.08 m 偏移也不能继续当真实基准。当前 URDF 根是 torso_base，在所查机器人 URDF/导航/感知配置中未发现独立 ground/base_footprint TF；`aft_mapped` 的现有契约是同底盘原点与轴向，并非地面帧。此结论不代表硬件或坡地标定。

| 用户地面分层边界（m） | 当前平地模型名义 base z（m，减 0.095） |
| --- | --- |
| 0.05 | -0.045 |
| 0.25 | 0.155 |
| 0.68 | 0.585 |
| 1.18 | 1.085 |
| 1.63 | 1.535 |

地图阶段完成后，独立 EnvelopeSlice 输出的最小复用点如下，仅为接口建议：

1. 复用 `robot_model.hpp:235–244` 的局部 `bodies`。它保存同一次 FK、误差膨胀后的每个形体 `z_min/z_max/footprint`，已经包含 attachments。无需重算 FK，也不从既有 0.25 m 粗切片反推细分层。
2. 在现有 `geometry()` 参数末尾增加默认空的显式区间列表，例如 `std::vector<std::array<double,2>>`；在 `RobotGeometry` 中增加独立结果字段，例如 `layered_slices`。现有调用默认仍走原输出，原 `slices` 计算块一行不改。非空列表只利用同一 `bodies`，按每段相交关系和 `convexHull` 填充新字段，不新增通用分层框架。
3. 新列表是同地图契约转换后的四段加 overflow。overflow 从地面 1.63 m 开始，延伸到既有有效投影覆盖顶；当前 fixed_v2 名义对应 base `[1.535,2.2]`。不能把 1.63 m 当机器人或载荷高度硬截断，也不能把 overflow 当无限观测。原 `height <= coverage_max` 门槛保持；超出现有观测覆盖仍拒绝完整性。
4. 在 `geometry_state_node.cpp:180–185` 同一次 calculate 内取新字段，复用 `EnvelopeSlice` 转换。独立 topic 可复用 `RobotGeometryState` 作为外层载体，沿用同一次采样的 header、sequence、model/attachment/source 身份和有效期，仅其独立 height_slices 使用新列表。原 `/navigation/geometry_state` 内容不变；新输出也必须经过原 `validateGeometryCompletion` 成功后才能声明 complete。完整配置身份、空层和 overflow 的消费契约应在地图输出确定后绑定，不能以裸多边形或一串无来源的切片代替。
5. `SliceProjector` 已有 `per_slice_ranges`，当前节点发布的是各层最小距离合并后的 scan；先保留地图层信息。`NavigationEnvelopeV2` 和现有 fixed core 目前只处理合并二维 footprint，没有消费 `height_slices`，所以独立包络输出仍不等于规划避障已耦合。后续接线须另行绑定分层数据与原版本/ACK，不在本阶段改动。

点云切片使用半开区间；机器人包络使用闭区间相交，以免边界形体漏包。名义 0.095 m 换算只适用于该平地基准；若地图阶段采用重力水平地面坐标，点云与机器人形体必须使用同一变换，不能只改 frame 名或单侧加减常数。

原始证据路径、哈希、关键源码行和计算输入见 `evidence.json`、`source_excerpts/`、`ground_model/`。Hold 回执暂停记录见相邻 `hold_observed_handoff_0610/paused_checkpoint.json`。
