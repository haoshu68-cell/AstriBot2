# Social navigation: observation boundary

本包接入人员观测，不输出速度、目标、路径或运动约束。导航侧显式选择 `social_navigation_stage:=h2` 后，由既有 PolicyNode 组合让行、等待和速度候选评分。普通入口默认关闭；跟随、队列、超越仍待后续阶段验收。

视觉/融合跟踪器发布 `astribot_navigation_msgs/SocialAgentArray` 到 `/perception/social_agents`。数组使用同一采集时刻与坐标系，速度是该坐标系表示的目标绝对速度（不是相对相机的光流）。姿态/速度协方差按 ROS 六维顺序填写，未知速度设 `velocity_valid=false`，禁止用全零速度冒充静止。人员 ID 在同一 source_epoch 内不复用；源重启递增 epoch。

`social_observer` 按采集时刻查询 TF，转换位置、方向、速度及协方差。重复帧不续期；0.3 s 超时、无效输入、TF 缺失及时间回跳输出 `valid=false`。无效空数组不能当作无人环境。`/social_navigation/observation_status` 以 20 Hz 发布来源、时效和原因；H2 使用校验后的 `/social_navigation/observed_agents`。

HuNav 接口是独立的 `/simulation/social_agents_truth`，保留真值标识，不输出人的目标和行为树意图。必须显式设置 `source:=hunav_truth use_sim_time:=true` 才可观察。真机不得使用该来源。视觉接口是预留的类型契约，并不代表已实现视觉检测/身份关联。

依赖通过仓库 `tools/setup_hunav_sim.sh` 在私有目录构建。HuNav 与 BT.CPP 4 不链接现有 Nav2 BT.CPP 3；正常导航无需安装这些依赖。

仿真场景支持 `wait_for_episode_start: true`。`/social_sim/episode` 的 `start`/`pause` 只控制行人，`/social_sim/episode_state` 返回状态。每个 Regular 行人的行为树在本次日志目录生成，不依赖场景名称碰巧对应已安装 XML。

空场景有两种用途：`h2_empty.yaml` 仅保留物理几何观察，供相机等轻量验证使用；导航回归使用 `h2_empty_observed.yaml`，通过 `enable_empty_observations: true` 启动实际 HuNav 空清单观测、启停确认和断流注入接口。后者没有行人模型，仍要求观测按仿真时间持续更新；缺失输入不能作为无人证据。

`hunav_truth_adapter` 的 `observation_mode` 为仿真故障注入接口：`normal` 正常发布、`drop` 不发布、`repeat` 重发旧帧但不更新时间戳/序号。仅该仿真适配器具有此参数；用例结束必须恢复 `normal`。场景与运行方法见 `tools/social_navigation/README.md`。
