# astribot_operator_station (P0/P1 implementation)

默认现已使用 **WorkstationPanel + operator_backend 租约网关 + C++ 只读 3D 回放**。
新增 P2 地图与工位页，见 [P2 事务实现](../../../docs/P2_MAP_STATION_TRANSACTIONS_20260919.md)。
当前接口、完整启动及验收边界以 [P0/P1 交付文档](../../../docs/P0_P1_COMPLETE_IMPLEMENTATION_20260919.md) 为准。
导航与路线、探索建图的端到端交互状态机见
[导航与路线、探索建图交互逻辑](../../../docs/NAVIGATION_ROUTE_EXPLORATION_INTERACTION.md)。
以下旧直连 Panel 的说明保留为兼容参考；默认配置不加载旧 Panel。

C++17 + Qt5 RViz Panel、C++ rosbag2 采集器、离线事件/参数回看。无 Python 运行节点或采集脚本；XML launch 只启动本包与 RViz，不启动设备或导航栈。

## 启动

先构建并 source 安装环境。Orin/采集机：

```bash
ros2 run astribot_operator_station diagnostics_recorder --ros-args --params-file <安装目录>/share/astribot_operator_station/config/recorder.yaml
```

工作站：

```bash
ros2 launch astribot_operator_station operator.launch.xml
```

仿真或同机导航栈可以直接把导航可视化和本工作站合并到一个 RViz 窗口。此时不要再单独启动
`operator.launch.xml`，在导航启动时传入合并配置：

```bash
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
  use_rviz:=true \
  rviz_config:=$(ros2 pkg prefix astribot_operator_station)/share/astribot_operator_station/config/operator.rviz
```

合并窗口包含地图、机器人模型、TF、激光、全局/局部代价地图、规划路径、MPPI 调试轨迹、
多点路线工具和 `WorkstationPanel`。Gazebo 仍是独立仿真窗口；`use_rviz:=false` 时可继续用
`operator.launch.xml` 单独打开工作站界面。

同机开发可加 `recorder:=true`，同一 ROS 图只运行一个采集器。仿真传 `use_sim_time:=true`；域配置必须与目标匹配。默认不自动采集，不自动发送控制。新包不替换现有采集脚本。

## 已实现控制接口

- 导航提交：`/navigate_to_pose` NavigateToPose，走既有 operator 仲裁；提交前显示目标及抢占提示。
- 导航取消：只取消本面板持有的目标 handle，不使用 cancel_all。
- 探索取消并存图：版本化 command 的 `cancel_save`；失败重试：`/mapping_session/retry`。取消前确认，保存结果见 `/mapping_session/status`；完成后不可直接恢复探索。
- 探索暂停/恢复/结束保存：`/exploration_coordinator_node/command` ExplorationCommand；旧 pause/resume/cancel Trigger 留作兼容。
- 诊断开始/停止/人工标记/参数快照：`/diagnostics_recorder/start`、`stop`、`mark`、`snapshot` Trigger。
- `/diagnostics_recorder/status` 输出录制状态、实际绝对目录、消息数、缺失话题和故障。
- 离线回看：打开 events.jsonl，选择事件或逐条播放/暂停，查看该接收序号处的关键参数；播放按每 0.5 s 一条推进，不是原始时间倍率，无 ROS 发布出口。

取消受理不代表停稳；导航 Action 终态也不替代独立停稳验收。UI 勾选只是防误操作，不是身份权限。生产部署还需后端控制租约/鉴权。无通用搬运启动、地图切换、真机抓放或参数写入按钮；显示未接入，不能把旧仿真 CLI 当成生产接口。

## 记录契约

默认保存 `/tmp/astribot_incidents/incident_<时间>_<PID>/`：session.log（spdlog）、events.jsonl、manifest.json、bag/。正式部署需把 output_root 指向持久化数据盘；不记录全量环境变量。session.log 是诊断采集器日志，现有全栈 session.log 保持原入口，本阶段未合并其内容。

`config/recorder.yaml` 明确话题与 `/节点:参数` 白名单；按当前部署扩展，不自动抓取全部参数（避免包含密钥和大模型文本）。每 10 s 回读，启动/手工请求立即采样；服务不可用/3 s 超时记录 parameter_gap；未声明参数保存 type=0/available=false；新增、修改、删除事件保留 ROS 类型和值。节点真实生效 revision 和 boot identity 尚未接入，值一律 observed/event_only，不冒充 effective。

schema v2 用 `/parameter_events` 发布端 DDS GID 区分节点实例。发布端更换、消失或同名多实例时清空旧参数证据；来源不匹配的事件拒收，旧会话/旧请求的迟到响应不入账。关闭参数事件发布的节点标 unknown；GID 不是设备 boot UUID，也不能证明参数服务与事件发布端来自同一进程。快照过程中观察到参数变化时放弃该次快照并标缺口，等待再次采样。

回读与已知值不一致且未观察到对应事件时，记录缺口及上次已知序号。离线查看该序号之后、发现缺口之前的区间会显示 unknown，并提供发现序号；新快照恢复 observed。相同值的中途变化又恢复等情况无法据回读检测。

事件排序依据采集接收 sequence；同时存 ROS、UTC、steady 时间。参数事件源 ROS 时间另存；跨节点一致快照、源时间乱序修正和事件缺口完整检测尚未完成。离线列表表达接收证据及后续发现的不确定性，不宣称精确历史控制周期。

每秒记录 data_quality，状态消息和 manifest 同时提供参数可用项数、节点身份质量及话题发布者数、消息/字节数、接收间隔与数据年龄。话题状态区分 missing_publisher、awaiting_data、stale、received；`topic_stale_sec` 默认 3 秒。`/map`、`/tf_static` 收到后按 latched_received 处理，不按周期判过期。时间依据采集端 steady clock，loss_count=null，不能推断网络丢包数或源数据新鲜度。

sqlite3 bag 每 256 MiB 分段、8 MiB 写缓存；每秒检查剩余空间，低于 1 GiB 结束并标 partial。当前没有总配额淘汰、故障前缓存、自动故障分类、资产哈希封装或吞吐验收。普通 topic best_effort，map/tf_static transient_local；需按真实 QoS 调整后验收。消息数不是无丢包证明。

采集器使用独立进程和单线程 ROS executor，文件写入不会阻塞控制节点，但尚未验证 Orin 资源预算。不在 GUI 线程等待 ROS 服务；ROS executor 在后台运行。事件文件异步加载，GUI 上限 10000 条，文件解析上限 64 MiB。

命令行只读查询（C++，不启动 ROS）：

```bash
ros2 run astribot_operator_station inspect_incident /绝对路径/events.jsonl 100
```

## 阶段边界

本包已实现操作接口、主动录制、接收序号参数回看、DDS 端点身份及数据质量记录。旧增量版本不含 3D 回放；当前版本已新增独立 C++ 只读 3D 回放（见顶部交付文档），现场资源预算仍待验收。原阶段顺序为图形 3D 回放与重建、资源预算验收 → P2 自动故障窗口与地图事务 → P3 抓放 → P4 搬运后端。

## 多点循环导航

新增“循环路线”Panel 和“添加路线点”Tool：地图点击拖动添加位置/朝向，
列表可排序、删除和保存 JSON；确认后由机器人端 C++ `astribot_route_executor`
按 `1→2→…→1` 循环，走 `/route/navigate_to_pose` 仲裁入口。
取消按路线 ID 作用于自己的在途目标，等待终态；失败或抢占停止，不自动跳点。
旧 LoopRoutePanel 关闭后不取消已受理路线；默认 WorkstationPanel 关闭后租约到期会请求取消，未确认终态不交接控制权。
详细设计、接口、启动与验证见
[RViz 多点循环导航](../../../docs/RVIZ_LOOP_ROUTE_DESIGN_20260919.md)。

## P0/P1 探索场景增量

探索按钮已改用版本化 ExplorationCommand 和结构化 operator_status，依状态门控，
不再仅根据 Trigger 服务在线就开放所有按钮。正常启动需要同步部署新探索节点和消息包；
旧节点缺少新状态时，探索按钮保持不可用，导航和诊断原入口不变。
详细场景、错误码、假后端与边界见
[P0/P1 探索交互实现](../../../docs/P0_P1_EXPLORATION_INTERACTION_IMPLEMENTATION_20260919.md)。
开发演示使用 `operator_fake.launch.xml`，默认场景 `waiting_map`，控制均映射至 `/operator_fake/*`；
按该文档使用独立 localhost 域，不通过生产 launch 启动假后端。

### 业务场景与设备标记

工作站新增“业务场景”页，支持五类业务、多实例、共享或独立地图、设备位置/轮廓/停靠位/等待位版本化保存，以及停靠位加入导航草稿。详见仓库 `docs/manuals/RVIZ_BUSINESS_SCENES.md`。需一并部署本次 `astribot_map_manager` 和 `astribot_operator_backend`；本页不执行设备开门、上下料或抓放。
