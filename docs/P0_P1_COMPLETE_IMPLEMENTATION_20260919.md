# P0/P1 上位机交付实现与验收边界

日期：2026-09-19。本文描述当前默认 WorkstationPanel；旧探索增量文档及 legacy Panel 仅供历史兼容参考。

## 交付清单

|阶段|实现|
|---|---|
|P0 接口|OperatorCommand v1、机器人 ID、后端 boot ID、命令 ID、租约、查询、结构化原因码|
|P0 控制权|独占协作租约；重复命令去重；重启隔离；失联请求停止；终态未确认禁止转交|
|P0 假后端|localhost 域 224，假导航/探索，真实路线执行器、网关、采集器和 Panel|
|P1 总览|来源新鲜度、控制权、电池可用性、导航/探索/地图/路线/采集状态|
|P1 导航|地图拖动选点、单点导航、精确取消；多点编辑/排序/保存/加载/循环/取消|
|P1 探索|暂停、恢复、结束并保存、保存失败重试、新建受管建图会话|
|P1 日志|C++ rosbag2、spdlog、业务事件、参数快照/变更/缺口、人工故障标记|
|P1 回放|独立只读 RViz、原始时间倍率、暂停、跳转、事件关联、关键参数历史与未知标识|

运行逻辑为 C++，新增启动封装为 XML；受管建图复用仓库已有 Python launch，未新增 Python 运行节点/采集脚本。未修改 SLAM 算法或其注释。

## 控制和接口契约

默认界面只经 /operator_backend/command 调用控制。网关转到现有导航仲裁、路线执行器和探索服务；不发布底盘速度。/operator_backend/status 是 schema_version=1 JSON，5 Hz，包含来源状态及新鲜度；/operator_backend/events 是命令生命周期。

OperatorCommand 请求带 schema_version=1、robot_id、command_id、expected_boot_id、lease_id、operation、payload_json。先读取状态中的机器人和 boot ID，再 acquire；返回令牌仅放响应中。默认租约 6 秒，Panel 每秒 renew。非 query/acquire 操作需要当前租约。同 ID/相同请求返回原结果，不重复执行；相同 ID/不同内容拒绝。查询返回已知结果，超时表示 UNKNOWN，不代表未执行，不能自动换 ID 重试。

|operation|payload_json|
|---|---|
|acquire|{"client_name":"workstation"}|
|renew / release / query|{}；query 的 command_id 指向原命令|
|navigate|{"frame":"map","x":1,"y":2,"yaw":0}；yaw 为弧度|
|cancel_navigation|{"operation_id":"原导航命令ID"}|
|start_route|{"points":[{"frame":"map","x":1,"y":2,"yaw":0},{"frame":"map","x":3,"y":2,"yaw":1.57}],"dwell_sec":0}|
|cancel_route|{"route_id":"当前路线ID"}|
|explore_pause / explore_resume / explore_cancel_save|{"expected_exploration_boot":"当前探索boot","expected_exploration_revision":当前版本}|
|retry_map / new_mapping_session|{}|
|record_start / record_stop / record_mark / record_snapshot|{}|

命令状态：ACCEPTED、RUNNING、SUCCEEDED、FAILED、CANCELED、PREEMPTED、UNKNOWN；拒绝为 REJECTED。控制状态 HELD / STOP_UNCONFIRMED / OBSERVER，释放响应 RELEASING。具体操作也可直接返回 ACCEPTED 后的终态。原因码包括 REQUEST.SCHEMA_MISMATCH、REQUEST.BOOT_MISMATCH、REQUEST.CONFLICT、REQUEST.RETIRED、REQUEST.INVALID_PAYLOAD、CONTROL.NOT_OWNER、CONTROL.BUSY、STATE.STALE、STATE.REVISION_MISMATCH、NAV.NO_MATCHING_OWNED_GOAL、ROUTE.NO_MATCHING_OWNED_ROUTE、MAP.SESSION_NOT_IDLE、MAP.NOT_FAILED、CAPABILITY.UNAVAILABLE。业务子系统的原因继续保留。

去重仅在当前进程 boot 内有效：最多 10000 条业务命令、约 16 MiB 请求记录和 4096 次租约获取；容量满明确拒绝，重启生成新 boot，不静默重放。持久化任务断点恢复属于 P4。

关闭默认 WorkstationPanel、断网或停止续约后，网关请求取消自有导航/路线、暂停自有探索；等待终态才能交接控制权。取消受理与 Action 终态都不等于物理停稳。旧 LoopRoutePanel 仍是无租约兼容入口，其“关闭仍循环”行为不适用于默认新界面。可信 ROS 图内的遗留直连接口仍可用；协作租约不等于身份认证，未经 SROS2/网络访问控制验收，不能声称防恶意绕过。

生产 navigation.launch.py 启动网关；网关退出触发该 launch 关闭，避免受管导航在失去上位机控制权管理后继续运行。独立 backend.launch.xml 供开发/集成使用，不具备整个导航 launch 的退出联动。设备 watchdog、制动和物理急停仍需独立验收。

## 探索场景交互

|场景|显示/动作|
|---|---|
|尚无地图、定位或后端|显示等待原因；不把“暂无前沿”当作建图完成|
|可运行/用户暂停|允许带当前版本恢复；暂停取消探索持有目标，保留会话|
|导航手动抢占|显示抢占/暂停原因；不会由 Panel 自动恢复探索|
|前沿耗尽且满足完成条件|探索结束；地图会话等待停稳、完成 SLAM、校验地图资产后报告 SAVED|
|取消探索|确认后 cancel_save；进入结束/保存事务，不再允许恢复旧会话|
|保存中|显示进度，禁用冲突操作；不因服务受理就显示保存成功|
|保存失败|显示失败原因，允许 retry_map；不启动下一段探索掩盖失败|
|保存成功|可新建受管会话；新会话初始暂停，需显式恢复|
|失联、同名重复发布者、boot/revision 变化|禁用过期操作，重新读取状态；确认框期间版本变化拒绝提交|
|控制权丢失|请求暂停自己启动的探索；未确认前不交接控制权|

new_mapping_session 由 C++ mapping_runtime 管理自己创建的进程组：仅在持续 2 秒新鲜低速里程计证据、无外部同名 SLAM/探索/地图管理节点和地图发布者时启动。再次创建仅允许自己拥有且已 SAVED 的会话，先退出旧进程组、等待图清除，再建立唯一目录/地图名。不会清理他人栈；停止超时转 FAILED，不强杀不明进程。运行时退出对其自有组发送 SIGINT；强制断电/kill 的恢复不在此承诺内。

## 启动与配置

构建需包括 operator_msgs、operator_backend、operator_station、route_executor、s1_exploration 及其仓库依赖，再 source 同一安装空间。

开发假后端：
```bash
ros2 launch astribot_operator_station operator_fake.launch.xml
```
固定 localhost 域 224，仅用于开发，不与生产导航混启。假导航 outcome 可选 success/reject/hold/cancel_unconfirmed；探索场景见原探索交互文档。假演示不启动 SLAM，因此新建地图会话不可用。

机器人导航启动继续使用现有入口（已带网关、mapping_runtime、路线执行器）；一个 ROS 图仅启动一个网关和采集器。工作站：
```bash
ros2 launch astribot_operator_station operator.launch.xml
```
采集同机可加 recorder:=true；仿真再加 use_sim_time:=true。跨机必须匹配 ROS 域和网络。默认不自动发送目标或开始采集。

受管建图默认 profile=""，明确禁用创建，避免猜测硬件标定。仿真部署可将 operator_runtime_params_file 指向 backend/config/mapping_runtime_sim.yaml；此功能需要已有传感器/里程计，且不能与已有独立 SLAM 栈争用。硬件配置 profile=hardware，并通过 launch_arguments 字符串数组显式提供 lidar_topic、lidar_topic_back、imu_topic、point_notime、imu_extrinsic_tran、back_extrinsic_tran、back_extrinsic_rota、map_topic、odom_topic、robot_base_frame。值必须来自当前设备标定，不能照搬仿真值。save_path 改为持久化目录。新会话不启动底盘/传感器驱动。

## 采集与回放

默认 incident 目录 /tmp/astribot_incidents/incident_<时间>_<PID>/，包含 session.log、events.jsonl、manifest.json、bag/。正式部署修改 recorder.yaml 的 output_root 为持久化数据盘。全栈既有 session.log 保留原入口；incident/session.log 是采集器自身日志。

关键参数采用显式白名单，含导航、探索、地图及网关/运行时配置。启动、手工快照和周期回读，加 /parameter_events 记录变更；数据缺失、同名节点、节点重启及快照竞争标 unknown。参数证据是 observed/event_only，不能冒充控制周期精确 effective 值。事件包含命令 ID、状态、原因和采集序号，保留 UTC/ROS/steady 时间。

Panel 打开独立 C++ replay_viewer，选择 incident 目录；也可运行：
```bash
ros2 run astribot_operator_station replay_viewer /绝对路径/incident目录
```
只读 RViz 与播放器使用 localhost 隔离域和每次跳转唯一话题前缀；只放行类型匹配的 TF、地图、路径、里程计、关节状态及完整 costmap。不会回放速度、轨迹控制、Action/Service 请求。倒跳会重建 RViz/播放器，防止未来 TF/map 缓存污染。支持 0.1–4 倍速；选择事件可定位相应 ROS 时间并查看参数证据。非单调 bag 时间拒绝回放，需要按时钟区段拆分。

当前为 sqlite3 可视化证据回放，不是控制算法确定性重执行。事件文件限 64 MiB、100000 条；高倍率 DDS 丢帧、长包启动耗时、相机/点云、MCAP、增量 costmap 重建及跨节点源时钟对齐尚未验收。不宣称完整网络丢包计数。故障前环形缓存、自动故障分类和长期归档归后续阶段。

## 验证范围

本轮使用隔离 localhost DDS 域、假 Action/服务和临时构建空间；未向真机发导航或速度，未重启共享仿真。测试覆盖重复提交、冲突/越权、过期取消、租约到期终态屏障、外部 SLAM 拒绝、Panel 门控、录制参数与业务事件、回放控制话题阻断，以及已有探索/地图会话回归。

本轮 4 个包构建通过；20 个 gtest 用例全部通过（colcon 汇总含 CTest 包装共 30 项，0 失败）。真实 Ogre/RViz 窗口连续两次拖动产生两行路线点，朝向一致，无启动请求。XML/YAML/RViz/Python 语法及假后端、受管建图 launch 参数展开通过。

验证日志：/tmp/astribot-full-p01-build.log、/tmp/astribot-full-p01-final-test.log、/tmp/astribot-full-p01-viewport.log；截图 /tmp/astribot_route_viewport_smoke.png。这些临时文件仅作为本轮本机证据，不是长期归档。完整 Nav2/Gazebo 搬动、新建 SLAM 会话正向保存、真机停止距离、Orin 资源预算、长时稳定性和网络延迟需要部署现场验收；不能用假后端/离线测试替代。P2 地图工位事务、P3 机械臂、P4 搬运/WCS 持久化恢复、P5 跨楼层现场验收未纳入本次实现。
