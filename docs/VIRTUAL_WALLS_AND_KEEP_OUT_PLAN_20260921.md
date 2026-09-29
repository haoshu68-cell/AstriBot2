# RViz 虚拟墙与禁区落地方案

## 目标与选择

借鉴扫地机的“地图上绘制虚拟墙/禁区—保存—任务遵守”交互。在现有工作站增加独立页，支持线段虚拟墙、矩形、多边形禁区；编辑、启停、删除、保存、重载。约束归属地图版本，所有业务场景共用；探索期间归属本次建图会话。场景切换不能暗中取消地图禁区。

三个可选实现：只画 Marker 没有控制作用，不采用；直接使用 Nav2 KeepoutFilter 能覆盖双 costmap，但缺少本项目需要的版本确认、失联关闭和探索原始地图过滤；本次采用独立 C++ 约束核心 + Nav2 自定义 Layer，在通用栅格接口上加版本/确认协议，复用现有 inflation、探索、仲裁及最终保护。

官方参考：
- Valetudo RestrictedZone capability：https://valetudo.cloud/pages/usage/capabilities-overview/
- Nav2 Humble KeepoutFilter：https://api.nav2.org/nav2-humble/html/keepout__filter_8cpp_source.html
- Nav2 双 costmap 配置建议及 inflation 区别：https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_filters/keepout_filter/

## 组件与边界

新增 `astribot_navigation_zones` 功能包：
1. 纯 C++ 几何/版本核心：简单多边形、自交与退化检查、线段胶囊距离、保守栅格化、圆包络与运动扫掠检查。
2. `zone_server`：唯一配置写入者，原子持久化、命令幂等、控制权/停稳/任务空闲检查，地图或建图会话上下文，消费者确认与心跳。
3. `ZoneLayer`：部署到 global/local costmap，位于障碍层之后、inflation 之前；设置 lethal 障碍且不能被清图清掉。局部 odom 坐标必须转换至 map；缺数据、TF 或心跳时 current=false 并阻塞区域。
4. 公共订阅适配：唯一来源、源时间、接收时间、版本与有界数据校验，供探索、仲裁、最终保护使用。

现有模块接入：
- RViz 新页只编辑几何并通过 operator_backend 提交，不发布速度。独立连续点击工具不污染路线点。
- operator_backend 维持控制租约，转发配置事务，显示保存/等待生效/已生效/失联状态。
- 探索在前沿搜索使用叠加禁区的地图副本，原 SLAM 地图不改写；禁区内及穿墙的候选被阻断。未覆盖但被人为封闭的区域报告边界受限，不伪装成全图建图完成。
- 导航仲裁同时保护上位机、路线、探索入口；约束版本或上下文变化时取消旧动作，等待终态，不自动恢复。
- 已启用的 final_protection 在现有唯一速度出口追加禁区预测检查，同时检查指令速度与实测速度、当前机器人包络和制动距离；不新增第二个底盘写入者。

## 数据与地图生命周期

区域字段：`id/name/type/enabled/points/width_m/margin_m`。坐标为 map，线段恰好两个不同点，矩形转换为四点，多边形 3～32 点，拒绝自交、重复边、非有限坐标；最多 64 区域。虚拟墙具有实际厚度，栅格化计入半栅格对角线以免薄墙漏格。

集合绑定 `context_id/revision/manager_boot`。加载地图使用 `map:<map_id>:<immutable_version>`；建图使用现有 mapping_session 发布的 `mapping:<mapping_session_id>`。新会话不沿用旧会话坐标。已有地图的禁区在重新加载该地图时自动恢复。建图会话与存图导入通过保存目录关联；继承到保存地图的限制必须重新核对后提交，不能把 SLAM 回环前的坐标当成现场标定。

存储为地图目录下的 `navigation_zones/zones.json`，独立于日志和 RViz 配置，记录完整版本、命令 ID 和操作者。不能仅因 UI 重启删除禁区。保存失败保持旧版本并返回明确错误。

## 生效事务

`绘制草稿 → 暂停/取消所属运动并确认停稳 → 提交(预期上下文+版本) → 原子落盘 → 发布版本 → 双 costmap 确认 → 保护层确认(启用时) → READY → 人工开始/恢复任务`。

提交默认只允许空闲停稳时进行。不能把“已保存”显示为“已生效”。消费者失联、版本不一致、地图不确定或 TF 过期进入 BLOCKED；恢复数据后不恢复已取消任务。机器人位于/触及禁区时拒绝继续运动，不自动穿墙退出。

初始无地图、无明确建图会话时显示 NO_CONTEXT。开始新建图会话的操作不依赖禁区 READY，以避免启动死锁；导航和探索恢复必须等待 READY。外部 SLAM 必须接入现有 mapping_session 的明确会话上下文，不能用任意 `/map` 冒充有效上下文。

## 仿真与真机

公共 navigation.launch 与 RPP/MPPI 的双 costmap 配置共用 C++ 插件；真机/仿真仅时钟、TF、定位和原有设备链不同。普通 Nav2 costmap 约束在 policy_stage=off 时仍生效；独立速度扫掠保护仅在既有 final_protection 链启用时生效，真机部署应启用该链并验证制动参数。虚拟墙不是硬件安全认证边界，不能替代硬急停或补偿定位漂移。

## 施工顺序和验收

1. 几何、版本、存储核心及边界测试。
2. C++ 服务与全局/局部 Layer，真实 Layer 更新/清图/失联/TF 测试。
3. 探索地图过滤、仲裁取消和既有最终保护接入。
4. RViz 绘制页、版本状态、持久化重载及地图切换交互。
5. 公共启动配置、诊断录制、操作手册。
6. 隔离验证：薄墙、斜墙、矩形、凹多边形、自交拒绝、地图版本切换、保存重载、过期心跳、不同坐标系、清图后仍有效、探索边界受限、旧动作取消、正确新目标放行。

本次不控制真机；单元、ROS 假后端和插件测试不能标为 Gazebo 闭环或真机验收。后续 Gazebo 验收应比较轨迹与区域的最小距离；真机最后按实测定位误差、完整包络、延迟与停止距离设定 margin 并做低速围栏测试。

用户已明确要求先方案后施工，本次按以上顺序在当前工作区实施，使用独立构建目录，不重复请求实施授权，不覆盖他人未提交改动。

用户补充（2026-09-21）：尽量与 SLAM 地图逻辑结合。实施为复用 mapping_session 的会话 ID、收尾与保存目录；map_manager 导入记录来源目录；区域数据位于 map_catalog/navigation_zones，并随地图载入选择版本。保持估计器原图不变。

## 实施细化

地图保存时，`mapping_session` 在提交 SLAM finish 前冻结当前约束快照，写入现有 `manifest.json.navigation_zones`，字段含 schema、frame、context_id、revision、regions。该清单本身的哈希作为地图版本，因此迁移地图文件夹时也携带约束，不依赖原计算机的绝对来源目录。`source_directory` 仅兼容同机旧地图关联。清单中的原始地图/点云哈希集合保持原语义。

已 SAVED 或正在收尾的建图会话禁止编辑禁区；需要先导入/加载已保存地图，再在地图版本下编辑。已加载地图的后续修订保存在 map_catalog/navigation_zones/zones.json，迁移这些修订时须迁移完整 map_catalog，不能只复制最初的 SLAM 输出文件夹。

运行时采用 `map` 平面。探索输入要求轴对齐的 OccupancyGrid 原点；旋转地图拒绝使用，不能静默忽略原点朝向。local costmap 的 odom→map 转换支持平移和偏航；缺失或过期时阻塞。圆包络扫掠保守覆盖整个当前机器人包络，会比精确多边形更早拦停，窄通道性能仍需闭环测量。

循环路线执行器也绑定开始时的禁区 token，包括两目标之间的等待阶段；不能在没有活动 Nav2 goal 的间隙跳过版本变化检查。RViz 路线文件增加 zone_context，防止不同建图会话的路线误用。旧路线文件在已有禁区上下文时需要重新标记、保存。
