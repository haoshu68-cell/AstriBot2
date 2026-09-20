# P2 地图、工位和人工换层事务实现

后续已补 C++ Voxel 受管加载适配器，最新能力及验证边界见 [适配器交付说明](P2_VOXEL_SESSION_ADAPTER_20260919.md)；本文原批次记录保留。

本轮完成 P2 的资产归档、工位版本、换层事务、RViz 入口和回归测试。属于 P2 第一批实现，不代表 P2—P5 已全部完成或现场验收通过。基于现有 P0/P1 网关继续施工，未修改 SLAM 算法及注释，未运行真机运动。

## 包与控制归属

- `astribot_map_manager`：新增 C++ Catalog（资产/持久化/版本）、MapManager（ROS 证据与事务）、隔离假切图适配器。
- `astribot_operator_backend`：继续唯一上位机入口、租约与命令去重，增加地图领域命令和运动阻止条件。没有复制 Nav2、SLAM 或机械臂调度。
- `astribot_operator_station/MapPage`：独立 C++/Qt 地图工位页，共享 WorkstationPanel 的控制权；UI 不直接操作机器人文件。
- `diagnostics_recorder`：记录 `/map_manager/status`、`/map_manager/events`、目录/适配器/超时配置；地图业务事件可进入现有事件回放，包含事务/工位版本和地图 manifest SHA256；逐文件哈希保留在地图资产目录，尚未自动打包复制到 incident。

生产 navigation.launch.py 和独立 backend.launch.xml 均启动地图管理器，并令网关 `require_map_manager=true`。若管理器异常退出、状态冲突或过期，新导航/路线/探索恢复会被拒绝。独立直接运行网关仍兼容旧 P1，但一旦观察到地图管理器，就不会在其消失后自动退回无地图监管状态。

## 地图资产与工位

导入已保存 Voxel SLAM 会话，要求 `manifest.json` 完成标记、完整哈希、地图 YAML/PGM、扫描位姿和关键帧存在；复用已有 `inspectSession()` 格式检查。拒绝导入根目录之外的会话、资产符号链接、路径穿越、损坏哈希或未完成会话。校验属于现有检查器定义的文件完整性，不证明点云几何或定位质量。

将资产复制到管理器自己的 `assets/<map_id>/<manifest SHA256>/`，再次校验，同步文件/目录，再提交目录索引；删除原导入目录不会破坏已归档版本。地图 ID 与内容/楼层元数据不可原地覆盖；新地图用新 ID。激活前重验哈希，不覆盖已归档但被人工损坏的文件。

工位引用明确 `map_id + map_version`，每次修改追加版本，并检查 `expected_station_version`。停靠姿态和作业位置分开保存；本批作业位置仅支持 map 平面 x/y/yaw，不能作为机械臂 6D 执行目标。停靠点还检查地图边界和已知空闲栅格；这是点位检查，不替代机器人完整 footprint、载荷包络及导航实时碰撞检查。历史版本保留在 catalog.json，界面显示最新版本；导航草稿仅能使用当前活动地图的工位。

资源上限：32 张地图、128 个工位、每工位 1000 版、4096 条持久化命令、目录账本 16 MiB、单次资产总量 8 GiB。达到上限拒绝，不自动清理证据。大地图校验目前在独立地图节点中同步执行，可能使该节点状态变旧；网关续租线程不受阻，地图状态变旧期间禁止新运动。大地图耗时/GUI 延迟和异步导入进度尚待专项优化。

## 人工换层与地图切换

状态：

`WAIT_TRANSFER → LOAD_INTENT → LOADING → COMMITTED`

同楼层可跳过 WAIT_TRANSFER。LOADING 的受理不等于成功；需要适配器对本次 attempt_id、transaction_id、目标 map_id/version 提供新鲜、唯一发布端的确认，且 localization、TF、地图、全局/局部 costmap 五项都为 true，连续保持至少 1 秒，才能提交活动地图。

前置条件包括唯一发布端、递增时间戳和源时间新鲜的里程计，低于 0.01 m/s 与 0.01 rad/s 持续 1 秒；导航无在途目标、路线不活跃、探索暂停或已结束且无在途目标、有效控制租约。这里是软件停稳观测，不是物理制动距离认证。

人工换层还需要部署适配器提供 `cargo_known`、`transport_ready` 和 `handover_ready`。界面的确认记录当前操作者和时间，不代替这些后端证据。需先导航到交接位再准备转运；本批不提供电梯控制或自动交接位导航。

失败、超时、控制权丢失、适配器重启转 RECOVERY_REQUIRED。未提交事务在进程重启后也转此状态；禁止自动重放。显式恢复只重新加载并验证选定目标地图，不恢复运动或机械臂动作。只有尚未确认转运、且适配器仍确认原活动地图定位时，才能放弃事务；已转运后不能用“回滚旧地图”解除运动阻止。

提交成功后持续检查活动地图确认；确认丢失会阻止新的运动请求。现有在途动作的制动/取消仍由既有控制和定位保护链负责，本模块不发布速度，也不把状态阻止等同硬件急停。

旧地图版本的导航请求拒绝；活动版本变化时 RViz 清空路线草稿。事务指令意图和每步结果通过 fsync + rename 持久化，单目录用 flock 防止双实例写入。I/O 故障退出服务以避免继续使用不确定账本；断电一致性尚未做物理故障注入验收。残留暂存目录需人工核查，不静默删除历史数据。

## 上位机接口

全部经过 `/operator_backend/command`，复用 OperatorCommand v1 的机器人、boot、命令 ID、租约。请求 payload 额外带当前 `expected_catalog_boot`、`expected_revision`。网关转发时将 boot 绑定到地图管理器并移除辅助字段。

| operation | 业务参数 |
|---|---|
| map_import | map_id、directory（机器人文件路径）、floor、可选 name |
| station_put | map_id、map_version、station_id、expected_station_version、kind、dock_pose、可选 work_pose |
| map_switch_begin | map_id、map_version、manual_transfer |
| map_transfer_confirm | transaction_id |
| map_abort | transaction_id；需要原地图定位确认 |
| map_recover | transaction_id；显式重新验证 |

姿态格式：`{"frame":"map","x":1.0,"y":2.0,"yaw":0.0}`。kind 为 shelf/workstation/standby/handover。请求去重和版本检查独立：同 ID 同请求重读原受理记录，不再执行；当前事务进度以状态快照为准。

新增原因码包括 MAP.CONTEXT_BLOCKED、MAP.VERSION_MISMATCH、MAP.MOTION_OR_STATE_UNCONFIRMED、MAP.ADAPTER_UNAVAILABLE、MAP.VERIFICATION_TIMEOUT、MAP.SOURCE_LOCALIZATION_UNCONFIRMED、RECOVERY.PROCESS_RESTART、STATION.VERSION_MISMATCH、ASSET.HASH_MISMATCH。非法结构与底层格式异常统一标 MAP.INVALID_REQUEST_OR_ASSET，详细原因保留在 message。

## 切图适配器边界

当前生产 Voxel SLAM 部署没有统一、可直接替换的“加载会话并确认定位/双 costmap”的端点。本批交付适配协议及 C++ 假后端，**没有用 AMCL LoadMap 代替现有 Voxel 定位链**。因此生产界面可以归档/编辑工位，切换按钮在适配器缺失时明确不可用。

部署适配器服务 `/map_session_adapter/command` 使用 OperatorCommand，operation=load_session；payload 带 transaction_id、attempt_id 和已验证 target 资产信息。服务响应 boot_id 必须与请求 expected_boot_id 一致。适配器必须幂等处理 attempt_id，不能把服务成功当定位成功。

适配器 `/map_session_adapter/status` 的 JSON：boot_id、supports_voxel_sessions、state、attempt_id、transaction_id、map_id、map_version、localization_ready、tf_ready、map_ready、global_costmap_ready、local_costmap_ready；人工转运另带 cargo_known/transport_ready/handover_ready。这些字段必须来自部署观测，不能简单常量置真。默认管理器只订阅此协议，不直接调用 SDK、map_server 或发布 initialpose。

假适配器只注册 `/operator_fake/map_adapter/*`，outcome=ready/reject 或其他（确认条件不满足），不启动 SLAM/设备。`operator_fake.launch.xml` 已加入它和独立假目录；假演示本身不生成里程计或伪造机器人停稳，切换验证由独立测试里的假依赖完成。

此协议运行在可信 ROS 图内，仍需部署侧 DDS 权限；P0 协作租约不是身份认证，原始 ROS 服务不应暴露给不受信任客户端。

## 启动和数据目录

构建需包含新增 `astribot_map_manager` 及 operator_backend/operator_station，source 相同安装空间。正常导航入口自动带地图管理器。工作站仍运行：

```bash
ros2 launch astribot_operator_station operator.launch.xml
```

生产导航 launch 参数 `map_manager_params_file` 或独立 backend.launch.xml 参数 `map_params` 指向部署配置；默认配置为新增包 `config/map_manager.yaml`。

默认账本 `/tmp/astribot_map_catalog/catalog.json`，资产 `/tmp/astribot_map_catalog/assets/`，导入根 `/tmp/astribot_slam_sessions`。这些是开发默认值；部署时必须改为持久化数据盘和实际 SLAM 保存根目录。工作站填写的是机器人路径，不是工作站本地文件。

## 验证与后续工单

自动化覆盖：100 次重复工位提交只增加一个版本、旧版本拒绝、历史版本保留、目录互斥、损坏资产/越界路径、原始目录删除、归档后篡改、人工确认事务 ID、加载超时不提交、显式恢复、租约丢失、活动定位证据失效、进程重启恢复态、网关地图阻止/旧版本拒绝、Qt 观察模式/确认请求版本绑定。测试仅在独立 localhost 域 216 等运行，没有真机导航或速度发布。

最终 27 个 gtest 用例通过；colcon 连同 13 个 CTest 包装合计 40 项，0 错误、0 失败、0 跳过。真实 Ogre/RViz 连续两次拖动选点验证通过，没有启动请求。XML/YAML/RViz 配置、launch 参数展开和 diff 空白检查通过。验证使用 /tmp/astribot-route-build 的构建/安装空间，未覆盖现有 ws_robot/install。

日志 `/tmp/astribot-p2-build.log`、`/tmp/astribot-p2-final-test.log`；最后地图增量构建/测试为 `/tmp/astribot-p2-map-final-build.log`、`/tmp/astribot-p2-map-final-test.log`，界面截图 `/tmp/astribot_map_page.png`。临时证据不等于长期归档。

下一批仍按阶段门槛推进：

1. P2：按现有 Voxel 会话加载/定位入口实现生产适配器，完成真正两地图切换、costmap/TF 确认；补远端地图预览、历史工位查询和故障前后缓存。
2. P3：复用 `/transport/plan_manipulation`、`/transport/plan_skill`；源码实际组名 arm_left/arm_right，与初稿 left_arm/right_arm 不同。先绑定计划、场景、地图、起始关节与载荷版本，再接执行。规划返回轨迹本身不证明安全执行。
3. P4：复用现有 transport 的资源/载荷状态与持久化语义，补上位机队列、WCS 幂等与回报投递，避免再建一条并行搬运动作链；当前运行时已有 Python 部分，C++ 迁移应按接口逐步替换。
4. P5：动态目标、跨层、载荷和故障现场验收；没有现场观测时不能生成成功率或停距结论。
