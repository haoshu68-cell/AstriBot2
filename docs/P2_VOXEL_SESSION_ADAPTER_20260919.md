# P2 Voxel 会话加载适配器

本轮补齐 C++ voxel_session_adapter、目录适配、受管 Voxel 进程和 Nav2 生命周期联动。代码已接入可选启动项；本轮使用隔离依赖和替代进程验证，**尚未做真实 Voxel + Nav2 两地图切换验收**。

## 实现与归属

未修改 SLAM 实现及注释。复用既有 voxel_slam.launch.py 的 mode=localization、previous_map、save_map=0；运行期编排、目录准备和证据检查均在 C++ 中完成。既有 launch 仍为 Python，没有新增 Python 脚本。

归档目录名为 manifest SHA256，原会话 YAML 名与 manifest.session 一致。直接把哈希目录当 previous_map 会找不到 YAML。materializeSession() 在私有运行根建立“attempt_id/原会话名”，复制并复核全部哈希、原 manifest 和既有会话格式，再调用 Voxel 加载入口，原归档保持只读用途。匹配阈值当前固定为 0.5，尚未做场地调优。

复制只允许 asset_root 内的资产，拒绝符号链接、路径穿越、重复 attempt、损坏文件和超过 8 GiB 的输入；保留至少 1 GiB 空间。失败只清理本次自己创建的暂存目录。成功目录保留用于诊断，尚未自动淘汰，部署需要容量规划。

## 事务顺序

1. 校验机器人、boot、attempt、停稳、事务 ID 和目标地图版本；后台线程准备目录。
2. 只停止自身创建的旧 Voxel 进程组，等待退出和 ROS 图清除。外部 SLAM/地图节点存在时拒绝，不接管、不杀掉别人的栈。
3. 校验全局 costmap 的 static_layer.map_topic、global_frame=map 和 always_send_full_costmap=true。
4. 请求 Nav2 lifecycle manager RESET，确认成功后启动自己的 Voxel 定位会话。
5. 等待 TRACKING、实时 camera_init→aft_mapped、map→底盘 TF 和新地图。
6. 请求 Nav2 STARTUP，再等待启动完成后生成的全局/局部完整 costmap。
7. 全部证据到齐才报告 READY，地图管理器仍独立完成持续确认和提交。

TRACKING 是状态变化通知，不能因两秒没有重复消息就认定失联。适配器保留该状态，以自有进程存活、唯一发布端及实时 SLAM TF 判断定位证据。新会话重建 TF 监听上下文，避免旧动态缓存，同时重新接收静态 TF。

costmap 必须尺寸有效、帧正确、时间新鲜、不是全未知，并且生成于本轮 STARTUP 之后。不能用进程存在或服务受理替代就绪。提交后持续报告质量，证据失效不再报告 READY。

失败或超时保持地图事务阻塞/恢复语义。未确认结束的生命周期请求会阻止新加载，迟到响应不能推进新 generation。同 ID/内容不会重复 RESET；同 ID/不同内容拒绝。适配器不发送导航目标、速度、initialpose 或机械臂命令。

## 配置

默认不启动适配器，也不授予 Nav2 重配置权限。必须显式选择部署，并确保它是该 ROS 图内 Voxel 会话的唯一所有者。

导航 launch：
- enable_voxel_adapter，默认 false。
- voxel_adapter_params_file，默认 config/voxel_session_adapter.yaml。
- map_manager_params_file 建议选择 config/map_manager_voxel.yaml，将事务超时设为 180 秒；适配器加载超时为 120 秒。

独立 backend.launch.xml 对应参数为 enable_voxel_adapter、voxel_adapter_params、map_params，另补 use_sim_time。独立入口不启动传感器/Nav2，也不具备完整导航 launch 的退出联动；不要和已有导航入口重复启动网关。

仿真模板为 config/voxel_session_adapter_sim.yaml。启用时地图管理器和适配器均需 use_sim_time=true；本轮修正了 navigation.launch.py 对地图管理器的时钟传递。

硬件要求 profile=hardware、allow_navigation_reconfigure=true，并在 launch_arguments 显式提供七项本机已校准参数：lidar_topic、lidar_topic_back、imu_topic、point_notime、imu_extrinsic_tran、back_extrinsic_tran、back_extrinsic_rota。不能照搬仿真值。

asset_root 必须对应地图管理器 storage_root/assets；runtime_root 放在有容量预算的持久化数据盘。另核对 map_topic、odom_topic、odom_frame 和 robot_base_frame。里程计应独立于被重启的 Voxel 进程，以持续提供停稳证据。全局和局部 costmap 均须发布完整数据；仓库当前 Nav2 默认配置均为 always_send_full_costmap=true。

## 能力边界

生产适配器的 cargo_known、transport_ready、handover_ready 均为 false。真实载荷/运输姿态/交接位证据尚未接入，不能为开放跨楼层按钮而伪造它们。因此本批支持同楼层会话加载；人工跨楼层仍由 CARGO.TRANSPORT_UNCONFIRMED 拦截，等待后续搬运适配。

costmap 确认依据“配置一致 + RESET/STARTUP + 新会话后的数据”，不宣称逐栅格与 SLAM 语义完全相等。真实 StaticLayer、重定位质量、两地图切换、长时运行及断电恢复仍需现场验证。新任务阻止不替代在途任务的定位保护、制动或急停。

P3 机械臂执行、P4 WCS/队列与 P5 现场验收未在本轮完成。

## 验证

本轮新增 4 个 gtest 用例，连同地图管理已有 6 个，共 10 个全部通过（含 CTest 包装共 14 项，0 失败）。隔离域为 localhost 215/216。正向测试将受管进程创建替换为本测试拥有的 sleep 进程，并提供假定位、TF、地图、costmap 和 lifecycle 服务，没有启动真实 Voxel、重置真实 Nav2 或发布速度。

覆盖：
- 原会话名恢复、哈希复核、损坏/越界/重复 attempt 拒绝。
- 七项就绪门槛、默认禁用、外部 SLAM 占用拒绝。
- 错误 map_topic 在 RESET 前拒绝；RESET→加载→STARTUP 顺序。
- 重复请求不重置，停止新观测后就绪失效，以及地图事务回归。

使用 /tmp/astribot-route-build 独立构建/安装，未覆盖 ws_robot/install。验证日志：
- /tmp/astribot-voxel-adapter-build.log
- /tmp/astribot-voxel-adapter-test.log
- /tmp/astribot-voxel-adapter-launch.log

启动 XML/YAML、navigation.launch.py 语法和 backend launch 参数展开已检查。诊断白名单新增 /map_session_adapter/status，以及部署模式、权限、话题、资产根和标定参数。
