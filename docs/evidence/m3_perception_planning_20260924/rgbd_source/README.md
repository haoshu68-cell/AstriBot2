# M3 受控工位 RGB-D Request 源交付

本子项从 2026-09-24 10:46 开始，11:46 为同一问题检查点。范围是 `astribot_s1_manipulation_perception` 内 C++ 库及定向验证。M1 拥有任务、工位、实例身份、场景和执行；M2 拥有仿真场次，M5 核对实际相机输入。本轮没有启动真实模型、GPU、Gazebo 或控制动作。

2026-09-24 11:44，总调度任务「设计机器人搬运仿真流程」（01a0c405-627f-7742-b500-e057726ed2e7）回传独立冻结审查通过：33包文件、4包外文件、22安装产物hash一致，13项测试/库绑定一致，72项离线检查零失败。该任务批准本子项标为 **OFFLINE_ACCEPTED**。审查来源是跨任务回传，详见status.json的review_provenance；实际VIEW_NOT_READY、模型与执行仍未验收。本次仅更新状态/交付清单，源码保持冻结，未启动ROS或重测试。

## 接口与边界

`SingleBoxRequestSource` 挂在 M1 已运行的节点，复用 M1 TF buffer。既有 `rgbd_pointcloud_node` 提供 decimation=1、保留 NaN 的全像素 XYZ；source 严格配对 RGB、XYZ、实际 CameraInfo 的采集戳、frame 和布局，并核对 K 投影回像素。

先按独立工作台范围筛选，再做橙色连通分离。范围不得来自目标实体位姿、GT bbox 或历史抓取点。单实例及身份版本由工位所有者给出，颜色不建立身份。实际有效点至少 2048、有效深度比例至少 80%；超出 12000 只减采样，不填充。Request 保留原 CameraInfo、采集 TF、工位 TF、模型和所有者上下文。

接收时记录 ROS 与 steady 时钟；迟到与到达后时间合并计算，重复帧不能续期。采集输入 250 ms、推理准入 0.5 s、总结果最多采集后 5 s；后两项原始 steady deadline 随 Request/PlannedPick 传递。相机内参 hash 和工位 region_revision 加入 Context。ROS 回退须新 clock_epoch 与新帧；同源/同标定版本内内参变化锁存拒绝。

调用方式和 ABI 变更见 `ws_robot/src/astribot_s1_manipulation_perception/PICK_PLANNING.md`。旧手工 Request 也必须提供原接收 deadline，不能在 `plan()` 中重设 `now+5s`。头文件与静态库必须整套更新。

## 验证

| 检查 | 最终证据 | 结果含义 |
|---|---|---|
| 新 source 纯 C++ 边界 | contract_final.xml | 13 例；布局、K、区域、点数、深度比例、身份/版本、时间边界 |
| 单进程 ROS 订阅夹具 | protocol_final.xml、protocol_final_session.json | 6 例；合成传感器发布/订阅、TF、epoch、内参、暂停和回退 |
| planning client 契约回归 | client_regression.xml | 10 例；新 ABI 和原约束 |
| planning client Action 回归 | client_protocol_regression.xml、client_protocol_session.json | 6 例；合成推理/规划对端、失败/取消/场景变化，无控制器 |
| 安装与外部 CMake 消费者 | install.log、consumer_configure_explicit.log、consumer_build.log、consumer_stdout.txt | 新库链接和调用成功；不创建 ROS 节点 |
| 既有服务定向回归 | baseline_inference_contract.xml、baseline_worker_process.xml、baseline_action_contract.xml | 9+4+24例；原服务当前源码、独立fixture worker，无真实模型 |

ROS 夹具在已授权 domain 100、localhost 下运行；运行前可读进程环境和无 daemon 图检查为空，结束后自有进程退出。它们不是实际相机、模型或机器人闭环。不把历史检查累计为模型成功率。

既有服务回归由总调度单独授权，用于核实可重建包中的baseline依赖；它不将旧服务算作本轮新增功能。`final_regression_sessions.json`、`baseline_preflight.json`、`baseline_graph.txt`记录该轮所有权与环境；`final_verification.json`核对冻结源码/安装hash和自有测试进程均退出。

独立 build/install 使用 `cmake --build … --parallel 1`，见 build_final.log、build_tf_fix.log。原阶段 colcon 的 `-j28 -l28` 审计保留在上级 build_parallelism_audit.json；不改写历史。最终源码与安装 hash 见清单。

保留的失败：初始空实现 9 例失败见 red.xml；初次配置混用链接签名见 configure_red.log。首次 TF 协议虽返回 PASS，但输出要求 dedicated thread 的 ERROR，已改用采集时刻 BufferCore 即时查询，最终 protocol_final.log 无此 ERROR。外部 CMake 首次未找到包：workspace setup 的 AMENT_PREFIX_PATH 含本包，CMAKE_PREFIX_PATH 缺本包；显式设置已安装 package_DIR 后配置、链接、调用成功，原 consumer_configure.log 保留。

## 真实输入前置

M5 的 `docs/evidence/mainline_m5_20260924/scene03_static_snapshot/region_counts.json` 对实际 26.5 s 同帧 RGB/深度/CameraInfo 与工位范围核对：stride 1 和离线 stride 2 的区域有效点均为 **0**；全图橙色背景不能算目标。当前 head 朝向未覆盖左侧工作台，状态 `VIEW_NOT_READY`。这不是模型失败，也不能用静态估算替代实测。该场次已结束，region 不作为下一场实时事实。

后续主线准备合法观察姿态、新工位绑定和相机/投影健康，确认真实目标点数，再部署 source 与两推理实例。M1 场景提案提交、独立读回、真实 MTC 时效和抓取/放置均未验证。FoundationPose 后端与多形状槽板装配属于后续阶段。

## 复现

先加载 `/opt/ros/humble/setup.bash`、仓库 `ws_robot/install/local_setup.bash` 与 `runs/m3_perception_planning_20260924/install/local_setup.bash`。本包 build 位于 `runs/m3_perception_planning_20260924/build/astribot_s1_manipulation_perception`；目标为 single_box_request_test、single_box_source_protocol_test、pick_planning_test、pick_planning_protocol_test。ROS 重跑前须重新检查会话所有权和 domain。

外部消费者在 `runs/m3_perception_planning_20260924/rgbd_consumer/`，配置时指定任务 install 下 `share/astribot_s1_manipulation_perception/cmake` 为 `astribot_s1_manipulation_perception_DIR`，再单编译构建。本轮不自行提交整个原本未跟踪的包；文件归属与历史 baseline 证据见上级 FILE_SCOPE.md、delivery_manifest.json。
