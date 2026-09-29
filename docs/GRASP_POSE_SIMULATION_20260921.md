# GraspNet 与物体 6D：实现与仿真验证记录

2026-09-21。本次实现真实 GraspNet 推理服务和**已知 CAD 物体**的完整 6D 姿态服务。运行时采用 C++；Python 用于模型准备、启动与验证。物体姿态与夹爪抓取姿态是独立结果。当前已打通躯干 RGB-D 的连续推理，**尚未全场景放行**。

后续相机安装修正单独记录于 [六相机参考安装与腕部外参](CAMERA_REFERENCE_MOUNTS_20260921.md)：依据用户照片解决头部嵌入和腹部/腕部安装问题。下文保留原推理验证条件和结果，不将历史推理指标视为新外参下的回归验收。

## 已落地的数据链路

```text
导航仓库 Gazebo RGB-D + CameraInfo
  → ros_gz_bridge（本地 16 MiB 共享内存）
  → C++ camera_health（完整帧配对、时效、源 epoch、标定版本）
  → 调用者提供分割目标 PointCloud2 + 同次采集快照
  → C++ manipulation_perception_server
      ├─ ComputeGrasps → 独立 LibTorch C++ GraspNet worker → 抓取候选
      └─ EstimateObjectPose → C++ PPF/ICP + CAD 可见性 → 物体 6D
  → 带来源/模型/场景/包络版本的观测或建议
```

本次输入分割使用明确标注的 HSV 颜色测试夹具；不是 YOLO 实例分割验收。真值由独立记录器读取，只供评分，不进入算法。仿真复用导航仓库与同一机器人资产，隔离域 87、partition `astribot_grasp_pose_20260921`；运行范围为传感器与物理，不宣称导航或机械臂抓放闭环通过。

新增包：`astribot_graspnet_runtime`、`astribot_object_pose_core`、`astribot_s1_manipulation_perception`。`astribot_perception_msgs` 更新为 0.3，ComputeGrasps、EstimateObjectPose 和观测消息携带完整快照字段。消息 ABI 已变化，构建脚本同时重建原有视觉与 manipulation 消费者。

## 模型与算法

GraspNet 固定官方 baseline 提交 `280c215129f759ed8649cb4e89fc5dfee55f4f80`。checkpoint SHA256 为 `60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868`。官方下载配额不可用时使用两个内容一致的镜像，来源、哈希与许可证完整保存在 [模型证据](evidence/grasp_pose_sim_20260921/graspnet/README.md)；哈希一致不等于发布方签名认证。

采用可脚本化 PointNet2 兼容算子，导出 TorchScript，由 LibTorch C++ 执行。已修复最远点采样并列距离的 CUDA 归约顺序差异，最终使用 **v2** 模型。CPU/CUDA 各 10 项算子测试通过；官方输入和变动输入共 4 次 eager/C++ 输出比较最大绝对差异为 0。没有编译原始 CUDA 扩展，因此不宣称与其完整逐位等价。详见 [v2 模型记录](evidence/grasp_pose_sim_20260921/graspnet/v2/README.md)。

物体姿态采用已知 CAD 全局 PPF 搜索、600 点 ICP 精配准、精确 KD-tree 和注册 CAD 的可见性检测。返回完整平移和旋转，不以身份四元数代替不可观测朝向。固定门槛包含可见模型覆盖率 0.75、场景覆盖率 0.65、RMSE 5 mm、至少两个有足够支持的非平行法向，以及竞争姿态歧义检查；没有逐场景放宽。未知模型、连续旋转对称或证据不足的结果拒绝。未标定协方差以大对角线显式表示，不能把残差当作融合协方差。

## 双 RGB-D 新鲜度修复

发现并修复两个独立问题：

1. **传输容量不足**：Fast DDS 默认 512 KiB 共享内存小于本次 640×360 RGB 的 691200 B 和深度的 921600 B。修复前 10 秒内每路 CameraInfo 100 帧，RGB 仅 51/54 帧、深度 65/73 帧。仅对本次 bridge 应用 16 MiB 配置后，两路都收到 100 组完整同戳数据。
2. **异步到达误判**：原健康检查比较各流“最新一帧”，下一帧 RGB 先到时会短暂错误判断失步。现在各流保留最多 32 条元数据，仅配对未消费过的完整 RGB/depth/info；在途帧不续期，也不推翻仍在有效期内的完整快照。0.25 秒时效、0.03 秒时间差、8 Hz 门槛不变。14 项健康测试通过。

真实推理负载下连续监测 60 秒，仿真时钟推进 59.298 秒：两路各 **593 组**完整同戳 RGB/depth/info，头、躯干健康检查均 **1199/1199 OK**。这是持续数据传输和时间健康证据，不证明头相机看到有效场景几何。详见 [前后对照](evidence/grasp_pose_sim_20260921/health_sync/under_inference_summary.json)。

默认本机模式禁用 builtin transport，使用 loopback UDP 与有界 SHM；LAN 模式继承调用方网络环境，避免屏蔽跨机 RViz。显式 profile 可覆盖。8 项离线 launch 契约检查通过；未做跨机吞吐测试。

## 连续实时服务结果

来源为实际运行中的躯干相机，目标静止、clear 视角；每次使用新采集时间。场景/包络版本 1 是明确的推理测试上下文，不能当作真实 MoveIt 场景版本或执行授权。

| 接口 | 成功数 | 完整请求耗时中位数 | 最小—最大 | 最大发送前数据龄期 |
|---|---:|---:|---:|---:|
| GraspNet GPU | 20/20 | 1.210 s | 1.175—1.277 s | 0.138 s |
| 已知 CAD 6D | 10/10 | 1.598 s | 1.572—1.633 s | 0.156 s |

GraspNet 每次产生 713 个原始 proposal，剔除 2 个非正分 proposal 后返回排序前 32 个。原始分数与有界排序值分别记录，后者不是概率。所有候选都保持 `collision_checked=false`、`collision_free=false`，须再经 MTC/MoveIt 碰撞、IK 和任务准入。服务没有控制执行器，也没有自称通过场景点云碰撞过滤。

结果保留采集时间，感知快照有效期最多 5 秒；这不改变下游执行新鲜度要求。超时、取消、相机 epoch/版本变化、模型变更、输出非法或过期都会清空可用结果。

[连续请求原始记录及输入](evidence/grasp_pose_sim_20260921/live_actions_v2/summary.json)。另采 5 帧 clear 姿态并独立记录 555 条 Gazebo 真值，采用每帧精确采集时刻 TF，5/5 满足 20 mm / 10°：最大平移误差 **2.646 mm**、旋转误差 **0.5811°**。目标和机器人在该窗口实测静止；不宣称动态精度。旧报告缺少 TF 时拒绝补算。详见 [独立评分说明](evidence/grasp_pose_sim_20260921/live_actions_v2/SCORING.md)。

再切换远距离与遮挡场景，每次取新帧：far 姿态 **3/3 成功**，occluded **3/3 明确拒绝**，没有有效位姿泄出；恢复 clear 场景。这里验证服务行为，不能将其与另有真值的离线精度统计混算。

真实 worker 的 **12/12 故障与恢复检查**通过：错 frame、旧采集时间、错误标定/场景/包络/epoch、非有限点、稀疏点、50 ms 推理超时、结果过期、取消、随后正常恢复。失败或取消均无候选返回。[故障注入记录](evidence/grasp_pose_sim_20260921/live_actions_v2/real_worker_faults.json)。首轮验证脚本意外修改共享缓存 Header，已修复为请求副本；该轮保留为 `invalid_harness_alias_run.json`，不计入服务验收。

## 七场景离线实测基线

输入是已保存的实际 Gazebo RGB-D，不是合成点云。独立真值以采集时刻 TF 和 Gazebo 模型状态建立；姿态验收标准统一为 20 mm / 10°。

| 场景 | v2 结果 | 平移误差 | 旋转误差 |
|---|---|---:|---:|
| clear | 通过 | 2.093 mm | 0.435° |
| far | 通过 | 2.942 mm | 0.577° |
| near | 拒绝 `no_acceptable_pose` | 不输出有效估计 | 不输出有效估计 |
| tilted | 拒绝 `no_acceptable_pose` | 同上 | 同上 |
| yawed | 拒绝 `no_acceptable_pose` | 同上 | 同上 |
| occluded | 拒绝 `no_acceptable_pose` | 同上 | 同上 |
| close | 拒绝 `no_acceptable_pose` | 同上 | 同上 |

因此是 **2/7 放行，5/7 拒绝**，不能表述为七场景全部姿态成功。已放行结果没有超过规定误差的假阳性。剩余原因分析与有界搜索结果另附；不能靠隐藏机器人网格、移动虚构外参或降低覆盖门槛取得通过。

[完整 v2 评分](evidence/grasp_pose_sim_20260921/simulation/pose_results_visibility_v2/summary.json)；[输入快照清单](evidence/grasp_pose_sim_20260921/simulation/dataset_manifest.json)。

有界扩展搜索把候选预算从 32 增至 256，结果仍为 2/7 通过，其余无候选同时满足全部门槛；默认算法没有改变。五个拒绝场景的最优候选可见覆盖分别为 near 41.5%、tilted 33.2%、yawed 51.0%、occluded 32.2%、close 71.0%，均低于 75%；tilted/close 另只有一个受支持法向。原始深度投影显示额外前景遮挡与缺失深度，但这些诊断以候选为假设，不能当作真值遮挡率或已准确定位。详见 [逐场景拒绝诊断](evidence/grasp_pose_sim_20260921/simulation/rejection_diagnosis_v2/README.md)。

## 验证与后续边界

独立 overlay 回归：Action/输入/worker **28 项 GTest**，原视觉组件 **48 项 GTest**，原抓取门控 **10 项 GTest**；物体配准 **23 项 CTest**，另保留两个原过滤器 CTest。Colcon 汇总 120 项包含 GTest 的 CTest 包装项，不能与上述数字再次相加。[构建测试记录](evidence/grasp_pose_sim_20260921/build_and_tests_final.json)。

[独立最终代码复审](evidence/grasp_pose_sim_20260921/review_final.md)未发现剩余可复现 P1/P2。桥接容量修复在当前独占会话通过进程替换实测；新的整栈启动入口做了参数/配置检查，本次未为它额外重启整套仓库。LAN 保留配置的检查不能代替跨机网络验收。

尚未整体放行的项目：

- 头相机原始深度无效，网格审计显示视线撞到自身父级网格；躯干相机部分视角同样被本体遮挡。真实标定待硬件可用后核对，本次未猜测修改外参。
- 被遮挡或只剩单平面的目标不能可靠输出完整 6D。需要取得额外有效视角；当前服务保持明确拒绝。
- YOLO 实例分割到目标点云的端到端接线、世界物体账本、MTC 候选选择和物理抓放不属于本次推理验收。VLA 和真机未测试。

## 复现入口

1. `bash tools/vision/build_manipulation_perception.sh /absolute/output/ros_ws` 同步构建消息与所有消费者。
2. `tools/vision/graspnet_setup.sh` 准备隔离依赖、固定权重、v2 算子与 C++ worker；完整路径与 hash 见 [运行产物清单](evidence/grasp_pose_sim_20260921/graspnet/v2/runtime_paths_v2.json)。
3. `tools/vision/sim_pose_session.py` 为本次专用实例取得锁并启动导航仓库传感器场景；默认读取 `runs/grasp_pose_sim_20260921/ros_ws/install`。当前会话在用时禁止重复启动。正式日志为 `runs/grasp_pose_sim_20260921/session/session.log`，替换节点日志及 PID 身份在同目录。
4. 来源域、partition 与 overlay 使用会话 `query_env.sh`；Action 配置为 `runs/grasp_pose_sim_20260921/action/inference_v2_final.yaml`。默认包内配置版本为 0，故意拒绝未配置请求。
5. `validate_inference_actions.py` 连续采集并请求；`verify_camera_sync.py` 记录健康；`sim_pose_evaluate.py` 读取固定点云独立评分。评测器调用前清除旧输出，禁止复用旧成功结果，5 项回归通过。

本次保留早期失败、v1 模型和基线证据，最终模型与报告单独命名。没有覆盖或清理其他任务的仿真，也没有提交工作区中已有的无关修改。
