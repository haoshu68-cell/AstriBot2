# READY 场次单帧：2466 个真实点满足点数门槛

原始目录：`/home/yjh/WorkSpace/astribot_validation/READY_profile_20260924_1133/ready_scene01`。本次只做离线组包、点数/ROI 计算与录像证据检查；没有启动 ROS/Gazebo/GPU、发送动作或修改运行时代码。

## 结论与范围

**当前 READY 视角的这张冻结帧，按专用 decimation=1 的几何与分割流程，可得到 2466 个实际有效点，满足 ≥2048 点及 ≥80% 有效率。** 独立分析任务复算一致。这里证明单帧数据量足够，不代表 live Request 准入、GraspNet 精度、抓放动作或持续覆盖通过。

本场采用 `transport_ready` 初始关节配置；本场真实初态验证见源目录 `fixtures/result.json.initial_ready`。它是独立正常抓放条件，**不是零位→READY 运动修复**。本张快照采于动作请求之前。

## 数据绑定

- RGB/depth/CameraInfo、CameraHealth 采集时刻、TF 求值与独立 Pose_V 精确匹配 **28.300 s**。图像 640×360，实际 K：fx/fy≈374.5375，cx/cy=320/180，D 全零；未套用静态 profile 主点。
- 标定修订 `2026092101`；相机 source epoch 原样保存在组包和点数报告。**43 ms 是采集结束时的历史龄期，不表示冻结文件现在有效**。
- region frame：`astribot_torso_base`；revision：`worktable_region_v1:41a07571b298e166ade98e2789cd4fe043049f58d249724bb92eb4ca9790e146`。
- table-only region 来自本场 `pick_station_region.json`；scene SHA256 为 `83e40641ebf997ff5c8a6e74ddf51d00ff9cdab373a4fa911b2dc45a385dce52`。核对 station 原对象、桌面 XY 支撑范围向上 0.3 m、水平扩展 0，以及八角点/AABB 均一致。没有使用箱体位姿、尺寸、pick_xyz 或上一场数值。
- station 的 scene stamp=0，区域只能作为本场静态场景范围，不提供同刻或运动场景认证，也不提供实时身份租期。

## M3 点数与有效率门槛

`request_point_gate.json` 是应交给 M3 的结果。`region_counts.json` 只是更早一级的区域统计，不能以所有区域点代替选中目标候选点。

| 层级 | 本帧数量 |
|---|---:|
| 全图有效深度点 | 217269 |
| 工位区域内所有颜色有效点 | 3422 |
| 全图橙色 HSV 有效像素 | 81468 |
| 工位剪裁后的橙色候选 mask 像素 | 2466 |
| 面积 ≥30 的 8 连通域 | **1** |
| 选中连通域面积 | **2466** |
| 选中域真实有限 XYZ 点 N | **2466** |
| 保留在分母的无效深度像素 | **0** |
| 有效率 N/面积 | **100%** |
| N≥2048 | **满足，多 418 点** |
| N/面积≥0.8 | **满足** |

离线复现当前 C++ 点数路径：按实际 K 和 CPU projector 计算方式产生 FLOAT32 XYZ → HSV `[5,100,45]..[30,255,255]` → 同采集时刻 station-camera TF 下的工位 AABB 筛选 → 对无效深度橙色像素保留与区域相交的 `[0.08,5]` 光学 Z 射线 → 8 连通域 → 恰好一个面积 ≥30 的候选 → 统计有限 XYZ、严格 `0.08<Z<5`、`|X|/|Y|≤10` 的实际点。

选中域 `xywh=[310,200,38,72]`。12000 点上限后仍为 2466。没有 voxel、形态学、复制、补点或模型重采样参与计数；后续 GraspNet 重采样到 20000 不能算新增观测点。

对同一选中域按 `(u,v)=(0,0)` 起步长 2 取子集，只剩 **618/618**，有效率仍 100%，但不满足 2048。它仅为抽样影响对照；当前 M3 要求 decimation=1 的全图逐像素布局，stride2 不能冒充合法 M3 输入布局。

`selected_measured_pixels.npz` 保存由原深度得到的真实 UV/XYZ 和选中 mask，仅供离线复核。未用 GT 选择这些像素。

## 独立 GT ROI 评分

独立 truth 日志解析出 **1397 条完整消息**，严格推进范围 **21.721–45.087 s**，无截断尾包；第 395 条（从 0 计）为采集时刻 28.300 s。组包使用精确同刻机器人实体、根绑定及 base-camera TF 求世界相机变换，不假定 world=odom。目标实体真值只送入 `prepared/` 与 `roi_score/` 的独立评分分支，不作为 M3 算法输入。

投影 ROI 为 2466 像素，有限深度 2466，但**有限深度不是目标表面吻合证明**：

- 独立筛出的候选 mask 与 GT ROI 相交 **2414** 像素，双方各有 **52** 像素不重合，不能因总数相同就称完全一致。
- 光学 Z 残差（实测减投影首表面）：P50≈**0.887 mm**，P95≈**1.042 mm**；绝对残差 P95≈**12.409 mm**，最大正残差≈**1.715 m**。该最大异常像素 `(343,202)` 不属于独立选中连通域。
- 表面容差没有冻结，遮挡/表面吻合判定为 `UNKNOWN_TOLERANCE`，覆盖验收仍为 `UNKNOWN_THRESHOLD`，完整三维表面覆盖 `NOT_MEASURED`。本次不更改阈值或把边缘差异归咎于某一已证原因。

## 录像与动作证据

真实录像：`/home/yjh/WorkSpace/astribot_validation/READY_profile_20260924_1133/ready_scene01/first_stage.mp4`；对应 `first_stage.json`、`first_stage.jpg`。首尾帧可解码，216 帧、10 fps、播放时长 **21.6 s**；采集记录墙钟窗口 **35.6958 s**。这是收到帧后的固定帧率编码，不能把播放秒数当现实性能时间或仿真时间。

| 视频帧范围（右端不含） | 播放秒 | 记录阶段 |
|---|---|---|
| [0,96) | [0,9.6) | WAITING_FOR_TASK |
| [96,97) | [9.6,9.7) | WAITING_FOR_NAVIGATION_REVOCATION_READBACK |
| [97,206) | [9.7,20.6) | PLANNING_COMPLETE_OPERATION |
| [206,216) | [20.6,21.6) | MTC_SCENE_CHANGED |

父 Action 于 **34.006 s** 提交；所有者报告规划于 52.314 s 成功，随后因 `MTC_SCENE_CHANGED` 终止且没有 child submission。1028 条左臂 JTC 反馈覆盖 **31.57–53.23 s**，最大观测关节变化约 `2.30e-17 rad`、最大观测速度约 `5.43e-14 rad/s`，与未执行臂轨迹一致。资源状态中的 `EXECUTING` 字样不作为运动证据。此录像没有可验收的实际臂动作片段，也不覆盖零位→READY。

所有者回执 `resources_released=true`、清理完成；录像记录器因 KeyboardInterrupt 收尾，MoveGroup 卸载 `-11` 由所有者单列。可解码与进程退出均不能升级成全节点干净退出。详见 `video_scope.json` 和原场次结果。

## 未验证项与复现

本次未采集/验证真实专用 PointCloud2 布局、ProjectionHealth 的 processing epoch/有效期、当前单实例身份与工位租期、各接收 steady deadline。故 `request_live_admission=NOT_EVALUATED`；冻结帧不得当当前有效帧送入实时 Request。

复算点数：在项目根目录执行 `python3 docs/evidence/mainline_m5_20260924/ready_scene01_static_snapshot/reproduce_region_counts.py`。组包/评分命令与所有来源/产物哈希见 `verification.json`；组包/评分要求新的输出目录，复跑另选目录以保留本次证据。

交给 M3：`request_point_gate.json`、`region_counts.json` 和本说明。`prepared/`、`roi_score/`、`roi_comparison.json` 为 M5 的 GT 独立评分，不能作为 Request 选点依据。验证层级为真实仿真快照的离线分析，不是实时感知推理或运动闭环验收。
