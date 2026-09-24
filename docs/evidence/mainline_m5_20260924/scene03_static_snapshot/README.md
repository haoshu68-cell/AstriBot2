# 第三场静态 RGB-D 快照：组包成功，当前视图无工位有效点

原始场次：`/home/yjh/WorkSpace/astribot_validation/M1_scene_prepare_20260924_1024/first_scene03`。本目录仅离线分析已有文件，未启动 ROS、Gazebo、GPU 推理或发送动作。

## 结论

- **同刻组包成功**：RGB、depth、CameraInfo、CameraHealth 的采集时刻，以及 TF 求值、独立 Pose_V 均匹配 **26.500 s**。图像 640×360，实际 K 的焦距约 374.5375、主点 320/180，D 全零；标定修订 `2026092101`。未套用静态配置内参。采集结束龄期 84 ms，原采集命令退出 0 的依据见 `context_audit.json`。
- **当前视图 NOT_READY**：仅由工作台构造的区域内，真实有效深度点为 **0**；全分辨率和 stride=2 对照均不能满足 ≥2048 点。不是抽稀造成的不足，不复制或补点。
- **evaluation-only 箱体 ROI 为 0**：精确同刻独立箱体真值的投影结果为 `NO_PIXEL_RAY_HIT`。该分支只用于离线覆盖诊断；目标实体位姿/尺寸未参与下面的工作台区域筛点，也未供推理使用。
- 本场采集在动作请求之前。后续 Action 因 `MTC_START_OUTSIDE_PLANNING_MARGIN:astribot_arm_left_joint_4` 被拒绝；本证据不表示 PREGRASP、Hold、抓取或运动覆盖通过。

## 真实点数

计算口径：有限且 `0.08 < optical Z < 5` 米；使用实际 K 投影，再用 **26.500 s** 的 `base_from_camera` 转入 `astribot_torso_base`，按工位区域闭区间筛选。stride 网格从 `(u,v)=(0,0)` 起。

| 指标 | 全分辨率 / stride 1 | stride 2 对照 |
|---|---:|---:|
| 图像网格像素 | 230400 | 57600 |
| 全图有效深度点 | 94184 | 23354 |
| 工位区域内有效点 | **0** | **0** |
| 全图橙色 HSV 有效像素 | 72824 | 18106 |
| 工位区域内橙色 HSV 有效像素 | **0** | **0** |

全图橙色统计仅采用项目 HSV 范围 `[5,100,45]..[30,255,255]`，未清理连通域，也没有目标身份含义。原始 RGB 显示前方货架和纸箱；不能把这些区域外像素当成目标箱体。

工作台区域在当前相机光学系的 Z 范围为 **[-0.0573153352, 0.0426870652] m**，整个区域都低于有效深度下限 0.08 m。相机光轴近似朝 base +X，工位在左侧。全图实际有效深度则为 **[2.6694524288, 4.9051370621] m**。因此当前朝向不提供工位有效点；增加采样密度不能解决这个视角问题。

两个 stride 是纯离线对照，不能视作实际在线 projector 参数；当前专用 projector 源配置已为 decimation=1。任何后续视角/动作调整由场次所有者按原有准入实施，本次没有发出此类请求。

## 来源与时间边界

工位区域来自同场 `pick_station_region.json`：scene 文件 SHA、嵌入的 station 对象、0.1×0.1 m 桌面向上延伸 0.3 m 后的八角点及边界均核验一致；没有使用目标位姿、目标尺寸或 pick_xyz。station 的 scene header stamp=0，只能证明静态来源，不能宣称区域与图像同刻或运动期间始终有效。

`world_poses.log` 共解析 **825 条完整消息**，时刻严格递增，范围 **20.163–33.990 s**；匹配第 377 条（从 0 计）为 26.500 s，包含实际实体 `astribot_s1` ID 136 与 `transport_box_01` ID 334。没有截断尾包；实际记录窗口不足请求的 24 秒，不以请求时长冒充实采时长。

模型根绑定由本场实际 robot_state_publisher URDF 和转换后 SDF 核验，`model_from_base` 为单位矩阵。世界相机变换使用同刻机器人真值 × 模型根绑定 × 同刻 base-camera TF，不假定 world=odom。TF 是 tf2 在采集时刻的求值，可含原 TF 消息插值。

clock epoch 标签关联本场会话；前后 geometry clock_epoch 都为 0，所记录真值段严格推进。这个证据不覆盖记录段外的全部历史。`task_id=M5_first_scene03_static_snapshot` 是离线快照任务；另行保存之后 Action 的真实 task_id，不把它误写成采集时已执行的阶段。

覆盖阈值仍为 `UNKNOWN_THRESHOLD`；零 ROI 的有效比例为空，不能写成 0% 或 100%。表面误差没有样本，完整三维表面覆盖 `NOT_MEASURED`。这里确认的是当前视角无可用工位点，不是识别模型失败或整条算法链验收。

## 文件与复现

- `prepared/sample.json`、`prepared/assembly.json`、`prepared/truth_at_capture.pbtxt`：独立真值评估组包，**仅评估使用，不作为 Request 输入**。
- `roi_score/report.json`、`roi_score/pixel_maps.npz`：离线 ROI 评分。
- `region_counts.json`：不使用目标真值的区域点数、筛点口径及原始文件哈希；M3 可引用此文件作为当前视图 NOT_READY 的证据。
- `context.json`、`context_audit.json`：会话、阶段、clock epoch、实体绑定及退出码依据。
- `verification.json`：执行与输入哈希核验。

在项目根目录运行 `python3 docs/evidence/mainline_m5_20260924/scene03_static_snapshot/reproduce_region_counts.py` 可重算点数。组包与 ROI 评分使用 `tools/vision/prepare_box_roi_snapshot.py` 和 `tools/vision/score_box_roi.py`，参数见 `verification.json`；它们要求新的输出目录，复跑时另选目录，保留本次结果。

本轮未修改运行时代码。一个独立只读分析任务得到相同点数及相同区域光学 Z 范围。没有新仿真、真机、持续运动或推理精度验收。
