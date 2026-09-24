# 正常 box 的离线 ROI / 三维表面评估契约

2026-09-24 M5 第二独立项，开始约 10:10 +08。仅新增离线评分脚本及证据；没有实时门控、感知库、ROS/GPU/仿真操作，也没有重新运行上一轮 25 项检查。

目标是让下一轮实际正常 box 图像可以计算：完整几何投影 ROI 中的有效深度、与目标表面一致的深度、前方遮挡证据和各可见面的像素支持。成功标准为数据身份/时间/坐标契约明确，合成正反例验证公式，真实输入缺口与未知判据明确。**本轮实现不是实际运动覆盖验收。**

## 最小输入和来源

评分入口为 `tools/vision/score_box_roi.py --sample <sample.json> --output <新目录>`。纯 NumPy 离线脚本，不导入 ROS、不发布话题、不供模型推理。成功退出仅表示评分计算完成，不表示覆盖通过。

完整可执行输入示例见 [合成 sample.json](roi_synthetic/front/sample.json)。它使用正常箱体的尺寸，但图像、位姿、内参和身份均明确为合成，不能当真实证据。

| 字段 | 契约 |
|---|---|
| schema / evaluation_only | `astribot.m5.box_roi/1` / `true` |
| 身份 | session_id、task_id、显式 phase、object_id、camera_id、source_epoch、clock_epoch；不得跨任务拼接 |
| calibration_revision | 采集对应 CameraHealth 的数字版本，不能从日期或当前配置补猜；旧 box 链 manifest SHA256 是另一种 epoch，可作为额外来源字段保留 |
| camera | 实际 CameraInfo K、D、width/height、三流共同光学 frame；RGB/Depth/Info 精确同 stamp；深度类型为米制 `optical_z_m` |
| files | RGB 文件、米制二维 `depth.npy`，可选独立 `visible_mask.npy`；路径相对 sample.json，文件 SHA256 写入结果 |
| world_from_camera | 采集时刻 `T_reference_optical` 4×4 刚体矩阵、reference frame、optical child frame、stamp、clock epoch、标定版本 |
| box_truth | 独立实际 `T_reference_box`、同刻 stamp/clock epoch/reference frame、object_id、box 三轴尺寸；来源为 Gazebo 实际 pose 或人工三维标注 |
| visible_annotation（可选） | 人工或 Gazebo 实例标签的可见目标二值 mask，含 annotation_id、来源、object_id、stamp/clock epoch、optical frame。不能用待评估算法自身的框/HSV/深度筛选结果冒充真值 |
| depth_policy | 深度有效范围上下界及出处，开区间。现有正常 box 模板使用 `0.08 < z < 5.0 m`；这是有效深度定义，不是相机全部工作距离或覆盖通过阈值 |
| surface_tolerance_m / source | 可为空；若分类目标表面一致/更近/更远，须提供非负深度容差及明确依据。合成样例的 5 mm 只是公式验证灵敏度，不是项目验收值 |

有独立二维可见 mask 时可不提供 box_truth/TF，仅评分标注 ROI 有效深度，三维结果为 `NO_BOX_TRUTH`。没有任一独立 ROI 来源则拒绝评分，不能用全图或中心裁剪回退。双目 RGB 相机可做后续二维人工标注分析，但当前没有其独立深度，不能伪造给这个 RGB-D 评分入口。

三维路径严格要求同一采集时刻的真值与 TF；本版不做最近样本匹配或自动插值。若 100 Hz 物体 pose 没有恰好同 stamp，后续应由采集所有者保留前后实际样本并明确插值契约；目前标输入未就绪。`world`、`map`、`odom` 不能被默认视为相同 frame，别名也必须有明确会话依据。

矩阵含义为 `p_reference = T_reference_child * p_child`，位置单位米。box 坐标原点为箱体中心，三轴尺寸沿物体自身坐标。光学坐标为 x 向右、y 向下、z 向前。摄像机内参采用消息中的实际 K，不套用 YAML 标称主点。当前只支持零畸变 pinhole 输入；裁剪或去畸变后须保存调整后的 K、图像变换和独立来源，不能混用原图参数。

RGB 被保留并哈希用于复核，评分器不解码 RGB，也不从颜色生成真值。完整 payload、CameraInfo 和时刻关系仍须由采集记录证明；JSON 声明本身不能认证采集过程。

## 现有正常 box 数值的实际含义

来源：`ws_robot/src/astribot_s1_transport/config/warehouse_transfer.json`、`astribot_s1_transport/rgbd.py` 与 `ros_backend.py`。

- 正常 box 为 `transport_box_01`，尺寸 `[0.06,0.06,0.12] m`。初始 Gazebo spawn 是单位姿态；配置中的 `[0.5,0.5,0.5,0.5]` 是 TCP 抓取目标，**不是 box 姿态**。携带时必须读取实际物体姿态。
- 模板检测候选连通域至少 30 px，有效点至少 30、有效比例至少 0.8；分母保留缺深度像素。这些定义在检测出的组件内，不能直接移植成完整几何投影 ROI 的“80%覆盖通过”。
- 模板可见表面逐轴 1%/99% 范围须在 `0.25×expected` 至 `expected+0.025m`；不能将它解释成物体三维表面积覆盖率。
- 放置位置容差 25 mm、落稳漂移 5 mm 都不是深度残差容差。现有项目未定义 ray-box 深度容差、遮挡通过率、各相机角色可见比例或整体三维覆盖阈值。

因此所有评分均保留 `coverage_acceptance=UNKNOWN_THRESHOLD`。即使所有像素与合成表面完全一致，也不能升级为实际任务覆盖通过。

## 公式与输出

评分先计算：

```text
T_optical_box = inverse(T_reference_optical) × T_reference_box
ray(u,v) = inverse(K) × [u,v,1]
```

像素坐标为 ROS 反投影使用的整数 u/v；ray 的 z 分量为 1，不单位化，所以 ray-box slab 首交点参数直接是光学 Z 深度，不能当欧氏距离。按物体自身六个面编号 x−/x+/y−/y+/z−/z+ 保存首交面。

投影 ROI 为图像中与实际 box 相交的像素中心射线集合，分母包含无效深度与遮挡像素。`NO_PIXEL_RAY_HIT` 只表示没有采样射线击中，可能是画外、后方或物体小于像素，不能直接判定物体在视场外。该离散采样不是精确投影面积，也不是整箱表面积。

对 ROI 内有效深度计算 `residual = observed_Z − expected_first_surface_Z`，保留 min/max、P05/P50/P95、绝对残差 P95。给定明确容差 ε 后：

- `consistent`：`abs(residual) ≤ ε`，表面深度一致；单凭此条件不证明物体身份。
- `closer`：`residual < −ε`，前方表面证据。在 pose/时间/标定正确且目标确实存在的前提下，支持遮挡解释；错误真值也可能产生相同结果。
- `farther`：`residual > ε`，后方表面或目标/模型/位姿不一致，不能归类成前方遮挡。
- `unknown`：NaN/Inf 或超有效深度范围，不从分母删除，也不等同于遮挡。

没有 ε 时仍输出 ROI 有效深度和残差分布，分类状态为 `UNKNOWN_TOLERANCE`。有人工可见 mask 时，单独保存其有效深度及与几何投影不一致的像素数；“投影内未标注”不能自动解释成某种遮挡原因。

输出 `report.json`、`pixel_maps.npz` 和输入 `sample.json` 副本。像素图含投影 ROI、预期 Z、首交面、有效深度、残差与明确容差下的分类；可逐像素复核。各面只统计相机可见首交面的像素支持，**不计算隐藏背面、连续运动视角融合或整箱三维表面积覆盖率**。

## 现有采集能复用什么

| 现有入口/证据 | 可复用 | 缺口与限制 |
|---|---|---|
| `tools/vision/sim_pose_capture.py` | 精确同 stamp 的 RGB/Depth/Info，米制 depth.npy，实际 K/D/P/R，采集时刻 base/odom→camera TF、健康/revision | 没有正常 box 的实际 pose；紫红 HSV mask 经过有效深度筛选和腐蚀，不能作橙 box 真值或无效深度比例分母 |
| `/simulation/transport_payload_pose` | 已有 PoseStamped 桥，正常模型 PosePublisher 配置 100 Hz | 下轮须落盘实际 pose source stamp/四元数/frame，并与图像对应；末尾放置 xyz 不能反推 earlier frame |
| kinematic_attachment/state | 记录 stamp_ns、actual_world_xyzw | 只能用 actual 字段复核；expected_world_xyzw 是期望值；需明确来源独立性与物体绑定 |
| `score_live_pose.py` / `sim_pose_truth.py` | 可参考采集时刻 TF、world/odom 锚点、独立真值保存方式 | 当前静止紫色夹具专用，不能直接用在运动正常箱体 |
| `runs/normal_grasp_20260923/view_capture.npz` | task02 单帧 RGB/depth/K/map←optical TF；关联诊断 stamp 426600000000 | 无同时刻独立物体 pose、D、标定版本。不能拼 task08 的 stamp 965100000000 或录像 task01 的 stamp 110550000000 |
| 成功 task08/录像 task01 events | 正常阶段、检测中心/可见尺寸/点数/像素框 | 没有对应整帧 payload/CameraInfo/TF/独立 pose；检测值是被评估输出，不是真值 |

实际 Gazebo pose 桥在 `transport_support.launch.py:67`；模型 PosePublisher 在 `ros_backend.py:1043`；`tools/sim/probe_payload_consistency.py` 已有源 pose stamp 查 TF 的用法。新评分器没有直接消费任一运行时通道，所有真值文件只在独立评估目录使用。

下一轮最小落盘增量是：在现有精确三流快照旁保存同一 stamp 的 `/simulation/transport_payload_pose` 实际完整姿态和显式 reference↔optical TF。相机/物体实体、模型尺寸、phase/请求身份、源时刻和标定版本同时落盘。此前 M5 40-topic 元数据工具不保存像素，不能替代这个快照包；现有旧快照也不能凭缺失字段默认值转换成合格输入。本轮未改采集器或启动此采集。

## 离线验证与复现

运行新增检查：

```bash
python3 -m unittest discover -s tools/vision -p 'test_score_box_roi.py' -v
python3 tools/vision/score_box_roi.py \
  --sample docs/evidence/mainline_m5_20260924/roi_synthetic/half_occluded/sample.json \
  --output /tmp/m5_box_roi_new_result
```

第二条输出须为不存在的新目录。合成证据已保存，不需要重复生成或覆盖。最初未实现时失败日志为 `roi_red_test.log`；最终结果和哈希见 `roi_validation.json`。

| 合成样本（正常 box 尺寸，模拟 36 px 投影） | 有效深度比例 | 与目标表面一致比例 | 更近表面比例 |
|---|---:|---:|---:|
| 正面完整目标 | 1 | 1 | 0 |
| 半幅前方遮挡 | 1 | 0.5 | 0.5 |
| 三分之一无效深度 | 0.667 | 0.667 | 0 |
| 未提供容差 | 1 | 未分类 | 未分类 |
| 仅独立可见 mask | 1 | 无三维真值 | 无三维真值 |

全部保留覆盖阈值未知。其他测试覆盖转动箱体/面编号、坐标变换方向、轴向深度与距离区别、后方深度、画外/后方/亚像素物体、重复/不一致身份时刻、未知标定/畸变/单位、非刚体矩阵、缺真值与输出防覆盖。上述均为合成/离线证据，没有实际运动或遮挡场景的验收结论。
