# 给 M2 的同刻快照与被动观察交接

仅由 M2 在总调度交接的现有会话内执行。本任务准备了命令、待填模板和纯离线组包器，没有操作仿真/GPU。先完整采集并量化，不定义新的覆盖通过阈值。

## 操作者先填写的真实上下文

复制 [real_snapshot_context.template.json](real_snapshot_context.template.json) 到本场次证据目录后填写。模板中的 null 是真实缺项，不是可用默认值；组包器会拒绝未填写内容。相机 K、D、时刻、frame、source_epoch、校准 revision 和 TF **从采集文件原样读取**，无需手抄。

| 字段 | 本场次须提供 |
|---|---|
| session_id / session_json | 本次明确拥有的会话 ID 和实际 session.json 路径 |
| task_id / phase / phase_evidence | M1 本次请求/任务身份、实际阶段或明确标注的观察窗口、M1 反馈/事件文件。仅人工窗口标签时不能声称阶段由系统证实 |
| clock_epoch / clock_epoch_evidence | 同一会话时钟 epoch 及其所有者事件/clock 记录依据。不能用 CameraHealth source_epoch 代替；发生回退/重置后拆分记录 |
| world_frame | 独立 Gazebo transport 的世界坐标名称，例如已核实 `/world/default/pose/info` 时可明确标作 `world/default`，不能填 map/odom 来假定坐标相同 |
| robot_model | 实际 Gazebo 模型名；通常 astribot_s1，但 spawn 开启 allow_renaming，须确认真实名字 |
| model_from_base / model_binding_evidence | 4×4 `T_model_astribot_torso_base`，以及本次实际加载 robot_description/SDF 等绑定文件。当前仓库模型根可为单位矩阵，但不能只凭默认实体名照填 |
| object_model / object_id | 实际 Gazebo 箱体模型名和任务对象 ID；通常 transport_box_01，仍按本场次确认 |
| size_m / object_geometry_evidence | 本次真实模型局部三轴尺寸及模型/场景来源；正常配置为 `[0.06,0.06,0.12]` m，不使用 TCP 四元数作为物体姿态 |
| snapshot_exit_code | 原快照命令实际退出码，组包要求 0；不能因为文件存在就把失败快照当成功 |

上述 evidence 字段和 session_json 是文件路径，绝对路径或相对填写后 context.json 的路径均可。组包器校验文件存在并记录哈希，但不会把操作者声明自动认证为事实；源会话/模型/阶段绑定仍由所有者核实。

## 同一个会话中的采集

由 M2 先加载本次核对过的完整查询环境/overlay，核实 ROS_DOMAIN_ID、ASTRIBOT_SIM_INSTANCE、IGN/GZ_PARTITION、实际 world/model/相机 topic 和校准来源。不得根据另一个会话的环境或旧 latest_sim 发起采样。下列变量均由操作者设为本场次新路径；所有输出目录必须未存在。

最小首张建议 `head_rgbd`；它已有健康链证据。腕部之前 raw 有数据但 CameraHealth 无样本，须由任务所有者确认所需消费者/健康已激活，采集器不代为激活。

**终端 A：在快照之前开始独立真值记录，保持到快照完成。**

```bash
if timeout --signal=INT --kill-after=2s 30s \
  ign topic -e -t /world/default/pose/info \
  > "$M5_TRUTH_FILE" 2> "$M5_TRUTH_STDERR"; then
  printf '%s\n' 0 > "$M5_TRUTH_EXIT_FILE"
else
  printf '%s\n' "$?" > "$M5_TRUTH_EXIT_FILE"
fi
```

这是现有 transport 的有界只读记录，未启动仿真、未发送动作。timeout 到时的 124 等退出码原样保存，不改写成 0；离线适配器只允许丢弃截断末包，内部消息损坏会失败。topic 中 `default` 须与实际 world 一致。本命令本身不保证录到图像时刻的 truth。

**终端 B：在上述 30 秒窗口内、M1 指定阶段执行现有快照入口。**

```bash
if python3 tools/vision/sim_pose_capture.py \
  --camera head_rgbd --seconds 6 --truth-stream "$M5_TRUTH_FILE" \
  --output "$M5_SNAPSHOT_DIR" \
  > "$M5_SNAPSHOT_STDOUT" 2> "$M5_SNAPSHOT_STDERR"; then
  printf '%s\n' 0 > "$M5_SNAPSHOT_EXIT_FILE"
else
  printf '%s\n' "$?" > "$M5_SNAPSHOT_EXIT_FILE"
fi
```

不要加 `--require-target`。正常橙箱只复用 `rgb.png`、`depth.npy`、`camera_info.json`；`mask.png/scene.xyz` 是紫色夹具的衍生物，不能作为橙箱真值。这个入口会覆盖已有目录内文件，所以操作者必须选择不存在的新目录。它最终只保存一帧，同步等待最多约 20 秒；不能把此命令当连续运动覆盖。

本轮新增可选 `--truth-stream` 仅增量读取记录文件的完整消息时间戳，缓存最近 512 个，只从相机已有同步帧中选取匹配项。下一条 header 到达后才认为上一条消息完整，避免文件写到一半时接受残包；时钟重复/回退会失败。历史 Pose_V 约 17 ms、图像 50 ms 的周期不同，原任取最终帧无法保证精确重合；此筛选在原有有界等待内等待交集，不插值或重盖时间戳。它仍不能保证有限窗口一定存在完整实体/健康/TF，离线组包继续核查。省略此参数时原调用方式不变。

保留双方命令、开始/结束时间、退出码、stdout/stderr 和当时会话/模型/时钟身份。不要直接运行 `sim_pose_session.py`、旧 `sim_pose_truth.py` 主程序或 `score_live_pose --run-live` 来替代上述步骤。

## 同场次持续元数据观察

仍使用既有 `capture_m5_observer.py`，在 M1 真实动作前开始，动作后至少保留 20 秒；完整正常窗口默认 300 秒，超过 260 秒的动作须预先安排更长窗口（工具上限 600 秒）。显式阶段 topic/type 由 M1 提供；尚无冻结 topic 时仅传真实所有者事件文件，不猜状态。

```bash
python3 tools/vision/capture_m5_observer.py \
  --topic-config docs/evidence/mainline_m5_20260924/topics.json \
  --session-json "$M5_SESSION_JSON" --output "$M5_CAPTURE_DIR" \
  --seconds 300 --phase-events "$M5_OWNER_EVENTS" \
  --reference "$M5_ACTUAL_MODEL_FILE"

python3 tools/vision/analyze_m5_capture.py \
  --capture "$M5_CAPTURE_DIR" --output "$M5_ANALYSIS_JSON"
```

若实际存在兼容阶段源，可显式增加 `--phase-topic /transport/status --phase-type std_msgs/msg/String`。native 阶段源按真实接口替换。保持 raw 接收间隔 250 ms 预算；不因本轮采样失败调宽。快照/真值记录本身也增加观察负载，所以本场次用于动作证据完整性，不能自动标为无观察扰动的性能 A/B；无重构建对照由总调度另定。

## 离线精确组包与评分

M2 采完填完 context 后即可运行，或把原始文件交回 M5 离线处理：

```bash
python3 tools/vision/prepare_box_roi_snapshot.py \
  --capture "$M5_SNAPSHOT_DIR" --truth "$M5_TRUTH_FILE" \
  --context "$M5_CONTEXT_JSON" --output "$M5_PREPARED_DIR"

python3 tools/vision/score_box_roi.py \
  --sample "$M5_PREPARED_DIR/sample.json" --output "$M5_SCORE_DIR"
```

纯离线组包器只接受 RGB/depth/info 精确同 stamp、同采集健康及非未来且不超过既有 250 ms 的采集结束龄期；TF 是 tf2 在指定时刻的求值，可包含原始 TF 消息插值，不冒充每条 TF 原消息也同 stamp。

独立 Pose_V 必须严格推进且有**恰好同一采集 stamp**的完整消息，同时含两个明确模型。不采用第一帧、最近帧、静止推断或自动插值。无精确匹配时会报最近前后时间戳用于诊断，保存原始文件并标输入缺口。若连续缺同刻 truth，先向总调度报告源采样对齐问题，不能填配置位姿获得通过。

坐标组合为：

```text
T_world_camera(t) = T_world_model(t) × T_model_base × T_base_camera(t)
T_world_odom(t)   = T_world_camera(t) × inverse(T_odom_camera(t))
```

第二项只是保存坐标锚点诊断，不以代数自洽宣称独立校准通过，也不套用旧静止夹具的 2 mm/0.2° 阈值。解析 protobuf 省略数值按 0 处理，含 quaternion w=0；缺完整姿态、非单位四元数、重复模型或时钟回退均拒绝。

输出 `sample.json`、`assembly.json`、`truth_at_capture.pbtxt`。sample 引用原始 RGB/depth 的绝对路径并保存原文件/上下文/依据哈希，须一起保留原始目录。surface_tolerance 固定留空：直接评分可量化有效深度、残差分布、各面支持；遮挡分类及覆盖通过阈值仍 UNKNOWN，不使用检测 80% 替代。

本轮新适配器只做离线合成验证；M2 的首次实际文件组包成功前，不宣称这条采集接线已在真实非 home 场次验证。
