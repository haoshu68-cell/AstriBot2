# 暂缓问题与难点清单

本清单不是验收通过记录。采用 [全窗口任务推进规则](TASK_EXECUTION_POLICY_20260923.md)，更新时保留原始失败与各次排查时间。共同清单由搬运仿真窗口维护，其他窗口回传自己的问题条目。

## SCAN-FRESHNESS-01：策略使用的扫描过期

- 最新覆盖：用户随后明确要求定位修复，本项于16:35:59重开一轮，17:35:59检查点；下列暂缓记录保留作历史。新证据与验收状态见 [修复记录](SCAN_FRESHNESS_RETURN90_REPAIR_20260923.md)。
- 当前状态：`deferred_after_checkpoint_partial_repairs_verified`。C++ TF修复后R1两次返程通过；R2/R3发布门禁和两段期限不续期的采集wire审计通过；R3还修复了控制器临时过期永久锁存。但R3最终返程仍因120.55s起步对齐超时失败，不能以R1成功记整体放行。17:33:54仿真已安全清理，本问题不再续开排查，17:35:59同一检查点及历史投入保留。
- 历史状态：`deferred_by_user`；此前用户要求先继续其他任务。历史累计排查时间待从各窗口记录归并，不能视为零。上一轮 A4 恢复始于 15:51:54，该时间不是本问题累计起点。
- 现象：返程 90°任务超时；同场景独立观测的源扫描新鲜，但策略使用扫描出现过期。
- 本轮证据摘要：返程首段到位成功，返程任务 180 s 超时；扫描独立观测 P95 58 ms/max 69 ms，策略扫描年龄 P95 476 ms/max 788 ms，510 次 STALE。安全停车、资源释放及会话清理已完成。指标以原始批次文件为准，不将不同阶段分位数相加。
- 原始证据根目录：`/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/I0_2_navigation_20260923_155154`，其中返程批次的 `freshness_analysis.json`、窗口 `window1_results.json` 和 `window1_cleanup.json`。
- 重开前已完成：保留冻结版本基线；原算法与默认关闭/启用的离线选帧对照各 12 场一致。当时R0尚未部署；当前R0实际加载及分段指标见修复记录。
- 重开前已知/未知：源接收与策略消费之间存在时效差；当时未确认内部各阶段贡献。当前已确认主要等待边，但不能把外部TF可查或单个候选视为全部根因闭环。
- 影响：该返程场景及依赖持续有效扫描的动态导航不能验收；不阻塞静止载荷接线或独立视觉接口工作。
- 剩余卡点：原始采集deadline保留后，短正向租约之间存在间隙，保护反复重置清除确认窗口，返程ALIGN_START难以推进。R3中POLICY_UNAVAILABLE 2807、PROTECTION_CLEAR_CONFIRMATION 8317为诊断采样计数，非独立故障次数。
- 下一轮最小实验：按R3冻结安全链关联逐条剩余租期、下一条间隔、snapshot/risk/start_maneuver耗时和确认重置；据同场证据减少主耗时，再补取消/ACK失联及多场景恢复。不能用延长300ms预算、改stamp或删除确认窗口换成功；R0缺陷已修复。导航窗口17:38后转独立质量验证脚本离线修复，未重启本问题仿真。
- 恢复条件：导航窗口独占仿真并与视觉窗口交接 GPU；300 ms 时效门槛保持原值。

## SCAN-DIAGNOSTICS-01：默认关闭的 R0 诊断候选审查项

- 当前状态：`fixed_and_loaded_in_r0`。两项缺陷均已先复现再修复，12项C++检查和3项实际绑定检查通过；绑定SHA `a7da70ec26936779e9851c9e5c2aaec210666858a69ee8a7d79f456982350eda`已在`scan_r0_01`实际加载。诊断仍默认关闭。
- 现象一：C++ 按 256 字节直接截断中文等 UTF-8 文本，pybind `snapshot()` 可能抛 UnicodeDecodeError；启用诊断前必须修复并验证完整绑定路径。
- 现象二：记录器 reset(1) 后仍可接收 epoch 0，形成顶层纪元与 successful 记录纪元不一致；这是诊断准确性问题，不是运动授权逻辑。
- 入口：`ws_robot/src/astribot_s1_navigation_policy_native/include/astribot_s1_navigation_policy_native/scan_timing.hpp`；已有 9 项 C++ 检查不足以覆盖上述两项。
- 已完成最小实验：实际绑定中文截断保持完整UTF-8码点；reset后旧纪元receive计入拒绝，不污染successful。证据为`runs/joint_acceptance_20260923/scan_timing_fix_20260923_1637/result.json`；未将非UTF-8 Python孤立代理字符支持混入ROS有效字符串验收。
- 计时：审查复现发生于本轮 15:51:54 后；精确本项开始时间未单独记录，标未知，不虚构耗时。

## FP-BACKEND-01：FoundationPose 后端镜像与依赖就绪

- 状态：`deferred`，视觉窗口已在检查点收尾并转S0独立离线任务，GPU交还导航；本清单仅汇总其回传。
- 计时：已存最早失败证据为15:38:27，替代此前约15:40的粗略估计；主动排查累计时间未完整单列，标未知。15:57:25–16:08:28资源交接和镜像传输/解包另列；本轮约16:36停止后端工作并交接资源，保留历史。
- 已确认：Docker/Toolkit安装、容器GPU可见、官方模型/样例/CAD和15项CPU检查完成；固定基镜像下载通过，两个ONNX模型完整语义/输入检查通过。尚未构建最终后端/engine或执行模型GPU推理。
- 待解：TRT 24.08 基镜像 TensorRT 10.3.0.26 与固定 common 要求的 10.3.0.30 不同，必须统一最终构建/运行版本后才生成 engine。
- 证据：`docs/FOUNDATIONPOSE_P0_PROGRESS_20260923.md`、`docs/evidence/foundationpose_p0_20260923/backend_checkpoint_1636.json`；v1检查器因tmpfs noexec失败的记录保留，v2通过不覆盖旧记录。
- 可独立下一项：装配 S0 几何/匹配检查或感知契约工作。容器显卡可见不代替 FoundationPose 推理验收。

## 协调状态

- 2026-09-23 16:28 左右：三个活动窗口均已确认执行新规则；FoundationPose 和导航窗口已回传各自问题清单。
- 载荷质量节点已交付精确二进制路径与 SHA256；下一验证要求读回来源身份、实际加载版本，避免 geometry-only EMPTY 兼容路径冒充真实 Consumer 接线。
- 静止载荷接线本轮导航侧 16:19 起复核；这只是本轮区间，先前 A4 相关投入另行保留，不能作为新问题清零。
- 视觉窗口17时段回传S0截面子项完成，Git `62e00dac`；6形状×3级间隙、27份DXF独立读回、18配对标称间隙及7几何边界测试通过。报告`docs/ASSEMBLY_S0_PROFILE_VALIDATION_20260923.md`。这不是3D/夹爪扫掠/IK/接触验收；圆件→六边孔及切角件→矩形孔存在可容纳错配，后续必须补物体实例→模型→指定槽位→槽位版本门禁。保持无GPU/ROS/下载/重型构建，资源继续归导航R2。

## NAV-COLLISION-08：历史 FOLLOW_POLYGON_SWEEP_BLOCKED 归因

- 状态：`deferred`。历史批次 08 的具体碰撞拒绝原因尚未复现确认；14 项 C++ 诊断测试已通过，但最新返程批次实际加载诊断后产生 0 次 COMMAND_SAFETY_REJECT，不能替代历史问题归因。
- 证据：`/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/I0_2_navigation_20260923_140545` 与同级 `I0_2_navigation_20260923_155154`。
- 后续最小实验：冻结历史场景、实际包络/地图/姿态与二进制，取得该拒绝发生点的轨迹扫掠和几何证据；保持碰撞门槛，不把无拒绝的新场景认作历史修复。
- 累计排查时间未完整单列，标未知；由导航窗口在重开前归并。

## INFRA-RSP-01：RSP 参数服务首回复丢失

- 状态：`underlying_cause_deferred_with_verified_mitigation`。bounded URDF 有界重试已在 3 次冷启动验证，其中 2 次首回复丢失后第二次恢复；启动永久等待已缓解，DDS 首回复丢失底因尚未确认。
- 历史始于 9/22，累计耗时未知；不因更换会话重开无目标排查。
- 证据：上述 `I0_2_navigation_20260923_155154` 及 `docs/evidence/joint_acceptance_20260923/a4_resume_155154` 的启动 helper / preflight 检查日志。
- 后续最小实验：保存精确服务请求与响应日志，在独占场景下核对响应路径；不能凭 Docker 后安装或共享仿真曾存在就推断根因。

## NAV-VISUAL-01：横穿场景完整视觉证据不足

- 状态：`evidence_pending`，尚非已证实算法故障。终点截图行人部分出画，不能确认全身脚部接地和全图动态障碍清除。
- 后续：由导航窗口在不改变原验收口径下补完整视角及对应时序证据；单张截图不足以证明全程状态。
- 目前未记录独立排查起点，不虚构已耗时；可在依赖满足的普通场景验证中补齐，不必先等待扫描问题破解。
