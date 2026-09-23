# 暂缓问题与难点清单

本清单不是验收通过记录。采用 [全窗口任务推进规则](TASK_EXECUTION_POLICY_20260923.md)，更新时保留原始失败与各次排查时间。共同清单由搬运仿真窗口维护，其他窗口回传自己的问题条目。

## SCAN-FRESHNESS-01：策略使用的扫描过期

- 状态：`deferred_by_user`；2026-09-23 用户要求先继续其他任务。历史累计排查时间待从各窗口记录归并，不能视为零。本轮 A4 恢复始于 15:51:54，该时间不是本问题累计起点。
- 现象：返程 90°任务超时；同场景独立观测的源扫描新鲜，但策略使用扫描出现过期。
- 本轮证据摘要：返程首段到位成功，返程任务 180 s 超时；扫描独立观测 P95 58 ms/max 69 ms，策略扫描年龄 P95 476 ms/max 788 ms，510 次 STALE。安全停车、资源释放及会话清理已完成。指标以原始批次文件为准，不将不同阶段分位数相加。
- 原始证据根目录：`/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/I0_2_navigation_20260923_155154`，其中返程批次的 `freshness_analysis.json`、窗口 `window1_results.json` 和 `window1_cleanup.json`。
- 已完成：保留冻结版本基线；原算法与默认关闭/启用的离线选帧对照各 12 场一致。R0 时序诊断未部署到仿真，不视为时效修复。
- 已知/未知：源接收与策略消费之间存在时效差；尚未确认内部 TF 等待、队列和处理耗时各自贡献，外部 TF 可查询不能证明内部及时可用。
- 影响：该返程场景及依赖持续有效扫描的动态导航不能验收；不阻塞静止载荷接线或独立视觉接口工作。
- 下一步：后续难点阶段固定同场景、同二进制/配置，记录内部接收、TF 首次检查就绪、选帧、处理和发布时间，再根据证据进行单因素改动。先修复下方 R0 的离线审查问题，再决定是否启用诊断。
- 恢复条件：导航窗口独占仿真并与视觉窗口交接 GPU；300 ms 时效门槛保持原值。

## SCAN-DIAGNOSTICS-01：默认关闭的 R0 诊断候选审查项

- 状态：`deferred_with_scan_work`，源码默认关闭，未部署，不影响当前冻结导航基线。无需占用下一项载荷验收的时间。
- 现象一：C++ 按 256 字节直接截断中文等 UTF-8 文本，pybind `snapshot()` 可能抛 UnicodeDecodeError；启用诊断前必须修复并验证完整绑定路径。
- 现象二：记录器 reset(1) 后仍可接收 epoch 0，形成顶层纪元与 successful 记录纪元不一致；这是诊断准确性问题，不是运动授权逻辑。
- 入口：`ws_robot/src/astribot_s1_navigation_policy_native/include/astribot_s1_navigation_policy_native/scan_timing.hpp`；已有 9 项 C++ 检查不足以覆盖上述两项。
- 下一步最小实验：实际绑定分别输入 100 个中文字符以及 reset 后旧纪元 receive/finish，先记录失败，再补 UTF-8 完整码点边界和旧纪元拒绝计数；重跑受影响的选帧对照。
- 计时：审查复现发生于本轮 15:51:54 后；精确本项开始时间未单独记录，标未知，不虚构耗时。

## FP-BACKEND-01：FoundationPose 后端镜像与依赖就绪

- 状态：`in_progress_with_checkpoint`，视觉窗口已确认执行新规则；此项由其维护详细证据，本清单只汇总回传。
- 计时：约 15:40 首次发现固定 isaac_ros_common 生成的镜像标签不存在，主动排查累计时间未完整单列，标未知；15:57–16:08 资源交接属于外部等待，下载等待另列。保守以 15:40 作为本轮起点，16:40 收尾检查，若未解决则暂缓并切换。
- 已确认：Docker/Toolkit 安装、容器 GPU 可见、官方模型/样例/CAD 和 15 项 CPU 检查完成；当前是镜像下载阶段，尚未执行模型 GPU 推理。
- 待解：TRT 24.08 基镜像 TensorRT 10.3.0.26 与固定 common 要求的 10.3.0.30 不同，必须统一最终构建/运行版本后才生成 engine。
- 证据：`docs/FOUNDATIONPOSE_P0_PROGRESS_20260923.md`、`runs/foundationpose_p0_20260923/container_gpu_probe`、`runs/foundationpose_p0_20260923/dependency_inspection_v1`。
- 可独立下一项：装配 S0 几何/匹配检查或感知契约工作。容器显卡可见不代替 FoundationPose 推理验收。

## 协调状态

- 2026-09-23 16:28 左右：三个活动窗口均已确认执行新规则；FoundationPose 和导航窗口已回传各自问题清单。
- 载荷质量节点已交付精确二进制路径与 SHA256；下一验证要求读回来源身份、实际加载版本，避免 geometry-only EMPTY 兼容路径冒充真实 Consumer 接线。
- 静止载荷接线本轮导航侧 16:19 起复核；这只是本轮区间，先前 A4 相关投入另行保留，不能作为新问题清零。

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
