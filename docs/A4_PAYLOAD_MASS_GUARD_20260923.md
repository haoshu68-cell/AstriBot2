# A4 载荷质量准入：C++ 实现与离线证据

修复前，真实2kg附件可以用0.001kg请求固定包络。新增检查把请求绑定到当前有效的完整附件账本，拒绝质量低报、身份或版本不一致及证据失效。属于同一 A4 任务，计时14:13:31–15:43:31 +08 不重置。

## 数据流和边界

独立物理库存 → 已确认 AttachmentState → 既有 C++ payload::Consumer → FixedEnvelopeCore → 包络及六消费者。

Consumer 负责明确配置的 environment/session/source、内容摘要、顺序、来源纪元和 ROS/steady 租约；核心核对确认状态、全量语义、attachment_revision、无重复且完全相同的物体 ID 集合。每个实际 weight 必须有限且大于零，零值不冒充已知质量；以 long double 累计并拒绝超过 DBL_MAX 的总和。请求可保守高报，仅容许一相邻 double 数值步长的舍入差，不新增业务质量容差。

已配置来源时 EMPTY 也持续核对账本；附着或卸载先于几何更新时立即撤销并锁存。旧帧、失效或不匹配不靠心跳恢复，必须以新的有效证据重新请求包络。有效期取几何、保持和账本租约最小值。来源未配置时有载拒绝；原已验证的 geometry-only 明确 EMPTY 路径保留兼容。

节点通过只读参数接收与账本/几何相同的来源身份，并用50ms墙钟定时检查处理 ROS 时间冻结。墙钟和账本回调沿用 tick(false)，避免正向 ACK 反馈放大。当前节点单线程执行；本次没有增加任意来源自动发现或身份认证能力。

## 已验证

- 修复前定向低报测试失败，日志 mass_red.log 保留。
- fixed_envelope_cpp、固定包络核心和链路测试构建通过；未覆盖正在使用的安装。
- 32项 C++ 链路测试通过，包括原18项回归及低报/等额/高报、无来源、版本和 ID 集合不一致、旧包重复且 ROS 冻结、重新请求恢复、EMPTY切换先于几何、同版本质量篡改、实际零质量、求和溢出、0.1+0.2及一/两 ULP边界。
- 独立 fixed_envelope_authority 可执行检查通过。
- 实际 navigation.launch.py 参数解析验证通过，确认三个 payload 身份参数转发至协调器。该检查没有启动 ROS 图。
- 独立代码审查未发现阻塞问题；其建议的数值边界已补齐并通过。

原始结果位于 [a4证据目录](evidence/joint_acceptance_20260923/a4/README.md)，其中 mass_flow_tests.xml/log、mass_authority_test.log、fixed_payload_launch_test.log、mass_source.sha256 与 mass_build_manifest.json 对应本修复。preparation_and_launch_tests.log 的24项是17项库存检查、6项准备条件和1项本次launch检查，不能重复加总为额外质量测试。

## 复现和待验收

当前独立构建路径为 runs/joint_acceptance_20260923/mass_build。按 manifest 记录的依赖前缀配置同一工作区后构建 fixed_envelope_cpp、fixed_envelope_core_test、fixed_hold_flow_test；运行 fixed_hold_flow_test 与 fixed_envelope_core_test（参数为 simulation.json）。Python检查使用既有 ROS 环境执行 tools/test/test_fixed_payload_launch.py。环境脚本只提供离线依赖，不赋予旧会话运行权限。

仓库仍含共同开发中的其他未提交源码；本提交是限定范围快照，并非全仓库可独立发布版本。顶层 native CMake 等依赖输入的 hash 另存 manifest，不混入本次修改。

**尚未验证**：新节点 DDS 实际接线与50ms墙钟实测撤销时延、新会话实际有载准入/六ACK、运动时质量变化和停止、左右非home受权执行、单/双/偏置载荷动态矩阵。当前导航 N4 冻结旧安装以维持对照一致，后续独占窗口再使用新构建。运动学质量仅支持该来源的几何/质量账本契约，不能证明真实惯性、抓持力或接触效果。
