# 项目 Skills 整改记录

2026-09-22。依据 [整体盘点](SKILLS_AUDIT_20260922.md) 的 F-01～F-08 及用户新增语言约定完成技能/文档整改。保留 15 个技能，修改其中 13 个入口及相关支持材料；未改机器人运行逻辑、停止工具或构建产物，未启动/停止 ROS、Gazebo 或硬件。

## 统一语言约定

**功能实现优先 C++；只有脚本和启动文件类优先 Python。**

- 已写入 [根 AGENTS.md](../AGENTS.md)，让普通项目开发也能发现该约定。
- [robot-runtime-cpp](../.agents/skills/robot-runtime-cpp/SKILL.md) 为具体分类和迁移边界的唯一入口：ROS 节点、算法、任务/协议服务、设备适配、区域/RViz UI、持久化、恢复与报警默认 C++；启动、运维辅助、一次性转换/采集/分析/验证脚本优先 Python。
- 其他领域技能统一引用该入口，删除“薄适配/REST/MQTT/区域编辑可默认 Python”的宽泛例外。按实际职责分类，不能把常驻功能命名为脚本来改变选择。
- 现存 Python 或第三方绑定可在兼容迁移期间保留，不因此默认新增 Python 业务逻辑；本次没有将语言约定扩大成全库重写。

## 盘点问题的处理

| 发现 | 修改 | 本轮验证与关闭范围 |
|---|---|---|
| F-01 清理未限制归属 | 移除全机名称匹配终止和全局 DDS 删除配方，要求有效启动句柄或执行点的进程身份/归属校验；未知对象保留 | 技能危险示例已移除；文字应用题覆盖其他会话、无 launch 父进程、PID 复用。没有实现或运行新停止器 |
| F-02 隔离/就绪说明过时 | 按当前参数化 domain、instance、partition 和端口更新；核对会话环境及 overlay；多帧/分层证据代替固定 CPU/话题数和超时结论 | 与当前启动/隔离源码核对；试评区分错查询环境、缺观测与不步进 |
| F-03 依赖与通信语义错误 | 源码依赖为适配器→核心/契约；另画运行调用和消息流。增加连续命令 Topic、授权 writer、时效与 watchdog | 静态复核及两道应用题符合；未改已有速度通信实现 |
| F-04 状态/释放顺序冲突 | 分开执行阶段、业务结果和资源处置；补分支转换及事件偏序、持久化失败、取消竞争和旧 epoch；中间交接使用检查点，不等整个任务终态 | 正常成功、拒绝、取消/成功/ACK、未停稳及落盘失败试答符合；11 类协议场景是后续验证设计，不声称已经执行 |
| F-05 必加载和产物过宽 | 一个主技能＋按影响补充；行业架构仅按需比较；局部设计提交差异。离线证据整理不必读栈操作，也不重跑全链 | 去重与 CSV 题不再加载不相关搬运/操作流程；独立复核无整机强制扩张发现 |
| F-06 当前资料入口过时 | 建统一源码索引；替换 C++ 仲裁及栅格处理入口，去掉固定总包数；手册声明历史主体范围 | 范围内本地引用有效；旧文件消失不能被解释为能力不存在 |
| F-07 语言例外不一致 | 根 AGENTS 接入，具体政策集中于 robot-runtime-cpp，其他领域引用 | 应用题明确 UI、网关、持久化 C++；启动及离线验证脚本 Python |
| F-08 行为验证不足 | 固定 12 道题及输入版本，整改前/后分别试答，另做独立只读复核；保留原始输出 | 本轮有限使用检查已补齐；不计算改善率，不声称长期稳定或穷尽边界 |

架构总入口见 [astribot-architecture-design](../.agents/skills/astribot-architecture-design/SKILL.md)。任务接收与搬运编排、场景设计与证据报告继续保持独立职责；`autonomy-stack-architecture` 保留可发现性，但不再作为普通架构设计的必读项。

## 新增支持材料

新增 7 份按需读取的支持材料，没有继续增加技能数量：

- [源码入口索引](../.agents/skills/astribot-architecture-design/references/project-map.md) 与 [固定使用题集](../.agents/skills/astribot-architecture-design/references/skill-use-cases.md)。
- [模块卡](../.agents/skills/robot-dataflow-module-design/assets/module-card.md) 与 [接口契约](../.agents/skills/robot-dataflow-module-design/assets/interface-contract.md)。
- [任务状态转换与事件偏序](../.agents/skills/robot-task-intake-execution/references/task-transitions.md)。
- [当前操作入口与限制](../.agents/skills/ros2-stack-ops/references/current-entrypoints.md) 与 [诊断经验](../.agents/skills/ros2-stack-ops/references/diagnostic-notes.md)。

## 验证结果与限制

- 格式校验：15/15 通过；全部技能 Markdown、AGENTS 和架构参考手册的 155 处本地引用均有目标。不包括全部外部 URL 和递归引用文档事实的复验。
- 整改后固定题：12/12 在限定题设下符合人工检查判据。基线已能给出多数正确判断；这不是新增 12 项能力，也不是 12 次机器人测试。
- 独立只读复核：未发现阻断问题。协议表、实际机器人实现、物理停止与硬件验收分别记录，不以文本一致性替代运行证据。
- 技能正文由 1514 行变为 1167 行，详细状态/模板按需读取。行数变化仅说明正文组织调整，没有测量耗时或 token 改善。
- 原始答题、复核、哈希与检查范围见 [验证记录](evidence/skills_remediation_20260922/validation.md)，可审查的完整前后差异见 [changes.patch](evidence/skills_remediation_20260922/changes.patch)。

**实际停止工具的限制仍然存在。** `tools/clean_sim_stack.sh` 代码未修改，技能已禁止推荐它作为共享环境清理入口；supervisor 的进程组/采样子树退出和真机任务工具的固定 ROS 端点未在本轮做隔离夹具或运行验证。技能要求不等于这些工具已经实现全部身份与竞态保护。后续若改停止器，需要独立的实现及故障验证，不能把本次技能整改记作该工具安全验收。
