# AstriBot 架构与操作手册

更新：2026-09-29。覆盖当前仓库的导航、感知、SLAM、双臂规划、载荷/资源、固定工位搬运、操作台及仿真/真机部署。

| 文档 | 内容 |
|---|---|
| [总架构](ARCHITECTURE_REFERENCE.md) | 总图、控制权、数据与坐标、部署、故障、证据及当前缺口 |
| [16 个模块细化](MODULE_ARCHITECTURE.md) | 每个模块的细化图、代码入口、已实现/部分实现/未实现、验证边界 |
| [完整代码包索引](PACKAGE_INDEX.md) | 49 个 ws_robot 包逐一映射，另列 SDK、工具及依赖目录 |
| [仿真操作手册](SIMULATION_OPERATIONS.md) | 构建、隔离启动、就绪、导航/探索、存图/载图、规划/搬运、日志与关停 |
| [真机操作手册](HARDWARE_OPERATIONS.md) | 部署、版本切换、环境、只读检查、感知、运动放行条件、停止及回滚 |
| [本次核验记录](DOCUMENTATION_AUDIT_20260929.md) | 文档覆盖、链接/图检查、参数转发修复及验证范围 |

建议按总图 → 模块 → 对应操作手册阅读。图使用 Mermaid，可在支持 Mermaid 的 Markdown 查看器中展开；另附[可缩放总览 SVG](ARCHITECTURE_OVERVIEW.svg)。

## 当前结论

- 源码基点 `2c5354c3fe2d4df46103684182c1ab852a1cf64e`，分支 `chassis-effort-drive`；本次另含文档和统一仿真 launch 参数转发修复。源码不代表当前安装已更新。
- scene96 已有固定工位抓取→导航→放置→空载→释放的仿真成功记录，使用临时 **3 cm / 0.1°** 到站门槛；严格 2 mm、接触/力控、真机均未因此通过。见[原始验收入口](../evidence/mainline_20260929/PLACE_ACCEPTANCE.md)。
- 默认策略 off 与上肢限速配置冲突仍阻塞原默认启动；仿真手册给出显式 p3 接线。真机无已验收策略 profile，不能直接改成 p3，也不能关闭保护伪装修复。
- 静态地图+SLAM 需要实测初始配准。真机旧停止器仍用 odom 判停，不符合项目的 SLAM 监测要求；真机运动步骤明确标为阻塞关闭后执行。
- 当前包络核心为五方 ACK，旧末级 FinalProtection 速度链、旧 SlipMonitor 和“19 个包”不再适用。

本轮未运行 ROS/Gazebo、未连接真机；检查只支持文档与启动参数接线，不提供新的运动性能或安全验收。

## 相关专题

[虚拟墙与禁行区](VIRTUAL_WALLS_AND_KEEP_OUT.md) · [业务场景/设备标记](RVIZ_BUSINESS_SCENES.md) · [日志](../LOGGING.md) · [跟踪指标](../PATH_TRACKING_METRICS.md) · [导航/路线/探索交互](../NAVIGATION_ROUTE_EXPLORATION_INTERACTION.md)

有日期的历史设计和验证报告保留其原始结论，不能直接取代本版的当前源码描述。部署现场以 source_manifest、实际安装、参数和会话证据为准；latest_* 只是索引。
