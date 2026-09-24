# M1 载荷命令与原生物理客户端阶段交付

日期：2026-09-24。范围仅为 native 包的 PayloadCommand / PayloadClient、必要测试和独立安装。首段源码 `b041e530` 未改，`install` 与 `next_install` 未覆盖。新库尚未接入执行器；新完整 Action 仍未生成或开放。

## 已修复的实际契约缺口

- 真实 `PAYLOAD_TRANSITION_OR_ERROR` 诊断的 `execution=[]`：先核对 source epoch、clock epoch、model identity，保持未确认，不误报执行身份缺失。该原因同时可能包含插件错误，因此客户端独立消费对应 model 的 physical state，明确 `error` 立即传播；不能把所有异常无限等待。
- 正常附着/解绑在两个库存采样之间完成时，生产者会发布 `INVENTORY_CHANGED_RECONCILIATION_REQUIRED`，仍保留 execution 内容。这是负证据，必须等待**严格更新 capture** 的完整库存；同 capture 转正不通过。完整样本仍校验 plugin epoch、实际 accepted/applied counter、新 revision 及目标附着状态。
- 来源 capture 的 300 ms 有效期按收到时的剩余部分投影到 steady clock。例：capture=1.1 s，收到时 ROS=1.3 s / steady=2.0 s，最晚 steady=2.1 s 即失效。重复 capture 不续期。
- physical state 回调自身保留首次收到时间；即使执行器尚未第一次 poll，重复消息也不能重置原有效期。

## 原生客户端边界

`PayloadClient` 用持有的 Ignition Transport Node 向已配置 model 的 kinematic attachment service 发单次命令；不启动 shell、Python 进程或额外控制器网关。command id 来自独立库存的当前实际计数，附着和解绑都递增，不从 1 重启。

入队成功 ACK 不证明物理应用。确认需要独立完整库存和新鲜物理 state，保持原 position_error + radius × rotation_error ≤ 0.006 m 判据；物理应用的 2 s 单调时钟截止沿用旧业务时限。错误/超时不自动变成终态证明，未确认命令保留 unresolved，禁止覆盖发送下一命令。调用方之后仍须完成 scene、ledger、geometry 和剩余路径重验，不得只凭此库释放任务资源或宣布全库存 EMPTY。

解绑命令发送 world pose；本库的应用确认**不等于放置落点/稳定验收**。真实落点及独立第二次采样、全部库存、保守碰撞体和剩余路径检查属于后续任务接线。

## 验证与失败留存

- `payload_red.log`：真实 pending 结构与两项有效期错误的 3 个行为 RED。
- `payload_reconciliation_red.log`：正常 reconciliation 被误判永久失败的 RED。
- `payload_client_red.log`：真实 Transport 回调在首次 poll 前因重复物理 capture 续期的 RED。
- `payload_client_missing_api.log`：新接口尚未实现时的编译缺失，仅是开发记录，**不是行为 RED**。
- 独立 Release 全包构建 `full_build_3.log`，**8/8 CTest 目标通过**，见 `full_ctest_3.log`。其中 PayloadCommand 有 **9 个**用例，PayloadClient 有 **5 个**用例；这些数量包含在 CTest 中，不重复累加。
- 安装后显式加载 `full_install/lib` 再跑 9 + 5 个用例，通过；日志 `payload_command_installed.log` / `payload_client_installed.log`。实际库解析见 `payload_client_installed_ldd.log`。
- 客户端使用真实 Ignition transport API 与同一测试进程内的合成服务，每例独立 `m1_payload_test_<pid>_<steady>` partition，具体身份见日志。未启动 Gazebo、ROS 栈或真机，没有物理运动结论。测试进程均正常退出。
- 首次配置误选旧 I0_2 CMake 前缀，首次 CTest 又被旧 `libarm_hold_core` 抢先解析，造成两个 loader error；原日志 `full_configure.log` / `full_ctest.log` 保留，修正显式 CMake / 动态库路径后通过。这些环境错误不算语义 RED。
- 保留原 resource_journal_test 的 symlink 返回值编译警告，不将此交付描述为全仓无警告。

## 复现范围与未完成

构建目录 `runs/m1_transport_20260924/full_build`，安装目录 `runs/m1_transport_20260924/full_install`。MTC 和 transport_msgs 的 CMake 路径显式固定为 `runs/mainline_20260924/payload_transition/install/<package>/share/<package>/cmake`，导航 msgs 使用 `ws_robot/install/astribot_navigation_msgs/share/astribot_navigation_msgs/cmake`。用 -j1 构建；测试/运行时必须让本候选 lib 优先于历史安装。

准备姿态与 head_pick 前置、同一 ResourceAuthority 的完整多段执行、物理库存→ApplyPlanningScene→独立账本读回→几何同版→RevalidatePayloadTransition→再次完整读回均待接入。准备链由总调度专属审查后统一实施；不扩改首段，不下调 margin，不引入第二个执行所有者。实际抓放、搬运、长期稳定性及性能 A/B 尚未验收。

哈希及精确交付文件见 `payload_client_manifest.json`。本轮由总调度精确 Git 留档，执行者未操作共享 index/分支/提交。
