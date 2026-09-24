# 首段真实准入与同上下文占据图重验证修复

## 原始失败与原因

真实首段问题沿总调度记录 **2026-09-24 10:35 +08** 计时，没有因 cold/warm 拆名重置。原首段 `7fafe827` 是离线/合成协议基线，真实非 home 尚未通过。

M2 `first_scene02` 原始结果：`/home/yjh/WorkSpace/astribot_validation/M1_scene_prepare_20260924_1024/first_scene02/first_stage/result.json`。错误 `M1_REQUIRES_FRESH_FIXED_HOLD`，envelopes 为空、无任务场景通过、无任务目标发送。M2 已完成所属会话退出。

真实 FixedEnvelopeCore 未收到 SetFixedEnvelope 前没有 V2 output；revoke 只锁存 navigation_allowed=false，不递增 epoch，也不将已有 mode 从 FIXED_POSTURE 改为 HOLD。负输出 valid_until=now。因此“先有未来有效的 HOLD 包络才能移动手臂，再由移动完成提供 Hold 建包络”是循环依赖。

## 最小修复

1. Goal 准入核对新鲜实测底盘停止、完整几何/控制器 claims、独占资源以及依赖可用。无包络本身不证明撤销。
2. 本任务 acquire 后发 SetRobotEnvelope(transport_ready=false)，捕获 task lease 和本地 request generation；3 s 单调超时。**accepted ACK 前不查询规划场景、不规划、不发送任何 JTC。** 服务生产者没有 task/lease/session 鉴权字段，该绑定由本节点回调实现，不是 DDS 访问控制。
3. 冷启动尚无 V2 输出时，本次 accepted ACK 的实际 epoch 是锁存撤销证据。暖启动已有 V2 时，有界等待本次请求后、匹配实际 epoch/已观察 session 的新负读回；服务 ACK 先于负话题到达时不把撤销前缓存 positive 误判为新的许可。负读回不要求未来 valid_until；HOLD/FIXED_POSTURE 两种负模式均符合实际生产者契约。
4. 屏障成立后才读取完整 scene、规划并执行。执行期间的新 positive、非法模式、session/epoch 改变、底盘移动仍停止。最终实测 Hold 且 guard disarm 后，允许 M2 用新 fixed epoch 显式接管导航。
5. 仅 occupancy 改变时，稳定 1 s（最多 15 s）后调用原 RevalidateManipulation(context,index,scene)，保留原计划/路径/期限；成功响应后再次独立完整读回必须仍同一 scene。拒绝、错误 context、读回再变或静态 world/ACM/附着几何变化都不放行。

## 验证与版本

独立候选构建目录 `runs/m1_transport_20260924/next_build`，安装 `runs/m1_transport_20260924/next_install`；原 `install` 未覆盖。构建以 -j1，7/7 CTest，日志 `next_build_warm.log` / `next_ctest.log` / `next_install.log`。编译保留既有 resource_journal_test.cpp 未检查 symlink 返回值警告；没有将其改写为全仓无警告。

真实契约 RED：179 cold_start 在原安装拒绝 goal；189 warm_ack_race 在中间候选因撤销前缓存 positive 报 MTC_NAVIGATION_NOT_REVOKED，未发送 MTC/JTC。占据图重验 RED178 在原安装报 MTC_SCENE_CHANGED，未发送 JTC。环境导入错误 occupancy_red.log 单独保留，不当作行为 RED。

最终安装的必要协议结果见同目录 `cold_repair_manifest.json` 及 `cold_repair_protocol/`：暖启动 ACK 顺序、coordinator 变化、cold 无包络、expired FIXED_POSTURE、未 ACK 超时、新 positive、拒绝，以及同 context 重验通过/拒绝/错误 context/读回变化。每场独立 ROS domain，未启动 Gazebo/真机；这些结果仍不能替代 M2 第三场真实非 home。

## 加载与运行

在 M2 已确认归属的会话环境中加载既有导航/几何/账本覆盖层，然后从仓库根目录：

```bash
source /opt/ros/humble/setup.bash
source ws_robot/install/astribot_navigation_msgs/share/astribot_navigation_msgs/local_setup.bash
source runs/normal_grasp_20260923/revalidation/install/astribot_transport_msgs/share/astribot_transport_msgs/local_setup.bash
source runs/mainline_20260924/mtc_export/install/local_setup.bash
source runs/m1_transport_20260924/next_install/share/astribot_s1_transport_native/local_setup.bash
runs/m1_transport_20260924/next_install/lib/astribot_s1_transport_native/trajectory_executor \
  --ros-args -p use_sim_time:=true -p simulation_commissioning:=true
```

仍是 `/transport/plan_to_hold` 与未修改的 PlanToHold.action，只运行完整计划首段并保持。沿旧交付 ActionClient 的 task/request/context、目标和 0.2 s 续约；取消后等真实资源释放。M2 仍需独立证明旧导航终态、底盘停稳和旧 Hold 释放。没有用本次修复绕过这些前置。

首次修复不加载新的 payload-transition MTC overlay；该新增接口留到完整抓放单独接入，避免扩大真实首段变量。

## 保留的未完成项

- 新 `ManipulationToHold.action` 仅设计草稿，未加入生成/安装或暴露 server；完整六阶段没有接通。
- PayloadCommand 核心六用例通过，但独立审查发现生产者 pending 诊断 execution=[] 与原剩余租期投影两项缺口，尚未修复，也未接到运行执行器。不能把这些单测记为物理事务完成。
- 同一更新计划必须经过实际保守附着几何重验，不能以 occupancy-only 服务替代；总调度提供的 payload-transition 接口待后续接入。
- 当前仅为首段候选修复交付，真实非 home→Hold、完整搬运、性能 A/B、长期稳定性和真机仍未由本证据证明。
