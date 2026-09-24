# M0/F3：兼容运输取消异常后的收尾

范围：只修 `astribot_s1_transport/ros_backend.py::stop_and_hold()`，不接通 fixed_v2 抓放、不改导航协调器、不启动或查询 ROS/Gazebo，不构建共享安装。

本项统一派发起点为 2026-09-24 09:50 +08:00；本子项首次工具时钟记录为 09:53:33。计时从派发起点保守计入，未重置。当前工作区在开始前已有 `EnvelopeInbox` 接线改动，本补丁完整保留。

## 确认的问题与根因

原 `stop_and_hold()` 先 `cancel_active()` 再 `change_envelope(False, checked=False)`。取消接受/终态等待抛异常时，后者根本不执行。旧 C++ 包络协调器 `envelope_coordinator_node.cpp:353-358` 持续重发当前许可，没有订阅 `/transport/status`，不能期待最后一条许可仅凭旧 lease 自然过期。

离线测试调用真实 `stop_and_hold`、`cancel_active`、`change_envelope` 和账本/任务事务，仅把外部 Action Future 与包络服务替换成确定性输入；不构造 ROS 节点。修复前六项中四项失败，见 `red.log`：

- 取消终态超时后，底盘已停稳的输入下，原许可仍为 true。
- pending 接受应答超时后，同样跳过 HOLD。
- 取消与 HOLD 均故障时，没有触发后者，也无法报告两项原因。
- 最终任务账本保留原任务故障，但遗漏未执行的 HOLD 故障。

## 最小修复

取消抛异常后仍尝试现有 `change_envelope(False)`。HOLD 成功时重抛原取消异常；HOLD 也失败时抛出包含两项原因的 `TaskFailure`，并用原取消异常作为显式 cause。成功取消后的原流程保持不变。

不清除 `active`/`pending_goal`，不提交物体解除附着，不减少不确定载荷，不把取消 ACK 或等待超时当作资源释放。新测试还核对最终账本同时保留原任务故障、取消故障和 HOLD 故障。

## 验证

- 定向回归：6/6，通过；修前 4/6 失败。
- 既有 `unittest` 包回归：96/96，通过。
- 完整 `pytest` 包回归：110/110，通过，最终命令如下。
- 独立源码/绑定路径与 SHA256 见 `result.json`；原始过程日志保留。`fix.patch` 仅含本项改动；原工作区快照保存在 `runs/mainline_20260924/m0_f3/ros_backend.before.py`。
- 独立只读审查未发现本限定补丁的阻塞问题，确认未知执行器/载荷状态保留，并强调不能外推到移动中撤销。审查者没有运行 ROS 或修改文件。

```bash
source /opt/ros/humble/setup.bash
source ws_robot/install/local_setup.bash
source /home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/narrow_arms_20260923_1902/install_v2/local_setup.bash
source runs/normal_grasp_20260923/revalidation/install/local_setup.bash
PYTHONPATH="$PWD/ws_robot/src/astribot_s1_transport:$PYTHONPATH" /usr/bin/python3 -m pytest -q ws_robot/src/astribot_s1_transport/test
```

初次仅使用共享安装时，缺 `RevalidateManipulation` 接口；加载已有 revalidation 安装后，`unittest` 通过。随后完整 `pytest` 发现六个既有 `test_envelope_inbox.py` 用例因共享 `_geometry_native` 无 `LeaseSelector` 失败（104 passed），见 `pytest.log`。第一次添加 lease 目录时顺序不当读取了旧 transport 源码，采集失败，见 `pytest_overlay.log`；将当前 transport 源码放最前后 110 项通过，见 `pytest_lease_overlay.log`。总调度提供较新的 `narrow_arms_20260923_1902/install_v2` 后只加载其库，再次 110 项通过，见 `pytest_final.log`，并核对实际绑定文件路径与哈希。没有因此修改源码依赖或共享安装。

## 尚未关闭的运行边界

此项只证明**取消异常后继续尝试现有撤销路径，且原故障和资源隔离不丢失**。`change_envelope()` 仍先等待 `stopped()`，旧协调器 `propose()` 也要求新鲜里程计证明停稳（`envelope_coordinator_node.cpp:335-338`）。若底盘仍运动，HOLD 可能等待/失败，不能称已经完成独立即时撤销或实测停稳。M2 负责核查该独立边界的现有接口。

本次没有实际运动、ROS Action/服务传输或 Gazebo 验证，不作为新版 fixed_v2 完整抓放或硬件验收证据。
