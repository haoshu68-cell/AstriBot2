# C92 后续：真实角点子状态的故障证据

本步只增加离线验证工具，控制器、运动参数和运行库没有变化；没有启动 ROS、Gazebo/RViz 或真机。

原 C91 夹具把进入 FOLLOW 当成 CORNER_RECOVERY，把 CORNER_APPROACH 中低速附近的条件当成停稳。新工具要求来自本次会话的显式 `CORNER_STATE`，缺少实际命中证据就不能将该阶段记为已测。

| 用例目标 | 必须出现的子状态 | 同时匹配的高层相位 |
|---|---|---|
| 接近角点 | APPROACH | CORNER_APPROACH |
| 转向前停稳 | SETTLING_BEFORE_TURN | CORNER_APPROACH |
| 转向 | TURNING | ALIGN_CORNER |
| 位置恢复 | RECOVERING | ALIGN_CORNER |
| 转向后停稳 | SETTLING_AFTER_TURN | ALIGN_CORNER |
| 换路径停稳 | REANCHOR_SETTLING | REANCHOR_SETTLING |

## 工具与数据流

`corner_phase_evidence.py` 是离线记录判据和未来注入器可调用的辅助模块，**不包含 ROS、服务请求、运动或进程操作**。它没有替换机器人运行时保护，也没有接通在线注入器。

```text
所属会话的控制器日志 + 对应位姿/高层相位 + 计划修订 + 时钟代次
  → parse_state：解析显式子状态
  → CornerPhaseGate：判定当前证据是否匹配目标阶段
  → 实际注入器（尚未接通）：重新检查时效，至多发送一次
  → 对应接口 ACK
  → 独立实际效果与控制结果观测（尚未执行）
```

录制器必须从实际会话元数据绑定 `session_id`、`clock_epoch` 和控制器发布者身份，提供该动作对应的 `revision`、`cursor`，按同一发布源接收次序分配 `sequence`。源位姿时间、接收 ROS 时间、接收单调墙钟分别保存；不能拿日志行的系统墙钟直接冒充 ROS 仿真时间。

工具拒绝或等待：旧/缺失上下文、不同路径、已越过的角点、源状态倒序、时钟回退、未来接收时间、过期状态/位姿、HOLD 相位、重复发起、失效判据及超期发起。新事件不会续期原注入期限。默认状态证据窗口为 0.3 s，这是验证工具的观察条件，**未修改控制器任何运动/停车阈值**；实际用例必须记录配置和测量延迟，缺证据标 INVALID，而不是放宽机器人条件。

日志目前按子状态变化输出，HOLD 后恢复若没有新的匹配状态事件，工具不会推断其仍处在可注入窗口。在线采集或受控夹具需要补齐这个观测条件；本轮没有添加心跳或宣称这类场景已经覆盖。

`READY_FOR_INJECTION` 仅表示此刻记录满足阶段时序要求；`INJECTION_ACKNOWLEDGED` 仅表示接口返回确认。两者均保留 `effect_validated=false`，不会输出机器人通过。实际刺激效果还要看独立的位姿、包络版本或行人实际轨迹；控制器响应、取消终态和实测停止再单独判断。

## 离线复现

在仓库根目录执行：

```bash
python3 tools/social_navigation/test_corner_phase_evidence.py
python3 tools/social_navigation/corner_phase_evidence.py \
  --input tools/social_navigation/cases/corner_phase_replay_example.json \
  --output /tmp/corner_phase_replay.json
```

示例输入明确标为 `SYNTHETIC_HELPER_CHECK`，不是真实 ROS 或仿真记录。22 项离线测试通过，覆盖上述正反例。独立审查发现 4 个误放行问题，均保留 22 项中 4 项失败的 RED 和修复后的 22/22 GREEN 记录。

另将解析器与 C92 实际 C++ 对象测试日志进行格式核对：154 条子状态输出中，6 条带有效非零源时间；其余 148 条对象夹具未填源位姿时间，被拒绝作为可注入证据。该结果说明解析格式匹配和无效时间处理，不能当成 154 次机器人阶段验证。

证据目录：`/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/C92_offline/phase_evidence`。

## 下一步与边界

已整理 6 类阶段 × 4 类刺激（取消、定位跳变、包络过期、行人横穿），共 24 张场景卡。每张均为 **BLOCKED / NOT_RUN**，不是 24 项通过。

- 恢复阶段须先用可追踪、在恢复范围内的受控漂移夹具确实进入 RECOVERING；该准备扰动与被测故障分开记录。
- 换路停稳须把注入上下文绑定到已确认的新路径修订，不能用旧修订继续等待。
- 实际采集器必须核验单一发布源和会话身份；本辅助模块无法证明外部填写的元数据是真实的。
- 在线动作取消、服务确认、实际刺激生效和旧执行隔离仍需实际 ROS/Gazebo 验证。C91 的旧推断方法不能直接用于新候选验收。
- 不启动仿真的要求仍有效；C/I0.1 未通过，后续阶段没有开放。
