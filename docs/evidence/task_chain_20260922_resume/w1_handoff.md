# 本轮交接：W1 与硬同步新增需求

本轮重新计时：2026-09-22 09:53:45 +08，暂停点11:23:45。重连未重置时限。运行记录与本轮结果均在 task_chain_20260922_resume，旧失败未删除。

## 已验证范围

- 空场景启动解除不必要的 HuNav 依赖，保留原生物理几何观察；地图 verified、三个区域消费者 ACK、物理几何新鲜度通过。
- Nav2 实际 haltAllActions 复现相同目标取消后执行ID复用；C++ 生命周期修复通过测试且真实第二次目标成功。
- transport Python 薄适配调用 C++ LeaseSelector，保持原有效期；新包络先于本地 /clock 1ms 到达不再错误撤销同上下文有效包络。负向状态、版本改变、缩短有效期仍立即生效。
- planner/controller 使用同样原则的 C++ EnvelopeEvidence。navigation_fix2 插件配置崩溃保留；全新单一构建 navigation_fix3 通过9组测试和两目标实际运动。旧崩溃可能与编译期间头文件变化/并发构建有关，未独立证明。禁止把构造插件测试当配置阶段ABI证明。
- motion_05_fixed 和 motion_07_clean 都通过固定紧凑姿态、空载、1.3m通道真实运动及保持租约超时注入。后者最小全身墙面净空下界0.28737m；567个FOLLOW样本，横向RMS0.00829m、P950.01972m；到位平面误差0.985/0.766mm，航向0.0292/0.0384deg。仿真真值定位，不是真机或SLAM精度。
- 新增 `astribot_sensor_sync`：C++有界检查、ROS强类型消息、可选启动、HardwareTriggerPort抽象接口和禁用的硬件模板。20项核心测试、14项消息流+5项私有时钟ROS检查通过。合成消息不等于Gazebo曝光触发；没有厂家驱动适配或物理板卡实现。

## 明确未通过

- motion_camera_04_clean：600.003秒完整记录，仿真时间推进235.933秒，平均RTF0.3932。四路深度源间隔均0.1仿真秒，观测端龄期均约0.001仿真秒；真实平均帧率约3.93Hz。监控接收间隔约55ms以内但样本本身wall龄期超250ms。
- 四源无效采样数分别 head2629、torso2604、left4992、right4989；这是健康状态采样数，不是独立丢帧数。正常档新鲜度未放行，不延长250ms门槛。
- 分桥与20Hz全分辨率候选不作为修复。分桥源码已撤回，历史patch保留在bridge_isolation_candidate.patch。低分辨率候选只用于诊断，最终记录见implementation_status.json；即使健康改善也不代表障碍覆盖或姿态质量等价。

## 正在/下一步

1. navigation_08_lowres诊断在142.612秒因续租REQUEST_CLOCK_AHEAD退出；原始数据和分析已保存。四源STALE采样5/4/9/9，不完整未验收。两个导航实例和相机探针均已清理，无遗留自有进程。本轮在90分钟上限前暂停，剩余预算不足重新跑600秒。以session.json的真实PID/环境验证归属，不能使用latest_sim判断存活。
2. W1仍需每角色ROI/有效深度/遮挡覆盖、全流程阶段延迟/队列峰值、TF/单流丢失/重启矩阵、30分钟组合压力和完整C++任务激活客户端/唯一provider准入。W2+保持pending。
3. 优先提升Gazebo实时推进效率或合理划分按需相机负载；单纯软/硬同步不能修复低RTF。低分辨率若采用，必须重验目标识别、点云密度和3D安全覆盖。
4. 硬同步需设备型号、固件、各color/depth外触发能力及板卡后才实现具体后端/接线；激光/IMU按PTP/PPS/原生时间域接入。当前只完成方案和证据检查底座，不能宣布所有传感器已硬同步。
5. VLA、真机和完整动态抓取搬运放置继续暂缓。固定姿态仿真导航不解除这些边界。

## 重现入口

- 当前运行环境：`runs/task_chain_20260922_resume/query_env.sh`，最后使用干净navigation_fix3；未覆盖共享install。
- 成功实际运动日志：`/home/yjh/WorkSpace/astribot_sdk_ros2/runs/task_chain_20260922_resume/navigation_07_clean/session.log`。
- 成功评分：`runs/task_chain_20260922_resume/motion_07_clean/summary.json`；低分辨率另有navigation_08_lowres/session.log。
- 冻结清单：`manifest_final.json`（1905文件、130ELF依赖无缺失；不是通用ABI证明），Python薄适配额外清单在runs/.../lease_python/manifest.json。
- 同步构建/安装：`runs/task_chain_20260922_resume/sync_overlay`；此overlay仅独立验证，未接默认导航。
- 同步设计：`docs/SENSOR_HARD_SYNC_DESIGN_20260922.md`；验证/哈希：本目录`sensor_sync_verification.json`。

最后诊断：续租拒绝的请求/响应状态时间戳都为66.730秒；服务判定和响应状态分别读取now，后者不能反证前者。验证脚本按明确REQUEST_CLOCK_AHEAD做原请求/原截止时间内重试（原0.2秒墙钟预算不变），4项单测通过，完整仿真尚未复验。见tools/vision/camera_clock_retry.py；没有放宽服务端时间检查。
