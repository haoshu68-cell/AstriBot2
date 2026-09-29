# scene39 PICK 至导航的 ArmHold 接收回执

原路径在 hold_confirmed 后同轮先调用 navigation start，发送 set_fixed 服务请求，到 tick 末才发布正向 ArmHold。scene39 首个正向源时间为 104.543 s，前一条 104.508 s 为 false，104.562 s 再次 false；PICK 的 LIFT 和 TRANSPORT_POSTURE 已确认，随后服务拒绝 ARM_HOLD_UNCONFIRMED。资源 journal 的 issued_at 是租约快照，不是每个事件的独立发生时刻。

本改动以已有 ArmHoldStatus 接收回执建立处理顺序。协调器由主线在 core hold 处理之后回显 /navigation/arm_hold_observed；本目录仅包含 native 四个文件的精确改动。回执表示已接收，未授予导航权限，原 set_fixed、五方 ACK 和测量门槛保留。

执行器首次实际发布 positive 后记录其源 stamp 下界和 steady 起点。PICK 到 NAV 必须收到 owner、hold、attachment revision 匹配且 positive 的新鲜回执；后续源 stamp 不早于下界的 heartbeat 也可匹配。回执源时效与扣除接收时已有源年龄的 steady 剩余时效相交；负值、未来/非法时间、非有限或不在 (0,0.5] 范围的 lease 拒绝。首次发布后最多等待 3 秒，心跳不重置起点；新任务 start 清除旧回执及下界。本地 hold 失效与取消沿原逻辑停止。

这里新增的是最多 3 秒的回执等待窗口；随后已有 set_fixed RPC 自身 3 秒回复上限及父任务总预算未改，不宣称整个交接总耗时上限只有 3 秒。没有 sleep、重试、请求刷新或忽略拒绝。

现有 ArmHold 新增 const 匹配方法，直接由 arm_hold_test 覆盖三组：首帧及后续 positive；false、旧下界/身份、非法时间及取消；source/steady 到期前 1 ns、恰好到期、冻结 ROS 时间以及非法 lease。该方法只验证回执，当前本地 hold 有效性由原 executor 每轮状态检查负责。3 秒窗口及实际跨通道交付仍须主线原场集成验证。

限定 git diff --check 和文本生命周期核对通过；未构建、未运行测试、未操作 ROS。主线构建目标 arm_hold_core、arm_hold_test、trajectory_executor、hold_executor。before/after 保存当时实际工作区字节，不覆盖已有的 scene38 失败诊断。
