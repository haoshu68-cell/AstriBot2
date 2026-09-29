# 主线时间契约实施与验证（2026-09-28）

本轮延续 9 月 26 日任务，不重置历史问题计时。按用户要求，删除已废弃内容，不在仓库另存源码备份。删除清单只记录路径、原因和 SHA；运行日志和验收结果保留。

## 已实施

采样时间用于同来源、同会话的最新选择，旧消息不覆盖新消息。样本领先本节点时钟、观察数据年龄或旧 valid_until 不再触发主线时效错误。RGB-D 使用最近可用配对，TF 使用可用源变换；原采样戳保留。资源租约、服务/动作预算、命令看门狗和停稳窗口使用 steady 时间；物理轨迹时序不改。缺数据、非法几何、身份/版本变化及明确负确认仍按业务处理。

覆盖载荷、几何、相机健康、RGB-D、点云过滤、map-odom、导航包络与策略消费、起点脱困、三段跟踪、搬运执行器和 MTC guard。仍实际使用的导航 Python 适配层同步移除旧年龄判断，没有新增 Python 常驻模块。补齐原来遗漏的 arm_chassis_speed_coupling；保持原臂展阈值和限速计算。

## 验证证据

- 输入侧：相机健康 28 项、RGB-D 30 项；载荷 3 组、几何 4 组；map-odom 核心和附件过滤确认通过。
- 导航侧：3 包构建、17 组针对性 CTest、6 项实际绑定行为、12 项 TF 隔离测试通过。
- 机械臂限速：4 组 CTest，包含两种活动度模式的 ROS 隔离验证通过。
- 搬运侧：core 8、导航 helper 18、实际 Ignition payload client 7 项通过；MTC guard 数值夹具通过。
- 现用 Python 策略适配层 7 项、主线准备与验证脚本 56 项通过。

以上是构建和隔离证据，不能代替整段搬运验收。各组包含的案例不同，不把组数与案例数混为总数。

## 整栈记录

front63 在发出动作目标前，执行器因临时资源 marker 丢失退出。持久账本最后记录为资源已释放；核对上一任务 RELEASE_CONFIRMED，在 marker 和 journal 同时独占锁下恢复 marker，账本未删除或改写。该次 owner 正常停栈，remaining_owned_pids 为空。

front64 已完成 PICK、附着、抬升、运输姿态和约 10.04 cm 实际后退；停后复检 START_READY。普通 GridBased 路径包络检查失败（原始搜索路径 segment=30），未进入 PLACE。任务资源 RELEASE_CONFIRMED，停稳确认通过。录屏 1093 帧已完整解码。底盘 idle_position_hold=true、idle_position_kp=3.0 和既定碰撞/到位阈值保持不变；六相机基线 SHA 未变化。

VLA、真机和新增场景未纳入本轮。

已删除废弃文件数量：106。清单：`runs/mainline_20260926/latest_time_contract/orchestration/deleted_obsolete_files.json`。

## 路径碰撞定位与剩余卡点

失败邻近地图重放显示原停靠目标静态包络无碰撞，但约 1.25 cm 扫掠采样余量触及放置台近缘的 lethal 栅格。线上原始失败路径未保存，重放曲率与线上不同，不能称逐点复现。详见 `runs/mainline_20260926/latest_time_contract/navigation/front64_analysis/REPORT.md`。

只调整场景输入：停靠目标 world (1.30,0.145,π/2) → (1.30,0.045,π/2)。物体和桌台位置、朝向、搜索、MPPI、碰撞阈值均未改。离线完整原路径扫掠通过。

front65 使用上述目标，PICK 到 TRANSPORT_POSTURE 时验证程序报 UNEXPECTED_MOTION_OUTSIDE_NAVIGATION，随后取消；原停稳确认窗口超时。任务资源已 RELEASE_CONFIRMED。之后独立 4 秒只读复查确认 phase=0/RELEASED，停稳窗口通过、速度为零；这是晚到停稳证据，不将原失败改为通过。视频已保存并完整解码。保持本会话空闲供后续复用，未再发布新目标。

当前仍未验收：后移目标的实际导航、额外 10 cm 操作距离下的 MTC PLACE，以及单次完整 PICK→NAVIGATE→PLACE。非导航底盘运动按用户“偶现问题先记录，不扩大矩阵”的要求留待专门核对。已保留既有起点/时间链路一小时暂缓记录，本轮不以换会话重置历史排查计时。
