# 正对操作台搬运与放置仿真验收

本轮主线通过：抓取 → 运输姿态 → 导航 → PREPLACE → PLACE_APPROACH → RELEASE → DETACH_CONFIRM → RETREAT → STOW → 空载确认。父任务返回 `TRANSFER_COMPLETE`，资源已释放，所属仿真进程已退出。

- 临时到站门槛：平移 3 cm，朝向 0.1°；严格 2 mm 精度仍未验收。
- 首次进入 PLACE 时最近 SLAM 样本：平移误差 1.854 cm，朝向误差 0.0315°。
- 最终账本：附件清单为空、载荷质量 0 kg、版本 4；物理附件来源、账本及几何版本一致。
- 撤手和收臂的规划轨迹摘要、实际发送、控制器成功终态与阶段确认已对应；最终 SLAM 停稳窗口 0.693 s 通过。
- 视频解码/帧索引/元信息均为 2527 帧，bag 返回 0，全部所属进程退出。视频按收到的画面以 10 fps 编码，不作为实时性能测量。
- 固定配置：idle_position_hold=true、kp=3.0；3.5 倍轨迹时间缩放；停靠(1.50,0.15,90°)；沿用上一轮工作台内15mm放置点偏置。

本轮只优化验证脚本采集：包络历史保存 CDR，结束时逐样本展开，先保存终态摘要。未改任务续约门槛、碰撞条件或控制参数。本轮无续约中断；scene95 中断根因仍未证明，不以单轮成功认定已经根治。

证据边界：Gazebo 运动学附着场景，通过本轮主线流程；不等于力/接触仿真或真机验收，也不外推为全部扩展场景通过。

[完整录屏](/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260928/workstation_alignment/full_transfer_runner/front_transfer_scene96_transfer_budget_place_3cm_world44_20260928/full_transfer.mp4) · [验收指标](/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260928/workstation_alignment/full_transfer_runner/front_transfer_scene96_transfer_budget_place_3cm_world44_20260928/acceptance.json) · [父任务终态](/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260928/workstation_alignment/full_transfer_runner/front_transfer_scene96_transfer_budget_place_3cm_world44_20260928/full_transfer/summary.json)
