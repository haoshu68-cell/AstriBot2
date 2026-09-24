# READY 仿真初态验证 — 2026-09-24

本项仅增加显式的 Gazebo READY 初始条件，默认 profile 为空。实际依赖工作区既有相机、控制与导航覆盖层；本次精确 Git 补丁不包含这些历史改动，不代表 HEAD 可独立部署。

- 静态边界 16 项通过，两包隔离构建完成。默认展开模型结构不变，READY 仅增加 22 个 initial_value。
- ready_scene01 真实 gz_ros2_control 初始化、22 轴 /joint_states 的位置与速度、源时间与接收时间、TF、独立完整 Scene 位置和碰撞核验通过。最大初值偏差 0.001901 rad，准备工装未发送运动命令。
- Scene 序列化中的 stamp=0 与空 velocity 保留原值；停止证明来自独立真实 JointState。7 轴历史反馈的离线回放不冒充 22 轴实时证据。
- 真实 MTC 规划成功，随后执行器在机械臂下发前报 MTC_SCENE_CHANGED，child_submission=0。此拒绝发生在人工调度中断前；完整首段/保持/抓放均未通过。
- 18 条捕获身份、12 个唯一 PID 全部退出，资源明确释放；MoveGroup SIGINT 卸载 -11 另记 clean_shutdown=false。
- 初始化到 READY 不证明 zero→READY 运动已修复；原问题 10:35–11:35 已依用户规则暂停。

原始场次、录像与大体积完整回包位于 `/home/yjh/WorkSpace/astribot_validation/READY_profile_20260924_1133/ready_scene01`。目录后缀时间曾误估，实际计时见 timing.json；本项开始 11:27。evidence_manifest.json 绑定原文件与留档副本，actual_initialization.json 仅逐字段摘录，未补造观测。
