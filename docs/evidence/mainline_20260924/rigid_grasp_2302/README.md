# 仿真刚体夹持目标修复

主线仍未完成。本改动只关闭当前仿真箱体的夹爪位置目标冲突，不代表力控抓持或真机验收。

- scene12：60 mm 刚体，默认 4 mm 预紧得到 56 mm 目标；实测夹口约 59.932 mm，关节偏差 0.038835 rad，原 0.02 rad 端点检查正确拒绝。独立模型/FK/网格分析见 geometry_analysis.json。
- 改动：仅 simulation-only MTC 显式设 grasp_preload_m=0；通用夹爪默认、端点精度、碰撞与时效门槛不变。独立构建和安装通过，候选绑定见 candidate_binding.json。
- scene13：真实 journal 确认 GRASP_CONFIRM 已通过并进入 ATTACH_CONFIRM；随后附件交易的不完整几何消息被误判为 MTC_CONTEXT_CHANGED。父任务失败，resources_released=false，domain78 原租约隔离记录保留。进程退出不视为业务资源释放。
- 记录开销：CDR 暂存后 scene12 最大观测 GC 为 62.17 ms，该场没有 ODOM_STALE；只作为此场观察，不作全栈性能保证。

所有原始失败、录像与完整结果留在 scene_results.json 中列出的绝对路径。这里只保存必要摘录，不删除原始数据。
