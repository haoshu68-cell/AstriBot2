# 架构模块链路图

按用户提供的深色圆角流程图样式整理，沿用 2026-09-24 14:08 汇报快照，不重新运行机器人。

- 01_overview：整机架构总览。
- 02_navigation：导航、避障、固定包络与底盘执行。
- 03_manipulation：相机感知、物体姿态、MTC 与搬运流程。

每图提供 PNG、SVG 与可编辑 Graphviz DOT。浏览 index.html 可以切换和缩放。
灰色实线代表已有实现或接口关系，不代表完整验收；金色虚线明确待贯通或待验收。

依据：上级 REPORT.md，源码入口索引，transport/transport_mtc README，FoundationPose P0 进展。
FoundationPose 不当作已运行后端；现有已知 CAD 6D 服务单列。普通演示完整成功与正式原生全阶段待验收分别保留。
