# 正常抓取、搬运与放置仿真验证（2026-09-23）

`task08` 已完成 RGB-D 定位、MTC 抓取、闭爪与附着确认、抬升收臂、两段携物导航、放置解除附着、退臂收臂和放置确认，终态 `SUCCEEDED`，耗时 **147.68 秒**。这是统一导航仓库中的一次正常兼容流程验证；暂停/继续与故障矩阵按本轮要求暂缓。

## 本轮修复

1. 搬运观测器接入统一导航环境实际发布的头部原始 RGB-D 话题，标定版本绑定本次相机配置和安装位置文件，并保留文件内容及哈希。
2. 根据现有 URDF 和工位位置，将抓取观测时的头部俯仰设为 0.65 rad；相机安装外参未改动。
3. 橙色箱体与黄色地面条纹在图像中连通，导致原识别器尺寸检查失败。既有 RGB-D 兼容识别器在连通域之前应用基于实测深度的目标搜索区域，保留原有效深度、尺寸和歧义检查。真实失败输入已裁剪为仓库内可重复测试夹具。
4. C++ / liboctomap 生成碰撞语义签名，排除纯 log-odds 更新造成的误失效；节点位置、深度、分类、树结构及外层坐标、几何、ACM 约束继续检查。
5. 头部或机械臂运动导致可见区域变化时，等待场景稳定，然后由新增 C++ `/transport/revalidate_manipulation` 服务重新检查原计划的全部剩余轨迹。只替换各阶段场景副本中的占据图，保留原轨迹、ACM 和预测附着转换。验证失败则停止，未降低碰撞或时效门槛。
6. 代码审查发现复核等待期间 HOLD 可能改变，因此补上等待后的执行条件重查。此项在 `task08` 进程启动后加入，已经离线回归验证；不把本次整链运行作为该补充检查的运行证据。

新增占据图处理、缓存及轨迹复核使用 C++。本轮只修复既有 Python 兼容适配，不新增 Python 运行时模块。

## 当前证据

| 项目 | 实测结果及口径 |
| --- | --- |
| 正常流程 | 1 次完整成功，`task08`，147.68 s |
| 识别来源 | 三帧 RGB-D 彩色夹具定位；没有使用真值替代观测 |
| 抓取 | 闭爪完成，账本 WORLD → ATTACH_PENDING → ATTACHED，抬升与运输姿态完成 |
| 携物导航 | 目标 x=0.45 m、1.10 m；任务 map→base 口径位置误差分别 0.509 mm、1.036 mm |
| 放置 | 账本 RELEASE_PENDING → PLACED；PlanningScene 位置误差 0.988 mm |
| Gazebo 放置确认 | 两次物体位姿观测，最终位置误差 1.180 mm，满足原静止条件 |
| 占据图变化 | 正常运行中 6 次剩余轨迹复核通过 |
| 服务独立探针 | 原完整六阶段计划复核通过；新增占据区域被真实环境碰撞检查拒绝 |
| 最终离线测试 | transport 包测试与相机接线测试共 109 passed；C++ canonical_octomap CTest 1/1 passed |
| 收尾 | 3 s 内 209 个关节样本、104 个里程计样本；底盘线/角速度为 0，臂/头/躯干最大速度约 5.1e-12 rad/s |
| 会话退出 | 本窗口 skills、support、supervisor 已退出；`remaining_owned_pids=[]`，资源显式交回导航窗口 |

导航误差不是外部物理定位精度。物体附着使用 Gazebo 运动学同步，未验收接触力、摩擦或真实抓持稳定性；本结果也不等于 fixed_v2 新执行链、GraspNet 到 MTC 执行链、VLA 模型或真机验收。

## 失败记录与边界

- task01–02：目标未入视野，以及条纹连通污染定位；已由头部观察姿态和深度区域检查解决。
- task03–04：占据图概率更新使原字节级场景校验拒绝；真实快照回放证实几何未变，已由 C++ 语义签名解决。
- task05：头部转向后实际可见几何尚未稳定；新增有界稳定窗口。
- task06：清理探针查询超时，未清夹具即发起新任务，任务正确拒绝重复物体；后续清理和启动改为失败即停。
- task07：实际 PREGRASP 已执行，机械臂运动改变占据图，接近前被旧校验拒绝；新增剩余轨迹复核后，task08 完整通过。
- MoveIt 旧会话关闭时有清理阶段崩溃日志，发生在本窗口主动停止之后；不归为正常抓取中断，也未在本轮扩展排查。

不把上述中间尝试合并为成功率；本轮只主张该固定场景的单次正常完整通过。未覆盖多物体、多载荷、扰动、暂停恢复、长期可靠性及真实模型执行。

## 复现与记录

运行目录：`/home/yjh/WorkSpace/astribot_sdk_ros2/runs/normal_grasp_20260923`。主会话 `stack02`，domain 90，partition `astribot_normal_grasp_20260923`，实际相机预设 `simulation_navigation_full/launch_preset.yaml`。使用独立 `revalidation/install` 覆盖层中的新消息和 MTC，未覆盖共享 MTC 安装。

构建先编译 `astribot_transport_msgs`，再编译 `astribot_s1_transport_mtc`，使用本目录 `revalidation_build.log` 对应的独立 build/install。加载环境时依次 source `env.bash` 和 `revalidation/install/local_setup.bash`，再使用源码中的 `transport_skills.launch.py`、`transport_support.launch.py`。统一仿真必须先取得会话所有权，采用新的 instance、domain、日志目录并同步两份 launch 的相机配置；旧运行目录不是启动许可。

正常任务入口为 `ros2 run astribot_s1_transport transport_task --scenario <源码>/config/warehouse_transfer.json --output <新证据目录> --navigation-geometry-mode legacy`。保持 fixed_v2 未完成执行器的拒绝门控。

可审查的精简证据位于 [evidence/normal_grasp_20260923](evidence/normal_grasp_20260923)：正常阶段事件、最终账本、指标、服务探针、停止观测及源码哈希。完整规划请求、轨迹、相机输入、构建和运行日志留在上述运行目录；`manifest.json` 明确最终源码与 task08 已加载版本之间的 HOLD 检查补充差异。
