# 张臂 T 形通道验证（2026-09-29）

## 结论

双臂已通过 MoveIt 规划及真实仿真关节控制张开、抬升至胸前。分层碰撞插件在两组记录数据的回放中正确区分“底盘和双臂均可过”与“底盘可过、双臂会碰撞”；独立 GEOS 几何核对结果一致。

**整栈 T 形通行验收未通过。** 正向场景在全局规划阶段被整机二维投影拒绝，没有进入 MPPI 执行，没有实际穿过通道。负向场景也被同一上游检查拒绝，不能把该次实际拒绝归功于新 critic。本轮未修改运行时代码，未绕过规划器检查，未合并 `chassis-effort-drive`。

## 场景与成功标准

T 指从正面观察的通道截面。复用原有仓库世界，增加静态物理碰撞模型 `validation_t_gate`，不修改原仓库文件。双臂在导航期间保持固定姿态。

| 项目 | 正向通道 | 胸前挡板对照 |
|---|---:|---:|
| 低墙高度（距地面） | 0.65 m | 0.65 m |
| 底盘通道净宽 | 1.0 m | 1.0 m |
| 胸前通道净宽 | 2.3 m | 1.0 m |
| 附加挡板高度范围（距地面） | 无 | 0.9～1.5 m |
| 通道厚度 / 中心 x | 0.35 m / 0.95 m | 相同 |
| 横梁下沿（距地面） | 2.3 m | 相同 |
| 导航目标（map） | (1.8, 0, yaw=0) | 相同 |

两侧低墙中心 y=±0.85 m、宽 0.7 m；立柱中心 y=±1.3 m、宽 0.3 m；负向挡板与低墙共用横向范围。地面世界坐标 z=0.034226 m，map 与 world 对齐。精确模型见 [正向 SDF](evidence/t_gate_20260929/positive_gate.sdf)、[负向 SDF](evidence/t_gate_20260929/negative_gate.sdf)。模型在空载观察基线建立前生成，并纳入该会话的 `world_reference.sdf`。

双臂目标关节角：左 `[0,-1.2,-1.1,0.9,0,0,0]`，右 `[0,-1.2,1.1,0.9,0,0,0]` rad。实际反馈得到的几何横向宽度约 1.619 m；加入现有每侧 0.08 m 净空后，臂层约 1.779 m，底盘层约 0.793 m。因此两组底盘间隙均足够，只有对照组的臂层间隙不足。没有改动默认净空、底盘保持开关或保持增益。

事先要求：正向实际到达且分层扫掠无碰撞；负向禁止碰臂轨迹，并保持底盘层可过；记录真实关节、SLAM、地图、包络和终态。当前第一项失败，不能据评分回放代替实际到达验收。

## 实际运行

| 证据 | 正向 `mppi_t_positive_20260929` | 负向 `mppi_t_negative_retry_20260929` |
|---|---|---|
| ROS domain | 99 | 34 |
| 两臂轨迹 Action | 均 SUCCEEDED，error_code=0 | 均 SUCCEEDED，error_code=0 |
| 关节目标最大误差 | 0.000805 rad | 0.000853 rad |
| 真实空载观察 | 通过 | 通过 |
| HoldResources / 五消费者确认 | 通过 | 通过 |
| NavigateToPose | ABORTED (6) | ABORTED (6) |
| 直接错误 | PATH_QUALITY_UNSAFE，segment=3 | 同左 |
| 实际 MPPI 可视化消息 | 0 | 0 |
| 清理时 SLAM 停稳窗口 | 7 样本，0.618 s，通过 | 7 样本，0.698 s，通过 |
| 资源释放 / 自有仿真退出 | 已完成 | 已完成 |

五个确认消费者为 controller、planner、local_costmap、global_costmap、policy。两次均在准入后约几十毫秒被全局规划器拒绝；导航观察窗口内没有收到速度命令或规划路径，SLAM 净位移小于 0.000001 m。不能把“没有命令样本”当成观测到零命令；停稳结论来自 `/slam/pose` 连续位姿窗口。

本次分层地图的实际来源为 `gazebo_collision_geometry`，分辨率 0.05 m。新增物理障碍被该来源栅格化后由 `/height_maps/snapshot` 提供。不是相机点云接入验收，也不证明 SLAM 点云在真机中的遮挡覆盖能力。

## 真实插件回放与独立核对

从已正常关闭的 rosbag 提取同次会话原始地图、正式允许导航的包络、SLAM 位姿和时钟，不改消息内容。加载本分支实际 `WholeBodyCollisionCritic` 动态库，输入两个几何探针：原地保持、固定朝向直线至 (1.8,0)。

| 检查 | 正向 | 胸前挡板对照 |
|---|---|---|
| 原地候选代价 | 0 | 0 |
| 直行候选代价 | 0 | infinity |
| 批次 fail_flag | false | false（仍有原地可行候选） |
| 分层连续扫掠 | 无碰撞 | 碰撞 |
| GEOS 独立连续扫掠 | 五层均无碰撞 | torso、head 高度层碰撞；其余三层无碰撞 |
| 整机二维投影叠加低层地图 | 碰撞 | 碰撞 |

`torso`、`head` 是现有高度层名称，张开的双臂也位于这两层；不是说身体躯干或头部本身撞上挡板。GEOS 检查使用每个记录凸多边形在直线平移两端的凸包，逐个求交实际占据/未知栅格单元，不调用被测 C++ 碰撞核。两组的底盘相关 `low_obstacle`、`main_nav` 层均无碰撞。

这里的直行序列是构造的几何诊断输入，不是在线 MPPI 采样结果，也未验证该序列的速度、加速度可执行性；没有发送给机器人。独立核对仍共用记录地图及包络，不能替代接触传感器、真实几何标定或真机安全证明。

## 整栈阻塞原因与后续最小范围

`ExactGoalPlannerBase::collisionIndex()` 在 `ws_robot/src/astribot_s1_path_tracking/src/exact_goal_planner.cpp` 约 292 行读取 `getRobotFootprint()`，对二维 costmap 检查整机投影。路径创建在约 116 行调用该检查，约 182 行抛出本次实际记录的 `PATH_QUALITY_UNSAFE`。低墙与张开的双臂在二维投影重叠，即便它们高度不相交，仍可能被拒绝。

本次代码链路、实际规划器拒绝日志以及“同一条直线在分层地图可过、在整机二维投影不可过”的对照相互吻合。没有记录被拒绝路径的全部内部候选，不能进一步声称已逐点复现规划器的 segment=3。

下个必要改动应聚焦现有全局路径碰撞校验：在 fixed_v2 使用已有分层快照与扫掠核，并核对控制阶段其余二维 footprint 检查是否仍重复拒绝抬臂低墙场景。不能通过关闭碰撞检查或随意缩小整机 footprint 来制造通过。改动后复跑这两份固定场景；正向真实穿过、负向禁止碰臂轨迹之前，不满足合并条件。本轮只完成场景验证和缺口定位，没有提前实现这部分改动。

## 证据、录屏与无效试验

代码分支 `codex/fixed-posture-whole-body`，运行时代码基线 `f40b35ec`。紧凑结果、来源哈希、地图版本、正式包络 epoch、Action 事件及 bag 计数见 [summary.json](evidence/t_gate_20260929/summary.json)。

本地证据根目录 `/home/yjh/WorkSpace/astribot_whole_body_nav/runs/`：

- `mppi_t_positive_20260929/`：正向原始录屏 123.467 s，bag 130806 条消息，含 910 条 SLAM、191 条分层地图。
- `mppi_t_negative_retry_20260929/`：负向原始录屏 112.867 s，bag 130566 条消息，含 989 条 SLAM、230 条分层地图。
- 两组的 `arm_plan.json`、`arm_result.json`、`empty_inventory/result.json`、`navigation/result.json`、`stack/session.log`、`replay_result.log`、`independent_sweep.json` 保留完整证据；`motion_bag/metadata.yaml` 在录制关闭后核对。
- `mppi_t_gate_20260929/replay_t_gate.cpp`、`build_replay.py`、`independent_sweep.py`：插件诊断及独立核对源码。GEOS Python 依赖仅装在该目录的 `oracle_dependencies/`。
- `mppi_t_gate_20260929/t_gate_validation.mp4`：48 s 原速摘录，含实际抬臂、正向场景、胸前挡板对照及结果字幕。画面是 RViz 实时 RobotModel；障碍显示按已生成物理模型的尺寸和坐标绘制，不是机器人穿行动画。源片段为正向第 23～38 s、77～92 s，负向第 85～103 s。

先前两次无效试验未删除：`mppi_t_gate_20260929` 在空载基线生成后才加障碍，正确触发 ATTACHMENT_UNCONFIRMED；`mppi_t_negative_20260929` 的 domain 100 存在历史账本但缺资源标记，触发 RESOURCE_MARKER_MISSING。均未进入正式导航，不能计入碰撞验收。历史账本未修改。负向重试的第一次 MoveIt 查询超时，尚未发送关节动作，保存在 `raise_arms_first.log`；后续查询和动作成功。

所有本任务 T 场景 supervisor 的 `remaining_owned_pids=[]`；补充 MoveIt、HoldExecutor、RViz、显示及录制进程已退出。没有清理其他会话资源。
