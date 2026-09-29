---
name: ros2-stack-ops
description: 在本仓库启动、停止、清理或在线探查 ROS2/Gazebo 会话时使用。按会话归属核对进程身份、实际查询环境和分层就绪证据，避免干扰其他会话或把缺观测误判为故障；只整理已有日志无需在线操作。
---

# ROS2/Gazebo 会话操作

先确定操作对象，再判断其状态。用户授权处理一个任务，不等于授权清理同机所有 ROS/Gazebo 进程。只操作本任务已确认拥有的资源；归属未知时保留该资源，继续可独立进行的只读分析或选择空闲隔离身份。

修改实现时遵循唯一语言约定 [robot-runtime-cpp](../robot-runtime-cpp/SKILL.md)。仅整理既有证据时使用 [robotics-validation-evidence](../robotics-validation-evidence/SKILL.md)，不因此启动或重测运行栈。

## 1. 会话与身份

操作记录至少关联：会话目录、启动命令、启动句柄/所有者、主机启动身份、PID 与进程启动时间、ROS domain、Gazebo partition/发现端口、实际安装及 overlay、日志路径。字段缺失必须说明，不能从进程名补猜所有权。

- `latest_sim` 是可变索引，不是活性或归属证据；先解析目标，再读取该会话的 `session.json` 和实际日志。
- 进程匹配读取可执行文件及参数令牌，避免把工具 shell 中的脚本文本当成栈进程。排除自身及工具 shell 只是避免误匹配，不能证明其余进程归本任务所有。
- PID 可复用；停止或计数前比较启动身份。直接启动、已脱离父进程的子进程也须有先前捕获的归属证据。单独的 domain、日志前缀、同一用户或同一可执行名都不足以授权停止。
- 启动前确认隔离身份未冲突；共享性能资源、安装目录或配置仍可能相互影响。构建/覆盖安装需独立路径，不能把 domain 隔离当成文件或性能隔离。

## 2. 当前启动与停止入口

使用启动接口前按 [当前入口与限制](references/current-entrypoints.md) 核对所读源码及实际安装版本，不把此表视为永久事实。

- 统一仿真入口是 [launch_sim_stack.sh](../../../tools/launch_sim_stack.sh) → [sim_stack.launch.py](../../../ws_robot/src/astribot_s1_navigation/launch/sim_stack.launch.py) → [sim_stack_supervisor.py](../../../tools/sim_stack_supervisor.py)。选择明确的 `instance`、`ros_domain_id` 和新日志目录；先检查解析后的启动配置，再在授权范围内启动。shell 的 `export` 不能代替 launch 参数。
- [warehouse_sim.launch.py](../../../ws_robot/src/astribot_s1_gazebo_bringup/launch/warehouse_sim.launch.py) 的 `ros_domain_id` 可配置，默认 25。supervisor 的隔离实例要求非空 `instance` 和 1–101 内且不为 25 的 domain，并生成 partition、发现端口及锁；实际约束以 [sim_isolation.py](../../../tools/sim_isolation.py) 为准。
- 停止前先停止本会话的任务接收/目标生成，经有归属的业务取消或停止接口处理活动任务，并保留反馈链路，按任务要求确认相关资源停稳；不能以先杀执行进程代替停车。纯仿真故障中反馈无法取得时，记录停车状态未知，仍可回收已确认拥有的仿真进程，但不输出物理停车成功。
- 停止仿真优先通过本任务保留的、已核实归属的启动句柄通知其 supervisor，让该所有者执行自身退出流程。退出后核对同一会话结果及自有进程身份；不能把“已发停止请求”当成“自有子树已退出”。
- 若只剩陈旧清单中的 PID，没有启动身份和有效所有者句柄，不按 PID 猜测发送信号。停止入口必须在执行点重新校验对象并防止 PID 复用；离线审查或手工前置检查不能替代这一约束。
- **不使用** [clean_sim_stack.sh](../../../tools/clean_sim_stack.sh) 作为共享环境的清理入口：当前实现仍按全机名称/路径收集进程并删除全局 DDS 共享内存。旧 `/tmp/rosops/cleanup.sh` 配方也已废弃。不改用另一份模式匹配脚本绕过限制。
- 不执行全局进程名终止、全局 Fast DDS 共享内存删除或无归属的 daemon 重启。DDS 状态异常先检查会话环境、transport 和发现信息，不能清空其他会话资源来“修复”。
- 真机停止须另按 [ros2-control-hardware-safety](../ros2-control-hardware-safety/SKILL.md) 核对执行权、停车反馈与设备保护。`robot_task_control.py` 不是仿真通用停栈入口，也不能凭 `--session` 就假定所有 ROS 停车请求已隔离，见参考材料。

## 3. 查询环境必须属于目标会话

从目标会话记录和已确认归属的进程交叉核对；不要从全机第一个 `controller_server`、SLAM 或发布者进程抄环境。

需要核对的环境包括 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY`、RMW 与 DDS profile，以及 Gazebo 的 `IGN_PARTITION`/`GZ_PARTITION`、发现端口和 IP。Gazebo transport 与 ROS domain 是不同通道，必须分别对齐。

supervisor 的 `env.sh` 保存导航查询环境，但当前仅 source 基础安装，不完整重建额外 overlay；需结合本次启动环境核对实际包/库路径。文件由可信目标会话生成且已检查后才可 source。启动早期失败也可能尚未生成 `env.sh`，缺文件不能推出“栈从未运行”。

即使环境对齐，“无消息”仍可能来自 QoS、命名空间、错误话题、观察窗口或暂停；先区分这些原因，再定位生产端故障。

## 4. 分层证据与结论

按任务依赖定位，某层未确认时将依赖该层的结论标为未知；可继续采集其他层只读证据，但不能跳过前提宣布全栈就绪。

| 层 | 要观察的证据 | 不足以支持结论的单项 |
|---|---|---|
| 仿真世界 | 目标 partition 和实际 world 的统计话题；一个观察窗口内时间/iterations 是否推进，paused 状态 | 话题数量、CPU 百分比、一帧 stats 或订阅超时 |
| ROS 时钟 | 对齐环境下 `/clock` 的连续样本及时间范围；消费者实际 `use_sim_time` | 有发布者、只有一帧、未确认 QoS 时的零帧 |
| TF | 所需 frame 链、时间戳、龄期和来源，按实际任务检查 `map`/`odom`/机器人 frame | 只存在一条变换、不相关 frame 的查询成功 |
| 生命周期/接口 | 目标节点所需状态、目标服务/Action 的归属与可用性 | 进程存活或名称相同 |
| 数据/执行 | 实测接收率、龄期、质量、QoS 兼容与下游实际消费；运动另需反馈 | `pub > 0`、命令已发布或 Action 已接收 |

报告至少区分“已观察到持续推进”“已观察到暂停/无推进”“查询对象或观测不足，尚不能判断”。受控暂停不是启动失败；查询超时不是物理一步未走。CPU 和进程/话题数量只作诊断线索，不设历史固定正常值。

时间不推进会影响依赖 ROS 时间的逻辑，但不能一概断言所有节点时钟为零或所有定时器停摆。定位加载阻塞、QoS、异常退出时按需读 [诊断经验](references/diagnostic-notes.md)，不要用历史表象直接认定根因。

## 5. 收尾与证据边界

输出操作对象、归属依据、实际环境/安装、观察窗口、执行过的操作、目标会话结果及未确认事项。停止后仅对已捕获身份的自有集合报告退出/残留；不能把全机“同名进程为零”或 `/clock` 无发布者作为清理成功证明。

“停栈完成”和“机器人已实测停稳”分开报告。发现 SIGINT/SIGTERM、进程死亡或日志中断时先核对操作与异常时间线：可能来自自己、其他所有者或故障，单条信号日志不能判定为外部干扰。

技能/资料静态修改只证明规则已修订；若没有隔离夹具或真实会话验证，明确未验证现有停止工具的运行行为，不声称已修复工具实现。
