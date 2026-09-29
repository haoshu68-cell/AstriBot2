# 当前操作入口及限制

核对日期：2026-09-22。以下来自源码静态阅读；没有运行或停止 ROS/Gazebo。使用时重新核对源码、实际采用版本和安装产物，特别是 overlay。

## 启动与会话文件

| 入口 | 已见实现 | 使用边界 |
|---|---|---|
| [launch_sim_stack.sh](../../../../tools/launch_sim_stack.sh) | 无参数或 `name:=value` 形式进入统一 launch；旧 `--flags` 形式直接进入 supervisor；可加载 `ASTRIBOT_OVERLAY_SETUP` | 脚本先设置 domain 25，目标 domain 应显式经 launch/supervisor 参数传入；不能仅 export 另一个 domain |
| [sim_stack.launch.py](../../../../ws_robot/src/astribot_s1_navigation/launch/sim_stack.launch.py) | 将 `instance`、`ros_domain_id`、`log_dir` 等转为 supervisor 参数；支持 `dry_run` | dry-run 只解析配置，不证明资源当前空闲或真实就绪 |
| [sim_isolation.py](../../../../tools/sim_isolation.py) | 隔离实例的 domain 为 1–101 且非 25；生成 `astribot_<instance>` partition、domain 对应端口和锁 | ROS/Gazebo 身份隔离不覆盖 GPU、性能、安装目录和未遵守该机制的启动者 |
| [sim_stack_supervisor.py](../../../../tools/sim_stack_supervisor.py) | 记录 `session.json`、`session.log`、导航查询 `env.sh`；对子进程建立独立进程组并捕获后代启动时间；正常退出时回收自有子进程和捕获的后代 | 清单存 `supervisor_pid`、`children[].pid`，当前没有持久化完整 boot/start 身份，不能只凭清单重接后发信号；采样子树也不是所有逃逸进程已覆盖的证明 |

`session.json` 中 `state`、`ended`、`remaining_owned_pids` 和测量记录需与本次进程身份及时间线对应；磁盘上的 `ready` 可能是历史状态。`env.sh` 保存指定环境变量并加载基础 `ws_robot/install`，未完整保存 overlay；不能宣称与启动侧包解析完全相同。

停止已持有的本任务 supervisor 时，由该启动句柄接收中断并执行退出路径；重新接管遗留会话需要额外可验证的身份，不能复制陈旧 PID。进程退出与停车反馈是不同证据。

## 当前不适合作为通用停止入口的工具

- [clean_sim_stack.sh](../../../../tools/clean_sim_stack.sh)：当前仍按全机可执行名、launch 文本及安装路径匹配并发送信号，且删除全局 DDS 共享内存。**不可推荐直接执行**；本次技能整改没有修改该工具。
- [robot_task_control.py](../../../../tools/robot/robot_task_control.py)：真机任务工具。`--session` 读取的是含 `boot_id`、`supervisor`、`processes` 的真机清单，与仿真 supervisor 清单不同。它对 PID 启动时间做校验并使用 pidfd 发送信号，但 ROS 停车请求仍使用当前调用环境里的固定 Topic/Service/Action 名称；没有自动从清单设置或核验目标 DDS 会话。未传 `--session` 时还会按角色、路径和 domain 扩选。**不能把文件参数当作完整授权或隔离保证**；需要另行核实请求端点归属与物理反馈。`--dry-run` 只列进程，不证明停车端点安全。

若需要新增安全停止器，先提供可评审设计和隔离验证：在执行点核验 `(boot_id, pid, start_time)`、受控子树/实例归属，防 PID 复用，拒绝未知对象；请求及反馈属于同一目标会话；限制升级策略并保留停车所需反馈链路。此项是后续工具工作，不能用说明文字冒充已实现。

## 只读辅助证据

- [record_simulation_ownership.py](../../../../tools/sim/record_simulation_ownership.py) 采样 Gazebo 实例、domain、partition、stdout 和负载，用于发现其他实例及观察性能干扰。它按部分可执行模式扫描，没有记录完整 boot/start 身份；其结果不是可直接用于停止的授权清单。
- [record_runtime_identity.py](../../../../tools/sim/record_runtime_identity.py) 按实例记录进程启动时间、环境、日志和加载原生库哈希，可辅助区分源码与实际运行库；仍需交叉核对会话所有者，实例环境字段自身不是不可伪造身份。
- [sim_stack_probe.py](../../../../tools/sim_stack_probe.py) 提供数据/导航就绪探测。解释输出前核对该次环境、配置的 scan、目标节点与所要求的证据层级，不能外推为整机/真机验收。
