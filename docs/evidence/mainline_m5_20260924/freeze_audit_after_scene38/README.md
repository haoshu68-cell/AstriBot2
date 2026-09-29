# scene38 后固定配置审计与唯一补漏

只读对照 scene38 保存副本与当前文件，未启动 ROS/build/GPU 或重解码。审计时11份 used_* 对应私有文件均逐字一致，实际 overlay 也一致；随后唯一获准变更是 reader 新增3行 kp 类型和值检查，其他脚本未修改。

| 固定项 | scene38实际证据及当前状态 |
|---|---|
| idle hold / kp | 已读回 bool true / double 3.0；当前私有配置相同。reader 原来仅校验 hold，现已补上 kp 必须 double且==3.0 |
| 放宽条件 | 两节点均读回 bool true；启动仍需显式 `--relax-base-motion`，不得把省略flag得到的严格场与此轮混用 |
| 轨迹 | 2.5时间缩放、0.1速度、0.1加速度、0.1关节余量；verifier已有提交前读回和数值拒绝 |
| 场景 | scenario SHA `ad21630122ec833dc7ca17de21117aed7c57a58e4263ba1b96b29cca180dce3a`；场景、夹具准备器、goal生成器均与scene38副本相同 |
| 实际目标 | PICK目标(0.1,0.7,1.195)、pre z=1.225、lift z=1.255；PLACE目标(1.15,0.72,1.195)、pre z=1.225；导航map目标(1.1,0,0)。任务/请求/context ID按场次更新，完整goal哈希不宜跨场强等 |
| 六相机 | preset `8942e6b0952a982ad3edb3b9fb8aa293f0c528a06ea6a720f3e145056cdd625f`、explicit_overrides为空；引用的六YAML/preset/mounts与当前文件逐字一致，实际读回两个URDF的六sensor相同 |
| 六相机规格 | 头/躯干640×360@20、双腕640×320@20、双目400×300@5，clip .08–5m；六相机开启，postprocess=false、pointcloud=true。配置不是接收帧率或六流质量验收 |
| overlay | `runs/mainline_20260925/base_motion_user_0542/overlay.bash` 与scene38保存副本相同，SHA `b949ed8b022102fa0cb1ae2e3999d0c67643e49fc051964e082fabae63036b83` |

审计时实际安装产物与scene38运行清单的哈希均相同：

- trajectory_executor：`5e5a17ec4c10127d77c4c76756b3e132a24907baa867157d4eaa75a5e6e60ce7`
- mtc_planner：`01abe4991268775f89da1d2cc51eed5237f84475bf78973fc1b3913bb9bb07a9`
- execution_guard：`961443c560237f6e34d332df76cff86012de42e8d34936560fe5f900a1e97596`
- omni_effort_drive_cpp：`e1878a05551861db984082c1a200fbd9d3e52b7c21ef1cdd73ee606d9f2c6b17`

后续已授权的失败快照诊断构建可产生新MTC/native哈希，应由root更新清单；以上仅是本次审计快照，不冒充后续构建身份。

仍存在的记录与硬拒绝边界：runner第102行只锁preset哈希，没有锁其引用六YAML/mounts，也没有断言explicit_overrides为空；第125–134行记录底盘binary哈希，组件清单也记录guard哈希，但当前expected比较没有覆盖这两个可执行文件。scene38当前值已核实，下一轮由root冻结输入并对照已存清单；本次未扩大防护工具或改runner。`--relax-base-motion`默认false，两节点都false会成为合法严格条件场，报告会区分，不自动保证本轮固定true。

## 唯一批准的补漏

private `read_full_executor_parameters.py` 在既有底盘读回后要求 `idle_position_kp.type==3` 且 `double_value==3.0`，否则 `CHASSIS_IDLE_POSITION_KP_MISMATCH`。两项生产片段离线验证通过：3.0通过、被覆盖为2.0拒绝。语法通过，移除新增3行后与原脚本逐字相同；没有新增通用工具。

**实际冻结 reader SHA256：`7368749120cc6ad099ff418b60a218e94c19aedb327f00b4bec4babdb9264f76`。** 已对生产文件和after副本分别复算一致。通信中一度误写的 `d75db08e…848c35` 已明确撤回，不对应本候选。scene38历史reader `6db3a221…5240c` 及其kp=3读回证据保持不变。

verifier仍为 `1333136611ff06fbf31d285858c61833893c6997f11a8937ec936b9ac3081184`；runner仍为 `8f3212167d0d423d561ff984149d900d3fb73eec05cc4b4e10a1bf79e6538cd1`。本目录保留reader before/after、最小patch、两项结果与哈希清单。
