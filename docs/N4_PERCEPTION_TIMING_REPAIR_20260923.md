# N4 传感时效修复设计与当前失败证据

本文件与共同基线协作，不另建运行分支。属于当前 A4 的分析交付，沿用14:13:31–15:43:31计时。**下述性能迁移尚未实施，也未证明能解决全部 N4 失败。** 到位精度任务负责当前仿真、safeCommand诊断改动和联合STATUS；本任务负责本设计、质量准入及夹具准备工具。

## 已观察到的事实

N4 07和08均通过实际物理EMPTY、完整PlanningScene回读、正式HoldResources与六消费者同版本ACK。首个0.8m目标到位误差分别为1.452mm/0.0129°、1.434mm/0.0257°；返回原点并指定90°终态未通过。07墙钟180秒超时，08 Action状态6。二者的自有目标清理、连续新鲜里程计停稳与保持释放均有记录。

08原始证据目录：`/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/I0_2_navigation_20260923_140545/fixed_navigation_08`。

基于407条抽样诊断，按实际nav_started/nav_terminal墙钟边界分段；P95采用nearest rank `ceil(0.95*n)`。统计可由 [分析脚本](../tools/sim/analyze_policy_freshness.py) 重算，结果和原始文件SHA256见 [n4_08_freshness_analysis.json](evidence/joint_acceptance_20260923/a4/n4_08_freshness_analysis.json)。这不是每个控制周期的完整记录，最大值也不是最坏情况上界。

| 指标 | 去程：102组观测/策略 | 返程：75组观测/策略 |
|---|---:|---:|
| tracks数量范围 | 129–2091 | 2091–2960 |
| scan处理P95 | 49.85ms | 72.66ms |
| snapshot处理P95 | 37.80ms | 50.08ms |
| risk处理P95 | 15.21ms | 22.78ms |
| scan采集龄期P95 / 样本最大 | 193 / 196ms | 359 / 479ms |
| coverage拒绝样本 | 0 | 17，全部scan STALE/ACQUISITION_EXPIRED |

观测scan和snapshot随占据数量增长，是主要耗时增长项。09之前08已记录停止状态、完整360°覆盖但scan过期，因此不能归为物理视野盲区。一次ROS63.616评估的scan阶段仅0.05ms，仍使用63.2采集的旧scan；说明该周期没有处理新的可用scan，而不只是risk计算慢。`can_transform`未就绪会排队，不增加errors，故errors=0不能排除TF等待。state的observation/arbitration/start_maneuver计时存在嵌套，不能相加。

后续09独立直线hold_cancel与新授权恢复通过。其149条scan源探针的接收龄期P95为58ms、样本最大59ms，独立TF查询均可用；148条策略观测的scan龄期P95为196ms、最大204ms，tracks129–2154，没有coverage过期拒绝。见 [09分析](evidence/joint_acceptance_20260923/a4/n4_09_freshness_analysis.json)。这是另一负载/路径，不能替代08返程；独立探针的TF缓冲也不是策略工作线程的TF缓冲。轨迹数增长本身不等于泄漏，未取得新鲜自由空间证据的占据应继续保留。

另一个**独立未归因项**：08日志07:14:55.995出现FOLLOW_POLYGON_SWEEP_BLOCKED，随后56.034出现ENVELOPE_V2_NOT_READY，56.284控制器patience超时。ROS67.5–68.2内外部探针收到的28条包络均READY_FIXED，最短剩余租约85.56ms、最大接收间隔51ms；controller/local_costmap共14条ACK均positive。这不能证明控制器调用瞬间一定就绪，也不能据此推断negative ACK导致失败。

`ArrivalController::safeCommand`的false还包括包络失效、costmap非current、footprint不足、越界和不同碰撞检查，现有统一错误名不足以判定真实几何阻挡。此项由导航任务新增C++分支原因诊断，**诊断实施中，判断顺序、返回结果、阈值保持不变**；运行时验收单列。

## 现有实现与迁移接口

当前Python `PolicyNode`继承 `PolicyObserver`；即使日志显示geometry_backend=cpp，scan对象组织、融合调用和逐对象快照构造仍在Python路径。现有C++来源可复用：

- `policy_observer_node.cpp`：ROS接收、最近5帧缓存、TF、地图、scan融合/清空、健康、风险及包络。
- `policy_fusion.cpp`：持久 `ConservativeFusion`，复用机器人几何snapshot内核。
- `policy_health.cpp`、`policy_risk.cpp`、`policy_sweep.cpp`：健康、风险、候选与连续扫掠。
- `policy_observer_core.cpp`：地图和任务/定位版本转换；已有冻结Python差分参考。

不直接打开旧 `ASTRIBOT_FUSION_NATIVE_SNAPSHOT` 作为修复。它每周期把Python tracks逐项打包成数组，再逐个构造Python合同对象，已有代码明确把该路径留作测量候选。新增C++ facade应让持久融合状态、预测行和风险计算留在同一个C++对象内，避免每个调用重复往返数千对象。

最小集成先保留Python route/corridor/start状态机的决策顺序和外部Action协议；新增运行逻辑写C++。兼容层只传递输入和轻量查询结果，后续再逐步迁移状态机，不能一次替换整个导航编排。不要额外启动一个发布同名包络ACK/约束的observer来争夺控制权。

## 一致快照的数据流

`scan/map/odom/TF/包络/任务输入 → 有界接收缓存 → C++单写者融合与计算 → 不可变DecisionSnapshot → route/corridor/start查询 → 发布前时效/版本复核 → 既有约束与执行门`。

建议 `DecisionSnapshot` 保留不可变共享所有权（in-process `shared_ptr<const ...>`；Python仅持有opaque handle）：

| 部分 | 必需内容 |
|---|---|
| 身份 | session、snapshot_seq、clock_epoch、任务ID、path身份/版本、map_epoch、localization_epoch、envelope coordinator/session/epoch/hash、model/attachment revision |
| 输入来源 | scan采集戳/首次接收steady、实际选择序号、TF捕获时刻/所用变换、odom采集戳、camera标定/来源版本、原始源有效期 |
| 一致结果 | 同一robot pose/twist、地图只读版本、完整WorldSnapshot、未关联观测、来源健康、risk、预测行缓存 |
| 有效性 | 处理开始/完成ROS及steady、valid_until、源截止时间、质量与拒绝原因；不能由完成时间重置源租约 |

不要依次更新可变的last_world、last_robot、health_registry、profile后让其他线程拼接读。一次原子替换handle；旧handle生命周期有界，过期只能用于诊断。最多保留当前和正在评估的有限个快照/候选；队列满时明确返回繁忙或撤销旧工作，不能无限积压。

现有Python消费者需要明确改造的接缝：

1. **route_coordinator.safe**：目前读取last_world/last_robot并调用candidate_clearance。改为以同一handle执行C++ `evaluate_candidate`，返回safe/clearance/reason及输入snapshot/version。异步规划结果回到提交点时重新取得当前有效handle核对任务、路径、地图、定位、包络及起点；不复用旧结果的许可。
2. **corridor_adapter.geometry_clear/rotation_clear**：目前生成prediction_rows并读取n.map。改为handle内的地图+预测行查询；返回命中物体ID、边界、原因和版本。通道策略状态仍由单一原状态机管理，禁止通道内旋转等规则不变。
3. **start_maneuver_adapter.advance**：目前再次调用fusion.snapshot扩大预测区域，叠加一次构造费用。改为同一输入快照的 `forecast(region)`，区域键包含max_distance和几何版本，不重新拉取最新融合状态；后退/旋转覆盖、实测起点与motion_safe均来自同一handle。
4. **policy主循环**：选择→start→corridor→route的顺序先保留。所有查询返回同一snapshot_seq。发送MotionConstraint前复核源、clock/geometry/任务纪元；过期输出原有HOLD语义，不将更新过的单独health与旧世界拼成可执行判断。

第一实现可通过本进程C++ facade逐步替换Python重计算，复用持久ConservativeFusion及只读输入，不在每周期传JSON世界。后续将ROS接收也收敛到C++组件。若采用独立ROS进程，必须另定义有界、带版本的完整快照传输/查询协议，不能只订阅已有风险摘要就称完成route/corridor迁移。

## 回调、线程与时间

- 接收回调只校验结构、记录原始采集/接收时间、写入有界缓存。每路QoS沿用已验证配置；地图和包络不能随意改为丢失历史的订阅。
- TF查询使用scan采集时刻。先保留最近5帧、最后一个到达且TF可用的scan选择语义；现有测试明确区分它与“最大时间戳”，优化不能顺手改变选择规则。未来改选择策略须单独评审、差分和故障矩阵。
- 一个C++处理所有者串行修改fusion、map和profile；工作线程不持Python对象、不调用Python回调，pybind纯计算可在复制完有限输入后释放GIL。读取不可变快照可并行，候选结果仍需提交屏障。
- 每阶段记录ROS采集时效与steady耗时，避免以RTF约等于1直接混减两种时钟。ROS冻结/回退、重复旧帧、将来帧、纪元变化必须按原合同失效，steady看门狗不能等待仿真时间恢复。
- 剔除的是过期**待处理帧/过期计算结果**，不能剔除未被新鲜自由空间证实清空的静态占据。动态物体的年龄、速度和协方差预测完整保留。
- 独立保护继续拥有自己的有效输入与停止判定，不能因主策略心跳新鲜而放行过期预测。

## 最小可独立交付切片

| 顺序 | 改动和证据 | 放行边界 |
|---|---|---|
| R0：端到端采样 | 保留同输入、仅添加有界诊断；采集源stamp、接收ROS/steady、入队、首次TF就绪、选择/丢弃原因、处理各阶段和发布 | 能关联同一scan；区分源传输、TF等待、排队和计算，不由errors=0猜测TF |
| R1：控制器拒绝原因 | 导航任务负责C++ safeCommand分支原因与当前快照；记录检查阶段、costmap current/update/frame、geometry guard原因/本地租约、命令/实测twist、命中格及采样索引 | 诊断实施中；原bool结果及控制路径不变；独立解释08碰撞名称，不混入时效结论 |
| R2：C++持久融合与快照facade | 复用现有ConservativeFusion，提供不可变handle和区域预测；先离线，含原Python误差/异常语义 | 同输入差分通过；占据不漏、原始版本/时效不变；新边界只增拒绝不能静默放行 |
| R3：候选/通道接缝 | C++批量查询替换重复world构造；Python原状态转换暂保留 | route异步旧结果、start区域预测、corridor命中ID与原判断一致；一回合一个handle |
| R4：性能对照 | 固定输入、版本和相同CPU预算，AB/BA测完整链路而非仅核函数 | 处理P95/最大与源龄期改善，同时安全决策一致；未达门槛保留旧路径，不默认启用 |
| R5：统一导航实跑 | 先重复N4直线/返回90°与两项故障，再增加多载、稀疏/遮挡/高度及通道矩阵 | N4所有必选行分别通过；质量准入f92c5552新节点另做接线和时钟冻结验证；VLA/真机仍不在本轮 |

R0应按 `sensor_id/frame/clock_epoch/capture_ns` 加接收序号关联，记录重复/乱序；不要用到达时间冒充采集时间。若TF始终不就绪，保留最后缺失边及未来/过去外推原因。若scan本身已经老，先修上游；若接收新鲜而选中旧帧，再修等待/计算，不能仅从snapshot P95下降推断整链路恢复。

## 差分、多场景和AB/BA

固定129、约2100、约3000和6000条占据负载，包含相同静态cell重复、动态追踪、未关联视觉、真实free-space清除、遮挡后旧占据保留。覆盖矩形及本次带手臂/相机的固定凸包、单/双/偏置载荷；源缺失、299999999/300000000/300000001ns边界、重复/乱序、TF延迟/缺失/未来、ROS冻结/回退、来源/地图/定位/包络换代均独立测试。

执行层覆盖正常直线、返回90°、侧移、起点倒退对齐、L/U和窄通道禁止旋转、动态障碍、hold撤销、ACK失联及新授权恢复。故障拒绝、成功到位、基础设施失败、未运行分别计数。不能让成功的直线故障场景覆盖失败的返回转向。

AB/BA每负载至少4对、每实现20次预热+100次有效样本，固定地图/点云/TF/path/包络快照及CPU亲和；性能窗口独占，不与Gazebo双跑或构建竞争。记录CPU/RSS、接收等待、TF等待、scan/snapshot/risk/候选、输出发布的P50/P95/样本最大和总采集龄期。当前300ms源租约与碰撞/净空标准保持；由数据分配各段预算，不能改大timeout、伪造时间戳或减少物体换取通过。

## 现存检查入口（本设计未新运行）

只在选定离线构建已经配置好本仓库依赖后使用下列命令；本轮A4的mass_build可作为依赖清单参考，但不假定所有目标已构建。根目录colcon需要明确 `--base-paths ws_robot/src`，避免扫描runs重复包。

```bash
task_build=/absolute/path/to/owned/configured/build
cmake --build "$task_build" --target policy_fusion_probe policy_observer_core_probe policy_risk_probe policy_sweep_probe policy_health_probe policy_observer_cpp -j1
ctest --test-dir "$task_build" --output-on-failure -R '^policy_(fusion|observer_core|risk|sweep|health)_differential$'
ctest --test-dir "$task_build" --output-on-failure -R '^policy_risk_independent_replay$'
```

ROS wire replay会创建节点，必须等当前仿真释放后确认自己的domain/topic所有权；测试默认domain164不可当成已分配域。显式指定 `POLICY_OBSERVER_DOMAIN`、`POLICY_OBSERVER_CPP` 与 `POLICY_OBSERVER_GEOMETRY_BINDING`；不要沿用过期/tmp二进制路径。

```bash
python3 -m pytest -q ws_robot/src/astribot_s1_navigation_policy_native/test/test_policy_observer_ros.py
python3 ws_robot/src/astribot_s1_navigation_policy_native/test/benchmark_policy_observer_ros.py --output /absolute/path/to/new/evidence --workloads 8 64 129 2100 3000 --warmup 20 --samples 100 --pairs 4
```

现有benchmark使用合成视觉轨迹；它没有自动覆盖本次静态scan cell累积、Python route/corridor或真实源时延。须新增同一07/08风格的scan/TF回放与完整策略查询对照，才能作为R4依据。现有冻结参考中保留的兼容行为不自动代表正确性；发现旧缺陷时单列修复，不能以差分相等覆盖缺陷。

最终证据还需记录选中源码/二进制hash、完整数据时间线、控制权清理结果，以及每个场景实际使用的camera/几何/载荷/地图版本。新的C++源码存在或离线通过，不升级为动态验收。
