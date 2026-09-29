# 只读probe静止入口修正设计（未实施）

2026-09-24。总调度批准设计，当前M2实际场占用资源，禁止构建/ROS/GPU。新发现的包络逻辑问题从约12:56记录，检查点13:56；原11:52→12:52的source/model场仍BLOCKED_PREREQUISITE、NOT_RUN，不重置或合并成模型失败。4eac3656源码/产物保持冻结。

## 1. 目标、边界与通过条件

只读验证必须同时满足：真实typed Hold持续有效；完整六ACK证明当前固定几何已安装；当前没有观测到导航任务进入活动状态；实际底盘命令、里程计速度及位姿持续满足项目已有停稳标准。`navigation_allowed=true`只是既有coordinator的几何就绪结果，不是probe拥有导航权限或底盘正在运动的证据。

拟将冻结probe的`!navigation_allowed && (HOLD || FIXED_POSTURE)`替换为**fresh FIXED_POSTURE + 当前六ACK + 真实typed Hold + 底盘静止观测**。具体要求envelope.mode=FIXED_POSTURE、navigation_allowed=true、limits.transport_ready=true及真实版本/有效期一致。未齐ACK、已撤销包络、HOLD模式或过期包络均不能作为本场就绪；不修改coordinator/ACK，不调用撤销服务制造false条件。

通过结论仅为一份在规定静止观测条件下有效的source/双模型结果；不宣称绝对零运动、机械装配精度、物理制动保证或生产执行权限。任何失效终止本次验证，最多1秒收集本次两个推理Goal的取消/终态；不取消导航任务、不发零速、不释放别人的Hold。

## 2. 精确新增输入

|输入|类型与QoS|绑定与用途|
|---|---|---|
|`/navigation/arm_hold`|astribot_navigation_msgs/msg/ArmHoldStatus，reliable/volatile/depth10|hold_confirmed=true；owner_id等于owner config新增必填hold_owner_id；hold_id等于当前owner配置及envelope；attachment_revision同时等于geometry/envelope；真实源戳/lease及原接收时钟有效|
|`/navigation/envelope_applied`|astribot_navigation_msgs/msg/EnvelopeApplyStatus，reliable/volatile/depth20|六消费者global_costmap、local_costmap、planner、controller、policy、protection；同coordinator_session_id、envelope_epoch、installed_geometry_hash，applied=true且源/接收均新鲜|
|`/odom`|nav_msgs/msg/Odometry，SensorDataQoS|真实header.frame_id=odom，与既有measured_stop一致；记录实际child_frame_id并绑定其本场值；有限位置/速度、单位四元数；原capture ROS和本地receipt ROS/steady用于年龄与窗口|
|`/cmd_vel`|geometry_msgs/msg/Twist，SensorDataQoS|本场最终保护节点输出、底盘实际订阅的命令，不读cmd_vel_policy_input。平面vx/vy/wz有限且接近零；消息无header，只能证明接收时间，不能宣称源端采集时间|
|`/navigation/execution_status`|astribot_navigation_msgs/msg/NavigationExecutionStatus，reliable/transient_local/depth10|覆盖正式arbiter各入口的task_id/source/sequence/state。注册后观察到ACCEPTED、EXECUTING或CANCELING即使速度仍零也终止；记录原始事件，随后terminal不能复活本次job|

其余geometry、envelope、CameraHealth、专用ProjectionHealth、RGB/Info/cloud、TF与Scene查询沿用冻结入口。新增hold_owner_id须取M2真实持有者，不能从probe自己task_id推导；该owner不等于相机source_epoch或coordinator_session_id。

### 导航入口证据的已知边界

execution_status是事件流，当前arbiter只在转换时emit，没有IDLE心跳或完整当前任务列表服务。不能用“没消息”、一个terminal或最近10条缓存证明没有在执行/排队的旧任务，也不为该事件流发明0.3秒心跳门槛。

沿用本次明确的M2独立冷启动场前提：M2按现有verify_fixed_mass的进程身份/图检查及真实Action所有权确认导航入口清空，并在owner_evidence中留证；从确认到probe结束不安排导航业务。probe订阅用于发现之后的活动事件。现有owner_evidence仅保留文件hash，**不代表probe已独立解析、验证了冷启动证明**；这一前置仍由M2承担。无法确认冷启动/入口归属，或publisher/进程更换，保持BLOCKED，不以配置布尔值声称已经证明空闲。无需新增状态发布器或生产权限服务。

`/cmd_vel`最终链的源码依据是final_protection_node.cpp:76与底盘launch/config；正式场必须核实实际参数、remap、唯一publisher和底盘subscription确实吻合。源身份由M2现有会话所有权记录绑定；不能只看同名topic。本入口提供失效检测，不替代命令独占和制动控制。

## 3. 阈值逐项溯源（不另造精度门槛）

首个登记前，所有数据必须来自本次入口就绪之后，按M2实际使用的`tools/sim/verify_fixed_navigation.py:33-60 measured_stop()`完成连续停稳窗口；此函数由`verify_fixed_mass.py:157,338-344`复用。其原值如下：

|判据|原阈值及比较符|来源|
|---|---|---|
|观测窗口|最近0.7秒源时间；至少12个独立源戳样本；跨度≥0.6秒；首样本晚于本次准入观察起点|measured_stop:34-37,55|
|样本连续性|相邻源戳0<Δt≤0.12秒；不以重复源戳增加样本数|measured_stop:37|
|里程计新鲜度|0≤now_ROS−stamp<0.3秒，0≤now_steady−receipt_steady<0.3秒|measured_stop:52-54|
|命令新鲜度|每个odom样本绑定其收到时最近命令，0≤odom_receipt−cmd_receipt<0.3秒；当前命令亦需持续<0.3秒|measured_stop:54及verify_fixed_mass:272,448的fresh(command)；“持续”是本次已批准的入口检查语义|
|速度|每个窗口样本平面速度≤0.01m/s，abs(wz)≤0.02rad/s|measured_stop:49-57；不采用较宽的单帧0.02/0.03来替代窗口|
|命令|abs(vx)、abs(vy)、abs(wz)各≤1e-6（分别为m/s、m/s、rad/s）|measured_stop:51,56；verify_fixed_mass:304|
|窗内位移/角度|相对窗口首帧最大XY漂移≤0.005m，解缠yaw最大变化≤0.01rad|measured_stop:41-45,57|
|慢漂移|窗口x/y线性拟合速度范数≤0.01m/s|measured_stop:46-48,57|
|固定基准位姿|登记时固定一次完整odom位姿；之后3D位移≤0.02m、四元数最短旋转角≤0.02rad，不随着窗口滑动更新该基准|hold_executor.cpp:605-612与execution_guard.cpp现有MTC_BASE_MOVED检查|
|位姿数值|每分量有限；abs(norm(q)^2−1)≤0.001；frame与本场基准一致|hold_executor.cpp:476-478、execution_guard.hpp executionPoseValid|
|typed Hold期限|0<lease_s≤0.5秒；now_ROS<stamp+lease，严格未到期；producer实际remaining≤0.3秒|fixed_envelope_core.cpp:158-164；arm_hold.cpp:134-149|
|六ACK|各源龄及receipt steady龄严格<0.5秒；future stamp不接受；同session/epoch/hash|fixed_envelope_core.cpp:198-213,237-242；verify_fixed_mass.py:46-61|
|geometry/envelope|原0.3秒ROS/steady年龄、原valid_until；不放宽|冻结probe的fresh()/current()|

原Python窗口选择有1e-8秒浮点松弛，只作用于0.7秒尾窗选样，不是许可有效期宽限。C++若使用整数纳秒，应将它明确限定为10ns尾窗边界，并与原函数的同输入结果对照；其余lease/capture截止不加epsilon。

typed Hold、odom、ACK等带源戳消息以**首次接收**建立ROS/steady锚点，重复同源戳不更新锚点、延长lease或增加停稳样本。同源戳不同内容为冲突，失败；回退在登记后终止。保持到期时间同时受`receipt_steady + (source_expiry − receipt_ROS)`限制，沿用冻结source/probe已有双时钟方法，晚到和暂停消耗剩余期限。

Twist无源戳，数值相同的零命令可以是真实周期发布，不能仅凭数值相同就判重放；其receipt只证明下游收到消息。未收到任何命令或断流是未知/失效，不能因为底盘watchdog可能置零就默认静止。六ACK同戳重复亦不续期；匹配当前绑定的negative ACK在登记后立即锁存失效，下一条positive不恢复旧job。

## 4. 最小执行与失败语义

- **登记前**：M2前置通过后，等待真实Hold、fresh FIXED_POSTURE、六当前ACK与原相机/geometry就绪；开始本次停稳窗口。仅缺样本时等待，沿用60秒总上限；不发任何Goal。数值/身份冲突显式拒绝，不用默认值填充。
- **登记时**：在首次Scene请求发出前进入已绑定阶段（arming）；从此刻开始锁存异常，而不是等registration.json写完才开始。冻结owner/hold、coordinator、epoch、installed_geometry_hash、geometry来源/模型/附件/clock、odom父子frame及基准位姿；然后按原流程新读完整Scene并生成30秒验证登记。不得把收集停稳窗口的时间算作给旧Scene续期。
- **登记后**：每次current()及新消息回调按各自边界检查；观测到非零命令、速度越限、Hold变false、相关negative ACK或新导航活动应锁存，使“坏→好”发生在两次轮询之间也不能漏掉已收到的坏消息。继续检查滚动停稳窗口、固定基准与原Scene/感知上下文。
- **失败**：若尚未发Goal，直接退出；若已发两个推理Goal，只取消其具体句柄，保留首次原因、UUID和逐端终态。失败后的正向消息不重开本job；不停止导航或改变Hold/包络/规划场景。
- **原合同保持**：输入准入0.5秒、capture+5秒结果、每端默认4秒、登记30秒、总60秒、Scene≤1秒读回、1秒取消收尾全部不变。未来raw bag仍必须覆盖实际capture，3秒发现/启动竞态是另一个采集缺口，不由本修正隐藏。

## 5. 精确建议修改范围（待实施授权及资源窗口）

- 修改`tools/vision/m3_source_probe/probe.cpp`：5条只读订阅、hold_owner_id、样本/初始基准、current gate与必要证据。删除旧导航禁止条件，按本设计替换；不改冻结感知库、coordinator、HoldResources或任何生产包。
- 修改该目录`CMakeLists.txt`：显式声明新增nav_msgs/geometry_msgs依赖，增加一个仅测试目标。
- 建议新增该目录`stationary_gate.hpp`与`stationary_gate_test.cpp`：小型probe私有纯判据供实时循环和测试共用，不安装为公共库，不做可配置策略框架。重复采样、时间与绑定判据用实际同一实现测试，避免继续仅测试复制表达式。若总调度要求严格两文件范围，需另定测试组织，不能把运行时代码复制一份声称测到同一实现。
- 新证据目录与新build/install/binary，不覆盖4eac3656及现有manifest。当前只是设计，以上文件尚未写入。

## 6. 最小验证计划（全部NOT_RUN）

|ID|不变量与刺激|独立判据/预期|层级|
|---|---|---|---|
|SG1|真实形态fresh FIXED_POSTURE、nav=true、六ACK、匹配typed Hold与停稳窗口；反例为缺一ACK/撤销/过期|前者可准入，反例拒绝；不要求coordinator输出互斥的false+六ACK|纯C++输入夹具|
|SG2|owner/hold/attachment/epoch/hash/frame逐项变更；同戳异内容；匹配negative ACK紧接positive|冻结绑定不能迁移；登记后锁存首错|纯C++参数化|
|SG3|与现有measured_stop共享原始样本；对速度/命令/漂移/拟合速度/跨度/间隔逐项t−δ、t、t+δ|对照原Python判据和明确比较符；距离/速度δ用相邻可表示double，时间δ=1ns，样本数11/12/13|纯C++与离线数据对照|
|SG4|ROS冻结、late delivery、重复odom/Hold/ACK、命令断流、未来戳、ROS回退|steady期限仍消耗；重复不延寿/增样本；无命令拒绝；窗口断裂不能登记|纯C++双时钟夹具|
|SG5|缓慢移动使每个滚动窗仍通过但相对固定基准超0.02m/0.02rad；q和−q等价；NaN/Inf/非单位q|基准不滚动；超限拒绝，等价四元数不误报，非法设备输入失败|纯C++边界|
|SG6|登记后命令坏→好或导航ACCEPTED→终态发生于两次current()之间|观测到的坏事件保留；不因恢复而接受原job|纯C++回调序列|
|SG7|两个合成推理Goal进行中注入运动/typed Hold失效；对端延迟接受或终态不明|仅本次Goal取消，≤原1秒收尾；原错误保留；无控制/Scene写客户端；迟到成功不通过|下一获准离线ROS窗口，最少2次合成job|
|SG8|M2拥有的新READY静止Hold场，一个真实source/双模型job|真实typed Hold/六ACK/odom/cmd/导航入口和raw同capture证据独立核对；分别判时效、算法、原始证据|总调度另行安排真实仿真；当前BLOCKED|

纯判据和接线验证通过才申请SG8。SG7复用既有取消/结果判据，只补新静止输入导致的取消，不重跑原72项/静止质量全矩阵。测试缺参数/会话权限时标BLOCKED+NOT_RUN；预期拒绝通过不计为模型业务成功。禁止在当前M2实际场启动这些测试。

## 7. 提请总调度确定的实施边界

本设计已有静止数值依据；还需确认采用上述4文件probe私有范围，以及M2冷启动/导航入口排他前置作为本轮明确所有者责任。该责任当前不能由事件流自动证明，若要求probe独立枚举所有旧导航任务，则需要额外接口，不属于本次最小修正。当前不增加该接口，也不以静默假定代替它。
