# W1 独占主机回归

重新计时：2026-09-22 15:46:13 +08，截止 17:16:13。用户授权关闭其他仿真并优先本窗口；首次完整进程核验未发现其他活动仿真。保留导航仓库资产、隔离 overlay 和所有安全门槛。

第一轮沿用最终 C++ 异步点云（27 ROS 契约测试 + 7 CPU/CUDA 后端测试通过；CPU-only 4 测试通过），Ogre4线程、原640宽20Hz相机、CPU头腹/CUDA双腕。实际进程与源/安装版本分开留证。独占性是 /proc 采样证据，非操作系统资源独占承诺；renderer_runtime_01.json 确认实际加载候选 core 和 ogre2 两个库。

15:53：motion_01 PASS，完成两个目标、原四个宽度/航向判定、1.3m通道穿越与保持失效撤销。安全净空下界0.287199686m，FOLLOW514样本 RMS0.009882986m/P950.022839269m；到位0.808/1.744mm和0.0420/0.0433deg。使用仿真位姿/TF，与物理精度区别。相机600秒仍采集中；CUDA GraspNet 500秒冻结点云压力重叠，既有模型，不是VLA或实时抓取。

上一轮 negative ACK 明确为 controller/local_costmap 上报 FOOTPRINT_OR_LEASE_MISMATCH 后撤销，非单纯ACK完全没到。新增仅诊断的细分原因源码（future/ROS stale/wall stale/frame/identity/footprint），尚未构建或进入本轮运行。当前数据尚不能唯一确定原负ACK原因。

16:00：完整独占 static-map/MPPI/GraspNet 子场景通过：600.000014s，RTF0.686293；四路健康全部OK，四路原始深度最大间隔202.46–207.13ms，>250ms均0。first_exclusive_result.json 已生成；独占性由初始枚举、运行快照和后续5秒采样支持，不是全时内核资源隔离。独占性能租约新增显式启动选项，普通启动共享锁/独占验证排他锁；4项文件锁测试通过，实际对运行中的未注册Gazebo拒绝启动且未创建第二实例。直接绕过本工具的启动仍需归属采样检查。

导航诊断源码已在冻结候选中构建，9/9测试组通过。但共享导航源码另有三阶段/到位控制改动（与11:07 manifest不同），因此本轮相机性能对照继续使用既有已验证 navigation_fix3 二进制；新候选未替换本轮控制库，不能声称只有诊断差异已部署。

16:03：mapping模式真实Voxel SLAM启动。最初带social-scenario的参数组合被启动器拒绝，未启动进程；去掉该仅允许baseline的夹具参数后进入原生导航仓库SLAM场景。生命周期active、扫描实际更新、map→base TF恢复；/slam/pose与/map_scan各8秒收到76帧且采集时间推进。此模式世界/扫描来源与social baseline不完全同一，性能不作严格单因素加速比。

16:08:25：camera_02_slam采集中出现原始stamp298.0s的共同STALE，最大已见头部wall_age373.819ms。恰与只读额外SLAM订阅探针启动重叠，尚无因果证明。原始失败保留，后续所有观察者在窗口之前启动再比较；不能排除故障而把本轮判PASS。相机会话与点云仍保持250ms门槛。

16:28：C++ 分段时延候选构建完成，27 个点云 ROS 用例、7 个 CPU/CUDA 后端用例、4 个固定内存 histogram 用例通过。节点实际安装到 metrics_overlay，未覆盖共享安装。

16:32：navigation_03_metrics_slam 使用 --exclusive-performance 启动；真实 Voxel SLAM、激活但静止的 Nav2、头腹 CPU 点云、双腕 CUDA 点云及 GraspNet CUDA 冻结输入重复推理。相机会话检查通过后开始 1800 秒捕获。运行时进程/二进制/renderer 哈希保存 rgbd_metrics_runtime_03.json。全部会话参数不变，诊断每5秒输出生命周期累计统计；不能把它们当严格捕获窗口分位数。

16:35：独占准入漏识别 Ruby/绝对路径启动形式，经 fixture 重现后修复。6项文件锁/识别测试通过；真实尝试启动第二监督器被 PERFORMANCE_LEASE_BUSY 拒绝，未创建日志目录或第二仿真。直接绕过工具的启动仍不受内核资源隔离限制。

16:41：采集器新增固定内存时钟统计；4项初始测试通过，补末尾静默后5项通过。camera_03 已在此前启动，使用旧采集器；下一场景使用新统计。新增验证 Python 不属于机器人运行时计算。末尾静默与相邻已收到消息间隔分开报告。

16:59：camera_03 尚在采集中，日志暂未出现 CAMERA_STALE，不能据此提前判 PASS。准备下一静止 Gazebo 暂停400ms的健康撤销/恢复场景。C++ 抽稀参数 int64→int 校验顺序存在溢出候选，新增4294967297回归用例，计划窗口结束后才构建验证。

17:05：用户取消本任务截止时间限制。task_timing.json 保留旧截止供追溯，当前 deadline=null，继续任务、不按17:16暂停。

17:07：camera_03完整1800.000088s通过采样新鲜度，四路原始深度最大接收间隔124.03–157.19ms，>250ms均0，RTF0.989854。889次GraspNet CUDA请求全部成功；请求序列与捕获重叠1776.873s（含请求间隙，不是GPU全忙时间）。真实SLAM+静止Nav2成立；实际运动+SLAM组合仍未验收。pressure_03_result.json 与 pressure_03_stage_metrics.json 已落盘。

17:07：C++抽稀溢出4294967297已RED复现，修复为int64范围检查后转换。28个ROS点云+7后端+4 histogram检查通过，安装在parameter_overlay；camera_03测量仍对应原metrics_overlay的冻结二进制。

17:10：时钟冻结测试第一次前置检查失败，无注入。查询侧按Gazebo父进程误设LOCALHOST_ONLY=1；实际/odom的parameter_bridge进程19559使用0及显式FastDDS相机profile。改为反读实际发布端配置后重试，不改任何时效/静止判据。首次失败输出保留clock_pause_04.json，不能当作被测保护失败。

17:23：camera_04完成600秒。实际暂停400.622ms；四路均撤销健康并由新帧恢复，但四路epoch均未变化，W1恢复版本门槛FAIL。保留clock_pause_04_analysis.json。后续C++修复：检测连续性断点时版本只递增一次，保留旧配对诊断；恢复需要新完整配对，会话重新有效后再接收完整帧；同token的任务身份变化/重复激活不得回到旧epoch。新增5个真实ROS回归用例，先在旧实现复现。

18:08续接核对：现存17:39的health_capture_boundary_red已复现延迟旧采集帧绕过；17:40加recovery_capture_after后GREEN，17:42补两次健康发布间短故障，17:46 CUDA构建14个CTest目标全通过（含27个CameraHealth用例）。这些产物已读取核实，不重复施工。navigation_03和04的自有清理remaining=[]；当前无Gazebo。将按已准备launch_navigation_05在CUDA独立安装重测。18:06文件名误带red的已有修复复查记录已更名reconnect_check，原真正RED证据不变。

18:15：相机版本恢复C++测试当前27/27；已加强真实CameraHealth→DetectionGate用例，旧候选未过期时明确CAMERA_SOURCE_MISMATCH，新epoch/新采集stamp正常输出，复验通过。只读复审无新增阻断。camera_05采集自18:09开始600秒；第一次注入前置检查未通过，无信号发送。加入失配分类后的重试18:12执行400.718ms暂停，停稳六维速度均0、odom墙钟16.74ms、clock墙钟3.35ms、ROS差14ms；判据未放宽。诊断曾记录消息到达次序导致6次odom超前clock；不能据此断定首次未记录原因的失败根因。

18:21：camera_05完整600.001160s；单次暂停400.718ms后的四路新epoch恢复全部通过。头/腹/左腕/右腕首健康观测109.58/60.38/77.47/73.67ms。故障窗口外、开始2s后无无效健康样本；恢复后epoch稳定直到结束。两个前置检查未过的尝试均未注入，保留记录，不作通过计数。腕部夹具清理remaining=[]；导航05新鲜反馈确认停稳后退出，自有残留=[]，当前枚举无Gazebo。W1整体仍未放行。下一施工细化为C++ GPU处理健康与持久子进程监督，计划文档已保存、未宣称实现。

2026-09-22T18:23:41.441652+08:00：收尾完整复验14/14 CTest目标通过（118个GTest用例，2个原生测试目标），无跳过/失败。implementation_status.json保存本次完成项、旧失败、未注入记录和后续门槛。当前无自有仿真残留，原90分钟截止已取消。
