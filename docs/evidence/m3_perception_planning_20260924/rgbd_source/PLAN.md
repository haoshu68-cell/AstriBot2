# RGB-D 单箱体请求源

2026-09-24 10:46 开始独立主线子项。总调度已批准本包内库实现，不启动模型/GPU/新仿真，不修改导航基线。前置核查：现有 object_pose 只给表面质心，旧 Python 分离器不是所需 C++ Request 生产者；现有 C++ projector 能提供实际 XYZ 与 ProjectionHealth，复用其 decimation=1 全像素（含 NaN）输出契约。

最小实现：同一已 spin 的 M1 节点挂 RGB/CameraInfo/XYZ/两类健康订阅，有界缓存严格相同采集戳；查询该戳 TF；用已有橙色 HSV 范围识别唯一连通区域，保留原始有限 XYZ，核对实际 K 与像素注册；2048 点下限、80% 有效深度门槛，最多12000点只减采样。不得读 Gazebo pose 或使用配置抓姿定位目标。工位唯一实例绑定和 identity_revision 由 M1 权威输入，颜色不能建立持续身份。

真实数据前置：注册的同frame RGB/深度、D=0实际CameraInfo、decimation=1原投影器、当前camera和processing epoch、TF、M1实际场景/标定/包络/clock版本。相机内参规范hash加入Context，原CameraInfo保留在Request，便于推理期间版本变化失效。当前真实目标点数未知，配置估算见sampling_feasibility.json，不能声称可用视角。

验收：纯C++正确选择已知合成颜色/真实值XYZ、原stamp/Info/health/TF/身份保持；多目标、点少、低深度有效率、错帧/布局/畸变/内参或epoch不一致、过期、工位身份不匹配拒绝；独立库构建、安装导出可用。ROS订阅联动/真实渲染帧/模型端到端另需所属会话验证。单编译使用MAKEFLAGS=-j1并核command.log或直接--parallel 1，不能仅依赖CMAKE_BUILD_PARALLEL_LEVEL。

实施中确认的必要边界：仓库背景存在同色橙色区域，必须先用独立工作台范围筛选；范围不可来自目标实体姿态。原始数据和健康回执记录ROS/steady双时钟，将迟到与后续暂停消耗计入同一个原始期限；Request和PlannedPick继续传递期限。独立审查指出的迟到200 ms后ROS暂停100 ms漏洞已纳入定向用例。最终证据和实际视角未就绪状态见README.md，不改变10:46起点。
