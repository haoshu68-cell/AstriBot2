# Final protection: final Release evidence, 2026-09-21

本轮使用最终源码对应的独立 Release 可执行文件：`/tmp/astribot_fixed_v2_clean_install/lib/astribot_s1_navigation_policy_native/final_protection_cpp`。其 SHA256 为 `fa6494752f8e229d7a9412b2adaa0258e573b007083968b0b4a0b55841416ee9`。源码与 no-compat 安装目录中的 `geometry_kernels.hpp` 逐字节相同，SHA256 为 `e36f9fa5fb5505c097caa32c54f601a2fbff7f7c493fd597a85b087e6ef69f17`；编译依赖文件指向该安装头。构建使用 Release / `-O3`、`ASTRIBOT_BUILD_PYBIND=OFF`，没有 geometry 源目录 include override。完整相关源码清单共 101 项，从测量开始到结束均未改变；详见 [source_manifest.json](source_manifest.json)、[构建来源](build_provenance.json)、CMakeCache/flags 文件与 [动态依赖](ldd.txt)。最终可执行的动态依赖全部解析，未链接 libpython 或 pybind。

CPU/RSS 数值为每种模式、每种实现 3 轮的中位数；箭头左为 Python，右为直接 C++。

| 模式 | 8 秒窗口 CPU 累计 | 单核 CPU 占用 | 采样 RSS | CPU 降低 | RSS 降低 |
| --- | --- | --- | --- | --- | --- |
| legacy | 2.57 → 0.16 s | 32.12% → 2.00% | 61.49 → 26.25 MiB | 93.8% | 57.3% |
| fixed_v2 | 2.82 → 0.20 s | 35.25% → 2.50% | 61.89 → 28.39 MiB | 92.9% | 54.1% |

每轮400帧，1080条激光射线，输入命令0.1 m/s、实测速度0.12 m/s，同样的proposal和包络。默认2 m静态激光环，固定40帧放入0.45 m近障碍；恢复仍经过原始0.6 s clear hold。每种模式分别采用Python→C++、C++→Python、Python→C++三组配对，legacy/fixed_v2组内次序也交替。12轮均实际接收400帧，共4800帧；实测输入速率为49.99904～49.99979 Hz。

每条接收的约束输出均检查序列单调、有限值、速度上限、禁止分量与HOLD零限速；每条Twist输出均检查有限速度和幅值边界。稳定障碍区间要求 `INDEPENDENT_SWEEP_RISK`，稳定净空区间要求 `CLEAR`，总计检查7674个稳定净空输出、791个稳定拒绝输出。正式12轮全部通过，无筛除失败trial。分离DDS话题的转换边界不用于严格逐帧原因比较，但所有边界输出也必须满足安全不变量。

测量只涵盖独立ROS节点进程，域114、localhost-only，没有Nav2、Gazebo、SLAM或硬件。统计采用owned child `/proc/PID/stat` 的user+system CPU，包含节点全部线程和DDS工作，排除fixture进程及启动预热；CPU计数分辨率为0.010秒。RSS每20帧采样，包含解释器、运行库和映射共享页，不等同独占物理内存。Python基线显式设置 `ASTRIBOT_NAV_NATIVE_KERNELS=0`，保留现有geometry扩展；该扩展自身的路径和SHA256也在build_provenance中。

**未测端到端延迟。** 输入Twist没有输入序号或时间戳，输出约束sequence属于输出端，无法可靠关联触发输入。fixture的wall时间、clock确认等待和deadline lateness只是输入节奏证据，不能解释为算法时延或速度收益。正式脚本在发送每帧输入前，用保护输出stamp确认该节点已处理相应 `/clock`，避免跨DDS话题到达次序造成未来source stamp；未放宽生产TTL或保护门槛。资源降低不意味着相同倍数的控制响应提升，不外推为硬实时最坏延迟、整栈或真机验收。

主机未锁定专用CPU；根代理同期有独立domain116 envelope实验，机器其他开发工作也可能影响绝对值。逐轮loadavg保存在原始记录，三组交错配对用于减弱次序影响；这些是本机资源样本，不提供统计置信区间。最终Release结果取代先前候选数值，先前 `/tmp/astribot_protection_benchmark_20260921_clock_barrier/` 候选数据未覆盖。

原始记录在 [final_release_results.json](final_release_results.json)，每轮 `sessions/*/wire.json` 保存逐输出字段、输入日程、资源采样和最终激光诊断，`sessions/*/session.log` 保存节点日志。[benchmark.log](benchmark.log)保留执行进度与汇总。这里的脚本/fixture副本用于冻结证据；实际复现应运行仓库 `ws_robot/src/astribot_s1_navigation_policy_native/test/benchmark_final_protection.py`，因为该脚本按仓库相对路径加载配置。

复现：先source ROS Humble、ws_robot消息/Python oracle overlay和 `/tmp/codex_geometry_cpp_20260921/install-no-compat/local_setup.bash`，设置 `ROS_DOMAIN_ID=114 ROS_LOCALHOST_ONLY=1 ASTRIBOT_NAV_NATIVE_KERNELS=0`，然后运行 `python3 <repository benchmark script> --binary /tmp/astribot_fixed_v2_clean_install/lib/astribot_s1_navigation_policy_native/final_protection_cpp --output <new directory>/final_release_results.json --pairs 3 --ticks 400 --rays 1080`。所有进程仅按实验持有的PID退出，不操作共享栈。
