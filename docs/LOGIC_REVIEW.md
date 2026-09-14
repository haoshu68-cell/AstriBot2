# 逻辑问题修复清单

按“保留核心特性和参数，修复不能降低路径跟踪效果”的要求，本轮只修改以下问题对应的分支。
三阶段切换、惯性制动、接近减速、到位控制、多定位源接口和规划算法保留。测试脚本仅放在 `/tmp/astribot_logic_fixes`，未重新加入仓库。

| 编号 | 位置 | 已实施修复 | 验证 |
|---|---|---|---|
| 1 | navigation/arm_speed_limiter_node.py | 使用原有 check_period 周期重发限速；状态变化日志保持原规则 | 真实 ROS 订阅晚启动及重建：旧版均 0 条，修复后各 4 条/2秒 |
| 2 | navigation/posture_monitor_policy.py | 恒定姿态不再触发自动停用；沿用 enable_posture_monitor 显式配置和原有采样窗口、阈值 | 静止 64 帧后仍监控，随后超限正确停车 |
| 3 | navigation/cmd_vel_body_to_world_node.py | 检查速度有限性、里程计四元数与时间戳；需要里程计时拒绝陈旧数据；独立稳态时钟看门狗处理命令断流，空闲不持续抢占其他输入 | 有效命令直通及旋转数值不变；陈旧/无效数据和断流输出零速 |
| 4 | dynamics_coupling/arm_chassis_speed_coupling_node.py | 要求监控 TF 齐全、新鲜、有限；失效时清空缓存并采用既有满量程保护；静态 TF 零时间戳可用 | 部分 TF 缺失、过期均不能复用旧收拢值；正常伸展曲线和平滑公式保持不变。导航二值限速节点同步校验 TF |
| 5 | tools/s1_hardware_bringup*.sh | 停止拷贝旧 YAML；通过 map_topic=/map_nav、map_transient_local=false 指定实机地图；旧 --keep-nav2-yaml 调用仍可解析 | 两份脚本静态检查、两种控制器 launch 参数展开检查；未执行实机运动 |
| 6 | perception/livox_preprocess_node.py | 直接按三维整数体素坐标分组，消除 XOR 碰撞 | 两个各含单点的碰撞体素：旧版错误保留 2 点，修复后正确丢弃 |
| 7 | perception/livox_fusion_node.py | 独立输出消息头，选取实际参与融合点云的最早时间；拒绝超前时间；不支持的 deskew 在启动和动态设置时明确拒绝 | 输入头不变、未来帧拒绝、单侧成功仍可发布；未宣称实现逐点去畸变 |
| 8 | navigation/navigation.launch.py | smoother 的 XY 上下限同步受 max_linear_speed 约束，保留 YAML 中更严格的限值及角速度配置 | MPPI/RPP × 0.2/0.5/1.0 六组合；运行实例读取为 ±[0.5,0.5,2.0] |
| 9 | navigation/nav2_full_bringup.launch.py | RViz 时钟跟随 sim/hardware 环境 | launch 展开分别为 true/false |
| 10 | tools/robot/env_robot.sh | SDK 根目录优先采用 ASTRIBOT_SDK_ROOT，否则识别同目录部署和 tools/robot 仓库布局；缺少 env.sh 明确失败 | 临时目录布局验证 |
| 11 | tools/robot/bringup_stages.sh | 按 NUL 分割环境，校验变量名并用 shlex.quote 转义；环境文件权限 0600 | 含换行、引号、反引号、$() 的值原样往返，未执行其中命令 |
| 12 | ThreePhaseController | 只将相位计算用位姿变换到路径坐标系；内层控制器输入和输出消息头保留原始坐标系 | 非恒等 map→odom 下，相同物理位姿给出相同旋转命令；缺失 TF 抛异常；旧库对应回归失败、新库通过 |
| 13 | navigation/package.xml | 增加 path_tracking 运行依赖 | 包依赖声明检查、全工作区自研包构建和实际插件加载 |

新增的里程计/命令超时默认均为 0.5 秒，TF 新鲜度默认 0.5 秒；融合帧允许最多 0.05 秒的轻微超前。
这些参数用于异常数据处理，不替换原控制参数。无姿态能力的里程计应显式关闭姿态监控，仍需姿态监控的部署必须配置正确的高度基准。

完整仿真对比、已知限制与证据见 [LOGIC_FIX_VALIDATION.md](LOGIC_FIX_VALIDATION.md)。
