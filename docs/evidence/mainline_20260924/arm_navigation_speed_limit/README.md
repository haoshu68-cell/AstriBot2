# 上肢限速职责前移（2026-09-24）

旧包32文件已归档并逐项SHA256读回验证。此改动仅将原比例算法的输出改为专用上游SpeedLimit；0%=HOLD，不得接到原生/speed_limit。去除Twist输入与输出，关节采集/接收时效均上限500ms，50ms重发保留源采集时间，缺失/无效/过期/乱序/时钟回退输出0。TF缺失沿用full-reach并立即采用minimum，不继承较高平滑值。

验证：2项源码边界测试通过；3个Python源文件AST检查通过。新增C++边界测试及单一有界ROS协议已编写，但本任务未构建/未运行ROS，交总任务在隔离域统一验证。原纯算法差分保留；旧Twist ROS差分已退出当前CMake入口，历史脚本与证据未改。

运行入口：DYNAMICS_CPP=<新构建可执行文件> DYNAMICS_DOMAIN=36 ROS_DOMAIN_ID=36 ROS_LOCALHOST_ONLY=1 DYNAMICS_EVIDENCE=<证据目录> python3 -m pytest -q ws_robot/src/astribot_s1_dynamics_coupling/test/test_navigation_limit_ros.py。域号只由总任务预检分配，不自动选择或探测其他会话。

本包另有本轮之前的Python移除/C++迁移未提交变更，本次精确范围见after_sha256.json与upstream_limit.patch，不可全包暂存覆盖历史归属。


## 最终验证与旧入口退休

总任务统一完成最终构建、安装；原纯算法差分与源码边界通过，核心测试浮点断言修复后单项复跑通过，最终 producer ROS 协议 1/1 通过（domain 36，节点与驱动 exit 0，无归属残留进程）。详细原始证据见 verified/ 与 verified_evidence_sha256.json；保留原核心失败日志，不替换失败历史。上述是隔离协议证据，尚非 Gazebo 全搬运闭环。

仅删除已校验原归档SHA的 test/test_ros.py、test/benchmark_ros.py；删除清单见 retired_entrypoints.json。核心差分依赖的 test/reference 未动，运行源/新测试未动。
