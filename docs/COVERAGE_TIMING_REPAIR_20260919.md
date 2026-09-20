# 覆盖时效与安装一致性修复（仿真）

本轮继续P3/P4仿真验收，未操作真机。证据目录 `/tmp/astribot-coverage-fix` 是临时实验记录，未视为持久归档。

## 定位与修改

r20b第二段导航766条有效包观测中，355条required_sensors_valid=false；扫描年龄中位0.355 s/P95 0.513 s，策略观测计算墙钟中位0.237137 s、CPU中位0.134120 s，轨迹记录约2403条。对应约束确实返回REQUIRED_COVERAGE_UNAVAILABLE，不能当作不必要的保护删掉。

核对实际安装发现两项可修复问题：

- 几何C++扩展的CMAKE_BUILD_TYPE为空，flags中没有优化级别。默认配置改为RelWithDebInfo，尊重显式Debug/Release和多配置生成器；未开启fast-math，不改变碰撞、时效、跟踪门槛。
- 隔离安装的navigation_policy仍为旧标量实现。原生接口回归21通过/1失败，失败为旧fusion不接受free_many；同步构建调用方后53项差分/时序/导航回归通过。此C++迁移本身为工作区已有实现，本轮修复安装一致性，不冒称新开发整个内核。

相同随机种子、3000盒、720射线、200次调用的scan_boxes_free基准：CPU中位2.400728 ms→0.694369 ms，输出SHA256一致；不能将单内核数字直接当作ROS端到端改进。启动前验证observer实际调用同一扩展的scan_boxes_free，记录模块路径/hash到runtime_modules.json。

## r21 仿真结果

实际domain213、partition=astribot_operator_validation_213；probe确认8控制器active和时钟/数据推进，Nav2全部active。保持true/kp10，MTC速度与加速度缩放0.03、关节余量0.1，原0.05 rad跟踪门槛保持。

视觉定位→抓取→附着→带载收臂→两个导航点均完成。第二段409条观测的required_sensors_valid均为true；计算墙钟中位0.153754 s、CPU中位0.100829 s，扫描年龄中位0.212 s/P95 0.286 s，记录数中位2881。是整套安装/优化组合的观察对照，不是控制所有负载因素的独立因果试验。

包内1312条sensor_health均为VALID，但最终约束仍有152条REQUIRED_COVERAGE_UNAVAILABLE，集中ROS时间120–139 s及150–159 s；该门禁尚未彻底消除。原因诊断存在采样时间差：observer在risk计算前发布health，policy在计算后重新检查时效。因此新增policy状态字段coverage_ok、coverage_motion、coverage_health，保存决策时刻的原始采集/有效期、必需标志及覆盖扇区；仅为已有Python适配层序列化，不增加算法或运动权。字段已构建，未在本轮r21中录制，需下一轮实跑验证。

### 新执行阻塞

预放置PREPLACE触发MANIPULATION_TRACKING_ERROR:astribot_arm_left_joint_6，整轮229.822 s，结果FAULT/RECOVERY_REQUIRED，不能算完整搬运成功。

ROS时间169.250 s控制器实际joint6=0.100499 rad、期望=0.048823 rad，误差0.051675 rad；后续169.270 s误差0.055159 rad。实际值没有处于之前joint3的3.1 rad硬上限，不能复用旧限位卡滞结论。保护取消返回terminal action_status=5，stop_error为空，载荷版本3/ATTACHED保留。下一步需结合完整轨迹、实际关节运动与接触/执行模型定位joint6偏差，不调大保护门槛。

录制正常stop成功后才读取SQLite；保留normal_r21.json、metrics_r21.json、constraints_r21.json、session_r21账本及incidents_r21。

## 后续顺序

1. 用新增决策快照细分剩余覆盖失效；定位joint6预放置跟踪偏差，完成同配置连续完整搬运与取消回归。
2. 单列DDS生命周期启动超时复现；r21本次全部激活成功，不代表偶发问题解决。
3. 继续P1/P2探索完成/取消存图、多点循环取消、回放与多地图事务验收。
4. P4队列/WCS幂等与事实恢复，之后P5动态工位/跨层/载荷验收；仍保持仿真先行。

r21结束后仅清理自有49个进程，保存PID/启动时间清单，remaining为空；无全局清理、共享内存删除或真机命令。

续作：[夹爪物理耦合与接触诊断](GRIPPER_MIMIC_AND_CONTACT_REPAIR_20260919.md)。r22捕获夹爪/台面接触并定位未注册的mimic；r23修复后完整通过且决策覆盖失败为0。该结果不回写成r21已经通过，重复轮次及后续验收以续作文档为准。
