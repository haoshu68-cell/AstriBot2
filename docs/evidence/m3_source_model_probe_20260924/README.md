# M3 source 与双模型一次性入口：离线检查点

> 历史版本留档：本文记录 `4eac3656` 初版及其当时路径，不是当前运行入口。初版 `probe_build` 已在逐文件归档校验后删除，映射见 [清理结果](../mainline_20260924/validated_version_cleanup_result.json)。当前静止入口修订为 `497c4e90`，20 项 C++ 检查通过，见 [修订证据](stationary_gate_revision/README.md)；实际 source/双模型仍未验。后续接线以 [当前操作清单](LIVE_OPERATION_CHECKLIST.md) 为准，包含必填 `hold_owner_id`；不得执行本文的旧二进制路径。

2026-09-24，问题起点11:52，原检查点12:52不变。本次仅完成入口实现、隔离构建及下列离线核验。没有启动ROS节点、Gazebo、GPU或模型，不能作为真实source/双模型闭环验收。前面的clock0修正另有7项合成ROS协议证据，本次不重复计入。

## 已核验

- 两文件C++入口编译、链接通过，显式单编译器；实际二进制`--help`通过。
- 精确抽取入口自身的观察/清理lambda，8项确定性纯C++检查通过，启用UBSan且无诊断。包含轮询间结果到达、等待接受、拒绝、UNKNOWN、失败终态与UUID、异常result future、取消异常不跳过peer、未发送。这里的对端是接口替身，不是ROS Action服务器。
- 实际二进制5项启动前配置拒绝检查通过：缺参数、多实例、零登记号、缺原始录制目录、已存在输出目录。均在rclcpp::init之前结束。
- 独立静态复核SHA256 `46bb78d7ba8f2876f0b1dee583a261aa96d1568f7385bc48614ca26fb9949c34`，未发现剩余可行动代码问题。首轮发现和修正包括逐端清理隔离、启动Scene时序、Goal/终态证据、时效与算法分离及单轮轮询一致性。
- 公共Scene规范库4个既有artifact hash重新匹配。没有复制或编译兄弟包实现。

## 加载路径与运行入口

必须先在本场所有者给出的ROS环境下source本目录`runtime_overlay.bash`，再做图、唯一发布者、实际进程/动态库和参数预检。此脚本仅设置probe依赖路径，不启动进程，也不授权运动。二进制为仓库下`runs/m3_source_model_probe_20260924/probe_build/m3_source_probe`，配置字段及调用见PLAN。

原始环境曾优先加载ws_robot/install中的navigation消息库，hash为903a5706…，与编译时I0_2冻结库6f3c536b…不同。`probe_ldd_before_binding.txt`保留现场；新环境明确绑定编译用消息库，`probe_ldd.txt`及manifest记录实际路径/hash。当前只验证直接依赖解析；真正ROS运行中按需加载的typesupport仍需本场进程读回。

## 实际场次的前提

M2最近一场没有建立成功Hold/六ACK，不能产生本probe租约。必须由M2重新建立真实静止Hold/六ACK、单实例所有权、专用全像素点云投影器及新鲜健康信息。probe独立读回完整Scene并用公共canonical语义登记验证会话快照，登记后任何合法Scene变化均失效；不代表生产M1 Context接口。

M5给M2准备head限定有界录制。配置input_recording只是关联目录，报告raw_input_evidence始终为EXTERNAL_RECORDING_PENDING_VERIFICATION；最终要用实包的同capture/stamp/epoch核对原RGB、完整provider点云、CameraInfo、CameraHealth、ProjectionHealth、TF、clock及外部接收时钟。目录存在、分割后Request CDR或收尾最新消息均不能替代原始同刻证据。

## 有效性与边界

原capture+5秒结果期限不延长。每端记录发送尝试、接受/终态轮询观察ROS/steady时刻、UUID、拒绝、取消与未知状态。both_within_original_deadline独立于source_and_models_validated；未有两端明确结果终态则前者为null。清理共享1秒截止，逐端错误保留，不能因一端错误跳过另一端。

本次6D后端仍是CAD object_pose_register，抓取后端是GraspNet。FoundationPose实际后端另在FP-BACKEND-01暂缓项中；本入口不证明它已运行。MTC、夹爪/机械臂/底盘执行均NOT_RUN，也不创建相关执行客户端。
