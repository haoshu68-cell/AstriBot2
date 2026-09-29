# 策略观测 C++ 候选：Stage 6B 隔离验证

**这是候选检查点，尚未切换生产 observer/controller，也没有删除它们仍使用的绑定。**
完整原生观测循环、地图/执行版本、包络消费、Vision/PointCloud 适配已实现。已知 JSON
整数与孤立 surrogate 字符串差异仍需修复，不能将本记录称为全输入等价或全项目去 pybind。

## 来源和范围

- Python oracle 逐字节冻结于 Git `965bf057` 的 16 个策略模块和 simulation.json，另明确加载此前基准 geometry binding；不从活动生产 Python 路径偷换参考实现。
- 纯核心 oracle 的 SHA256 由测试强制验证。`scoped_source_manifest.json` 记录本阶段完整包快照；`scoped_paths.json` 是相对上一记录的迁移文件。
- 共享 CMake/package/final_protection 同时有其他任务的 zones 改动。本阶段另从 `965bf057` 解出包，再仅叠加迁移文件与三个 CMake include，独立构建、安装和测试。没有收录/回退该并行改动。
- 只使用 localhost 的独立 ROS domain 164/165，全部话题按独有前缀重映射；只给本实验子进程发退出信号。没有启动 Gazebo、控制机器人或替换共享 install。
- `fresh_install_elf.json` 列出六个安装 ELF 的 SHA256/依赖，全部解析且无 libpython/pybind。它不能证明旧生产链已删除。

## 验证

- 完整共享候选和上述独立源码快照：各 **11/11 CTest**。后者见 `frozen_ctest_final.log`。
- 观测核心：**24/24** Release 与 UBSan，包括冻结来源校验、3,000 个版本转换操作、3,208 个定位边界操作、80 个随机旋转地图及边缘查询、四元数和完整 uint64 版本。
- 实际 ROS：先 **66/66**（33 场景×两实现），追加 ROS Time 秒范围边界 **2/2**；独立安装 C++ **34/34**。见 `ros_all_66.log`、`ros_time_boundary_final.log`、`ros_installed.log`。保存每用例原始日志和输出的 `*_artifacts.json`，没有将重复安装验证累加成新场景。
- 包络差分 **99/99**，适配器 **35/35** 及 ASan+UBSan+LSan；完整 wire-width 差分 **157/157** 与 UBSan 为相邻证据目录的已记录结果，和上述核心/CTest 有包含关系。
- ROS 覆盖启动无运动发布权、采集时间/过期/未来、五帧邮箱与最后可变换到达项、迟到 TF、时钟回退、路径替换/重试、image 不确定性、部分清除后异常、异常 sensor、完整 uint64 包络版本、匹配心跳的异步 ACK、撤销/恢复、PointCloud、CameraInfo、延迟配置错误和超出 ROS Time 的健康到期时间。

修复及保留原行为：

1. C++ TF Buffer 的 ROS-time 查询需要显式 dedicated-thread 标记，即使零等待；实际 TF 回调在默认组、处理在独立组且执行器有三线程。原失败日志保留。
2. 定位距离的阈值舍入差异已有 RED，使用前一阶段精确范数实现修复，未放宽阈值。
3. 完整 uint64 任务 sequence、版本/包络 epoch 不再落入有符号窄化；CameraInfo 维度涵盖 uint32。
4. 保留 normalize→采集/标定预检→coverage→fusion→health→逐项 resolution 的原事务顺序。错误 resolution 后已完成的清除不回滚；错误配置延迟到原消费阶段才拒绝。
5. 已确认原 Humble CameraInfo.K 给出 numpy.float64，冻结严格契约拒绝它。原生适配保持这个已存在的拒绝；**没有借迁移静默解锁要求标定的 depth 来源**。可接受普通 float 的纯标定核心仍独立测试。修复该原有功能问题应是另一个明确记录的行为变更。
6. 健康到期超过 ROS int32 秒上限时，两侧均停止发布并以错误退出，C++ 不再回绕为负时间。这是旧错误行为的保留，不是保证该极端时钟下持续运行。

`review/` 保存独立只读复核及真实 generated CameraInfo 复现。`frozen_ctest.log` 是错误地在首次构建结束前启动测试、且未解出相邻 profile 时的环境失败；补齐原 Git profile 并等待构建完成后的 `frozen_ctest_final.log` 才是验收结果。`ros_source_red*` 保存迁移真实差异；部分早期用例另有测试驱动错误（同话题类型冲突、未等待前次 ACK/异步消息），最终用例显式区分这些原因。

## 性能：同输入、实际隔离 ROS 进程

每负载四组交替 AB/BA，每进程预热20帧、测100帧，合计 **1,600 个测量帧**。
每个负载的所有测量帧逐字段对比业务输出（只排除三个计时字段；浮点容差 rel=2e-12/abs=3e-12）。
驱动固定CPU0、观测进程固定CPU1；三个执行线程共享该CPU，两实现相同。
Python保留原 geometry binding，未打开可选 navigation native kernels。

下表为四组指标的中位数，原始每帧/每组值、CPU亲和、ELF/绑定哈希保存在 `benchmark/`。

| 目标数 | 指标 | Python | C++ |
|---:|---|---:|---:|
| 8 | tick处理 P50 / P95 | 1.735 / 1.944 ms | 0.179 / 0.206 ms |
| 8 | tick线程CPU均值 | 1.735 ms | 0.178 ms |
| 8 | 全节点CPU / 帧 | 7.50 ms | 0.70 ms |
| 8 | 时钟发布→结果收到 P95 | 6.038 ms | 0.950 ms |
| 8 | 末次采样RSS | 62.213 MiB | 27.713 MiB |
| 64 | tick处理 P50 / P95 | 3.978 / 4.387 ms | 0.557 / 0.623 ms |
| 64 | tick线程CPU均值 | 3.941 ms | 0.554 ms |
| 64 | 全节点CPU / 帧 | 17.05 ms | 1.70 ms |
| 64 | 时钟发布→结果收到 P95 | 9.790 ms | 1.156 ms |
| 64 | 末次采样RSS | 64.082 MiB | 28.740 MiB |

处理P95分别降低约89.4%和85.8%。tick计时包含该轮scan/health/snapshot/risk，**不包含先前单独的 vision callback**；全进程CPU包含ROS回调。外部延迟从推进时钟开始，不把预先投递和等待输入的时间算成完整传感器端到端延迟。CPU采自/proc，以100帧摊销10ms系统计数粒度；RSS是当前采样，不是继承的ru_maxrss。

本次每组短窗口中，C++ RSS 增长中位约0.053/0.064MiB，Python约0.309/0.469MiB；这是样本结果，不能证明长期无泄漏。没有整栈吞吐、闭环导航、长时或真机性能结论。性能测量绑定 `benchmark/summary.json` 的共享候选ELF；独立安装ELF另有身份与完整ROS重验，不把不同ELF哈希混为一份实测。

## 尚待完成

- 真实有效 JSON 的 calibration_epoch 在2^63～UINT64_MAX、以及更大的内部整数，ImageBox 超64位维度/整型坐标必须保真；普通double解析会静默丢位。不能靠提前拒绝代替Python原事务。
- Python接收的 escaped lone-surrogate（包括未使用字段）目前被严格JSON parser拒绝。传感器字符串的ROS UTF-8出口错误与普通JSON中的ID保留需分别处理。
- 完成这些差异后重新跑对照/边界/性能，再切换observer入口。controller全P2/P3/P4/P5/H2、transport、bridge/SDK等闭包仍须继续迁移。
- Python测试oracle只在test/reference内，不安装、不添加兼容运行入口。尚未迁出的生产Python属于真实剩余工作，不以“保留兼容”宣布迁移完成。
