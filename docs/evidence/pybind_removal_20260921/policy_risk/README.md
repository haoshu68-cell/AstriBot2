# 策略纯核心阶段：差分、边界和性能证据

本阶段新增类型化 C++ contracts、持续融合、传感器健康/标定、多边形连续扫掠和风险组合。
它们是后续 observer/controller 的内部库，尚未切换两个策略 ROS 入口；未减少仍在使用的
79 个 Python 运行时文件，也未删除它们仍依赖的绑定。历史 Python 仅复制到不安装的
test/reference，作为测试输入，不新增生产兼容入口。

## 正确性与故障回归

- 整包 `ASTRIBOT_BUILD_PYBIND=OFF` Release 构建、**8/8 CTest** 通过；其中新核心
  pytest 为 fusion **47**、health **41**、sweep **18**、risk **20**，合计 **126**。
- 独立风险回放 **1,783 场景**通过：350 个混合世界的五种调用、18 个停止时域边界、
  15 组多边形接触边界。种子和输入生成器随源码保存。
- risk **20**、sweep **18** 及上述 **1,783** 回放在 UBSan 下通过，无 sanitizer 诊断。
  fusion 的 ASan/UBSan/LSan 与 health 的 UBSan 证据在相邻目录。
- 冻结的九个 Python 文件与 Git `694a99d8` 字节一致。差分精确比较决策布尔值、
  障碍物 owner、行序及纳秒；浮点间距使用明确误差上限，并额外精确比较 clearance>0。
- 原始失败保留：零速度的负零翻转、有限输入产生无限预测距离、严格 0.1 m 移动边界。
  没有放宽门槛；分别复用原有平移语义、保留原终点/空路径处理、使用补偿范数计算修复。
- 临时将多边形替换成矩形的错误实现被测试发现：**3 失败 / 15 通过**。该变异仅存在于
  `/tmp`，没有写入生产源码。
- 共享 snapshot 头的默认空回调仍保留旧行为；既有单元测试通过，额外 3,000 轨迹
  输出与变更前头文件字节相同，详见相邻 fusion 目录。

保留 `initial_*` 和 `*_red` 中的真实中间失败；它们不是最终候选失败。溢出测试产生的
6 个 NumPy warning 来自原 Python 参考，最终差分及 UBSan 通过日志保留这些输出。

## 核心性能

同一 CPU 26，四组交替 AB/BA，每进程预热 20 次、采样 150 次；每场景、每实现
600 次计时。场景是非对称 footprint、20 步预测，障碍数分别为 8 和 64。
Python 参考显式加载原有 geometry 绑定；可选 navigation native 开关保持源码默认关闭。
所有循环输出一致，配对结果一致。只测进程内 evaluate_risk，排除 JSON/进程启动/ROS。

| 场景 | 指标 | Python | C++ |
|---|---|---:|---:|
| 8 障碍 | P50 | 1.3371 ms | 0.04441 ms |
| 8 障碍 | P95 | 1.3775 ms | 0.04643 ms |
| 8 障碍 | CPU/次 | 1.3425 ms | 0.04408 ms |
| 8 障碍 | 结束 RSS | 33.85 MiB | 4.54 MiB |
| 8 障碍 | 全部样本最慢 | 1.5014 ms | 0.08751 ms |
| 64 障碍 | P50 | 2.7859 ms | 0.30002 ms |
| 64 障碍 | P95 | 2.8506 ms | 0.31567 ms |
| 64 障碍 | CPU/次 | 2.7913 ms | 0.30192 ms |
| 64 障碍 | 结束 RSS | 34.69 MiB | 4.94 MiB |
| 64 障碍 | 全部样本最慢 | 3.0461 ms | 0.36823 ms |

表中 P50/P95/CPU/RSS 为四次运行指标的中位数，最慢项为全部样本最大值。
P50 分别下降约 **96.7% / 89.2%**。原始每次 wall/CPU 纳秒、输出、顺序、ELF 哈希
均在 `benchmark/`。保存但不使用 `ru_maxrss` 做收益结论：该计数可能继承启动器历史高水位，
不等于当前 ELF 的独立峰值。RSS 也不是每个 ROS 节点的增量内存。

这些短时微基准不证明整机加速、硬实时界限、长期无泄漏或完整策略迁移完成。
ROS 队列、TF、双时钟、包络 ACK/租约、P2/P3/P4/P5/H2、取消及整栈验证仍在后续范围。

## 复现与身份

从记录分支独立工作树运行：

```sh
python3 docs/evidence/pybind_removal_20260921/policy_risk/reproduce.py
python3 docs/evidence/pybind_removal_20260921/policy_risk/reproduce.py --ubsan
```

依赖 C++17、nlohmann JSON、Python3/pytest/NumPy。脚本只在自有 `/tmp` 目录构建和运行
离线 probe，不安装、不启动 ROS。`build_recipe.json` 保存整包构建配置与 benchmark 参数。
`source_hashes.json` 关联最终源码、CTest、安装审计与 ELF；历史中间日志不重新标成最终 ELF。
新鲜独立安装仅有原五个 C++ 运行入口，无新 Python 包/绑定/参考测试安装项，所有已安装
运行 ELF 的动态依赖没有 Python 或本项目绑定。共享安装未更新。
