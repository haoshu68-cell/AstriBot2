# b5 五轮真机最小起步与到点前后参数表（2026-09-14）

## 测试范围
- 测试轮次：`轮1 x+`、`轮2 x-`、`轮3 y+`、`轮4 y-`、`轮5a yaw+(中止)`、`轮5b yaw+`。
- 工具与入口：`mode=min-speed`，`--execute` 真机闭环。
- 数据源：`runs/b5_five_round_analysis_20260914.json`。
- 当前 `hardware_validated=false`，`ground_truth_verified=false`（无独立真值回放）。

## 核心推荐参数（可先用于场景配置草案）
| 参数 | 建议值 | 依据 |
|---|---:|---|
| `minimum_continuous_linear_speed` | `0.002 m/s` | x±/y±四方向全部通过（21/21），候选取到最低档 |
| `minimum_continuous_yaw_rate` | `0.015 rad/s` | yaw+ 完整轮次 12/12，候选为 0.015 rad/s |
| `yaw_test_min_level` | `0.015 rad/s` | 0.005/0.0075/0.01 在停稳门槛下触发静态偏置导致未能开始下一档 |
| `wz_feedback_bias` | `+0.00323 rad/s`（静止基线估计） | 轮5a 中止轮静止基线测到的反馈 |
| `baseline_discard_s` | `1.0 s` | 与本批次分析参数一致 |
| `motion_position_resolution_m` | `0.0005 m` | 保留，抑制微抖；原始累计与分辨率累计同时保留日志 |
| `motion_rotation_resolution_rad` | `0.001 rad` | 保留，已通过门槛与原始累计限幅验证 |
| `settle_speed_m_s` | `0.01 m/s` | 与现有测试参数一致 |
| `settle_yaw_rate_rad_s` | `0.02 rad/s` | 现有参数一致；后续建议先做偏置补偿再评估是否可降 |
| `levels (x/y)` | `[0.002,0.004,0.006,0.008,0.01,0.015,0.02]` | 已在四方向轮次稳定通过 |
| `levels (yaw)` | `[0.015,0.02,0.03,0.04]` | 中止轮剔除后建议保留四档用于回归 |

## 轮次级参数记录
| 轮次 | 轴向/方向 | 执行状态 | 通过率 | 候选 | 候选档位 pass/n | 候选残差均值 | 停稳状态 | 停靠偏差（分辨率累计） |
|---|---|---|---:|---:|---:|---:|---|---:|
| 轮1 x+ | x / 1 | completed | 21/21 | 0.002000 | 3/3 | -0.000237 | manufacturer_feedback_settled | 直移 1.365593/raw 1.503133 |
| 轮2 x- | x / -1 | completed | 21/21 | 0.002000 | 3/3 | 0.000063 | manufacturer_feedback_settled | 直移 1.366338/raw 1.508920 |
| 轮3 y+ | y / 1 | completed | 21/21 | 0.002000 | 3/3 | -0.000063 | manufacturer_feedback_settled | 直移 1.365889/raw 1.505822 |
| 轮4 y- | y / -1 | completed | 21/21 | 0.002000 | 3/3 | -0.000031 | manufacturer_feedback_settled | 直移 1.365925/raw 1.504495 |
| 轮5b yaw+ | yaw / 1 | completed | 12/12 | 0.015000 | 3/3 | 0.003378 | manufacturer_feedback_settled | 直移 0.024001/raw 0.417242 |
| 轮5a yaw+(中止) | yaw / 1 | aborted | — | — | — | 0.003230 | unconfirmed | raw rot 0.002583 / resolved 0.000888 |

## 风险与待办
- yaw 偏置会放大低速判据冲突：`settle_yaw_rate=0.02` 与最低档 `0.005`，会导致“未能稳定停稳”误报。
- 当前结果尚非完整硬件放行，需独立真值复核到点精度与路径跟踪误差。
- 建议先保持 `hardware_validated=false`，将上述作为 `hardware.candidate` 供现场评审确认。

