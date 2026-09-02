# 脱困设计：起点落在致命区 / 控制器在开阔区走不动

> 状态：**设计稿，未实现、未验证**。文中标 ✅ 的是已实测数据，标 ⬜ 的是设计推断。
> 基线提交见 `git log -1`；实测数据来自 2026-08-31 Gazebo 仿真（本栈真机数据为零）。
>
> 读之前先读：[`technical_operations_manual.md`](technical_operations_manual.md) §3.2/§3.3/§8.5。
> 数字只在一处维护 —— 本文引用参数名，不复制阈值。

---

## 0 · 这份设计要解决的两个现象

| | 现象 A：起点落在致命区 | 现象 B：开阔区却走不动 |
|---|---|---|
| 表面症状 | 机器人不动，探索零派发 | 机器人蠕行或不动，最终 `Failed to make progress` |
| 日志特征 | `planner_server: Starting point in lethal space` | 控制器持续输出极小速度，或输出零速 |
| ✅ 实测量级 | 一轮里 **356 次**，连续 **177 秒**未自行恢复 | `/cmd_vel` 线速度 mean **0.019 m/s**，实测车速 mean **0.009 m/s** |
| 当前有恢复吗 | **没有**。协调器只会拒掉候选、重新采样，永久空转 | 只有 progress checker 判死 + 换目标，不解决根因 |

两者都源自"机器人处在一个规划器/控制器无法从中产生运动的位姿"，
但**根因不同、解法不同，必须先分类再动作** —— 这是本设计的核心。

---

## 1 · 现象 A 的根因分类（先分类，再脱困）

### 1.1 实测事实

✅ 卡住时刻的实测（机器人位于 map (-0.384, 2.322)）：

```
costmap 代价          = 253      ← 语义：机器人中心在此则足迹必然碰撞
SLAM /map 该格占据值  = 0        ← 该格本身是空闲的
0.65m(inflation_radius) 内 SLAM 占据格 = 54/517
```

253 = `INSCRIBED_INFLATED_OBSTACLE`，赋给**距任一致命格 < `robot_radius`(0.42) 的格**。
所以：机器人所在格自己是空闲的，但 **0.42 m 内存在 SLAM 地图里的真实障碍** ——
机器人是真的贴到墙上了。

### 1.2 由此得到的分类（决定用哪种脱困动作）

| 类别 | 判据 | 含义 | 正确动作 |
|---|---|---|---|
| **A1 真实贴障** | 自身格 costmap ≥ 253 **且** 附近有 SLAM 占据格 | 真的贴着墙/货架 | **必须移动**。清代价地图无效 —— static_layer 来自 SLAM 地图，清了下一周期又回来 |
| **A2 幻影障碍** | 自身格 costmap ≥ 253 **但** `inflation_radius` 内 SLAM 全部空闲/未知 | obstacle_layer 被一帧误观测污染（自身点云、反光、动态物） | **清代价地图**即可（`clear_around_*`），不要贸然移动 |
| **A3 未知区** | 自身格 costmap == 255 | 机器人处在没扫过的区域 | 等 SLAM 出图；`allow_unknown: false` 下规划必然失败，这不是故障 |

⚠️ **A1/A2 分不清就动作，是这份设计里最危险的错误**：
- 把 A1 当 A2 → 清了代价地图，下一个 costmap 周期（`update_frequency: 1.0`）静态层重新膨胀，
  253 原样回来，白清一轮，且期间规划器可能规划出一条穿墙的路径。
- 把 A2 当 A1 → 在其实没有障碍的地方倒车，纯属无谓移动，且开环推进有打滑风险。

判据实现就是 §1.1 那段测量：**同时读 `/global_costmap/costmap_raw` 与 `/map`，比同一世界坐标**。
两张图的分工与坐标换算见 `costmap_adapter.hpp` 文件头。

---

## 2 · 现象 A 的脱困设计

### 2.1 状态机：新增 `ESCAPING`

放在协调器里，与 `BOOTSTRAP` 同层（都是"本节点自己驱动底盘"的状态，
由 `drivesChassisDirectly()` 统一表达）：

```
NAVIGATING ─┐
GEN_NEXT_POINT ─┼─ 连续 N 次规划失败且自检确认起点致命 ─→ ESCAPING ─→ IDLE
VALIDATING ─┘                                              │
                                                    安全门拒绝 / 次数用尽
                                                           ↓
                                                        PAUSED（等人工）
```

进入条件（全部满足才进，避免把普通规划失败当成致命起点）：

1. 连续 `escape_trigger_plan_failures`（建议 3）次规划失败，且失败原因含 `lethal`；
2. 自检：自身格 costmap ≥ `escape_lethal_threshold`（253）；
3. 分类为 **A1 或 A2**（A3 不进，等图）;
4. `escape_count_ < escape_max_attempts`；
5. 无在途导航目标（与 Nav2 抢底盘是绝对禁止的）。

退出一律回 `IDLE`，由 `IDLE` 重新走前置条件 —— 与 `BOOTSTRAP` 同一约定。

### 2.2 A2 的动作：清代价地图（不动车）

```
调 /global_costmap/clear_around_global_costmap  (reset_distance = escape_clear_radius)
调 /local_costmap/clear_around_local_costmap    (同上)
等 1 / update_frequency + 余量，重新自检
自检通过 -> 回 IDLE；仍 >=253 -> 重新分类（很可能其实是 A1）
```

用 `clear_around_*` 而不是 `clear_entirely_*`：后者会把整张图的实时障碍全抹掉，
包括机器人身后刚观测到的真实障碍，代价远大于收益。
`ClearCostmapAroundRobot` 的接口只有一个 `reset_distance` 字段（✅ 已确认）。

⚠️ `clear_*` **不会**清 static_layer。这正是 A1 不能靠它解决的原因。

### 2.3 A1 的动作：沿"离障最远"方向挪一小步

**为什么不用 nav2 现成的 `/backup`。** ✅ 已确认 `behavior_server` 起了
`spin` / `backup` / `drive_on_heading` / `wait` 四个行为且 action 都在线。
但 `BackUp` 的 goal 只有 `target`(Point) + `speed`，语义是**沿车体 −x 直线后退**；
`DriveOnHeading` 是沿 +x。本底盘是 X 型全向轮，脱困的最优方向通常既不是 +x 也不是 −x
（贴墙时往往需要横移）。把方向限制在车体 x 轴上会：
- 要么先原地转到那个方向（贴墙时转身本身就可能碰撞），
- 要么走一条比必要长得多的路。

所以 A1 直接复用**自举那一套已验证的底盘驱动通路**：

| 复用项 | 已验证的性质 |
|---|---|
| 发到 `bootstrap_cmd_vel_topic`（`/cmd_vel_nav_body_raw`）| ✅ 保留 smoother → 倾倒监控 → 臂-底盘耦合限速 → 力矩闭环 leash 四层 |
| 独立高频定时器重发 | ✅ 底盘 `cmd_vel_timeout_sec=0.5`，20 Hz 不会被反复超时归零 |
| 离开状态在 `transitionTo` 里统一发零速 | ✅ 一处覆盖所有出口 |
| 激光新鲜度安全门 | ✅ 无数据一律拒绝运动 |
| 转角/位移**实测**而不是看指令 | ✅ 自举实测下游总缩放约 0.85，与名义值差很多 |

**脱困方向怎么算（纯函数，可离线测）：**

```
输入：costmap 快照、机器人位姿、robot_radius
1. 在半径 escape_search_radius（建议 1.5 m）内，找所有 costmap < 253 且
   距任一 254 格 >= robot_radius 的候选格（= 机器人中心可以合法站的格）
2. 取其中距机器人最近的一个作为脱困目标点 P
3. 脱困方向 = 归一化(P - 机器人位置)，在**车体系**下分解成 (vx, vy)
4. 找不到任何候选格 -> 不动，转 PAUSED 并显式报"周围 1.5m 内无合法站位"
```

这一步刻意**不调规划器**：起点已经在致命区，规划器本来就会拒绝（那正是现象 A）。
纯几何搜一个最近合法站位，是这个状态下唯一可行的算法。

**执行：**

```
以 escape_linear_vel（建议 0.08 m/s，与自举同量级的保守值）朝该方向平移，
时长上限 escape_duration_sec（建议 3.0 s），期间：
  · 每拍重查安全门（激光新鲜 + 该方向 escape_min_clearance 内无障碍）
  · 每拍重查自身格代价；一旦 < 253 立刻停车并回 IDLE（提前成功退出）
  · 超时/安全门拒绝 -> 停车、记一次失败、回 IDLE 由上层决定是否再试
结束后**实测位移**（在 odom 系，理由同自举：map->odom 由 SLAM 发布，不可依赖）
位移 < escape_min_displacement（建议 0.05 m）-> 显式报错并指出最可能原因（下游限速/被物理卡住）
```

### 2.4 参数（全部进 yaml，禁止硬编码）

```yaml
    # ---- 脱困（起点落在致命区）----
    escape_enabled: true
    escape_trigger_plan_failures: 3      # 连续多少次含 lethal 的规划失败才判定
    escape_lethal_threshold: 253         # 与 validation.costmap_lethal_cost_threshold 同源
    escape_clear_radius: 1.0             # A2 清障半径(m)
    escape_search_radius: 1.5            # A1 搜合法站位的半径(m)
    escape_linear_vel: 0.08              # 脱困平移速度(m/s)
    escape_duration_sec: 3.0             # 单次脱困时长上限(s)
    escape_min_displacement: 0.05        # 实测位移下限，达不到要显式报错(m)
    escape_min_clearance: 0.30           # 脱困方向上的最小净空(m)
    escape_max_attempts: 3               # 次数上限；成功派发目标后清零
    escape_cmd_rate_hz: 20.0             # 与 bootstrap_cmd_rate_hz 同理
```

必须加的耦合校验（配错会让保护失效，参照 §5.3 的 🔒 条目）：

| 校验 | 理由 |
|---|---|
| `escape_lethal_threshold` == `validation.costmap_lethal_cost_threshold` | 两处用不同阈值 = 判据不同源，会出现"协调器说致命、脱困说不致命" |
| `escape_search_radius` > `robot_radius` | 小于足迹半径时搜索域内不可能有合法站位，脱困永远失败 |
| `escape_min_displacement` < `escape_linear_vel × escape_duration_sec × 最坏耦合缩放` | 否则即使一切正常也判"没动"，与自举那条同理 |
| `escape_cmd_rate_hz` ≥ 2 / 底盘 `cmd_vel_timeout_sec` | 发得比超时慢会被反复归零 |
| `escape_max_attempts` ≥ 1 且 ≤ 10 | 禁止死循环重试 |

### 2.5 状态上报（禁止静默失败）

`/exploration/state` 增加字段，与 `bootstrap_*` 同格式：

```
escape=1/3 escape_class=A1 escape_result=实测位移 0.083m，已脱离致命区
```

被安全门拦下、方向找不到、位移不达标，都必须能从这一行看出来，不用翻日志。

---

## 3 · 现象 B 的设计：开阔区却走不动

### 3.1 先把"开阔"量化，再谈解法

✅ 已实测的反例教训：我曾把一次卡死归因为"MPPI 窄通道零梯度"，
实测那个点 **costmap 代价 0、沿 x 可通行 2.55 m、沿 y 3.75 m** —— 远宽于
零梯度阈值 1.62 m，假设被自己的测量推翻。**所以必须先量，不能按现象猜。**

量化判据（纯函数，可离线测）：

```
freeWidth(costmap, 位姿, 方向) = 过机器人中心、沿该方向、代价 < 253 的连续段长度
open_area = min(freeWidth(沿路径切线), freeWidth(垂直于路径切线))
```

`open_area > mppi_zero_gradient_width`（实测 1.62 m）即认定"开阔"。

### 3.2 分类

| 类别 | 判据 | ✅/⬜ | 已知机制 |
|---|---|---|---|
| **B1 命令即低速** | `/cmd_vel_nav_body_raw` 线速度持续 < `stall_cmd_threshold`(0.05)，而目标还很远 | ✅ 实测 mean 0.019 m/s | 控制器自己选择了低速。**不是**下游限速：✅ 实测 `/speed_limit` 20 s 内零消息，raw 在源头就是低值 |
| **B2 命令正常但车不动** | cmd_vel 正常，odom 速度 ≈ 0，轮实际转速 ≈ 0 | ⬜ 未在开阔区观测到 | 力矩不足 / 物理卡住 |
| **B3 轮转车不动** | 轮转速非零、odom ≈ 0 | ⬜ 未确认 | 打滑 |
| **B4 窄通道零梯度** | `open_area < 1.62 m` | ✅ 已记录（占可行域 35%）| `consider_footprint: true` 下代价饱和成 253、梯度为零 |

⚠️ **抓取器判据必须同时看平移和转角。** ✅ 实测教训：第一版只看位移，
把每一次 `ALIGN_START` 的正常原地旋转都判成卡死（124 次触发里绝大多数是假阳性），
并据此给出了"打滑"这个明确错误的结论。原地旋转时线速度本来就是 0、位移本来就不变。

### 3.3 B1 的解法（本轮唯一有实测支撑的一类）

B1 的性质是"控制器能动但选择不动"，所以解法方向是**换一个在这种局面下更果断的控制器**，
而不是调 MPPI 的 critic 权重（权重改动会全局影响跟踪质量，风险面大得多）。

**方案 B1-a：fallback 控制器（推荐）**

`controller_server` 已经支持多控制器实例（本项目已有 `FollowPath` /
`FollowPathThreePhase` / `FollowPathExplore` 三个）。再加一个基于
RPP（`nav2_regulated_pure_pursuit_controller`）的实例作为兜底：

```
协调器检测到 B1（开阔 + 命令低速 持续 stall_detect_sec）
  -> 取消当前 FollowPath
  -> 用 follow_controller_id = <fallback 实例> 重发同一条已校验路径
  -> 走通了：继续；仍不动：按现有导航失败流程另选目标
```

✅ 可行性已确认（不用新装任何东西）：
- `libnav2_regulated_pure_pursuit_controller.so` 在 `/opt/ros/humble/lib/` 里，
  插件类 `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` 已注册；
- 一整套可用参数已经存在于 `nav2_params_rpp.yaml:94-98`（`desired_linear_vel: 0.5`、
  `lookahead_dist: 0.6` 等），直接抄成 mppi 配置里的第四个实例即可，不需要重新调参。

代价与前提：
- RPP 不是全向控制器，会退化成"先转向再直行"。在开阔区这恰恰是想要的果断行为，
  但它**不发 `vy`**，所以贴墙横移那类动作它做不到（那属于现象 A，由 §2.3 处理）；
- fallback 实例**必须与 MPPI 实例用同一个 goal checker**，否则两者到位判据不同，
  会出现"换了控制器就永远收敛不了"。⬜ 这条是推断，但同类错误已实测过一次：
  `goal_checker_id` 配错时每次 FollowPath 直接 abort（手册 §3.3 陷阱 1）；
- ⬜ **端到端完全未验证**。验证前不要把它设成默认。

**方案 B1-b：只在检测到 B1 时临时抬高速度下限** —— 不推荐。
MPPI 的低速是它自己代价函数的输出，外部强行抬速等于绕过它的避障判断，
在贴障场景下直接危险。

### 3.4 B4 的解法（已有记录，本设计不展开）

窄通道零梯度是 `consider_footprint: true` 的固有代价。
已实测：改 `false` 在这张地图上**测不出差别**（已回退），
所以不要据此结论推广。可选方向（都⬜未验证）：
`nav2_collision_monitor` 做位姿相关动态足迹、或对窄通道单独降 `inflation_radius`。

---

## 4 · 与现有机制的关系（不要重复造）

| 已有机制 | 覆盖什么 | 不覆盖什么 → 本设计补什么 |
|---|---|---|
| `progress checker`（`PoseProgressChecker`）| 检出"不动" | 不解释原因、不脱困。它只让当前目标失败 |
| `max_invalid_replan_attempts` | 当前路径判死后放弃目标（✅ 17.0s → 4.00s）| 起点致命时根本规划不出路径，走不到这一步 |
| `BOOTSTRAP` | 冷启动地图空 | 地图有了、但机器人站在致命区 |
| nav2 默认行为树的 clear/backup 恢复 | 起点致命的标准解法 | **`follow_path` 模式绕开了 bt_navigator，这套恢复全部拿不到** —— 这是现象 A 无人处理的直接原因 |

⚠️ 值得单独记的一条：选择 `follow_path` 复用已校验路径，代价是**同时失去了 BT 的整套恢复行为**。
`follow_max_retries` 只补回了 `RecoveryNode` 的重试次数，
`ClearCostmap` / `BackUp` / `Spin` 这些一个都没补。本设计补的是其中最要紧的一条。

---

## 5 · 实施顺序与验证判据

**判据先写，避免事后挑数据。**

| 阶段 | 内容 | 通过判据 |
|---|---|---|
| 1 | A1/A2 分类的纯函数 + 单测 | 两类各有正反例；A1 误判成 A2 有专门哨兵 |
| 2 | 脱困方向搜索的纯函数 + 单测 | 无合法站位时返回"失败"而不是返回一个凑合方向；搜索半径小于足迹半径时必须报配置非法 |
| 3 | 配置一致性测试（跨包）| §2.4 的 5 条耦合各有一条测试，且**每条都做故障注入验证** |
| 4 | 在线：人为把机器人开到贴墙位姿 | 出现 `escape_class=A1` → 实测位移 ≥ `escape_min_displacement` → 自身格代价 < 253 → 恢复派发 |
| 5 | 在线：回归 | 换路径倍率仍 ≤ 2.0、到位偏差不退化、`Starting point in lethal` 连续簇长度从实测 **177 s** 降到 < 15 s |
| 6 | 在线：不该触发时不触发 | 一轮正常探索里 `escape=0/3`，即无误触发 |

**一键回退**：`escape_enabled: false` 即完全恢复现状（与 `bootstrap_mode: disabled` 同一约定）。

---

## 6 · 明确未解决 / 未验证的部分

| # | 项 | 状态 |
|---|---|---|
| 1 | **机器人为什么会走进致命区** | ⬜ 未定位。本设计只做脱困，不治本。两个待验证的猜想：(a) 新观测把机器人当前所在格刷成障碍（那一瞬间机器人"凭空"进入致命区）；(b) 贴地层 `z_min` 比设计高 1.5 cm，5~6.5 cm 的矮障碍看不见（手册 §5.4 第 3 条），撞上后被旁边可见障碍的膨胀圈困住 |
| 2 | B2 / B3 是否真实存在 | ⬜ 抓取器修正判据后尚未取得干净样本 |
| 3 | fallback 控制器方案 | ⬜ 完全未验证，含"两实例必须共用 goal checker"这条约束也只是推断 |
| 4 | 脱困动作在真机上的安全性 | ⬜ 本栈真机数据为零。开环平移在真机上的打滑风险显著高于仿真 |
