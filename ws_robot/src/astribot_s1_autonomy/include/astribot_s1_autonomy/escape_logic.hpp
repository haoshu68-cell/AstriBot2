// Copyright 2026 Astribot.
//
// 膨胀带脱困的**纯几何/判据层**（不依赖 ROS、不依赖 costmap 对象，可离线逐条测）。
//
// ============================ 这一层为什么存在 ============================
//
// 实测死锁（2026-08-31，3h41m 一次运行）：
//     planner_server: GridBased: failed to create plan, invalid use:
//                     Starting point in lethal space! Cannot create feasible plan.
//     出现 25652 次；14 次到位全部集中在开头 2.5 分钟，之后 222 分钟只再到位 1 次。
//
// 成因链（逐层取证）：
//   1. SmacPlanner2D 的 Node2D::isNodeValid 走 GridCollisionChecker 的**单格**变体，
//      判据是 cost >= INSCRIBED(253)（nav2_smac_planner/constants.hpp）。
//   2. 253 = INSCRIBED_INFLATED_OBSTACLE 是**膨胀层**写的，不是障碍层。
//      本机 inflation 把墙面向内吃掉一个内切半径 0.388m，机器人中心一旦进这条带，
//      全局规划器就拒绝从这里起步 —— 去任何目标都拒绝。
//   3. nav_dispatch_mode=follow_path 绕过了 BT，**clear_costmap / backup / spin 全部拿不到**。
//   4. 协调器的「自动恢复」只是 transitionTo(kIdle)，**物理上什么都没做** ——
//      机器人还站在那个格子里，于是重新校验、重新失败，永久死锁。
//
// 三条由此确定的设计约束（写错任何一条，脱困都会变成危险动作）：
//
//   A) **旋转无效**。判据是单个中心格，原地转不改变中心格代价。
//      现有 BOOTSTRAP 状态只做原地旋转，救不了这个 —— 脱困必须含平移。
//
//   B) **不能用足迹代价做梯度**。实测（nav2_params_mppi.yaml:769 那一节）：
//      consider_footprint 下足迹代价在窄于 1.62m 的通道里恒为 253、零梯度，
//      取值集合 = [253]；而中心格代价有梯度 = [0,118,125,134,137]。
//      本层因此**根本不用梯度**，改用「最近可规划格」搜索：
//      目标定义为「三态 != 100」，也就是 raw < 253，正是规划器会接受的起点。
//      这比梯度多一个保证 —— 到了那里 SmacPlanner2D **必然**接受起点，
//      而梯度只是朝代价低的方向挪，不保证越过 253 边界。
//
//   C) 🔴 **安全红线：只穿膨胀带，绝不穿真实障碍。**
//      「规划器视角」用 costmap 三态（阈值 253，与 CostmapAdapter 同一口径），
//      「物理真值」用 /map（SLAM 的占据栅格，不含 nav2 膨胀）。
//      /map 判占据或未知 ⇒ 一律禁止脱困。未知不等于可通行 ——
//      本项目已在探针脚本上踩过「没收到数据被当成前方无障碍」这一类错误。
//
// ========================================================================
//
// 角度约定：弧度，(-pi, pi] 归一化，正为逆时针（右手系绕 +z），与 align_math 一致。

#ifndef ASTRIBOT_S1_AUTONOMY__ESCAPE_LOGIC_HPP_
#define ASTRIBOT_S1_AUTONOMY__ESCAPE_LOGIC_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "astribot_s1_autonomy/frontier_search.hpp"    // GridMap
#include "astribot_s1_autonomy/path_validator.hpp"     // PlanarPoint

namespace astribot_s1_autonomy
{

/// 三态栅格的判定阈值。与 FrontierSearch / PathValidator 同一套口径。
///
/// 刻意再声明一份而不是复用那两个 params 结构：本层只需要这两个数，
/// 拖进整个 PathValidatorParams 会把「目标净空半径」之类无关语义也带进来，
/// 而那些语义在脱困场景下是**有害**的（脱困目标本来就贴着膨胀带）。
struct EscapeGridThresholds
{
  /// >= 此值判「占据」。三态图里占据恒为 100，所以任何 (free, 100] 内的值等价。
  int occupied_threshold{65};
  /// <= 此值判「空闲」。三态图里空闲恒为 0。
  int free_threshold{25};
};

/// 机器人所在格在两张图上的读数。
///
/// 两张图分工是本仓库已确立的纪律（探索校验就是 /map 找前沿、costmap 判可站），
/// 这里沿用：costmap 回答「规划器让不让我从这儿起步」，/map 回答「这儿物理上堵没堵」。
struct CellReading
{
  /// costmap 三态是否可用（收到过帧、未超龄、坐标在图内）。
  bool costmap_valid{false};
  /// costmap 三态值：-1 未知 / 0 空闲(含膨胀梯度) / 100 致命(raw >= 253)。
  int costmap_tri{0};
  /// /map 三态是否可用。
  bool map_valid{false};
  /// /map 三态值：-1 未知 / 0 空闲 / 100 占据。
  int map_tri{0};
};

/// 触发判定的结论。
enum class EscapeVerdict
{
  /// 不需要脱困（起点在规划器眼里是合法的，失败原因在目标侧）。
  kNone,
  /// 条件 A + B' 成立：起点被膨胀判致命、但物理可通行 ⇒ 允许脱困。
  kEscape,
  /// 🔴 /map 判占据 ⇒ 物理真堵。必须停机告警，**禁止任何脱困动作**。
  kBlockedPhysically,
  /// 数据不足（两张图之一不可用，或 /map 判未知）⇒ 保守拒绝运动。
  kDataInsufficient
};

const char * toString(EscapeVerdict v);

/// 触发条件配置。
struct EscapeTriggerConfig
{
  /// 连续多少次 ComputePathToPose 失败才认为「不是偶发」。
  int trigger_failures{5};
  EscapeGridThresholds thresholds{};
};

/// 触发判定。**只做判断，不产生任何指令**，因此可以单独测。
///
/// 判定顺序是刻意的，不能调换：
///   ① /map 占据/未知 → 先否掉（红线优先于一切，包括 costmap 说什么）
///   ② 数据不可用     → 保守拒绝
///   ③ costmap 非致命 → kNone（起点合法，失败原因在目标侧，该换候选点而不是脱困）
///   ④ 失败次数不够   → kNone（避免偶发抖动就触发运动）
///   ⑤ 其余           → kEscape
///
/// @param reading              机器人所在格的两图读数
/// @param consecutive_failures 连续规划失败次数
/// @param cfg                  配置
/// @param[out] why             人类可读原因（无论成功失败都填，供日志与状态话题）
EscapeVerdict evaluateEscapeTrigger(
  const CellReading & reading,
  int consecutive_failures,
  const EscapeTriggerConfig & cfg,
  std::string & why);

/// 单格分类：是否「规划器眼里致命」（三态 == 占据，即 raw >= 253）。
bool isPlannerLethal(int tri_value, const EscapeGridThresholds & th);
/// 单格分类：是否「物理占据」。
bool isPhysicallyOccupied(int tri_value, const EscapeGridThresholds & th);
/// 单格分类：是否「未知」。
bool isUnknownCell(int tri_value);

/// 面包屑：机器人走过的位姿采样点。
struct Breadcrumb
{
  PlanarPoint p{};
  /// 采样时刻（秒，单调时钟口径由调用方保证）。越大越新。
  double stamp_sec{0.0};
};

/// 搜索配置。
struct EscapeSearchConfig
{
  /// 搜索半径上限(m)。超出即放弃 —— 脱困是短距离动作，不是重规划。
  double search_radius_m{1.5};
  /// 方向约束：候选目标方位与参考朝向的最大夹角(rad)。默认 ±90°。
  double heading_tol_rad{1.5708};
  /// 直线可通行性采样步长(m)。<=0 表示取 map.resolution/2。
  double segment_step_m{0.0};
  /// 单条线段最多采样点数（防御性上限，禁止无界循环）。
  std::size_t max_segment_samples{512U};
  /// 目标格额外要求：其自身在 /map 上必须空闲（不只是非占据）。
  bool require_target_map_free{true};

  /// 红线判据的**足迹半径**(m)。默认 0.386 = 实测轮系包络半径。
  ///
  /// 为什么必须有：只查中心线等于把机器人当质点。中心线离墙 0.05m 时中心线
  /// 全程「空闲」，但轮子已经压在墙上。这是本层最初实现的一个真实缺口。
  ///
  /// 用 0.386（真实轮系包络）而不是 0.42（外接圆）：红线的语义是
  /// 「物理上会不会撞」，外接圆是八边形的顶点距离，用它会把八边形边中点
  /// 附近本来能过的地方判成撞 —— 那会让脱困在恰好需要它的窄处无解。
  ///
  /// 0 表示退回「只查中心线」的旧行为。**仅供回归对比，不得上线**。
  double footprint_radius_m{0.386};
  /// 足迹扫掠单次最多检查的栅格数（防御性上限）。超出即判不可通行，
  /// 绝不放大步长偷偷降分辨率 —— 那会让「漏检一格墙」静默发生。
  std::size_t max_footprint_cells{20000U};

  EscapeGridThresholds thresholds{};
};

/// 线段在 /map 上是否物理可通行（不穿占据、不穿未知、不出图）。
///
/// 为什么必须有这一步：脱困是**直驱 Twist**，没有规划器介入。
/// 若只挑「最近的可规划格」而不查这条直线，机器人可能被要求穿过一道薄墙 ——
/// 目标格本身合法、起点合法、中间是墙。
///
/// 出图按**不可通行**处理（不是可通行）：图外没有信息，
/// 「无数据当成安全」是本项目已踩过的一类错误。
///
/// ---- 两级判据，严格程度刻意不同 ----
///
/// 中心线（机器人中心真正扫过的那条线）：**未知也判不可通行**。
///   理由：中心线上是未知，意味着要把车开进从没看见过的地方。
///
/// 足迹环带（中心线之外、footprint_radius_m 之内）：**只拦物理占据，
///   不拦未知**。理由：探索期地图边缘天然被未知包围，若环带里的未知也拦，
///   脱困会恰好在最需要它的时候（贴着未知边界）永远无解。而用户定的红线
///   原话是「不能穿过 obstacle 层**占据**栅格」，未知不在其中。
///
/// 这条差异是判断，不是疏漏 —— 改动前请先看 test_escape_logic 里
/// FootprintAnnulusUnknownDoesNotBlock / CenterLineUnknownAlwaysBlocks 两条。
bool segmentPhysicallyClear(
  const PlanarPoint & a,
  const PlanarPoint & b,
  const GridMap & map,
  const EscapeSearchConfig & cfg);

/// 一级方案：反向沿来路。
///
/// 为什么优先于搜索：机器人是**自己开进来的**，所以来路可通行是可证明的，
/// 不是推断的。这同时绕开了纯梯度那两个已知毛病（原地转圈、局部极小）。
///
/// 选点规则：在轨迹里从**新到旧**扫，取第一个同时满足
///   · costmap 三态非致命（规划器会接受）
///   · /map 空闲
///   · 与机器人之间的直线在 /map 上可通行
///   · 距离在 (0, search_radius_m] 内
/// 的点。从新到旧是因为越新的点越近，脱困要走最短的路。
///
/// @param[out] target 选中的目标（世界坐标）
/// @param[out] why    原因
/// @return 是否找到
bool pickBreadcrumbTarget(
  const std::vector<Breadcrumb> & trail,
  const PlanarPoint & robot,
  const GridMap & costmap,
  const GridMap & map,
  const EscapeSearchConfig & cfg,
  PlanarPoint & target,
  std::string & why);

/// 二级方案：最近可规划格搜索（面包屑不可用时兜底）。
///
/// **不用代价梯度**，理由见文件头约束 B。目标定义为「costmap 三态非致命」，
/// 也就是 raw < 253 —— 到了那里 SmacPlanner2D 必然接受起点。
///
/// 两趟搜索，顺序不能反：
///   趟 1：只接受方位在 ref_heading ± heading_tol_rad 内的候选（尊重任务方向）
///   趟 2：趟 1 无解时才放开方向约束，并置 used_relaxed_pass=true 供调用方告警
/// 这样既实现了「拒绝背向任务路径逃逸」，又不会因为方向约束而彻底卡死。
///
/// @param ref_heading            参考朝向(rad)。通常取「机器人 → 当前候选前沿点」
///                               的方位；取不到时调用方应传机器人当前朝向。
/// @param[out] used_relaxed_pass 是否用了放开方向约束的第二趟
bool findNearestPlannableCell(
  const PlanarPoint & robot,
  double ref_heading,
  const GridMap & costmap,
  const GridMap & map,
  const EscapeSearchConfig & cfg,
  PlanarPoint & target,
  bool & used_relaxed_pass,
  std::string & why);

/// 脱困速度限制。**全部低速**——脱困是故障恢复态，不是正常导航。
struct EscapeLimits
{
  /// 线速度上限(m/s)。实测本底盘 0.02 m/s 就能动（无静摩擦地板），
  /// 0.08 时 4s 滑行 < 0.03m，远小于 253 带宽 0.388m。
  double max_linear{0.08};
  /// 角速度上限(rad/s)。实测 wz=0.05 就能动，跟踪比 0.73~0.92。
  double max_angular{0.20};
  /// 到目标多近算「本段走完」(m)。
  double arrive_tol_m{0.05};
  /// 朝向对齐容差(rad)：航向误差大于此值时先转再走，避免斜着蹭墙。
  double align_tol_rad{0.35};
};

/// 一拍的脱困速度指令。全向底盘，vx/vy 都可用。
struct EscapeCommand
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
  /// 是否已到达本段目标（调用方据此判断要不要重新选点）。
  bool arrived{false};
};

/// 由「机器人位姿 + 目标点」算受限速度指令。
///
/// 策略：车体系直接给 (vx, vy)，不强制先转向 —— 这是全向底盘，
/// 侧移比「转身再走」快且不改变足迹朝向（转身反而可能让顶点扫进 254）。
/// wz 只用于把车头缓慢对齐运动方向，幅值受 max_angular 限制，
/// 且**航向误差在 align_tol_rad 内时 wz 归零**，避免原地抖动。
EscapeCommand escapeVelocity(
  const PlanarPoint & robot,
  double robot_yaw,
  const PlanarPoint & target,
  const EscapeLimits & lim);

/// 出带判据：连续 need 拍读到「非致命」才算真出带。
///
/// 要求连续多拍而不是一拍：栅格在边界上会抖，一拍就判出带会让状态机
/// 在 ESCAPE 与 GEN_NEXT_POINT 之间来回跳。
bool escapeCleared(int consecutive_clear_ticks, int need);

}  // namespace astribot_s1_autonomy

#endif  // ASTRIBOT_S1_AUTONOMY__ESCAPE_LOGIC_HPP_
