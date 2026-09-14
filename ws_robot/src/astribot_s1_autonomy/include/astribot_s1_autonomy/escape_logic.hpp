// Copyright 2026 Astribot.

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

  /// 红线判据的**足迹半径**(m)。默认 0.4384 = 正方形足迹(半边长 0.31)的外接半径
  double footprint_radius_m{0.4384};
  /// 足迹扫掠单次最多检查的栅格数（防御性上限）。超出即判不可通行，
  /// 绝不放大步长偷偷降分辨率 —— 那会让「漏检一格墙」静默发生。
  std::size_t max_footprint_cells{20000U};

  EscapeGridThresholds thresholds{};
};

/// 线段在 /map 上是否物理可通行（不穿占据、不穿未知、不出图）。
bool segmentPhysicallyClear(
  const PlanarPoint & a,
  const PlanarPoint & b,
  const GridMap & map,
  const EscapeSearchConfig & cfg);

/// 一级方案：反向沿来路。
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
/// @param ref_heading            参考朝向(rad)。通常取「机器人 → 当前候选前沿点」
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
  /// 0.08 时 4s 滑行 < 0.03m，远小于 253 带宽 0.310m。
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
