// Copyright 2026 Astribot
//
// 窄通道贴边通行的**纯几何/判据层**（不碰 ROS、不碰 costmap 对象，可离线逐条测）。
//
// ============================ 适用范围（很窄，务必先读）============================
//
// 本层只处理**一档**情况：机器人中心不在致命带、但足迹碰到致命带。
// 用 clearance（中心到最近障碍的距离）表述，配 8 边形足迹（内切 0.388 / 外接 0.42）：
//
//   clearance < 0.388          机器人中心在此 ⇒ 足迹必然碰撞 ⇒ **物理放不进去**
//                              占本地图自由域 31.26%（实测 warehouse_explored_auto_grid.pgm）
//                              → 不是本层职责。该交给协调器的 ESCAPE（把机器人挪出来）。
//
//   0.388 ≤ clearance < 0.42   ⇐⇐ **本层唯一的目标域** ⇒ 占自由域 3.42%
//                              Smac2D 出得了路径（中心格 < 253），
//                              但 MPPI 的 footprintCostAtPose 取足迹**最大值**、
//                              恒返回 253 ⇒ 由它反推的 dist_to_obj 成常数 ⇒
//                              critical/repulsion 对所有候选轨迹完全相同 ⇒ **零梯度**。
//                              实测（nav2_params_mppi.yaml:769 那节）沿通道 13 点采样：
//                                足迹最大代价取值集合 = [253]         ← 饱和
//                                中心格代价    取值集合 = [0,118,125,134,137]  ← 有梯度
//
//   clearance ≥ 0.42           足迹不碰致命带，MPPI 正常工作 ⇒ 不需要本层。
//
// 换成通道物理宽度 W（= 2×clearance，通道中线处）：
//   W < 0.776    任何朝向都放不进去
//   0.776 ≤ W < 0.84   **只有「八边形平边正对通道壁」才过得去**   ⇐ 本层
//   W ≥ 0.84     任意朝向都能过
//
// ============================ 由此确定的两条设计约束 ============================
//
// A) **朝向是能不能过去的决定因素，不是优化项。**
//    在 0.776~0.84 这个区间里，机器人必须把八边形的**平边**正对通道壁；
//    顶点正对时需要 0.84 的宽度，会卡住。八边形每 45° 复现一次有利朝向，
//    所以朝向误差要归一到最近的 45° 整数倍，而不是归一到通道方向本身。
//    只做「横向对齐最低代价中线」而不控朝向，在这个区间过不去。
//
// B) **横向扫描不能查单格，必须用连续位姿的足迹代价。**
//    本档的宽度只有 0.42 − 0.388 = **0.032 m**，而栅格分辨率是 0.05 m ——
//    比目标区间还大。查单格在这一档上没有任何分辨力。
//    所以扫描步长取 0.01 m 量级，并且用「把足迹放在该位姿上求最大代价」这个量，
//    由调用方注入（本层不依赖 costmap 类型）。
//
// 🔴 安全红线：真碰撞（254 = LETHAL_OBSTACLE）绝不允许穿越。
//    253 是膨胀语义（可以贴），254 是真障碍（不可以）。两者必须分开判。
//
// 角度约定：弧度，(-pi, pi] 归一化，正为逆时针（右手系绕 +z），与 align_math 一致。

#ifndef ASTRIBOT_S1_PATH_TRACKING__NARROW_MATH_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__NARROW_MATH_HPP_

#include <cstddef>
#include <functional>
#include <vector>

#include "astribot_s1_path_tracking/align_math.hpp"     // PlanarPoint / normalizeAngle 约定

namespace astribot_s1_path_tracking
{

/// nav2 代价语义。刻意写成常量而不是散在判断里 —— 253/254 的区别就是
/// 「可以贴」和「不可以撞」的区别，混淆一次就是安全事故。
struct NarrowCostValues
{
  /// 膨胀层写的「中心在此⇒足迹必然碰撞」。可以贴，不算真碰撞。
  static constexpr double kInscribedInflated = 253.0;
  /// 真障碍。红线，绝不穿越。
  static constexpr double kLethal = 254.0;
  /// 未知。
  static constexpr double kNoInformation = 255.0;
};

/// 触发判定结论。
enum class NarrowVerdict
{
  /// 不在窄通道档：足迹没碰致命带，MPPI 正常工作。
  kNone,
  /// 本层目标域：中心格可站、足迹碰致命带 ⇒ 接管跟踪。
  kNarrow,
  /// 机器人中心已在致命带内 ⇒ 物理放不进去。**不是本层职责**，
  /// 应由协调器 ESCAPE 把机器人挪出来。本层拒绝接管并如实上报。
  kCenterLethal,
  /// 足迹已压到真障碍（254）⇒ 红线。停车告警，不许贴边硬挤。
  kPhysicallyBlocked,
  /// 没有可用的全局路径 ⇒ 本层以路径为核心约束，无路径不接管。
  kNoPath
};

const char * toString(NarrowVerdict v);

/// 触发判定配置。
struct NarrowTriggerConfig
{
  /// 足迹代价达到多少算「碰致命带」。默认 253，与 MPPI 的饱和阈值一致。
  double footprint_lethal_threshold{NarrowCostValues::kInscribedInflated};
  /// 中心格代价达到多少算「中心已致命」。默认 253，与 SmacPlanner2D 判起点一致。
  double center_lethal_threshold{NarrowCostValues::kInscribedInflated};
  /// 连续多少拍满足条件才接管。避免边界抖动导致反复切换控制律。
  int trigger_ticks{3};
};

/// 触发判定。**只做判断，不产生指令。**
///
/// 判定顺序刻意如此，不能调换：
///   ① 最有利朝向下仍压真障碍(254) → kPhysicallyBlocked（红线优先于一切）
///   ② 中心格致命                  → kCenterLethal（物理放不进去，本层不接管）
///   ③ 无路径                      → kNoPath
///   ④ 足迹未碰致命带              → kNone
///   ⑤ 其余                        → kNarrow
///
/// ⚠️ **红线判据用的是「最有利朝向下的足迹代价」，不是当前朝向下的。**
///
/// 为什么：足迹是正八边形，外接 0.420m、内切 0.388m，**顶点比边中点多伸出
/// 0.034m**。通道宽度落在 0.772~0.840m 时，车顶点朝墙就压到 254、边朝墙就过得去
/// —— 而这一档正好是本层唯一存在的理由（占可行域 3.42%）。
///
/// 用当前朝向判红线，实测后果是本层**在自己的目标域上把自己否掉**：
/// 机器人以不利朝向进窄处 → 红线立刻抛异常 → 朝向闸门还没来得及把车转正。
/// 一条实测时序里中心代价 0 → 218 → 229 → 致命，机器人被一路推进膨胀带深处，
/// 最后自己变成了 ESCAPE 的对象。
///
/// 所以正确的问法不是「现在撞不撞」，而是「**转到最有利朝向后还撞不撞**」：
///   · 最有利朝向下不撞 ⇒ 不是物理堵死，接管并先转朝向（闸门负责）
///   · 最有利朝向下仍撞 ⇒ 真的过不去，红线成立
/// 这**不是放宽安全判据** —— 转朝向本身是本层的既有动作，问的是同一台车
/// 在它能达到的姿态下能否通过。
///
/// @param center_cost      机器人中心格代价（0~255），与朝向无关
/// @param footprint_cost   **当前朝向**下的足迹最大代价，用于判「是否在窄通道档」
/// @param favorable_cost   **最有利朝向**下的足迹最大代价，用于判红线
/// @param have_path        是否有可用全局路径
NarrowVerdict evaluateNarrowTrigger(
  double center_cost,
  double footprint_cost,
  double favorable_cost,
  bool have_path,
  const NarrowTriggerConfig & cfg);

/// 八边形（或任意 n 边正多边形）的**有利朝向误差**。
///
/// 为什么不是「朝向对齐通道方向」：正八边形每 45° 就复现一次相同的横向包络，
/// 所以有利朝向不是一个角度，而是一个**同余类**。把误差归一到最近的
/// period 整数倍，机器人就会转到最近的有利朝向，而不是绕远路去凑通道方向。
///
/// @param yaw               机器人当前朝向(rad)
/// @param corridor_heading  通道方向(rad)，通常取全局路径在机器人附近的切向
/// @param period_rad        有利朝向周期。正八边形 = pi/4；必须 > 0
/// @return 有向角误差，落在 [-period/2, +period/2]。正值表示应顺时针修正
///         （即 wz 应取其相反数）。
double favorableYawError(double yaw, double corridor_heading, double period_rad);

/// 横向扫描的一个采样结果。
struct LateralSample
{
  /// 相对当前位置的横向偏移(m)，正为通道方向左侧。
  double offset_m{0.0};
  /// 该偏移处的足迹最大代价。
  double footprint_cost{0.0};
  /// 是否可用（未压真障碍、未越界）。
  bool usable{false};
};

/// 横向扫描结果。
struct LateralScanResult
{
  bool valid{false};
  /// 最优横向偏移(m)。
  double best_offset_m{0.0};
  double best_cost{0.0};
  /// 扫描到的可用样本数（0 表示两侧全被真障碍堵住）。
  std::size_t usable_count{0U};
  /// 代价是否**完全饱和**（所有可用样本代价相同）。
  ///
  /// 这一项必须显式返回：饱和时「最低代价中线」是无意义的（任选一点都一样），
  /// 此时应当放弃横向寻优、纯粹沿路径走，而不是把噪声当梯度去追。
  /// 实测本项目就是在饱和区把 [253] 当成了有梯度的量。
  bool saturated{false};
};

/// 代价查询回调：给定世界位姿，返回足迹最大代价。
///
/// 做成回调是为了让本层不依赖 nav2_costmap_2d ——
/// 上线时注入 FootprintCollisionChecker::footprintCostAtPose，
/// 测试时注入一个解析函数，两者走完全相同的判据代码。
using FootprintCostFn = std::function<double (double x, double y, double yaw)>;

/// 垂直于通道方向扫描，找足迹代价最低的横向偏移。
///
/// ⚠️ 步长必须远小于栅格分辨率能分辨的量：本档目标区间只有 0.032m 宽
/// （外接 0.42 − 内切 0.388），而栅格是 0.05m。所以默认步长 0.01m。
///
/// @param robot            机器人当前位置
/// @param yaw              机器人当前朝向（求足迹代价要用）
/// @param corridor_heading 通道方向；横向 = 通道方向逆时针 90°
/// @param half_width_m     单侧扫描范围(m)，必须 > 0
/// @param step_m           扫描步长(m)，必须 > 0
/// @param cost_fn          足迹代价查询
LateralScanResult scanLateral(
  const PlanarPoint & robot,
  double yaw,
  double corridor_heading,
  double half_width_m,
  double step_m,
  const FootprintCostFn & cost_fn);

/// 贴边通行的速度限制。全部低速 —— 这是受限通行，不是正常跟踪。
struct NarrowLimits
{
  /// 沿通道前进速度(m/s)。
  double v_along{0.10};
  /// 横向修正速度上限(m/s)。
  double v_lateral_max{0.05};
  /// 角速度上限(rad/s)。
  double wz_max{0.20};
  /// 横向修正比例增益。
  double kp_lateral{1.0};
  /// 朝向修正比例增益。
  double kp_yaw{1.5};
  /// 朝向误差大于此值时**不前进**，先转到有利朝向(rad)。
  ///
  /// 这是本层最关键的一条：在 0.776~0.84 区间里，朝向不对就是过不去，
  /// 带着错的朝向往前走等于往卡死里走。
  double yaw_gate_rad{0.12};
};

/// 一拍的贴边通行指令（车体系，全向底盘）。
struct NarrowCommand
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
  /// 是否因朝向未对齐而暂停前进（诊断用，让「原地转」这件事可见）。
  bool holding_for_yaw{false};
};

/// 由「通道方向 + 横向偏移目标 + 朝向误差」算受限速度。
///
/// @param yaw              机器人当前朝向
/// @param corridor_heading 通道方向
/// @param lateral_offset_m 期望的横向偏移（scanLateral 的 best_offset_m）
/// @param yaw_error        favorableYawError 的结果
NarrowCommand narrowVelocity(
  double yaw,
  double corridor_heading,
  double lateral_offset_m,
  double yaw_error,
  const NarrowLimits & lim);

/// 由路径估计机器人所在处的通道方向（切向）。
///
/// 用「最近点往前 lookahead 弧长」而不是相邻两点：相邻点间距可能只有几毫米，
/// 方向被噪声主导（与 align_math::pathStartHeading 同一理由）。
///
/// @return 是否估计成功。失败时**不返回 0 冒充成功** —— 调用方必须区分。
bool corridorHeadingFromPath(
  const std::vector<PlanarPoint> & path,
  const PlanarPoint & robot,
  double lookahead_m,
  double & heading);

/// 退出判据：连续 need 拍足迹代价低于阈值才算脱离窄通道。
///
/// 要求连续多拍：边界上足迹代价会抖，一拍就判脱离会让控制律反复切换。
bool narrowCleared(int consecutive_clear_ticks, int need);

/// 「连续贴边失败」计数的推进规则。
///
/// ⚠️ 语义是**连续失败次数**，不是「本条路径贴过几次边」。
/// 起初实现成后者，把成功穿越也算进上限，实测 13 次接管里 10 次是正常穿过
/// （足迹已脱离致命带），却仍触发 2 次「判定该路径不可行」——
/// 把一条沿途有 4 个窄处、但完全走得通的路径判成了不可行。
/// **长路径经过多个窄处是常态，不是异常。**
///
/// @param cleared_through 本次接管是否以「穿过去了」结束
/// @param count           连续失败次数，原地更新
void updateNarrowFailureCount(bool cleared_through, int & count);

/// 是否应就当前目标放弃贴边（抛「路径不可行」）。
///
/// already_gave_up 这一项必须有：抛异常到 FollowPath 真的 abort 之间有延迟，
/// 期间 controller_server 仍以 20Hz 调进来。实测每次放弃刷了 **15 条**同样的
/// WARN，多抛的 14 次没有任何作用，只是把这条本该被看见的告警淹掉。
bool shouldGiveUpNarrow(int consecutive_failures, int max_attempts, bool already_gave_up);

/// 机器人在路径上的**弧长进度**：路径起点到最近点的累计弧长。
///
/// 为什么用弧长而不是「到终点的直线距离」：贴边通行会横移，直线距离会被
/// 横移污染（横着挪 5cm 也能让直线距离变小或变大）。弧长只沿路径累计，
/// 横移不产生假进展。
///
/// ⚠️ **不要用它做卡住判据** —— 用 pathRemainingArc。这个量的原点是
/// 「当前这条路径的起点」，而路径会被重规划替换，原点因此会跳。
/// 实测两种表现（v4 五轮）：
///   · 进度涨到 1.5037m 后换路径 → 掉回 ≈0 → 永远超不过高水位 → 6s 后误判卡住
///   · 新路径起点就在机器人身上 → 进度恒 `0.000000m`，一次都没涨过
/// 两次事件里机器人都在被指令以 0.10m/s 前进（指令 (0.076,0.065)/(0.099,-0.011)）。
/// 保留本函数只为单测与调试观测。
///
/// @return false = 路径退化（点数 < 2）。此时**不返回 0 冒充成功**。
bool pathArcProgress(
  const std::vector<PlanarPoint> & path, const PlanarPoint & robot, double & progress_m);

/// 机器人到**路径终点**的剩余弧长：最近点投影处沿路径到最后一个点的长度。
///
/// 这是卡住判据该用的量，理由是它**对重规划不敏感**：
/// 同一个目标重规划出来的新路径，剩余长度与旧路径相近（都是"还要走多远"），
/// 而"从起点累计"的量在换路径时会被重置到 ≈0。
///
/// 单调性：机器人沿路径前进 ⇒ 剩余弧长单调减。穿弯道时也成立
/// （弯道只让剩余长度减得慢，不会让它变大），所以不会像"累计弧长"
/// 那样在折返处停止增长而被误判。
///
/// 横移不产生假进展：与 pathArcProgress 同理，量沿路径计，不含横向分量。
///
/// @return false = 路径退化（点数 < 2）
bool pathRemainingArc(
  const std::vector<PlanarPoint> & path, const PlanarPoint & robot, double & remaining_m);

/// 「有没有进展」的判据状态。调用方持有，跨拍保留。
struct NarrowProgressState
{
  /// 迄今**最小**的剩余弧长（高水位，只减不增）。
  double best_remaining_m{0.0};
  /// 上一次刷新 best_remaining_m 的时刻(s)。
  double last_gain_sec{0.0};
  /// 是否已初始化（进入接管时置位）。
  bool initialized{false};
};

/// 卡住判据：**按进展判，不按时长判**；进展用「剩余弧长在减少」度量。
///
/// ⚠️ 这里连续踩过两个坑，两次都是「拿衡量正常的量去衡量失败」：
///
/// 坑 1（时长）：最初是「接管持续超过 narrow_timeout 秒即失败」。而
///   narrow_timeout 25s x v_along 0.10 m/s = 2.5 m
/// 等于给窄通道悄悄设了 **2.5 米的长度上限** —— 一个我从没打算设、
/// 也从没写下来的上限。实测一条 2.5m 长的窄通道被连砍 3 次
/// （每次都恰好停在 25.0x 秒），期间机器人全程以 0.10m/s 正常前进、
/// 朝向误差始终在 ±0.008rad 内、离路径最远 0.112m —— 完全健康。
///
/// 坑 2（从起点累计的弧长）：改成按弧长进展判之后，用的是
/// pathArcProgress = 「路径起点 → 投影点」。但 replan_policy: on_invalid
/// 会替换路径，**新路径的起点就在机器人附近**，于是这个量的原点会跳。
/// 实测两种表现（v4 五轮）：
///   · 涨到 1.5037m 后换路径 → 掉回 ≈0 → 永远超不过高水位 → 6s 后误判卡住
///   · 新路径起点就在机器人身上 → 恒 `0.000000m`，一次都没涨过
/// 两次事件里机器人都在被指令以 0.10m/s 前进。
///
/// 现在用**剩余弧长**（pathRemainingArc）：对换路径不敏感、穿弯道单调。
///
/// 「真的卡住」的特征因此是：时间在走，而**剩余弧长不再减少**。
///
/// @param remaining_m        当前到路径终点的剩余弧长
/// @param now_sec            当前时刻(s)
/// @param min_gain_m         剩余弧长至少要减少这么多才算有进展（滤噪声）
/// @param stall_timeout_sec  多久没有进展算卡住
/// @param state              跨拍状态，原地更新
/// @return true = 卡住
bool narrowStalled(
  double remaining_m,
  double now_sec,
  double min_gain_m,
  double stall_timeout_sec,
  NarrowProgressState & state);

/// 朝向闸门是否关闭（= 只转不走）。
///
/// 抽成函数是因为有**两个**调用点：narrowVelocity 用它决定是否置零平移，
/// 卡住判据用它决定这一拍是否计入无进展窗口。两处曾各写一遍同样的
/// `fabs(yaw_error) > yaw_gate_rad`，靠注释约定"必须逐字一致"——
/// 那种约定会漂，而漂的后果是"闸门关着却在累计无进展"，即误杀。
[[nodiscard]] bool yawGateHolding(double yaw_error, double yaw_gate_rad);

/// 路径是否被换掉了（重规划）。换了就必须重开进展窗口。
///
/// 为什么必须有这一条：高水位是跨拍保留的，而剩余弧长虽然对"同一目标的
/// 重规划"不敏感，但**换目标**时会整体跳变（新目标可能更远）。
/// 若不重开窗口，换目标那一刻剩余弧长变大，永远超不过旧高水位 → 误判卡住。
///
/// 判据用「终点位置 + 路径点数」而不是逐点比对：足够区分换路径，
/// 又不会因为重规划带来的微小顶点抖动而误判成换了路径。
///
/// @param path      当前路径
/// @param last_end  上一次记录的终点，原地更新
/// @param last_size 上一次记录的点数，原地更新
/// @return true = 认为路径已被替换（调用方应重置 NarrowProgressState）
bool pathWasReplaced(
  const std::vector<PlanarPoint> & path, PlanarPoint & last_end, std::size_t & last_size);

}  // namespace astribot_s1_path_tracking

#endif  // ASTRIBOT_S1_PATH_TRACKING__NARROW_MATH_HPP_
