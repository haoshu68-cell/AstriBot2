// Copyright 2026 Astribot
//
// 三段式路径跟踪的**纯几何/相位判断**。
//
// 为什么单独拆一层：这里全部是自由函数、不碰 ROS、不碰 costmap，
// 因此可以离线逐条测。相位跃迁一旦写错，在线表现是「机器人转个不停」或
// 「跳过某一段」，而这两种都很难从日志反推，所以判断逻辑必须能单独验证。
//
// 角度约定：全部用 (-pi, pi] 归一化后的弧度；正为逆时针（右手系绕 +z）。

#ifndef ASTRIBOT_S1_PATH_TRACKING__ALIGN_MATH_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__ALIGN_MATH_HPP_

#include <cstddef>
#include <vector>

namespace astribot_s1_path_tracking
{

/// 平面点，避免这一层依赖 geometry_msgs。
struct PlanarPoint
{
  double x{0.0};
  double y{0.0};
};

/// 判断两条路径是否指向「同一个目标」。
///
/// 为什么需要它：nav2 的默认行为树**每秒重规划一次**，每次都会调 setPlan()。
/// 若把每次 setPlan 都当成新目标而重置到 ALIGN_START，机器人就会每秒掉回
/// 起步对齐段 —— 实测 86 次重规划里有 12 次真的停下来原地转，最长 6.76s，
/// 把连续行驶切成一段段。所以只有**终点真的换了**才算新目标。
///
/// @param prev_end  上一条路径的终点
/// @param cur_end   当前路径的终点
/// @param eps_m     判定阈值（m），必须 > 0
bool isSameGoal(const PlanarPoint & prev_end, const PlanarPoint & cur_end, double eps_m);

/// 三段式的相位。kDone 只表示「本控制器认为没有更多主动动作」，
/// 是否真的算到达仍由 nav2 的 GoalChecker 判定 —— 两者刻意分开。
///
/// 因此 kDone **不是吸收态**：它是一个可撤回的意见。路径末端一旦跑出位置容差
/// （1Hz 重规划换路径、恢复行为把机器人原地转走都会造成这个），必须退回 kFollow
/// 继续开。写成吸收态的后果是永久停车 + 整段目标跑到超时，见 align_math.cpp
/// 里 kDone 分支的实测记录。
enum class Phase
{
  kAlignStart,
  kFollow,
  kAlignGoal,
  kDone,
};

const char * toString(Phase p);

/// 把任意角归一化到 (-pi, pi]。
double normalizeAngle(double a);

/// 从 a 转到 b 的最短有向角差，结果在 (-pi, pi]。
/// 符号即旋转方向 —— 取近路，不允许绕远。
double shortestAngularDiff(double from, double to);

/// 由路径起始一段估计「出发朝向」。
///
/// 不用相邻两点：相邻点间距可能只有几毫米，方向被噪声主导。
/// 做法是从起点沿路径累积弧长，走过 lookahead_m 后用该点与起点连线定方向；
/// 路径总长不足 lookahead_m 时退化为「起点到终点」。
///
/// @param path         路径顶点，至少 2 个
/// @param lookahead_m  前视弧长，必须 > 0
/// @param[out] heading 估计到的朝向（弧度）
/// @return 是否估计成功（点数不足或路径退化为单点时返回 false，
///         **不抛异常也不返回 0 冒充成功** —— 调用方必须区分这两种情况）
bool pathStartHeading(
  const std::vector<PlanarPoint> & path, double lookahead_m, double & heading);

/// 判断是否需要为「起步对齐」而原地旋转。
/// 误差小于 min_angle_rad 时不值得转 —— 直接进跟踪段，避免原地抖动。
bool needsStartAlign(double heading_error_rad, double min_angle_rad);

/// 原地旋转的角速度指令。
///
/// 比例律 + 双向限幅：靠近目标时降速以压小滑行过冲（实测原地旋转发零速后
/// 仍余转 0.006~0.033 rad，速度越高越大），远离时不超过 max_vel。
/// floor_vel 是「能真正转起来的最小角速度」，只在**尚未达标**时施加，
/// 避免比例律算出一个小到驱动不了底盘的值而卡死。
///
/// @param error_rad  有向角误差（shortestAngularDiff 的结果）
/// @param kp         比例增益，> 0
/// @param max_vel    角速度上限，> 0
/// @param floor_vel  角速度下限（绝对值），>= 0
/// @param tol_rad    达标阈值；|error| <= tol_rad 时返回 0
double alignAngularVelocity(
  double error_rad, double kp, double max_vel, double floor_vel, double tol_rad);

/// 相位推进。**只做判断，不产生指令**，因此可以单独测。
///
/// @param current            当前相位
/// @param start_error_rad    起步朝向误差（有向）
/// @param goal_error_rad     终点姿态误差（有向）
/// @param dist_to_goal_m     到路径终点的距离
/// @param xy_tol_m           位置容差（来自 nav2 GoalChecker，不自带一份）
/// @param align_tol_rad      对齐达标阈值
/// @param start_min_rad      起步对齐的触发下限
/// @param align_goal_enabled 是否启用终点对齐（探索场景为 false）
/// @return 推进后的相位
Phase advancePhase(
  Phase current,
  double start_error_rad,
  double goal_error_rad,
  double dist_to_goal_m,
  double xy_tol_m,
  double align_tol_rad,
  double start_min_rad,
  bool align_goal_enabled);

/// 相位计时器是否应当重置。
///
/// 这是一条曾经被写错、且后果是**永久锁死整套导航**的不变式，所以单独抽成
/// 纯函数并配测试：原实现是"相位值没变就直接 return"，于是当上一个目标恰好在
/// ALIGN_START 段被中止、而新目标又要求进 ALIGN_START 时，计时器保持上一次的值
/// —— 新目标第一拍就判"对齐超时"并抛异常，之后每个目标都瞬间失败且永不恢复。
/// 实测证据：日志读到 "ALIGN_START 段超时 1108.297s > 15.0s"（约等于开机总时长），
/// 恢复探索后 12 个目标 12 个失败、0 次进度停滞（机器人根本没开始动）。
///
/// 规则：相位变了要重置；**相位没变但这是一个新目标，也必须重置**；
/// 同一目标的周期性重规划刻意不重置（否则对齐段永远等不到超时，保护形同虚设）。
bool shouldRestartPhaseTimer(Phase before, Phase requested, bool is_new_goal);

/// 这次 setPlan 是不是「同一目标的**新一次** FollowPath 下发」。
///
/// ⚠2026-09-04 实机根因。上面那条 shouldRestartPhaseTimer 只补住了
/// 「新目标 + 相位没变」这一个洞，**同一目标被重新下发**那个洞还开着：
/// setPlan 判出 same_goal 后会提前 return，压根走不到 enterPhase，
/// 于是相位计时器继续沿用上一次的值。实机时间轴：
///   · 734.0s  最后一次真·新目标 -> 计时器重置
///   · 之后协调器把**同一个**目标 (1.68, 8.43) 每 1.5s 重下发一次
///   · 844.8s  计时器读到 110.708s > 15.0s，每条新路径第一拍就抛超时
/// 后果与上面那条一模一样：Controller patience exceeded ×43 ->
/// Aborting handle ×48 -> 上游 3 连败 -> PAUSED -> 自动恢复 3 次全在 3s 内
/// 再死 -> 永久 parked。ALIGN_START 超时读数**单调爬升**就是这个洞的指纹。
///
/// 判据不能看路径内容（同一目标的两次下发路径几乎一样），只能看**时间空档**：
/// nav2 在一个 FollowPath action 存续期间会以 controller_frequency 持续调
/// computeVelocityCommands；action 一结束（成功/中止/取消）调用就停了。
/// 所以「上一次 tick 距今超过 gap_threshold」⇔「上一个 action 已经结束」，
/// 这一次 setPlan 就是新一次尝试，必须给它一份完整的对齐预算。
///
/// 阈值取 controller_frequency 的若干倍（默认 0.5s = 20Hz 下 10 拍）：
/// 大于单拍抖动、又远小于 align_timeout(15s)，不会把周期重规划误判成新尝试。
///
/// \param has_prev_tick 之前是否 tick 过（首次 setPlan 时为 false）
/// \param idle_gap_sec  距上一次 tick 的秒数（has_prev_tick 为 false 时忽略）
/// \param gap_threshold_sec 判定空档的阈值，必须 > 0
bool isFreshFollowAttempt(bool has_prev_tick, double idle_gap_sec, double gap_threshold_sec);

}  // namespace astribot_s1_path_tracking

#endif  // ASTRIBOT_S1_PATH_TRACKING__ALIGN_MATH_HPP_
