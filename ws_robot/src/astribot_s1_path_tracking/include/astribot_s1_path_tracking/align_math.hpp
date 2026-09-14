// Copyright 2026 Astribot

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
/// @param prev_end  上一条路径的终点
/// @param cur_end   当前路径的终点
/// @param eps_m     判定阈值（m），必须 > 0
bool isSameGoal(const PlanarPoint & prev_end, const PlanarPoint & cur_end, double eps_m);

/// 三段式的相位。kDone 只表示「本控制器认为没有更多主动动作」，
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
/// @param path         路径顶点，至少 2 个
/// @param lookahead_m  前视弧长，必须 > 0
/// @param[out] heading 估计到的朝向（弧度）
/// @return 是否估计成功（点数不足或路径退化为单点时返回 false，
bool pathStartHeading(
  const std::vector<PlanarPoint> & path, double lookahead_m, double & heading);

/// 判断是否需要为「起步对齐」而原地旋转。
/// 误差小于 min_angle_rad 时不值得转 —— 直接进跟踪段，避免原地抖动。
bool needsStartAlign(double heading_error_rad, double min_angle_rad);

/// 原地旋转的角速度指令。
/// @param error_rad  有向角误差（shortestAngularDiff 的结果）
/// @param kp         比例增益，> 0
/// @param max_vel    角速度上限，> 0
/// @param floor_vel  角速度下限（绝对值），>= 0
/// @param tol_rad    达标阈值；|error| <= tol_rad 时返回 0
double alignAngularVelocity(
  double error_rad, double kp, double max_vel, double floor_vel, double tol_rad);


/// 发零速后还会继续转过的角度（>= 0）。
/// @param wz       当前实测角速度 [rad/s]（符号无关，取绝对值）
/// @param lag_s    指令→实际的死时间 [s]，>= 0
/// @param decel_rad_s2 松手后的等效减速度 [rad/s^2]，> 0
/// @return 余转角 [rad]；参数非法或 wz 为 NaN 时返回 0
double coastAngle(double wz, double lag_s, double decel_rad_s2);

/// 按当前转速预测"现在松手会停在哪"的剩余误差（有向，同 error_rad 的符号约定）。
double predictedHeadingError(double error_rad, double wz, double lag_s, double decel_rad_s2);

/// 惯性补偿版的原地旋转指令。
double alignAngularVelocityWithInertia(
  double error_rad, double wz, double kp, double max_vel, double floor_vel, double tol_rad,
  double lag_s, double decel_rad_s2);

/// 航向是否**真的**到位：误差在容差内 **且** 已经停下来。
/// @param settled_wz 判"已静止"的角速度阈值 [rad/s]，必须 > 0 且 < floor 速度下的
bool headingSettled(double error_rad, double wz, double tol_rad, double settled_wz);

/// 相位推进。**只做判断，不产生指令**，因此可以单独测。
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

/// 相位推进（惯性补偿版）。相对 8 参版只多一件事：**离开对齐段要求"已静止"**，
/// 判据即 headingSettled。8 参版等价于本函数取 wz=0、settled_wz=+inf。
///
/// @param wz         当前实测角速度 [rad/s]
/// @param settled_wz 判"已静止"的阈值 [rad/s]，语义见 headingSettled
Phase advancePhase(
  Phase current,
  double start_error_rad,
  double goal_error_rad,
  double dist_to_goal_m,
  double xy_tol_m,
  double align_tol_rad,
  double start_min_rad,
  bool align_goal_enabled,
  double wz,
  double settled_wz);

/// 接近段线速度上限：离终点越近，允许的速度越小（线性收敛 + 下限）。
/// @param dist_to_goal_m 到路径终点的距离（m）
/// @param approach_dist_m 收敛区长度 D（m），必须 > 0
/// @param v_min 速度下限（m/s），保证底盘还能动（实测 0.02 m/s 即可平动）
/// @param nominal_speed 不限速时的速度（m/s），通常传内层算出的 ‖v‖
/// @return 允许的线速度模长上限；`dist >= D` 时原样返回 nominal_speed
double approachSpeedCap(
  double dist_to_goal_m, double approach_dist_m, double v_min, double nominal_speed);

/// 相位计时器是否应当重置。
bool shouldRestartPhaseTimer(Phase before, Phase requested, bool is_new_goal);

/// 这次 setPlan 是不是「同一目标的**新一次** FollowPath 下发」。
/// \param has_prev_tick 之前是否 tick 过（首次 setPlan 时为 false）
/// \param idle_gap_sec  距上一次 tick 的秒数（has_prev_tick 为 false 时忽略）
/// \param gap_threshold_sec 判定空档的阈值，必须 > 0
bool isFreshFollowAttempt(bool has_prev_tick, double idle_gap_sec, double gap_threshold_sec);

}  // namespace astribot_s1_path_tracking

#endif  // ASTRIBOT_S1_PATH_TRACKING__ALIGN_MATH_HPP_
