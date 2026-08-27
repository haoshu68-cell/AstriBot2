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

/// 三段式的相位。kDone 只表示「本控制器认为没有更多主动动作」，
/// 是否真的算到达仍由 nav2 的 GoalChecker 判定 —— 两者刻意分开。
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

}  // namespace astribot_s1_path_tracking

#endif  // ASTRIBOT_S1_PATH_TRACKING__ALIGN_MATH_HPP_
