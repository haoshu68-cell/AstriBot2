// Copyright 2026 Astribot. Apache-2.0.
//
// 曲率-速度耦合的纯函数层（不依赖 ROS / xtensor，可离线单测）。
//
// 拆开的理由与 align_math.hpp 相同：这层写错时的在线现象是"机器人过弯偏慢/偏快"，
// 极难反推到某个公式；而单测能直接钉住"给定曲率应许多少速度"。
//
// ============================ 核心力学 ============================
// 设某条 rollout 在某一拍的平面速度模长 v = hypot(vx, vy)、角速度 wz。
// 该拍的轨迹曲率与向心（侧向）加速度分别是：
//
//     kappa = |wz| / v                       (v -> 0 时无定义，见 kEpsSpeed)
//     a_lat = v^2 * kappa = v * |wz|         <-- 恒等式，**不含除法**
//
// 于是"限制侧向加速度"与"按曲率限速"是同一件事：
//
//     v * |wz| <= a_budget   <==>   v <= sqrt(a_budget / kappa)
//
// 证明：a_lat = kappa * v^2；kappa*v^2 <= a_budget <==> v <= sqrt(a_budget/kappa)。
//
// 这条恒等式是本文件的全部设计依据 —— 打分时用左式（无除法、v→0 天然为 0、
// 原地旋转自动免罚），对外解释与标定时用右式（就是需求里的 vx = f(kappa)）。
//
//
// ============================ 软带的必要性 ============================
// 惩罚从 soft_ratio * a_lat_max 起算，而不是从 a_lat_max 起算。
// 理由是本仓库已两次栽在"判据区间内梯度恒为 0"上：
//   · MPPI 足迹代价在窄通道里饱和成 253（饱和到顶，无梯度）
//   · 全局规划在宽于 2*inflation_radius 的通道里代价恒 0（饱和到底，无梯度）
// 硬 hinge 同样是这一类：未越限时代价恒 0，MPPI 无法在"还没超"的样本之间
// 分出好坏，只能等超了才反应，表现为过弯先冲出去再修。软带让梯度提前出现。

#ifndef ASTRIBOT_S1_PATH_TRACKING__CURVATURE_SPEED_MATH_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__CURVATURE_SPEED_MATH_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace astribot_s1_path_tracking
{
namespace curvature_speed
{

/// v 小于此值时曲率无定义，一律按"直行/原地"处理（不罚）。
constexpr double kEpsSpeed = 1e-3;
/// 三点几乎共线或重合时的长度下限，低于它判 kappa=0。
/// nav2 的 Path 里**确实存在重复点**（平滑器与 prune 都会留），不能不设。
constexpr double kEpsLen = 1e-6;

/// 曲率-速度耦合的四个标定量。字段含义与 yaml 同名参数一一对应。
struct Limits
{
  /// 侧向加速度预算 [m/s^2]。惩罚的硬边界。
  double a_lat_max{0.35};
  /// 软带起点占硬边界的比例，(0, 1]。取 1.0 = 退化成硬 hinge（不推荐，见文件头）。
  double soft_ratio{0.6};
  /// 过弯速度地板 [m/s]。曲率再大也不要求慢过这个值，否则大曲率处会被要求近乎停住，
  /// 与 PathFollowCritic 的前推直接对抗，表现为在弯前僵住（progress checker 会超时）。
  double v_min_turn{0.08};
  /// 直行时的速度上限 [m/s]，取 vx_max。曲率趋 0 时 v_allow 收敛到它。
  double v_max{1.0};
};

/// 参数自检。返回 false 时 why 写明哪一条不成立（照抄到 ROS 日志即可）。
bool validate(const Limits & lim, std::string & why);

/// 给定侧向加速度预算与曲率，返回允许速度 sqrt(a_budget/kappa)，夹到 [v_min_turn, v_max]。
/// kappa <= 0 视为直行，返回 v_max。
double speedForLateralAccel(double a_budget, double kappa, const Limits & lim);

/// 硬上限：v_allow(kappa)，用 a_lat_max。用于日志/标定，不直接进代价。
double allowedSpeed(double kappa, const Limits & lim);

/// 软上限：v_allow(kappa)，用 soft_ratio * a_lat_max。惩罚从这里开始。
double softSpeedCap(double kappa, const Limits & lim);

/// 由 (v, wz) 算轨迹曲率 |wz|/v；v < kEpsSpeed 时返回 0（原地旋转不算有曲率）。
double trajectoryCurvature(double v, double wz);

/// 【项1 反馈项】单拍的侧向加速度超出量，已归一化到 a_lat_max。
/// = max(0, v*|wz| - soft_ratio*a_lat_max) / a_lat_max
/// 恰好到硬边界时返回 (1 - soft_ratio)；越界后继续线性增长。
double lateralAccelExcess(double v, double wz, const Limits & lim);

/// 【项2 前馈项】单拍速度超出"路径曲率允许值"的量 [m/s]。
/// = max(0, v - softSpeedCap(kappa_path))
double speedExcessOverPathCap(double v, double kappa_path, const Limits & lim);

/// 三点 Menger 曲率：kappa = 2*|cross(b-a, c-b)| / (|b-a| * |c-b| * |c-a|)。
/// 任一边长小于 kEpsLen（重复点）时返回 0。
double mengerCurvature(
  double x0, double y0,
  double x1, double y1,
  double x2, double y2);

/// 从 start_idx 起沿路径累计弧长 lookahead_dist 之内的**最大** Menger 曲率。
/// 取 max 而不是 mean：一段直路末尾接一个急弯时，mean 会被直路稀释掉，
/// 而需要减速的恰恰是那个急弯。窗口内不足 3 点时返回 0。
double maxCurvatureInWindow(
  const std::vector<double> & xs,
  const std::vector<double> & ys,
  std::size_t start_idx,
  double lookahead_dist);

}  // namespace curvature_speed
}  // namespace astribot_s1_path_tracking

#endif  // ASTRIBOT_S1_PATH_TRACKING__CURVATURE_SPEED_MATH_HPP_
