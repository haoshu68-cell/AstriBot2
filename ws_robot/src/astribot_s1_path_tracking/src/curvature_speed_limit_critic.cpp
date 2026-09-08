// Copyright 2026 Astribot. Apache-2.0.
// CurvatureSpeedLimitCritic 实现。设计依据、与既有 critic 的关系、标定口径
// 全部写在同名 .hpp 文件头，改这里之前先读那一段。

#include "astribot_s1_path_tracking/curvature_speed_limit_critic.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "astribot_s1_path_tracking/cost_accumulate.hpp"

#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

namespace mppi
{
namespace critics
{

void CurvatureSpeedLimitCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 2);
  getParam(weight_, "cost_weight", 6.0);
  getParam(path_weight_, "path_cost_weight", 8.0);
  getParam(path_lookahead_dist_, "path_lookahead_dist", 1.5);
  getParam(threshold_to_consider_, "threshold_to_consider", 0.5);

  double a_lat_max = 0.35, soft_ratio = 0.6, v_min_turn = 0.08;
  getParam(a_lat_max, "a_lat_max", 0.35);
  getParam(soft_ratio, "soft_ratio", 0.6);
  getParam(v_min_turn, "v_min_turn", 0.08);

  // v_max 不另设参数：直接取父控制器的 vx_max，避免同一个物理量在 yaml 里出现
  // 两次然后漂开（ConstraintCritic 读父命名空间也是这个做法）。
  double vx_max = 1.0, wz_max = 2.0;
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);
  getParentParam(vx_max, "vx_max", 1.0);
  getParentParam(wz_max, "wz_max", 2.0);

  limits_.a_lat_max = a_lat_max;
  limits_.soft_ratio = soft_ratio;
  limits_.v_min_turn = v_min_turn;
  limits_.v_max = vx_max;

  std::string why;
  if (!curvature_speed::validate(limits_, why)) {
    config_invalid_ = true;
    RCLCPP_ERROR(logger_, "%s 参数非法，本 critic 已停用：%s", name_.c_str(), why.c_str());
    return;
  }

  // ===== 空操作自检（必须打出来，否则这个 critic 会静默地什么都不做）=====
  // 侧向加速度的物理上界就是 vx_max * wz_max：两者都取满时 a_lat 最大。
  // a_lat_max 若不小于它，惩罚项恒为 0 —— 现象与"critic 没装上"完全一样，
  // 而 nav2 只会打一条 "Critic loaded"，不会告诉你它无效。
  const double a_lat_ceiling = vx_max * wz_max;
  if (limits_.a_lat_max >= a_lat_ceiling) {
    RCLCPP_WARN(
      logger_,
      "%s: a_lat_max=%.3f >= vx_max*wz_max=%.3f，本 critic 在当前速度上限下"
      "**恒为空操作**（侧向加速度物理上到不了这个值）。要么调小 a_lat_max，"
      "要么它就是白装的。",
      name_.c_str(), limits_.a_lat_max, a_lat_ceiling);
  }

  RCLCPP_INFO(
    logger_,
    "%s 已装载: a_lat_max=%.3f soft_ratio=%.2f(软带起点 %.3f) v_min_turn=%.3f "
    "v_max=%.3f(取自父 vx_max) w=%.2f path_w=%.2f power=%u lookahead=%.2fm | "
    "允许速度示例: kappa=0.5(R=2m)->%.3f  1.0(R=1m)->%.3f  2.0(R=0.5m)->%.3f m/s",
    name_.c_str(), limits_.a_lat_max, limits_.soft_ratio,
    limits_.soft_ratio * limits_.a_lat_max, limits_.v_min_turn, limits_.v_max,
    weight_, path_weight_, power_, path_lookahead_dist_,
    curvature_speed::allowedSpeed(0.5, limits_),
    curvature_speed::allowedSpeed(1.0, limits_),
    curvature_speed::allowedSpeed(2.0, limits_));
}

void CurvatureSpeedLimitCritic::score(mppi::CriticData & data)
{
  using xt::evaluation_strategy::immediate;

  if (!enabled_ || config_invalid_) {
    return;
  }
  // 终段让位：距目标已在容差内时，速度由 GoalCritic 与接近段限速接管。
  // 这里继续压速度会和它们叠乘，把终段拖成龟速（两层限速串联叠乘，见
  // memory two-speed-limiters-multiply-in-series）。
  if (mppi::utils::withinPositionGoalTolerance(
      threshold_to_consider_, data.state.pose.pose, data.path))
  {
    return;
  }

  const size_t time_steps = data.state.vx.shape(1);
  if (time_steps == 0) {
    return;
  }
  // 全时域参与，不做时间维降采样。
  // 曾加过 trajectory_point_step_ 想省 CPU，去掉的两个理由：
  //   1. xt::range(带 step) 存进变量再喂给 xt::sum(..., immediate) 编译不过
  //      （strided view 的 data_offset 里 static_cast<size_t>(xrange_adaptor) 非法）。
  //   2. 本 critic 只是两趟 elementwise + 一次归约，规模与 ConstraintCritic 完全相同，
  //      而它也是全量算的 —— 省这一下没有依据，属于凭感觉优化。
  const auto & vx = data.state.vx;
  const auto & vy = data.state.vy;
  const auto & wz = data.state.wz;

  // 保留了回退 Omni 的开关，那时用 |vx| 会低估真实速度而漏罚。
  const auto v = xt::sqrt(xt::square(vx) + xt::square(vy));

  const float dt = data.model_dt;

  // ---------------- 项1 反馈：罚 rollout 自身的侧向加速度超出量 ----------------
  // a_lat = v*|wz| 是恒等式（= kappa*v^2），不经过除法：v->0 天然为 0，
  // 原地旋转（ALIGN_START 相位）自动免罚。
  const float a_soft = static_cast<float>(limits_.soft_ratio * limits_.a_lat_max);
  const float a_norm = static_cast<float>(limits_.a_lat_max);
  const auto excess_alat = xt::maximum(v * xt::abs(wz) - a_soft, 0.0f) / a_norm;

  // 累加**必须**走 accumulateInPlace：写 data.costs += ... 会把 nav2 的缓冲区
  // 换成我们这边 std::allocator 分配的那块，nav2 按 32B 对齐读它 → SIGSEGV。
  // 实测证据与探针见 cost_accumulate.hpp 文件头。
  const xt::xtensor<float, 1> term_alat = xt::pow(
    xt::sum(excess_alat, {1}, immediate) * dt * weight_, power_);
  astribot_s1_path_tracking::cost_accumulate::accumulateInPlace(data.costs, term_alat);

  // ---------------- 项2 前馈：罚超出"路径曲率允许速度"的部分 ----------------
  if (path_weight_ <= 0.0f) {
    return;
  }
  const size_t path_size = data.path.x.shape(0);
  if (path_size < 3) {
    return;   // 路径太短算不出曲率；不是错误，直接不加这一项。
  }

  std::vector<double> xs(path_size), ys(path_size);
  for (size_t i = 0; i < path_size; ++i) {
    xs[i] = static_cast<double>(data.path.x(i));
    ys[i] = static_cast<double>(data.path.y(i));
  }
  // 锚点取"离轨迹起点（即机器人当前位置）最近的路径点"，向前看一段弧长。
  // 用 furthest_reached_path_point 是错的：那是所有 rollout 末端能摸到的最远点，
  // 前馈要的是**机器人前方紧接着**那段的曲率。
  const size_t anchor = mppi::utils::findPathTrajectoryInitialPoint(data);
  const double kappa_path = curvature_speed::maxCurvatureInWindow(
    xs, ys, anchor, static_cast<double>(path_lookahead_dist_));

  if (!(kappa_path > 0.0)) {
    return;   // 前方是直路，前馈项无事可做。
  }
  const float v_cap = static_cast<float>(
    curvature_speed::softSpeedCap(kappa_path, limits_));

  const auto excess_speed = xt::maximum(v - v_cap, 0.0f);
  const xt::xtensor<float, 1> term_path = xt::pow(
    xt::sum(excess_speed, {1}, immediate) * dt * path_weight_, power_);
  astribot_s1_path_tracking::cost_accumulate::accumulateInPlace(data.costs, term_path);
}

}  // namespace critics
}  // namespace mppi

#include <pluginlib/class_list_macros.hpp>
// 查找名必须逐字等于 "mppi::critics::" + yaml 里 critics 列表中的那个名字，
// 因为 CriticManager::getFullName() 无条件拼这个前缀（理由见 .hpp 文件头）。
PLUGINLIB_EXPORT_CLASS(
  mppi::critics::CurvatureSpeedLimitCritic,
  mppi::critics::CriticFunction)
