// Copyright 2026 Astribot

#include "astribot_s1_manipulation/singularity_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <rclcpp/logging.hpp>

namespace astribot_s1_manipulation
{

namespace
{
constexpr const char * kLoggerName = "astribot_s1_manipulation.singularity";
}  // namespace

bool SingularityMonitor::configure(const SingularityParams & params, std::string & error)
{
  error.clear();

  if (!(params.min_singular_value > 0.0)) {
    error = "min_singular_value must be > 0, got " + std::to_string(params.min_singular_value);
    return false;
  }
  if (!(params.max_condition_number > 1.0)) {
    // 条件数天然 >= 1（σ_max >= σ_min），阈值给 <= 1 等于"任何构型都判奇异"。
    error = "max_condition_number must be > 1.0, got " +
      std::to_string(params.max_condition_number);
    return false;
  }
  if (!(params.degenerate_jacobian_epsilon > 0.0)) {
    error = "degenerate_jacobian_epsilon must be > 0, got " +
      std::to_string(params.degenerate_jacobian_epsilon);
    return false;
  }

  params_ = params;
  configured_ = true;
  return true;
}

SingularityReport SingularityMonitor::check(
  const moveit::core::RobotState & state,
  const moveit::core::JointModelGroup * jmg,
  const std::string & tip_link_name) const
{
  SingularityReport report;

  if (!configured_) {
    report.reason = "SingularityMonitor not configured";
    return report;
  }
  if (jmg == nullptr) {
    report.reason = "joint model group pointer is null";
    return report;
  }
  if (tip_link_name.empty()) {
    report.reason = "tip link name is empty";
    return report;
  }

  // getJacobian 要求该组是一条运动链。双臂组(dual_arm)不是链，
  // 必须由调用方拆成左右两条分别检测 —— 这里显式拦住，
  // 否则 MoveIt 内部会打断言/返回垃圾数据。
  if (!jmg->isChain()) {
    report.reason = "group '" + jmg->getName() +
      "' is not a kinematic chain; check each arm separately";
    return report;
  }

  const moveit::core::RobotModelConstPtr & model = state.getRobotModel();
  if (!model) {
    report.reason = "robot model is null";
    return report;
  }
  const moveit::core::LinkModel * tip_link = model->getLinkModel(tip_link_name);
  if (tip_link == nullptr) {
    report.reason = "tip link '" + tip_link_name + "' not found in robot model";
    return report;
  }

  // 本函数是 const，但 FK 需要 update()。拷一份可写状态：
  // 拷贝 RobotState 的开销远小于一次 SVD，也避免了对调用方状态的副作用。
  moveit::core::RobotState local_state(state);
  local_state.updateLinkTransforms();

  Eigen::MatrixXd jacobian;
  const bool ok = local_state.getJacobian(
    jmg, tip_link, Eigen::Vector3d::Zero(), jacobian, false);
  if (!ok) {
    report.reason = "getJacobian failed for group '" + jmg->getName() +
      "' tip '" + tip_link_name + "'";
    return report;
  }
  if (jacobian.rows() == 0 || jacobian.cols() == 0) {
    report.reason = "jacobian is empty";
    return report;
  }
  // 数值防护：任何 NaN/Inf 进 SVD 都会得到无意义结果（甚至不收敛）。
  // 宁可判为"检测无效"，也不能拿垃圾数值当合法结论。
  if (!jacobian.allFinite()) {
    report.reason = "jacobian contains non-finite values";
    return report;
  }

  // 只要奇异值，不要 U/V，所以不传 ComputeThinU/V —— 省一半计算量。
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  const Eigen::VectorXd & sv = svd.singularValues();
  if (sv.size() == 0) {
    report.reason = "SVD produced no singular values";
    return report;
  }

  // Eigen 的奇异值按降序排列，取首尾即为 max/min。
  report.max_singular_value = sv(0);
  report.min_singular_value = sv(sv.size() - 1);
  report.valid = true;

  // 退化保护：σ_max 都接近 0，说明雅可比整体退化，不能拿它算条件数（会除零）。
  if (report.max_singular_value < params_.degenerate_jacobian_epsilon) {
    report.condition_number = std::numeric_limits<double>::infinity();
    report.singular = true;
    std::ostringstream oss;
    oss << "degenerate jacobian: sigma_max=" << report.max_singular_value
        << " < epsilon=" << params_.degenerate_jacobian_epsilon;
    report.reason = oss.str();
    return report;
  }

  // σ_min 可能正好是 0，此时条件数为无穷 —— 用 epsilon 兜住除法，
  // 并把条件数标为 inf，而不是让它变成一个巨大但有限的假数字。
  if (report.min_singular_value < params_.degenerate_jacobian_epsilon) {
    report.condition_number = std::numeric_limits<double>::infinity();
  } else {
    report.condition_number = report.max_singular_value / report.min_singular_value;
  }

  if (!params_.enabled) {
    // 关闭检测时仍然返回数值（便于观察），但一律判非奇异。
    report.singular = false;
    report.reason = "singularity check disabled by configuration";
    return report;
  }

  std::ostringstream oss;
  if (report.min_singular_value < params_.min_singular_value) {
    report.singular = true;
    oss << "sigma_min=" << report.min_singular_value
        << " below threshold " << params_.min_singular_value;
    report.reason = oss.str();
    return report;
  }
  if (report.condition_number > params_.max_condition_number) {
    report.singular = true;
    oss << "condition_number=" << report.condition_number
        << " above threshold " << params_.max_condition_number;
    report.reason = oss.str();
    return report;
  }

  report.singular = false;
  oss << "ok: sigma_min=" << report.min_singular_value
      << " cond=" << report.condition_number;
  report.reason = oss.str();
  return report;
}

SingularityReport SingularityMonitor::checkStates(
  const std::vector<moveit::core::RobotState> & states,
  const moveit::core::JointModelGroup * jmg,
  const std::string & tip_link_name,
  std::size_t * worst_index) const
{
  SingularityReport worst;
  if (states.empty()) {
    worst.reason = "no states to check";
    return worst;
  }

  double worst_sigma = std::numeric_limits<double>::infinity();
  std::size_t worst_i = 0;
  bool any_valid = false;
  // 「逃离段」标志：轨迹开头一段可能仍然贴着奇异构型 —— 从奇异点出发，
  // 头几个点必然还在奇异邻域里（实测：全零起步时 waypoint 1 的
  // sigma_min 仍只有 0.0104）。只豁免第 0 点是不够的。
  // 规则：在**首次出现非奇异点之前**的所有点都算逃离段、一律豁免；
  // 一旦出现过非奇异点，就说明已经脱离奇异邻域，之后任何奇异点
  // 都是"走进奇异"，必须否决。
  // 这个规则自带终止性（不需要配一个拍脑袋的豁免点数上限），
  // 而且严格实现了"允许离开、禁止进入"的语义。
  bool still_escaping = params_.allow_singular_start;

  for (std::size_t i = 0; i < states.size(); ++i) {
    const SingularityReport r = check(states[i], jmg, tip_link_name);
    if (!r.valid) {
      // 某一点算不出雅可比属于检测失败，直接上报：不能当作"没问题"跳过。
      if (worst_index != nullptr) {
        *worst_index = i;
      }
      return r;
    }
    any_valid = true;

    if (!r.singular) {
      // 见到第一个非奇异点：逃离段结束，后面严格检查。
      still_escaping = false;
    } else if (still_escaping) {
      RCLCPP_INFO(
        rclcpp::get_logger(kLoggerName),
        "waypoint %zu is still singular (%s) but exempt as part of the escape "
        "from a singular start; the trajectory is allowed to move away from it",
        i, r.reason.c_str());
    } else {
      // 已经脱离过奇异邻域又走回去 —— 这才是要拦的情况。
      // 立即返回：轨迹只要有一点奇异就整条不可用，没必要算完剩下的点
      // （省时间，且错误定位更准）。
      if (worst_index != nullptr) {
        *worst_index = i;
      }
      return r;
    }

    if (r.min_singular_value < worst_sigma) {
      worst_sigma = r.min_singular_value;
      worst_i = i;
      worst = r;
    }
  }

  if (!any_valid) {
    worst.valid = false;
    worst.reason = "no valid jacobian on any state";
  }
  if (worst_index != nullptr) {
    *worst_index = worst_i;
  }

  // 走到这里说明**没有任何需要否决的点**：真正奇异的非豁免点会在循环里
  // 提前 return。但 worst 可能是被豁免的起点的报告，它自己的 singular 标志
  // 是 true —— 直接返回会让调用方误判"整条轨迹奇异"。
  // 所以这里显式把聚合结论置为"不奇异"，同时保留实测数值供观察。
  if (worst.valid && worst.singular) {
    std::ostringstream oss;
    oss << "ok (start waypoint exempt): worst sigma_min=" << worst.min_singular_value
        << " at index " << worst_i
        << ", no non-exempt waypoint is singular";
    worst.reason = oss.str();
    worst.singular = false;
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger(kLoggerName),
    "checked %zu states, worst sigma_min=%.6f at index %zu",
    states.size(), worst.min_singular_value, worst_i);
  return worst;
}

}  // namespace astribot_s1_manipulation
