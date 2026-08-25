// Copyright 2026 Astribot

#include "astribot_s1_manipulation/gripper_commander.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <thread>
#include <utility>

#include <moveit/robot_state/robot_state.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace astribot_s1_manipulation
{

namespace
{

/// 建张口表时在 [open, closed] 上取多少个采样点。
/// 61 个点对 0.93 rad 的行程约合 0.0155 rad/点，线性插值残差远小于
/// converge_tolerance_rad，够用；再密只是白算。
constexpr int kJawTableSamples = 61;

/// 判张口表单调性时允许的反向抖动（m）。
/// 表是用 FCL 的轴对齐包络算的，网格三角面片的包络会有亚毫米级毛刺，
/// 不留容差会把正常的几何噪声判成「表不单调」。
constexpr double kMonotonicSlackM = 1.0e-4;

/// 从张开到闭合，张口至少要变化这么多（m），否则判定配置有误。
/// 5mm 是很宽松的下界：任何真能夹东西的夹爪行程都远大于此。
constexpr double kMinJawStrokeM = 5.0e-3;

/// 沿给定方向，量一个 link 的碰撞包络在该方向上的投影区间。
///
/// 用包络的 8 个角点投影取 min/max，而不是只投影原点 —— 指垫有厚度，
/// 只看原点会系统性高估张口宽度。
/// 返回 false 表示这个 link 根本没有碰撞几何（那它不能当指垫）。
bool projectLinkOnAxis(
  const moveit::core::RobotState & state,
  const moveit::core::LinkModel * link,
  const Eigen::Isometry3d & tcp_inverse,
  const Eigen::Vector3d & axis,
  double & lo, double & hi)
{
  if (link == nullptr || link->getShapes().empty()) {
    return false;
  }
  const Eigen::Vector3d extents = link->getShapeExtentsAtOrigin();
  const Eigen::Vector3d offset = link->getCenteredBoundingBoxOffset();
  const Eigen::Isometry3d link_in_tcp = tcp_inverse * state.getGlobalLinkTransform(link);

  lo = std::numeric_limits<double>::infinity();
  hi = -std::numeric_limits<double>::infinity();
  for (int sx = -1; sx <= 1; sx += 2) {
    for (int sy = -1; sy <= 1; sy += 2) {
      for (int sz = -1; sz <= 1; sz += 2) {
        const Eigen::Vector3d corner = offset + Eigen::Vector3d(
          0.5 * sx * extents.x(), 0.5 * sy * extents.y(), 0.5 * sz * extents.z());
        const double p = axis.dot(link_in_tcp * corner);
        lo = std::min(lo, p);
        hi = std::max(hi, p);
      }
    }
  }
  return true;
}

}  // namespace

GripperCommander::GripperCommander(rclcpp::Node::SharedPtr node)
: node_(std::move(node))
{
  open_angle_ = std::numeric_limits<double>::quiet_NaN();
  closed_angle_ = std::numeric_limits<double>::quiet_NaN();
}

PlanErrorCode GripperCommander::configure(
  const moveit::core::RobotModelConstPtr & model,
  const GripperConfig & config,
  std::string & detail)
{
  detail.clear();
  configured_ = false;

  if (node_ == nullptr) {
    detail = "节点指针为空";
    return PlanErrorCode::kInvalidInput;
  }
  if (model == nullptr) {
    detail = "RobotModel 为空";
    return PlanErrorCode::kRobotModelUnavailable;
  }
  if (config.group_name.empty() || config.action_name.empty()) {
    detail = "group_name / action_name 不能为空";
    return PlanErrorCode::kInvalidInput;
  }
  if (config.jaw_axis_in_tcp.size() != 3U) {
    detail = "jaw_axis_in_tcp 必须是 3 个数";
    return PlanErrorCode::kInvalidInput;
  }

  model_ = model;
  config_ = config;

  const moveit::core::JointModelGroup * jmg = model_->getJointModelGroup(config_.group_name);
  if (jmg == nullptr) {
    detail = "SRDF 里找不到夹爪组 '" + config_.group_name + "'";
    return PlanErrorCode::kPlanningGroupNotFound;
  }

  // 夹爪只允许 1 个主动自由度。多于 1 个说明 SRDF 把 mimic 从动关节也写进组里了，
  // 那会让 JTC 收到多关节目标而控制器只认 1 个，报的错离根因很远，所以在这里就拦住。
  const std::vector<const moveit::core::JointModel *> & active = jmg->getActiveJointModels();
  if (active.size() != 1U) {
    detail = "夹爪组 '" + config_.group_name + "' 有 " + std::to_string(active.size()) +
      " 个主动关节，期望 1 个（mimic 从动关节不应写进组里）";
    return PlanErrorCode::kInvalidInput;
  }
  joint_name_ = active.front()->getName();

  // ---- 开合角从 SRDF 的 group_state 读，不写死 ----
  std::map<std::string, double> open_values;
  std::map<std::string, double> closed_values;
  if (!jmg->getVariableDefaultPositions(config_.open_state_name, open_values)) {
    detail = "SRDF 里夹爪组 '" + config_.group_name + "' 没有 group_state '" +
      config_.open_state_name + "'";
    return PlanErrorCode::kInvalidInput;
  }
  if (!jmg->getVariableDefaultPositions(config_.closed_state_name, closed_values)) {
    detail = "SRDF 里夹爪组 '" + config_.group_name + "' 没有 group_state '" +
      config_.closed_state_name + "'";
    return PlanErrorCode::kInvalidInput;
  }
  const auto open_it = open_values.find(joint_name_);
  const auto closed_it = closed_values.find(joint_name_);
  if (open_it == open_values.end() || closed_it == closed_values.end()) {
    detail = "group_state 里没有主动关节 '" + joint_name_ + "' 的值";
    return PlanErrorCode::kInvalidInput;
  }
  open_angle_ = open_it->second;
  closed_angle_ = closed_it->second;
  if (std::abs(open_angle_ - closed_angle_) < 1.0e-6) {
    detail = "SRDF 里 '" + config_.open_state_name + "' 与 '" + config_.closed_state_name +
      "' 的值相同(" + std::to_string(open_angle_) + ")，无法区分开合";
    return PlanErrorCode::kInvalidInput;
  }

  // ---- 量张口用的三个 link 必须都在模型里 ----
  for (const auto & pair : {
      std::pair<const char *, const std::string &>{"tcp_link", config_.tcp_link},
      std::pair<const char *, const std::string &>{"left_pad_link", config_.left_pad_link},
      std::pair<const char *, const std::string &>{"right_pad_link", config_.right_pad_link}})
  {
    if (pair.second.empty()) {
      detail = std::string(pair.first) + " 不能为空";
      return PlanErrorCode::kInvalidInput;
    }
    if (!model_->hasLinkModel(pair.second)) {
      detail = std::string(pair.first) + " '" + pair.second + "' 不在模型里";
      return PlanErrorCode::kInvalidInput;
    }
  }

  const PlanErrorCode table_code = buildJawTable(detail);
  if (table_code != PlanErrorCode::kSuccess) {
    return table_code;
  }

  action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    node_, config_.action_name);
  joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    config_.joint_states_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::ConstSharedPtr msg) {onJointStates(msg);});

  configured_ = true;
  return PlanErrorCode::kSuccess;
}

void GripperCommander::onJointStates(const sensor_msgs::msg::JointState::ConstSharedPtr & msg)
{
  const std::size_t count = std::min(msg->name.size(), msg->position.size());
  for (std::size_t i = 0; i < count; ++i) {
    if (msg->name[i] == joint_name_) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      measured_angle_ = msg->position[i];
      have_measured_ = true;
      return;
    }
  }
}

bool GripperCommander::measuredAngle(double & angle_rad) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!have_measured_) {
    return false;
  }
  angle_rad = measured_angle_;
  return true;
}

PlanErrorCode GripperCommander::buildJawTable(std::string & detail)
{
  table_angle_.clear();
  table_width_.clear();

  const moveit::core::JointModelGroup * jmg = model_->getJointModelGroup(config_.group_name);
  if (jmg == nullptr) {
    detail = "建张口表时拿不到夹爪组";
    return PlanErrorCode::kPlanningGroupNotFound;
  }

  Eigen::Vector3d axis(
    config_.jaw_axis_in_tcp[0], config_.jaw_axis_in_tcp[1], config_.jaw_axis_in_tcp[2]);
  if (axis.norm() < 1.0e-9) {
    detail = "jaw_axis_in_tcp 是零向量";
    return PlanErrorCode::kInvalidInput;
  }
  axis.normalize();

  const moveit::core::LinkModel * tcp = model_->getLinkModel(config_.tcp_link);
  const moveit::core::LinkModel * left_pad = model_->getLinkModel(config_.left_pad_link);
  const moveit::core::LinkModel * right_pad = model_->getLinkModel(config_.right_pad_link);
  const moveit::core::JointModel * master = model_->getJointModel(joint_name_);
  if (master == nullptr) {
    detail = "拿不到主动关节 '" + joint_name_ + "' 的 JointModel";
    return PlanErrorCode::kRobotModelUnavailable;
  }

  moveit::core::RobotState state(model_);
  state.setToDefaultValues();

  const double lo_angle = std::min(open_angle_, closed_angle_);
  const double hi_angle = std::max(open_angle_, closed_angle_);

  for (int i = 0; i < kJawTableSamples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(kJawTableSamples - 1);
    const double angle = lo_angle + t * (hi_angle - lo_angle);

    // !!! 必须用 setJointPositions(master, ...)，不能用 setJointGroupPositions !!!
    //
    // 实测踩坑（这个错误被本包的单测抓住了，否则完全不会报错）：
    //   setJointGroupPositions() 内部走 updateMimicJoints(group)，它只遍历
    //   **组内**的 mimic 关节（group->getMimicJointModels()）。而 SRDF 里夹爪组
    //   刻意只声明主动关节 joint_L1、不含 5 个 mimic 从动关节 —— 于是从动关节
    //   一个都不会被更新，指垫停在初始位置。
    //   后果是张口的变化率只有真实值的一半（合成模型上实测 0.035 而非 0.070），
    //   反解出的抓取角随之错一倍，而全程没有任何报错。
    //
    //   setJointPositions(master, ...) 走的是 updateMimicJoint(joint)，它遍历
    //   master->getMimicRequests() —— 所有跟随这个主动关节的从动关节，
    //   不管它们在不在组里。这才是四连杆夹爪要的语义。
    state.setJointPositions(master, &angle);
    state.update();

    const Eigen::Isometry3d tcp_inverse = state.getGlobalLinkTransform(tcp).inverse();
    double l_lo = 0.0, l_hi = 0.0, r_lo = 0.0, r_hi = 0.0;
    if (!projectLinkOnAxis(state, left_pad, tcp_inverse, axis, l_lo, l_hi)) {
      detail = "left_pad_link '" + config_.left_pad_link + "' 没有碰撞几何，量不出张口";
      return PlanErrorCode::kInvalidInput;
    }
    if (!projectLinkOnAxis(state, right_pad, tcp_inverse, axis, r_lo, r_hi)) {
      detail = "right_pad_link '" + config_.right_pad_link + "' 没有碰撞几何，量不出张口";
      return PlanErrorCode::kInvalidInput;
    }

    // 两指垫相向面之间的间隙。谁在轴的正侧由包络中点决定，不假定 left 一定在 +x。
    const double l_mid = 0.5 * (l_lo + l_hi);
    const double r_mid = 0.5 * (r_lo + r_hi);
    const double gap = (l_mid < r_mid) ? (r_lo - l_hi) : (l_lo - r_hi);

    table_angle_.push_back(angle);
    table_width_.push_back(gap);
  }

  // ---- 单调性校验 ----
  // 反解用的是线性插值，前提是张口随角度单调。四连杆在行程内本该单调，
  // 但如果 URDF 的 mimic 比例被改错（比如某一侧符号写反），表就会先减后增，
  // 那时插值会静默给出一个错误角度 —— 宁可在这里明确失败。
  const bool open_is_lo = (open_angle_ <= closed_angle_);
  double worst_reverse = 0.0;
  for (std::size_t i = 1; i < table_width_.size(); ++i) {
    // 角度从 lo 到 hi 递增；张开在 lo 侧时张口应递减，反之应递增。
    const double delta = table_width_[i] - table_width_[i - 1];
    const double reverse = open_is_lo ? delta : -delta;
    worst_reverse = std::max(worst_reverse, reverse);
  }
  if (worst_reverse > kMonotonicSlackM) {
    detail = "张口随角度不单调（最大反向 " + std::to_string(worst_reverse) +
      " m > 容差 " + std::to_string(kMonotonicSlackM) +
      " m），无法反解抓取角。两种常见成因：mimic 的 multiplier 符号写反；"
      "或闭合位设得越过了两指对穿点（合过头之后张口会重新变大）";
    return PlanErrorCode::kInvalidInput;
  }

  // ---- 行程校验 ----
  // 如果 jaw_axis_in_tcp 指错方向、或者 pad link 选错（选成了不相向的两个 link），
  // 张口会几乎不随角度变化 —— 这**通不过**下面的判断，但**能**通过上面的单调性判断
  // （处处 delta≈0 也算单调）。那时错误会推迟到 graspAngleForWidth 才暴露，
  // 报的却是"物体比闭合间隙还窄"，指向完全错的方向。所以在这里就拦住。
  const double stroke = std::abs(table_width_.front() - table_width_.back());
  if (stroke < kMinJawStrokeM) {
    detail = "从张开到闭合，张口只变化了 " + std::to_string(stroke) +
      " m（< " + std::to_string(kMinJawStrokeM) +
      " m），几乎没动。八成是 jaw_axis_in_tcp 方向不对，或 left/right_pad_link "
      "选了两个并不相向的 link";
    return PlanErrorCode::kInvalidInput;
  }

  return PlanErrorCode::kSuccess;
}

double GripperCommander::jawWidthAtAngle(double angle_rad) const
{
  if (table_angle_.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double lo = table_angle_.front();
  const double hi = table_angle_.back();
  const double clamped = std::min(std::max(angle_rad, lo), hi);

  // 表按角度递增建好，可以直接二分。
  const auto upper = std::upper_bound(table_angle_.begin(), table_angle_.end(), clamped);
  if (upper == table_angle_.begin()) {
    return table_width_.front();
  }
  if (upper == table_angle_.end()) {
    return table_width_.back();
  }
  const std::size_t idx = static_cast<std::size_t>(upper - table_angle_.begin());
  const double a0 = table_angle_[idx - 1];
  const double a1 = table_angle_[idx];
  const double w0 = table_width_[idx - 1];
  const double w1 = table_width_[idx];
  const double span = a1 - a0;
  if (std::abs(span) < 1.0e-12) {
    return w0;
  }
  return w0 + (w1 - w0) * (clamped - a0) / span;
}

PlanErrorCode GripperCommander::graspAngleForWidth(
  double width_m, double & angle_rad, std::string & detail) const
{
  detail.clear();
  angle_rad = std::numeric_limits<double>::quiet_NaN();
  if (table_angle_.empty()) {
    detail = "张口表未建立（configure 没跑或失败）";
    return PlanErrorCode::kNotConfigured;
  }
  if (!(width_m > 0.0)) {
    detail = "物体宽度必须为正，收到 " + std::to_string(width_m);
    return PlanErrorCode::kInvalidInput;
  }

  const double target = width_m - config_.grasp_preload_m;
  const double w_open = jawWidthAtAngle(open_angle_);
  const double w_closed = jawWidthAtAngle(closed_angle_);

  if (target > w_open) {
    detail = "物体宽 " + std::to_string(width_m) + " m（含预紧后目标张口 " +
      std::to_string(target) + " m）超过最大张口 " + std::to_string(w_open) +
      " m，夹不下";
    return PlanErrorCode::kGraspWidthUnreachable;
  }
  if (target < w_closed) {
    // 完全闭合后间隙仍大于物体宽度 —— 两指合到底也碰不到物体。
    // 这种情况碰撞检测查不出来（没有接触就没有碰撞），只能在这里拦。
    detail = "物体宽 " + std::to_string(width_m) + " m（含预紧后目标张口 " +
      std::to_string(target) + " m）小于完全闭合时的间隙 " + std::to_string(w_closed) +
      " m，合到底也夹不住";
    return PlanErrorCode::kGraspWidthUnreachable;
  }

  // 在表里找目标张口所在的区间，线性插值出角度。
  // 表的张口方向可能递增也可能递减（取决于 open/closed 哪个角更小），两种都处理。
  for (std::size_t i = 1; i < table_width_.size(); ++i) {
    const double w0 = table_width_[i - 1];
    const double w1 = table_width_[i];
    const bool inside = (target <= std::max(w0, w1) + 1.0e-12) &&
      (target >= std::min(w0, w1) - 1.0e-12);
    if (!inside) {
      continue;
    }
    const double span = w1 - w0;
    const double t = (std::abs(span) < 1.0e-12) ? 0.0 : (target - w0) / span;
    angle_rad = table_angle_[i - 1] + t * (table_angle_[i] - table_angle_[i - 1]);
    return PlanErrorCode::kSuccess;
  }

  // 上面两个量程判断已经把 target 夹在 [w_closed, w_open] 内，
  // 走到这里说明表本身有断裂（不该发生）。宁可明确报错，不返回一个凑出来的角度。
  detail = "目标张口 " + std::to_string(target) + " m 落在张口表的区间之外（表有断裂）";
  return PlanErrorCode::kInvalidInput;
}

PlanErrorCode GripperCommander::sendTrajectory(double angle_rad, std::string & detail)
{
  detail.clear();
  const auto wait_budget = std::chrono::duration<double>(config_.server_wait_sec);
  if (!action_client_->wait_for_action_server(
      std::chrono::duration_cast<std::chrono::nanoseconds>(wait_budget)))
  {
    detail = "夹爪动作 " + config_.action_name + " 未就绪（控制器起了吗？）";
    return PlanErrorCode::kGripperActionUnavailable;
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names.push_back(joint_name_);
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.push_back(angle_rad);
  point.velocities.push_back(0.0);
  const auto move_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(config_.move_time_sec));
  point.time_from_start.sec = static_cast<std::int32_t>(move_ns.count() / 1000000000LL);
  point.time_from_start.nanosec = static_cast<std::uint32_t>(move_ns.count() % 1000000000LL);
  goal.trajectory.points.push_back(point);
  // header.stamp 留 0 = "立刻开始"。不要打墙钟戳：仿真下 tf 与控制器都用 sim time，
  // 墙钟戳会落在未来，轨迹被判成过期。
  goal.trajectory.header.stamp.sec = 0;
  goal.trajectory.header.stamp.nanosec = 0U;

  // 阻塞等待一律用 future.wait_for()：本节点已被外部执行器 spin，
  // 再调 spin_until_future_complete 会变成两个执行器抢同一个节点。
  auto goal_future = action_client_->async_send_goal(goal);
  const auto result_budget = std::chrono::duration<double>(config_.result_timeout_sec);
  const auto result_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(result_budget);
  if (goal_future.wait_for(result_ns) != std::future_status::ready) {
    detail = "夹爪目标提交超时（" + std::to_string(config_.result_timeout_sec) + "s）";
    return PlanErrorCode::kGripperTimeout;
  }
  auto handle = goal_future.get();
  if (handle == nullptr) {
    detail = "夹爪目标被控制器拒绝（关节名不匹配或控制器未 active）";
    return PlanErrorCode::kGripperGoalRejected;
  }

  auto wrapped_future = action_client_->async_get_result(handle);
  if (wrapped_future.wait_for(result_ns) != std::future_status::ready) {
    detail = "等夹爪结果超时（" + std::to_string(config_.result_timeout_sec) + "s）";
    return PlanErrorCode::kGripperTimeout;
  }
  const auto wrapped = wrapped_future.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    detail = "夹爪轨迹未成功：ResultCode=" +
      std::to_string(static_cast<int>(wrapped.code)) +
      " error_code=" + std::to_string(wrapped.result ? wrapped.result->error_code : 0);
    return PlanErrorCode::kGripperGoalRejected;
  }
  return PlanErrorCode::kSuccess;
}

GripperOutcome GripperCommander::moveTo(double angle_rad)
{
  GripperOutcome outcome;
  outcome.measured_rad = std::numeric_limits<double>::quiet_NaN();
  if (!configured_) {
    outcome.code = PlanErrorCode::kNotConfigured;
    outcome.detail = "GripperCommander 未 configure";
    return outcome;
  }

  const double lo = std::min(open_angle_, closed_angle_);
  const double hi = std::max(open_angle_, closed_angle_);
  const double target = std::min(std::max(angle_rad, lo), hi);
  if (std::abs(target - angle_rad) > 1.0e-9) {
    outcome.detail = "目标角 " + std::to_string(angle_rad) + " 超出 [" +
      std::to_string(lo) + ", " + std::to_string(hi) + "]，已夹到区间内；";
  }
  outcome.target_rad = target;
  outcome.target_width_m = jawWidthAtAngle(target);

  const auto started = std::chrono::steady_clock::now();
  // 第三方 action / MoveIt 都可能抛异常，本层边界上一律转错误码，绝不上抛。
  try {
    std::string send_detail;
    const PlanErrorCode send_code = sendTrajectory(target, send_detail);
    if (send_code != PlanErrorCode::kSuccess) {
      outcome.code = send_code;
      outcome.detail += send_detail;
      outcome.elapsed_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
      return outcome;
    }

    // ---- 控制器报完成之后必须再驻留 ----
    // JTC 的 SUCCEEDED 只代表它自己的容差满足。实测过「报完成时还在收敛」，
    // 不驻留就读，读到的是运动中的值，会把「正常收敛」误判成 NOT_CONVERGED。
    if (config_.settle_time_sec > 0.0) {
      std::this_thread::sleep_for(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(config_.settle_time_sec)));
    }

    double measured = 0.0;
    if (!measuredAngle(measured)) {
      outcome.code = PlanErrorCode::kGripperNotConverged;
      outcome.detail += "收不到 " + config_.joint_states_topic +
        " 上的 " + joint_name_ + "，无法确认是否到位";
      outcome.elapsed_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
      return outcome;
    }
    outcome.measured_rad = measured;
    const double error = std::abs(measured - target);
    if (error > config_.converge_tolerance_rad) {
      outcome.code = PlanErrorCode::kGripperNotConverged;
      outcome.detail += "控制器报完成，但实测 " + std::to_string(measured) +
        " 与目标 " + std::to_string(target) + " 偏差 " + std::to_string(error) +
        " rad，超过容差 " + std::to_string(config_.converge_tolerance_rad) + " rad";
    } else {
      outcome.code = PlanErrorCode::kSuccess;
    }
  } catch (const std::exception & ex) {
    outcome.code = PlanErrorCode::kExceptionCaught;
    outcome.detail += std::string("夹爪动作抛异常: ") + ex.what();
  }
  outcome.elapsed_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started).count();
  return outcome;
}

GripperOutcome GripperCommander::open()
{
  if (!configured_) {
    GripperOutcome outcome;
    outcome.code = PlanErrorCode::kNotConfigured;
    outcome.detail = "GripperCommander 未 configure";
    return outcome;
  }
  return moveTo(open_angle_);
}

GripperOutcome GripperCommander::close()
{
  if (!configured_) {
    GripperOutcome outcome;
    outcome.code = PlanErrorCode::kNotConfigured;
    outcome.detail = "GripperCommander 未 configure";
    return outcome;
  }
  return moveTo(closed_angle_);
}

GripperOutcome GripperCommander::closeToWidth(double width_m)
{
  GripperOutcome outcome;
  outcome.measured_rad = std::numeric_limits<double>::quiet_NaN();
  if (!configured_) {
    outcome.code = PlanErrorCode::kNotConfigured;
    outcome.detail = "GripperCommander 未 configure";
    return outcome;
  }

  double angle = 0.0;
  std::string why;
  const PlanErrorCode code = graspAngleForWidth(width_m, angle, why);
  if (code != PlanErrorCode::kSuccess) {
    outcome.code = code;
    outcome.detail = why;
    return outcome;
  }
  return moveTo(angle);
}

}  // namespace astribot_s1_manipulation

