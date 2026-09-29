#include "astribot_trajectory_bridge_native/arm_executor.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "astribot_trajectory_bridge_native/arm_math.hpp"

namespace astribot_trajectory_bridge_native {
namespace {

constexpr const char *kIdle = "IDLE";
constexpr const char *kStreaming = "STREAMING";
constexpr const char *kSettling = "SETTLING";
constexpr const char *kHolding = "HOLDING";
constexpr const char *kDone = "DONE";
constexpr const char *kAborted = "ABORTED";
constexpr const char *kCanceled = "CANCELED";

constexpr int kSuccessful = 0;
constexpr int kInvalidGoal = -1;
constexpr int kInvalidJoints = -2;
constexpr int kOldHeaderTimestamp = -3;
constexpr int kPathToleranceViolated = -4;
constexpr int kGoalToleranceViolated = -5;

std::string fixed(double value, int precision) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}

double max_abs_error_checked(const std::vector<double> &actual,
                             const std::vector<double> &target) {
  if (actual.size() != target.size())
    throw std::invalid_argument("维度不符：" + std::to_string(actual.size()) +
                                " vs " + std::to_string(target.size()));
  double worst = 0.0;
  for (std::size_t i = 0; i < actual.size(); ++i)
    worst = std::max(worst, std::abs(actual[i] - target[i]));
  return worst;
}

double oob_amount(const std::vector<double> &q,
                  const std::vector<double> &lower,
                  const std::vector<double> &upper) {
  double total = 0.0;
  for (std::size_t i = 0; i < q.size() && i < lower.size() && i < upper.size();
       ++i) {
    if (q[i] < lower[i])
      total += lower[i] - q[i];
    if (q[i] > upper[i])
      total += q[i] - upper[i];
  }
  return total;
}

} // namespace
// These checks previously lived in the Python ArmBridgeConfig constructor.
static void validate_arm_config(const ArmConfig &c) {
  if (c.stream_freq <= 0 || c.settle_tolerance_rad <= 0 ||
      c.settle_timeout_sec <= 0 || c.max_tracking_error_rad <= 0 ||
      c.hold_still_epsilon_rad <= 0 || c.hold_still_ticks_required < 1 ||
      c.hold_timeout_sec <= 0 || c.oob_start_tolerance_rad <= 0 ||
      c.oob_start_tolerance_rad > 0.5)
    throw std::invalid_argument(
        "Invalid arm timing, tolerance, or hold configuration");
  if (c.add_default_torso)
    throw std::invalid_argument(
        "add_default_torso must remain false for planned arm trajectories");
}
struct ArmTrajExecutor::Impl {
  Impl(const ArmConfig &config, ArmSessionPort &session, ClockPort &clock)
      : session_(session), clock_(clock) {
    validate_arm_config(config);
    part_name_ = config.part_name;
    joint_names_ = config.joint_names;
    stream_freq_ = config.stream_freq;
    control_way_ = config.control_way;
    use_wbc_ = config.use_wbc;
    add_default_torso_ = config.add_default_torso;
    interp_ = config.interp;
    max_traj_duration_ = config.max_traj_duration_sec;
    limit_margin_ = config.limit_margin_rad;
    cross_check_urdf_ = config.cross_check_urdf;
    limit_cross_check_tol_ = config.limit_cross_check_tol_rad;
    strict_limit_check_ = config.strict_limit_check;
    max_tracking_error_ = config.max_tracking_error_rad;
    abort_tracking_error_ = config.abort_on_tracking_error;
    settle_tolerance_ = config.settle_tolerance_rad;
    settle_timeout_ = config.settle_timeout_sec;
    hold_still_epsilon_ = config.hold_still_epsilon_rad;
    hold_still_ticks_required_ = config.hold_still_ticks_required;
    hold_timeout_ = config.hold_timeout_sec;
    allow_recovery_from_oob_ = config.allow_recovery_from_oob;
    oob_start_tolerance_ = config.oob_start_tolerance_rad;
  }

  const std::string &phase() const { return phase_; }
  int error_code() const { return error_code_; }
  const std::string &detail() const { return detail_; }
  std::vector<double> lower() const { return lower_; }
  std::vector<double> upper() const { return upper_; }

  std::vector<ArmEvent> events() const { return events_; }

  std::vector<ArmFeedback> feedbacks() const { return feedbacks_; }

  std::vector<ArmEvent> drain_events() {
    auto out = events();
    events_.clear();
    return out;
  }

  std::vector<ArmFeedback> drain_feedbacks() {
    auto out = feedbacks();
    feedbacks_.clear();
    return out;
  }

  std::pair<bool, std::string>
  load_limits(std::optional<std::vector<double>> urdf_lower,
              std::optional<std::vector<double>> urdf_upper) {
    try {
      const auto pair = session_.get_joints_position_limit({part_name_});
      const auto &lower_rows = pair.first;
      const auto &upper_rows = pair.second;
      if (lower_rows.empty() || upper_rows.empty())
        throw std::runtime_error("SDK 限位返回为空");
      const auto lower = lower_rows.front();
      const auto upper = upper_rows.front();
      if (lower.size() != upper.size()) {
        const std::string reason =
            "lower 长度 " + std::to_string(lower.size()) + " != upper 长度 " +
            std::to_string(upper.size());
        emit("LIMIT_SOURCE_MISMATCH", reason);
        return {false, reason};
      }
      for (std::size_t i = 0; i < lower.size(); ++i) {
        if (lower[i] > upper[i]) {
          const std::string reason =
              "关节 " + std::to_string(i) + " 的 lower(" + fixed(lower[i], 6) +
              ") > upper(" + fixed(upper[i], 6) +
              ")，SDK 返回顺序与 astribot_client.py:141 的约定 (lower, upper) "
              "不符。"
              "注意 examples/100:49 的解包顺序是反的，不要以它为准。";
          emit("LIMIT_SOURCE_MISMATCH", reason);
          return {false, reason};
        }
      }
      lower_ = lower;
      upper_ = upper;

      if (cross_check_urdf_ && urdf_lower.has_value()) {
        const auto ulower = urdf_lower.value();
        const auto uupper = urdf_upper.value();
        double worst = 0.0;
        std::string worst_desc;
        const std::size_t n = std::min(lower_.size(), ulower.size());
        for (std::size_t i = 0; i < n; ++i) {
          const double dl = std::abs(lower_[i] - ulower[i]);
          if (dl > worst) {
            worst = dl;
            worst_desc = "关节 " + std::to_string(i) +
                         " 的 lower 限位：SDK=" + fixed(lower_[i], 6) +
                         " URDF=" + fixed(ulower[i], 6) +
                         " 偏差=" + fixed(dl, 6);
          }
          const double du = std::abs(upper_[i] - uupper[i]);
          if (du > worst) {
            worst = du;
            worst_desc = "关节 " + std::to_string(i) +
                         " 的 upper 限位：SDK=" + fixed(upper_[i], 6) +
                         " URDF=" + fixed(uupper[i], 6) +
                         " 偏差=" + fixed(du, 6);
          }
        }
        if (worst > limit_cross_check_tol_) {
          const std::string reason = worst_desc + "（超过容差 " +
                                     fixed(limit_cross_check_tol_, 6) + ")";
          emit("LIMIT_SOURCE_MISMATCH", reason, worst, limit_cross_check_tol_);
          if (strict_limit_check_)
            return {false, reason};
        }
      }
      return {true, ""};
    } catch (const std::exception &error) {
      const std::string detail = std::string(error.what());
      emit("SDK_CALL_FAILED", "读限位失败：" + detail);
      return {false, "get_joints_position_limit 失败：" + detail};
    }
  }

  ArmStartResult start(const std::vector<std::string> &joint_names,
                       const std::vector<double> &times,
                       const JointTrajectory &positions,
                       std::optional<JointTrajectory> velocities) {
    phase_ = kIdle;
    error_code_ = kSuccessful;
    detail_.clear();
    cancel_requested_ = false;
    settle_start_.reset();
    hold_target_.reset();
    hold_start_.reset();
    hold_last_actual_.reset();
    hold_still_ticks_ = 0;

    if (lower_.empty())
      return reject(kInvalidGoal, "限位未加载，必须先成功调用 load_limits()");
    if (times.empty() || positions.empty())
      return reject(kInvalidGoal, "空轨迹");
    if (times.size() != positions.size())
      return reject(kInvalidGoal, "times 长度 " + std::to_string(times.size()) +
                                      " != positions 长度 " +
                                      std::to_string(positions.size()));
    if (!joint_names_.empty() && joint_names != joint_names_)
      return reject(kInvalidJoints, "关节名或顺序与本组不符。收到 " +
                                        py_repr(joint_names) + "，期望 " +
                                        py_repr(joint_names_));
    for (std::size_t i = 1; i < times.size(); ++i) {
      if (times[i] <= times[i - 1]) {
        return reject(kInvalidGoal,
                      "time_from_start 非单调：time_list[" + std::to_string(i) +
                          "]=" + fixed(times[i], 6) + " <= time_list[" +
                          std::to_string(i - 1) +
                          "]=" + fixed(times[i - 1], 6));
      }
    }
    if (times.back() > max_traj_duration_)
      return reject(kInvalidGoal, "轨迹总时长 " + fixed(times.back(), 3) +
                                      "s 超过上限 " +
                                      fixed(max_traj_duration_, 3) + "s");

    bool allow_start_oob = false;
    const auto actual_now = read_actual();
    for (std::size_t i = 0; i < positions.size(); ++i) {
      const auto check = within_limits(positions[i]);
      if (check.first)
        continue;
      if (i == 0 && actual_now.has_value() && allow_recovery_from_oob_) {
        const double near = max_abs_error_checked(positions[i], *actual_now);
        if (near <= oob_start_tolerance_) {
          allow_start_oob = true;
          emit("LIMIT_VIOLATION",
               "起点越限但等于当前实测位置（偏差 " + fixed(near, 4) +
                   " rad），按恢复轨迹放行：" + check.second);
          continue;
        }
        emit("LIMIT_VIOLATION", "路点 0 越限且与当前实测位置不符（偏差 " +
                                    fixed(near, 4) + " rad > " +
                                    fixed(oob_start_tolerance_, 4) +
                                    "），拒绝：" + check.second);
        return reject(kInvalidGoal,
                      "路点 0 越限且不等于当前位置（偏差 " + fixed(near, 4) +
                          " rad），不是恢复轨迹：" + check.second);
      }
      emit("LIMIT_VIOLATION",
           "路点 " + std::to_string(i) + "：" + check.second);
      return reject(kInvalidGoal,
                    "路点 " + std::to_string(i) + " 越限：" + check.second);
    }
    if (allow_start_oob && oob_amount(positions.back(), lower_, upper_) >
                               oob_amount(positions.front(), lower_, upper_)) {
      return reject(
          kInvalidGoal,
          "起点越限，且这条轨迹的终点越限更严重 —— 不是恢复轨迹，拒绝。");
    }

    times_ = times;
    positions_ = positions;
    velocities_.clear();
    if (velocities.has_value())
      velocities_ = velocities.value();
    t0_ = now();
    last_desired_ = positions_.front();
    phase_ = kStreaming;
    return {true, kSuccessful, ""};
  }

  void request_cancel() { cancel_requested_ = true; }

  const std::string &step() {
    if (phase_ == kStreaming)
      return step_streaming();
    if (phase_ == kSettling)
      return step_settling();
    if (phase_ == kHolding)
      return step_holding();
    return phase_;
  }

private:
  using Event = ArmEvent;
  using Feedback = ArmFeedback;

  static std::string py_repr(const std::vector<std::string> &values) {
    std::ostringstream out;
    out << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (i)
        out << ", ";
      out << "'" << values[i] << "'";
    }
    out << "]";
    return out.str();
  }

  static std::pair<bool, std::string> within(const std::vector<double> &q,
                                             const std::vector<double> &lower,
                                             const std::vector<double> &upper,
                                             double margin) {
    if (q.size() != lower.size() || q.size() != upper.size())
      return {false, "位置维度 " + std::to_string(q.size()) + " 与限位维度 (" +
                         std::to_string(lower.size()) + ", " +
                         std::to_string(upper.size()) + ") 不符"};
    for (std::size_t i = 0; i < q.size(); ++i) {
      const double lo = lower[i] + margin;
      const double hi = upper[i] - margin;
      if (lo > hi)
        return {false, "关节 " + std::to_string(i) + " 的 margin=" +
                           fixed(margin, 4) + " 过大，可行区间为空"};
      if (q[i] < lo || q[i] > hi)
        return {false, "关节 " + std::to_string(i) + " 目标 " + fixed(q[i], 6) +
                           " 越界 [" + fixed(lo, 6) + ", " + fixed(hi, 6) +
                           "]（含 margin " + fixed(margin, 4) + "）"};
    }
    return {true, ""};
  }

  std::pair<bool, std::string>
  within_limits(const std::vector<double> &q) const {
    return within(q, lower_, upper_, limit_margin_);
  }

  double now() const { return clock_.now(); }

  std::optional<std::vector<double>> read_actual() {
    try {
      const auto rows = session_.get_current_joints_position({part_name_});
      if (rows.empty())
        throw std::runtime_error("SDK 实际位置返回为空");
      return rows.front();
    } catch (const std::exception &error) {
      emit("SDK_CALL_FAILED",
           "读实际关节位置失败：" + std::string(error.what()));
      return std::nullopt;
    }
  }

  void set_position(const std::vector<double> &position) {
    session_.set_joints_position({part_name_}, {position}, control_way_,
                                 use_wbc_, add_default_torso_);
  }

  void emit(const std::string &code, const std::string &detail,
            double metric_1 = 0.0, double metric_2 = 0.0) {
    events_.push_back({code, detail, metric_1, metric_2});
  }

  ArmStartResult reject(int code, const std::string &detail) {
    phase_ = kAborted;
    error_code_ = code;
    detail_ = detail;
    return {false, code, detail};
  }

  const std::string &step_streaming() {
    if (cancel_requested_) {
      if (hold_current()) {
        phase_ = kHolding;
        detail_ = "收到取消请求，正在保持当前位置";
        return step_holding();
      }
      phase_ = kCanceled;
      detail_ = "收到取消请求，但读不到实际位置、无法保持";
      return phase_;
    }

    const double t = now() - t0_;
    const bool use_cubic = interp_ == "cubic" && !velocities_.empty() &&
                           !velocities_.front().empty();
    const auto q =
        interpolate_trajectory(times_, positions_, velocities_, t, use_cubic);
    last_desired_ = q;
    try {
      set_position(q);
    } catch (const std::exception &error) {
      emit("SDK_CALL_FAILED", "下发关节位置失败：" + std::string(error.what()));
      phase_ = kAborted;
      error_code_ = kPathToleranceViolated;
      detail_ = "SDK 下发失败：" + std::string(error.what());
      return phase_;
    }

    const auto actual = read_actual();
    if (!actual.has_value()) {
      phase_ = kAborted;
      error_code_ = kPathToleranceViolated;
      detail_ = "无法读取实际关节位置";
      return phase_;
    }
    const double error = max_abs_error_checked(*actual, q);
    feedbacks_.push_back({t, q, *actual, error});
    if (error > max_tracking_error_) {
      emit("TRACKING_ERROR_EXCEEDED",
           "跟踪误差 " + fixed(error, 4) + "rad 超过阈值 " +
               fixed(max_tracking_error_, 4) + "rad",
           error, max_tracking_error_);
      if (abort_tracking_error_) {
        phase_ = kAborted;
        error_code_ = kPathToleranceViolated;
        detail_ = "跟踪误差超阈：" + fixed(error, 4) + "rad";
        return phase_;
      }
    }
    if (t >= times_.back()) {
      phase_ = kSettling;
      settle_start_ = now();
    }
    return phase_;
  }

  const std::string &step_settling() {
    if (cancel_requested_) {
      if (hold_current()) {
        phase_ = kHolding;
        detail_ = "收敛期间收到取消请求，正在保持当前位置";
        return step_holding();
      }
      phase_ = kCanceled;
      detail_ = "收敛期间收到取消请求，但读不到实际位置";
      return phase_;
    }

    const auto target = positions_.back();
    const auto actual = read_actual();
    if (!actual.has_value()) {
      phase_ = kAborted;
      error_code_ = kGoalToleranceViolated;
      detail_ = "收敛判断时无法读取实际关节位置";
      return phase_;
    }
    try {
      set_position(target);
    } catch (const std::exception &error) {
      emit("SDK_CALL_FAILED", "收敛期下发失败：" + std::string(error.what()));
      phase_ = kAborted;
      error_code_ = kGoalToleranceViolated;
      detail_ = "SDK 下发失败：" + std::string(error.what());
      return phase_;
    }
    if (max_abs_error_checked(*actual, target) <= settle_tolerance_) {
      phase_ = kDone;
      error_code_ = kSuccessful;
      detail_ = "settled";
      return phase_;
    }
    const double waited = now() - *settle_start_;
    if (waited > settle_timeout_) {
      const double residual = max_abs_error_checked(*actual, target);
      emit("SETTLE_TIMEOUT",
           "等待 " + fixed(waited, 3) + "s 仍未收敛，残余误差 " +
               fixed(residual, 4) + "rad（容差 " + fixed(settle_tolerance_, 4) +
               "rad）",
           residual, settle_tolerance_);
      phase_ = kAborted;
      error_code_ = kGoalToleranceViolated;
      detail_ = "收敛超时，残余 " + fixed(residual, 4) + "rad";
    }
    return phase_;
  }

  bool hold_current() {
    const auto actual = read_actual();
    if (!actual.has_value()) {
      emit("SDK_CALL_FAILED", "取消时读不到实际位置，无法保持");
      return false;
    }
    hold_target_ = *actual;
    hold_start_ = now();
    hold_last_actual_ = *actual;
    return true;
  }

  const std::string &step_holding() {
    if (!hold_target_.has_value()) {
      phase_ = kCanceled;
      return phase_;
    }
    try {
      set_position(*hold_target_);
    } catch (const std::exception &error) {
      emit("SDK_CALL_FAILED", "保持期下发失败：" + std::string(error.what()));
      phase_ = kCanceled;
      detail_ = "取消后保持失败：" + std::string(error.what());
      return phase_;
    }
    const auto actual = read_actual();
    if (!actual.has_value()) {
      phase_ = kCanceled;
      detail_ = "取消后保持期读不到实际位置";
      return phase_;
    }
    const double moved = max_abs_error_checked(*actual, *hold_last_actual_);
    hold_last_actual_ = *actual;
    const double elapsed = now() - *hold_start_;
    if (moved < hold_still_epsilon_)
      ++hold_still_ticks_;
    else
      hold_still_ticks_ = 0;
    if (hold_still_ticks_ >= hold_still_ticks_required_) {
      phase_ = kCanceled;
      detail_ =
          "已保持并停稳（保持 " + fixed(elapsed, 3) + "s，最终偏离保持点 " +
          fixed(max_abs_error_checked(*actual, *hold_target_), 4) + " rad）";
      return phase_;
    }
    if (elapsed > hold_timeout_) {
      emit("SETTLE_TIMEOUT",
           "取消后保持 " + fixed(elapsed, 3) + "s 仍未停稳，最近一拍位移 " +
               fixed(moved, 4) + " rad",
           elapsed, hold_timeout_);
      phase_ = kCanceled;
      detail_ = "取消后保持超时（仍在运动）";
    }
    return phase_;
  }

  std::string part_name_;
  std::vector<std::string> joint_names_;
  double stream_freq_ = 250.0;
  std::string control_way_;
  bool use_wbc_ = false;
  bool add_default_torso_ = false;
  std::string interp_;
  double max_traj_duration_ = 60.0;
  double limit_margin_ = 0.0;
  bool cross_check_urdf_ = true;
  double limit_cross_check_tol_ = 0.01;
  bool strict_limit_check_ = false;
  double max_tracking_error_ = 0.1;
  bool abort_tracking_error_ = true;
  double settle_tolerance_ = 0.02;
  double settle_timeout_ = 2.0;
  double hold_still_epsilon_ = 0.001;
  int hold_still_ticks_required_ = 25;
  double hold_timeout_ = 2.0;
  bool allow_recovery_from_oob_ = true;
  double oob_start_tolerance_ = 0.05;

  ArmSessionPort &session_;
  ClockPort &clock_;
  std::string phase_ = kIdle;
  int error_code_ = kSuccessful;
  std::string detail_;
  std::vector<double> lower_;
  std::vector<double> upper_;
  std::vector<double> times_;
  JointTrajectory positions_;
  JointTrajectory velocities_;
  double t0_ = 0.0;
  std::optional<double> settle_start_;
  bool cancel_requested_ = false;
  std::vector<double> last_desired_;
  std::optional<std::vector<double>> hold_target_;
  std::optional<double> hold_start_;
  std::optional<std::vector<double>> hold_last_actual_;
  int hold_still_ticks_ = 0;
  std::vector<Event> events_;
  std::vector<Feedback> feedbacks_;
};

struct WaypointDispatcher::Impl {
  Impl(const ArmConfig &config, ArmSessionPort &session, bool enabled)
      : session_(session), enabled_(enabled) {
    validate_arm_config(config);
    part_name_ = config.part_name;
    use_wbc_ = config.use_wbc;
    add_default_torso_ = config.add_default_torso;
    limit_margin_ = config.limit_margin_rad;
  }

  WaypointResult dispatch(const JointTrajectory &waypoints,
                          const std::vector<double> &time_list,
                          std::optional<std::vector<double>> lower,
                          std::optional<std::vector<double>> upper) {
    if (!enabled_)
      return {false, "DISABLED_BY_CONFIG",
              "enable_waypoints_service=false，方案 A 未启用", 0, 0};
    if (waypoints.size() != time_list.size())
      return {false, "SHAPE_MISMATCH",
              "路点数 " + std::to_string(waypoints.size()) +
                  " 与 time_list 长度 " + std::to_string(time_list.size()) +
                  " 不符",
              0, 0};
    JointTrajectory kept;
    std::vector<double> kept_times;
    std::size_t dropped = 0;
    for (std::size_t i = 0; i < time_list.size(); ++i) {
      if (time_list[i] <= 0.0) {
        ++dropped;
        continue;
      }
      kept.push_back(waypoints[i]);
      kept_times.push_back(time_list[i]);
    }
    if (kept.empty())
      return {
          false, "NO_POINTS_AFTER_DROP",
          "丢弃 t<=0 的点后没有路点剩下（examples/206：t=0 由当前位置隐含）", 0,
          dropped};
    for (std::size_t i = 1; i < kept_times.size(); ++i) {
      if (kept_times[i] <= kept_times[i - 1])
        return {false, "TIME_NOT_MONOTONIC",
                "time_list[" + std::to_string(i) +
                    "]=" + fixed(kept_times[i], 6) + " <= time_list[" +
                    std::to_string(i - 1) + "]=" + fixed(kept_times[i - 1], 6),
                0, dropped};
    }
    if (lower.has_value()) {
      const auto lo = lower.value();
      const auto hi = upper.value();
      for (std::size_t i = 0; i < kept.size(); ++i) {
        const auto check = within(kept[i], lo, hi, limit_margin_);
        if (!check.first) {
          events_.push_back({"LIMIT_VIOLATION",
                             "路点 " + std::to_string(i) + "：" + check.second,
                             0.0, 0.0});
          return {false, "LIMIT_VIOLATION",
                  "路点 " + std::to_string(i) + " 越限：" + check.second, 0,
                  dropped};
        }
      }
    }
    try {
      session_.move_joints_waypoints({part_name_}, {kept}, kept_times, false,
                                     add_default_torso_);
    } catch (const std::exception &error) {
      const std::string text = std::string(error.what());
      events_.push_back({"SDK_CALL_FAILED", text, 0.0, 0.0});
      return {false, "SDK_CALL_FAILED", "move_joints_waypoints 失败：" + text,
              0, dropped};
    }
    return {true, "SUCCESS", "", kept.size(), dropped};
  }

  std::vector<ArmEvent> events() const { return events_; }

  std::vector<ArmEvent> drain_events() {
    auto out = events();
    events_.clear();
    return out;
  }

private:
  using Event = ArmEvent;
  static std::pair<bool, std::string> within(const std::vector<double> &q,
                                             const std::vector<double> &lower,
                                             const std::vector<double> &upper,
                                             double margin) {
    if (q.size() != lower.size() || q.size() != upper.size())
      return {false, "位置维度 " + std::to_string(q.size()) + " 与限位维度 (" +
                         std::to_string(lower.size()) + ", " +
                         std::to_string(upper.size()) + ") 不符"};
    for (std::size_t i = 0; i < q.size(); ++i) {
      const double lo = lower[i] + margin;
      const double hi = upper[i] - margin;
      if (lo > hi)
        return {false, "关节 " + std::to_string(i) + " 的 margin=" +
                           fixed(margin, 4) + " 过大，可行区间为空"};
      if (q[i] < lo || q[i] > hi)
        return {false, "关节 " + std::to_string(i) + " 目标 " + fixed(q[i], 6) +
                           " 越界 [" + fixed(lo, 6) + ", " + fixed(hi, 6) +
                           "]（含 margin " + fixed(margin, 4) + "）"};
    }
    return {true, ""};
  }
  std::string part_name_;
  bool use_wbc_ = false;
  bool add_default_torso_ = false;
  double limit_margin_ = 0.0;
  ArmSessionPort &session_;
  bool enabled_ = false;
  std::vector<Event> events_;
};

ArmTrajExecutor::ArmTrajExecutor(const ArmConfig &c, ArmSessionPort &s,
                                 ClockPort &clock)
    : impl_(std::make_unique<Impl>(c, s, clock)) {}
ArmTrajExecutor::~ArmTrajExecutor() = default;
std::pair<bool, std::string>
ArmTrajExecutor::load_limits(std::optional<std::vector<double>> lo,
                             std::optional<std::vector<double>> hi) {
  return impl_->load_limits(lo, hi);
}
ArmStartResult ArmTrajExecutor::start(const Names &n,
                                      const std::vector<double> &t,
                                      const JointTrajectory &q,
                                      std::optional<JointTrajectory> v) {
  return impl_->start(n, t, q, v);
}
void ArmTrajExecutor::request_cancel() { impl_->request_cancel(); }
const std::string &ArmTrajExecutor::step() { return impl_->step(); }
const std::string &ArmTrajExecutor::phase() const { return impl_->phase(); }
int ArmTrajExecutor::error_code() const { return impl_->error_code(); }
const std::string &ArmTrajExecutor::detail() const { return impl_->detail(); }
std::vector<double> ArmTrajExecutor::lower() const { return impl_->lower(); }
std::vector<double> ArmTrajExecutor::upper() const { return impl_->upper(); }
std::vector<ArmEvent> ArmTrajExecutor::events() const {
  return impl_->events();
}
std::vector<ArmFeedback> ArmTrajExecutor::feedbacks() const {
  return impl_->feedbacks();
}
std::vector<ArmEvent> ArmTrajExecutor::drain_events() {
  return impl_->drain_events();
}
std::vector<ArmFeedback> ArmTrajExecutor::drain_feedbacks() {
  return impl_->drain_feedbacks();
}
WaypointDispatcher::WaypointDispatcher(const ArmConfig &c, ArmSessionPort &s,
                                       bool e)
    : impl_(std::make_unique<Impl>(c, s, e)) {}
WaypointDispatcher::~WaypointDispatcher() = default;
WaypointResult
WaypointDispatcher::dispatch(const JointTrajectory &q,
                             const std::vector<double> &t,
                             std::optional<std::vector<double>> lo,
                             std::optional<std::vector<double>> hi) {
  return impl_->dispatch(q, t, lo, hi);
}
std::vector<ArmEvent> WaypointDispatcher::events() const {
  return impl_->events();
}
std::vector<ArmEvent> WaypointDispatcher::drain_events() {
  return impl_->drain_events();
}
} // namespace astribot_trajectory_bridge_native
