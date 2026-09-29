#include "astribot_trajectory_bridge_native/chassis_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "astribot_trajectory_bridge_native/chassis_math.hpp"

namespace astribot_trajectory_bridge_native {
namespace {

constexpr const char *kDisabled = "DISABLED";
constexpr const char *kEnabled = "ENABLED";
constexpr const char *kLeashTripped = "LEASH_TRIPPED";
constexpr const char *kStoppedNoPose = "STOPPED_NO_POSE";
constexpr const char *kStoppedStaleScan = "STOPPED_STALE_SCAN";

std::string fixed(double value, int precision) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}

Vec3 vec3(const std::vector<double> &values) {
  if (values.size() != 3)
    throw std::invalid_argument("期望 3 个底盘自由度");
  return {values[0], values[1], values[2]};
}

std::vector<double> vector3(const Vec3 &value) {
  return {value[0], value[1], value[2]};
}

bool finite3(const std::vector<double> &values) {
  return values.size() == 3 &&
         std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

using Event = ChassisEvent;

struct VelWindow {
  int ticks = 0;
  std::optional<double> t0;
  std::optional<double> t1;
  double in_peak = 0.0;
  double local_peak = 0.0;
  double in_wz_peak = 0.0;
  double local_wz_peak = 0.0;
  int zeroed = 0;
  double cmd_path = 0.0;
  double corr_path = 0.0;
  double dtheta_integrated = 0.0;
  int pose_rebases = 0;
  double lead_xy_peak = 0.0;
  double lead_theta_peak = 0.0;
  std::optional<Vec2> cmd_xy0;
  std::optional<Vec2> cmd_xy1;
  std::optional<double> cmd_th0;
  std::optional<double> cmd_th1;
  std::optional<Vec2> act_xy0;
  std::optional<Vec2> act_xy1;
  std::optional<double> act_th0;
  std::optional<double> act_th1;
};

} // namespace
// Preserve the safety checks performed by the Python configuration before
// construction. Timing/geometry normalization is explicit for direct C++ hosts.
static void validate_chassis_config(const ChassisConfig &c) {
  if (!std::isfinite(c.freq) || c.freq <= 0 || !std::isfinite(c.outer_rate) ||
      c.outer_rate <= 0 || c.outer_rate > c.freq)
    throw std::invalid_argument("Require finite 0 < outer_rate <= freq");
  if (c.cmd_vel_timeout_sec <= 0 || c.leash_xy_m <= 0 || c.leash_theta_rad <= 0)
    throw std::invalid_argument(
        "Command watchdog and SDK leash must remain positive");
  if (c.max_tick_dt_sec < 1.0 / c.freq)
    throw std::invalid_argument("max_tick_dt_sec must cover nominal period");
  if (!std::isfinite(c.slam_max_age_sec) || c.slam_max_age_sec <= 0)
    throw std::invalid_argument("slam_max_age_sec must be finite and positive");
  if (c.require_fresh_scan &&
      (c.scan_max_age_sec <= 0 || c.scan_loss_grace_sec < c.scan_max_age_sec))
    throw std::invalid_argument("Invalid fresh-scan interlock configuration");
  for (double preview : {c.pose_preview_xy_sec, c.pose_preview_theta_sec})
    if (!std::isfinite(preview) || preview < 0 || preview > 2)
      throw std::invalid_argument("Pose preview must be in [0, 2] seconds");
  if (!std::isfinite(c.pose_preview_max_xy_m) || c.pose_preview_max_xy_m <= 0 ||
      !std::isfinite(c.pose_preview_max_theta_rad) ||
      c.pose_preview_max_theta_rad <= 0 ||
      c.pose_preview_max_theta_rad >= std::acos(-1.0))
    throw std::invalid_argument("Invalid pose-preview displacement bound");
  if (c.pose_source != "slam" && c.pose_source != "ground_truth")
    throw std::invalid_argument("pose_source must be slam or ground_truth");
  (void)to_local_velocity({0, 0, 0}, c.input_frame, 0);
}
struct ChassisBridgeCore::Impl {
  Impl(const ChassisConfig &config, JointSessionPort &session, PosePort &pose,
       ClockPort &clock)
      : session_(session), pose_port_(pose), clock_(clock) {
    validate_chassis_config(config);
    part_name_ = config.part_name;
    freq_ = config.freq;
    input_frame_ = config.input_frame;
    theta_reference_ = config.theta_reference;
    cmd_vel_timeout_ = config.cmd_vel_timeout_sec;
    leash_xy_ = config.leash_xy_m;
    leash_theta_ = config.leash_theta_rad;
    require_slam_ = config.require_slam_to_enable;
    slam_max_age_ = config.slam_max_age_sec;
    slam_loss_grace_ = config.slam_loss_grace_sec;
    slam_jump_threshold_ = config.pose_source == "ground_truth"
                               ? std::numeric_limits<double>::infinity()
                               : config.slam_jump_threshold_m;
    odom_window_ = config.odom_drift_window_sec;
    odom_warn_ = config.pose_source == "ground_truth"
                     ? std::numeric_limits<double>::infinity()
                     : config.odom_drift_warn_m;
    max_tick_dt_ = config.max_tick_dt_sec;
    require_fresh_scan_ = config.require_fresh_scan;
    scan_max_age_ = config.scan_max_age_sec;
    scan_loss_grace_ = config.scan_loss_grace_sec;
    preview_xy_sec_ = config.pose_preview_xy_sec;
    preview_theta_sec_ = config.pose_preview_theta_sec;
    preview_max_xy_ = config.pose_preview_max_xy_m;
    preview_max_theta_ = config.pose_preview_max_theta_rad;
    sdk_history_ = SdkPoseHistory(slam_max_age_ + 0.2);
    reset_vel_window();
  }

  const std::string &state() const { return state_; }
  std::optional<Vec3> pos_cmd() const { return pos_cmd_; }
  std::optional<ChassisStop> last_stop() const { return last_stop_; }
  std::optional<IntegratorState> integrator_state() const {
    if (!integrator_)
      return std::nullopt;
    return IntegratorState{integrator_->stamp(), integrator_->pose(),
                           integrator_->anchor(), integrator_->integral(),
                           integrator_->velocity()};
  }
  std::vector<ChassisEvent> drain_events() {
    auto out = std::move(events_);
    events_.clear();
    return out;
  }

  void submit_twist(double vx, double vy, double wz) {
    last_twist_ = {vx, vy, wz};
    last_twist_time_ = now();
    repeat_event_at_.erase("CMD_VEL_TIMEOUT");
  }

  void submit_scan_seen(std::optional<double> stamp) {
    last_scan_time_ = stamp ? *stamp : now();
  }

  std::pair<bool, std::string> enable() {
    std::vector<double> seed;
    try {
      const auto rows = session_.get_current_joints_position({part_name_});
      if (rows.empty())
        throw std::runtime_error("SDK 返回空位置");
      seed = rows.front();
      if (!finite3(seed))
        throw std::runtime_error("SDK 实际位置必须为有限数值");
    } catch (const std::exception &error) {
      emit("SDK_CALL_FAILED",
           "使能时取积分种子失败：" + std::string(error.what()));
      return {false,
              "get_current_joints_position 失败：" + std::string(error.what())};
    }
    if (seed.size() != 3) {
      const std::string detail =
          "底盘自由度=" + std::to_string(seed.size()) +
          "，期望 3。ROBOT_TYPE 未设为 S1 时是 2（astribot_base.py:36-38）";
      emit("SDK_CALL_FAILED", detail);
      return {false, "底盘自由度不是 3"};
    }

    accepted_pose_.reset();
    const auto checked = lookup_pose_checked();
    if (!checked.has_value() && require_slam_) {
      emit("SLAM_UNAVAILABLE_OPEN_LOOP",
           "require_slam_to_enable=true 且位姿源不可用，拒绝使能");
      return {false, "位姿源不可用且 require_slam_to_enable=true"};
    }
    integrator_.reset();
    if (checked.has_value())
      integrator_.emplace(checked->first, checked->second, vec3(seed));
    sdk_history_ = SdkPoseHistory(slam_max_age_ + 0.2);
    sdk_history_.append(now(), vec3(seed));
    pos_cmd_ = vec3(seed);
    theta_ref_ = theta_reference_ == "at_enable" ? (*pos_cmd_)[kTheta] : 0.0;
    last_twist_ = {0.0, 0.0, 0.0};
    last_twist_time_.reset();
    applied_velocity_ = {0.0, 0.0, 0.0};
    repeat_event_at_.clear();
    prev_tick_time_.reset();
    reset_tick_window();
    reset_vel_window();
    p_slam_prev_.reset();
    pose_lost_since_.reset();
    scan_stale_since_.reset();
    drift_window_.clear();
    state_ = kEnabled;
    emit("OK", "enabled");
    return {true, "enabled"};
  }

  std::pair<bool, std::string> disable() {
    state_ = kDisabled;
    emit("NOT_ENABLED", "已停用");
    note_stop(kDisabled, "外部调用 ~/disable 主动停用");
    return {true, "disabled"};
  }

  std::pair<bool, std::string> reset_leash() {
    if (state_ != kLeashTripped)
      return {false, "当前状态 " + state_ + " 不是 LEASH_TRIPPED，无需复位"};
    return enable();
  }

  bool inner_tick() {
    if (state_ == kDisabled || state_ == kLeashTripped ||
        state_ == kStoppedNoPose || state_ == kStoppedStaleScan)
      return false;

    const double now_tick = now();
    const auto tick =
        measure_tick_dt(now_tick, prev_tick_time_, 1.0 / freq_, max_tick_dt_);
    if (!tick_first_time_.has_value())
      tick_first_time_ = now_tick;
    prev_tick_time_ = now_tick;
    const double dt = tick.dt;
    ++tick_count_;
    tick_dt_sum_ += dt;
    if (tick.has_raw && tick.raw > 0.0) {
      ++tick_window_ticks_;
      tick_max_raw_ = std::max(tick_max_raw_, tick.raw);
      if (tick.raw > tick_window_max_raw_) {
        tick_window_max_raw_ = tick.raw;
        tick_window_max_at_ = now_tick;
      }
      if (tick.raw > 2.0 / freq_)
        ++tick_window_over_count_;
    }
    if (tick.clamped) {
      ++tick_clamp_count_;
      emit("TICK_DT_CLAMPED", tick.reason, tick.has_raw ? tick.raw : 0.0,
           max_tick_dt_);
    }

    Vec3 raw_in = last_twist_;
    Vec3 vel_in = last_twist_;
    if (!last_twist_time_.has_value()) {
      vel_in = {0.0, 0.0, 0.0};
    } else {
      const double idle = now() - *last_twist_time_;
      if (idle > cmd_vel_timeout_) {
        vel_in = {0.0, 0.0, 0.0};
        emit_repeated("CMD_VEL_TIMEOUT",
                      "/cmd_vel 已 " + fixed(idle, 3) + "s 无输入，速度置零",
                      idle, cmd_vel_timeout_);
      }
    }

    if (require_fresh_scan_) {
      const auto scan = scan_age();
      if (scan.first > scan_max_age_) {
        vel_in = {0.0, 0.0, 0.0};
        if (!scan_stale_since_.has_value())
          scan_stale_since_ = now();
        const double stale_for = now() - *scan_stale_since_;
        if (scan.second) {
          emit("SCAN_NEVER_RECEIVED",
               "从未收到 /scan（已等 " + fixed(stale_for, 2) +
                   "s），速度置零。先查话题名与 QoS —— "
                   "BEST_EFFORT 发布配 RELIABLE 订阅会一帧都收不到且只有一条 "
                   "WARNING",
               stale_for, scan_max_age_);
        } else {
          emit("SCAN_STALE",
               "/scan 龄期 " + fixed(scan.first, 3) + "s 超过 " +
                   fixed(scan_max_age_, 3) + "s，速度置零",
               scan.first, scan_max_age_);
        }
        if (stale_for > scan_loss_grace_) {
          state_ = kStoppedStaleScan;
          const std::string reason = "/scan 已持续陈旧 " + fixed(stale_for, 2) +
                                     "s 超过宽限 " +
                                     fixed(scan_loss_grace_, 2) + "s";
          emit("SCAN_LOST_STOPPED", reason, stale_for, scan_loss_grace_);
          note_stop(kStoppedStaleScan, reason, stale_for, scan_loss_grace_);
          return false;
        }
      } else {
        scan_stale_since_.reset();
      }
    }

    const auto actual_opt = read_actual("读实际位置失败：");
    if (!actual_opt.has_value())
      return false;
    const Vec3 actual = *actual_opt;
    const auto checked = lookup_pose_checked();
    sdk_history_.append(now(), actual);
    if (!checked.has_value()) {
      vel_in = {0.0, 0.0, 0.0};
    } else if (!integrator_.has_value()) {
      integrator_.emplace(checked->first, checked->second, actual);
    } else if (integrator_->observe(checked->first, checked->second)) {
      ++vel_window_.pose_rebases;
      const Vec3 reference = sdk_history_.at(checked->second);
      const std::array<double, 3> horizons = {preview_xy_sec_, preview_xy_sec_,
                                              preview_theta_sec_};
      for (int i = 0; i < 3; ++i)
        if (horizons[i] > 0.0)
          integrator_->reanchor_axis(i, reference[i]);
    }
    if (tick.has_raw && tick.raw <= 0.0)
      vel_in = {0.0, 0.0, 0.0};

    double heading = (*pos_cmd_)[kTheta];
    if (integrator_.has_value()) {
      const auto anchor = integrator_->anchor();
      const auto integral = integrator_->integral();
      heading = anchor[kTheta] + integral[kTheta];
    }
    const Vec3 v_local =
        to_local_velocity(vel_in, input_frame_, heading - theta_ref_);
    const Vec3 pos_before = *pos_cmd_;
    Vec3 candidate = *pos_cmd_;
    if (integrator_.has_value()) {
      const Vec3 preview_times{preview_xy_sec_, preview_xy_sec_,
                               preview_theta_sec_};
      const Vec3 current_applied = applied_velocity_;
      for (int i = 0; i < 3; ++i)
        if (preview_times[i] > 0.0 && v_local[i] != 0.0 &&
            current_applied[i] == 0.0)
          integrator_->reanchor_axis(i, actual[i]);
      const Vec3 predicted = integrator_->preview_target(
          v_local, dt, preview_times,
          std::min(preview_max_xy_, 0.95 * leash_xy_),
          std::min(preview_max_theta_, 0.95 * leash_theta_));
      const Vec3 legacy = integrator_->target(v_local, dt);
      const std::array<double, 3> horizons = {preview_xy_sec_, preview_xy_sec_,
                                              preview_theta_sec_};
      for (int i = 0; i < 3; ++i) {
        if (v_local[i] != 0.0)
          candidate[i] = horizons[i] > 0.0 ? predicted[i] : legacy[i];
        else if (horizons[i] > 0.0 && current_applied[i] != 0.0)
          candidate[i] = actual[i];
      }
    }

    const auto leash = leash_check(candidate, actual);
    if (leash.first) {
      state_ = kLeashTripped;
      pos_cmd_ = actual;
      emit("LEASH_TRIPPED", leash.second, leash.third, leash.fourth);
      note_stop(kLeashTripped, leash.second, leash.third, leash.fourth);
      return false;
    }
    try {
      set_position(candidate);
    } catch (const std::exception &error) {
      emit("SDK_CALL_FAILED", "下发位置失败：" + std::string(error.what()));
      return false;
    }
    pos_cmd_ = candidate;
    if (integrator_.has_value()) {
      const Vec3 before = integrator_->integral();
      integrator_->commit_preview(v_local, dt);
      const std::array<double, 3> horizons = {preview_xy_sec_, preview_xy_sec_,
                                              preview_theta_sec_};
      for (int i = 0; i < 3; ++i)
        if (horizons[i] == 0.0)
          integrator_->set_integral(i, before[i] + v_local[i] * dt);
    }
    applied_velocity_ = v_local;
    record_velocity(raw_in, vel_in, v_local, dt, actual, pos_before);
    return true;
  }

  void outer_tick() {
    if (state_ != kEnabled)
      return;
    const auto checked = lookup_pose_checked();
    if (!checked.has_value()) {
      if (!pose_lost_since_.has_value())
        pose_lost_since_ = now();
      const double lost = now() - *pose_lost_since_;
      if (require_slam_ && lost > slam_loss_grace_) {
        state_ = kStoppedNoPose;
        const std::string reason = "位姿源丢失 " + fixed(lost, 2) +
                                   "s 超过宽限 " + fixed(slam_loss_grace_, 2) +
                                   "s，已停车";
        emit("SLAM_LOST_STOPPED", reason, lost, slam_loss_grace_);
        note_stop(kStoppedNoPose, reason, lost, slam_loss_grace_);
      }
      return;
    }
    pose_lost_since_.reset();
    if (p_slam_prev_.has_value()) {
      const double jump = pose_jump_distance(checked->first, *p_slam_prev_);
      p_slam_prev_ = checked->first;
      if (jump > slam_jump_threshold_) {
        drift_window_.clear();
        emit("SLAM_RELOCALIZED",
             "位姿跳变 " + fixed(jump, 4) +
                 "m，重置诊断窗口；请检查定位坐标是否重置",
             jump, slam_jump_threshold_);
        return;
      }
    } else {
      p_slam_prev_ = checked->first;
    }
    update_drift(checked->first);
  }

  TickStats tick_stats() const {
    const bool live = state_ == kEnabled;
    if (tick_count_ == 0 || !tick_first_time_.has_value())
      return {0, 0.0, 0.0, tick_clamp_count_, 0.0, live, 0.0};
    const double elapsed = *prev_tick_time_ - *tick_first_time_;
    const double rate = elapsed > 0.0 ? tick_count_ / elapsed : 0.0;
    return {tick_count_,
            tick_dt_sum_ / tick_count_,
            rate,
            tick_clamp_count_,
            static_cast<double>(tick_clamp_count_) / tick_count_,
            live,
            tick_max_raw_};
  }

  TickWindowGap consume_tick_window_gap() {
    const TickWindowGap out{tick_window_max_raw_, tick_window_max_at_,
                            tick_window_over_count_, tick_window_ticks_};
    tick_window_max_raw_ = 0.0;
    tick_window_max_at_.reset();
    tick_window_over_count_ = 0;
    tick_window_ticks_ = 0;
    return out;
  }

  void reset_tick_stats() { reset_tick_window(); }

  VelTrace consume_vel_trace() {
    const double wall = vel_window_.t0.has_value() && vel_window_.t1.has_value()
                            ? std::max(0.0, *vel_window_.t1 - *vel_window_.t0)
                            : 0.0;
    const auto net = [](const std::optional<Vec2> &a,
                        const std::optional<Vec2> &b) {
      return a.has_value() && b.has_value()
                 ? std::hypot((*b)[0] - (*a)[0], (*b)[1] - (*a)[1])
                 : 0.0;
    };
    const auto dtheta = [](const std::optional<double> &a,
                           const std::optional<double> &b) {
      return a.has_value() && b.has_value() ? wrap_angle(*b - *a) : 0.0;
    };
    const double frame_dx =
        integrator_.has_value() ? integrator_->integral()[0] : 0.0;
    const double frame_dy =
        integrator_.has_value() ? integrator_->integral()[1] : 0.0;
    const double frame_dt =
        integrator_.has_value() ? integrator_->integral()[2] : 0.0;
    const VelTrace out = {vel_window_.ticks,
                          wall,
                          vel_window_.in_peak,
                          vel_window_.local_peak,
                          vel_window_.in_wz_peak,
                          vel_window_.local_wz_peak,
                          vel_window_.zeroed,
                          vel_window_.cmd_path,
                          net(vel_window_.cmd_xy0, vel_window_.cmd_xy1),
                          net(vel_window_.act_xy0, vel_window_.act_xy1),
                          dtheta(vel_window_.cmd_th0, vel_window_.cmd_th1),
                          dtheta(vel_window_.act_th0, vel_window_.act_th1),
                          vel_window_.corr_path,
                          vel_window_.dtheta_integrated,
                          vel_window_.pose_rebases,
                          frame_dx,
                          frame_dy,
                          frame_dt,
                          vel_window_.lead_xy_peak,
                          vel_window_.lead_theta_peak};
    reset_vel_window();
    return out;
  }

private:
  using Stop = ChassisStop;

  double now() const { return clock_.now(); }

  void emit(const std::string &code, const std::string &detail,
            double metric_1 = 0.0, double metric_2 = 0.0) {
    events_.push_back({code, detail, metric_1, metric_2});
  }

  void emit_repeated(const std::string &code, const std::string &detail,
                     double metric_1, double metric_2) {
    const double current = now();
    auto it = repeat_event_at_.find(code);
    if (it != repeat_event_at_.end() && current >= it->second &&
        current - it->second < 1.0)
      return;
    repeat_event_at_[code] = current;
    emit(code, detail, metric_1, metric_2);
  }

  void note_stop(const std::string &state, const std::string &reason,
                 double metric_1 = 0.0, double metric_2 = 0.0) {
    last_stop_ = Stop{state, reason, metric_1, metric_2, now()};
  }

  std::optional<std::pair<Vec3, double>> lookup_pose_checked() {
    try {
      const auto result = pose_port_.lookup();
      if (!result)
        return std::nullopt;
      const auto &pose = result->pose;
      const double stamp = result->stamp;
      if (!finite3(pose) || !std::isfinite(stamp))
        throw std::runtime_error("pose 必须为 3 个有限数值且时间戳必须有限");
      const double current = now();
      if (stamp > current) {
        emit("SLAM_STALE", "位姿时间戳超前于控制时钟");
        return std::nullopt;
      }
      Vec3 accepted_pose = vec3(pose);
      double accepted_stamp = stamp;
      if (accepted_pose_.has_value() && stamp <= accepted_pose_->second) {
        accepted_pose = accepted_pose_->first;
        accepted_stamp = accepted_pose_->second;
      }
      const double age = current - accepted_stamp;
      if (age < 0.0 || age > slam_max_age_) {
        emit("SLAM_STALE",
             "位姿龄期 " + fixed(age, 3) + "s 超过阈值 " +
                 fixed(slam_max_age_, 3) + "s",
             age, slam_max_age_);
        return std::nullopt;
      }
      accepted_pose_ = std::make_pair(accepted_pose, accepted_stamp);
      return std::make_pair(accepted_pose, accepted_stamp);
    } catch (const PortError &error) {
      emit("POSE_PORT_FAILED",
           "位姿源实现抛异常（契约要求返回 (None, None) 而非抛）：" +
               std::string(error.what()));
      return std::nullopt;
    } catch (const std::exception &error) {
      emit("POSE_PORT_FAILED", std::string(error.what()));
      return std::nullopt;
    }
  }

  std::optional<Vec3> read_actual(const std::string &prefix) {
    try {
      const auto rows = session_.get_current_joints_position({part_name_});
      if (rows.empty())
        throw std::runtime_error("SDK 实际位置返回为空");
      if (!finite3(rows.front()) || rows.front().size() != 3)
        throw std::runtime_error("SDK 实际位置必须为 3 个有限数值");
      return vec3(rows.front());
    } catch (const std::exception &error) {
      emit("SDK_CALL_FAILED", prefix + std::string(error.what()));
      return std::nullopt;
    }
  }

  void set_position(const Vec3 &position) {
    session_.set_joints_position({part_name_}, {vector3(position)}, "filter",
                                 false, true);
  }

  std::pair<double, bool> scan_age() const {
    if (!last_scan_time_.has_value())
      return {std::numeric_limits<double>::infinity(), true};
    return {now() - *last_scan_time_, false};
  }

  struct LeashResult {
    bool first;
    std::string second;
    double third;
    double fourth;
  };

  LeashResult leash_check(const Vec3 &candidate, const Vec3 &actual) const {
    const auto error = leash_error(candidate, actual);
    if (error[0] > leash_xy_)
      return {true,
              "xy 偏差 " + fixed(error[0], 4) + "m > 阈值 " +
                  fixed(leash_xy_, 4) + "m",
              error[0], error[1]};
    if (error[1] > leash_theta_)
      return {true,
              "theta 偏差 " + fixed(error[1], 4) + "rad > 阈值 " +
                  fixed(leash_theta_, 4) + "rad",
              error[0], error[1]};
    return {false, "", error[0], error[1]};
  }

  void update_drift(const Vec3 &pose) {
    const auto sdk = read_actual("漂移诊断读实际位置失败：");
    if (!sdk.has_value())
      return;
    const double current = now();
    drift_window_.push_back(
        {current, {(*sdk)[0], (*sdk)[1]}, {pose[0], pose[1]}});
    const double cutoff = current - odom_window_;
    while (drift_window_.size() > 1 && drift_window_.front().first < cutoff)
      drift_window_.erase(drift_window_.begin());
    if (drift_window_.size() < 2)
      return;
    const auto &first = drift_window_.front();
    const auto &last = drift_window_.back();
    const double drift = odom_drift(
        {last.second[0] - first.second[0], last.second[1] - first.second[1]},
        {last.third[0] - first.third[0], last.third[1] - first.third[1]});
    if (drift > odom_warn_) {
      const double moved = std::hypot(last.third[0] - first.third[0],
                                      last.third[1] - first.third[1]);
      emit("ODOM_DRIFT_HIGH",
           fixed(current - first.first, 2) + "s 窗口内里程计与位姿源位移差 " +
               fixed(drift, 4) + "m 超过阈值 " + fixed(odom_warn_, 4) + "m",
           drift, moved);
    }
  }

  void reset_tick_window() {
    prev_tick_time_.reset();
    tick_first_time_.reset();
    tick_count_ = 0;
    tick_dt_sum_ = 0.0;
    tick_clamp_count_ = 0;
    tick_max_raw_ = 0.0;
    tick_window_max_raw_ = 0.0;
    tick_window_max_at_.reset();
    tick_window_over_count_ = 0;
    tick_window_ticks_ = 0;
  }

  void reset_vel_window() { vel_window_ = VelWindow{}; }

  void record_velocity(const Vec3 &raw_in, const Vec3 &vel_in,
                       const Vec3 &v_local, double dt, const Vec3 &actual,
                       const Vec3 &pos_before) {
    auto &w = vel_window_;
    const double current = now();
    ++w.ticks;
    if (!w.t0.has_value())
      w.t0 = current;
    w.t1 = current;
    const double raw_n = std::hypot(raw_in[0], raw_in[1]);
    const double in_n = std::hypot(vel_in[0], vel_in[1]);
    const double local_n = std::hypot(v_local[0], v_local[1]);
    w.in_peak = std::max(w.in_peak, raw_n);
    w.in_wz_peak = std::max(w.in_wz_peak, std::abs(raw_in[2]));
    w.local_peak = std::max(w.local_peak, local_n);
    w.local_wz_peak = std::max(w.local_wz_peak, std::abs(v_local[2]));
    if (std::max(raw_n, std::abs(raw_in[2])) > 1e-9 &&
        std::max(in_n, std::abs(vel_in[2])) <= 1e-9)
      ++w.zeroed;
    w.cmd_path += local_n * dt;
    w.dtheta_integrated += v_local[kTheta] * dt;
    w.lead_xy_peak =
        std::max(w.lead_xy_peak, std::hypot((*pos_cmd_)[0] - actual[0],
                                            (*pos_cmd_)[1] - actual[1]));
    w.lead_theta_peak = std::max(
        w.lead_theta_peak, std::abs(wrap_angle((*pos_cmd_)[2] - actual[2])));
    if (!w.cmd_xy0.has_value()) {
      w.cmd_xy0 = Vec2{pos_before[0], pos_before[1]};
      w.cmd_th0 = pos_before[2];
      w.act_xy0 = Vec2{actual[0], actual[1]};
      w.act_th0 = actual[2];
    }
    w.cmd_xy1 = Vec2{(*pos_cmd_)[0], (*pos_cmd_)[1]};
    w.cmd_th1 = (*pos_cmd_)[2];
    w.act_xy1 = Vec2{actual[0], actual[1]};
    w.act_th1 = actual[2];
  }

  struct DriftSample {
    double first;
    Vec2 second;
    Vec2 third;
  };

  std::string part_name_;
  double freq_ = 250.0;
  std::string input_frame_;
  std::string theta_reference_;
  double cmd_vel_timeout_ = 0.3;
  double leash_xy_ = 0.25;
  double leash_theta_ = 0.35;
  bool require_slam_ = false;
  double slam_max_age_ = 0.5;
  double slam_loss_grace_ = 2.0;
  double slam_jump_threshold_ = 0.3;
  double odom_window_ = 2.0;
  double odom_warn_ = 0.15;
  double max_tick_dt_ = 0.04;
  bool require_fresh_scan_ = true;
  double scan_max_age_ = 0.5;
  double scan_loss_grace_ = 2.0;
  double preview_xy_sec_ = 0.5;
  double preview_theta_sec_ = 0.5;
  double preview_max_xy_ = 0.2;
  double preview_max_theta_ = 0.34;

  JointSessionPort &session_;
  PosePort &pose_port_;
  ClockPort &clock_;
  std::string state_ = kDisabled;
  std::optional<Vec3> pos_cmd_;
  std::optional<Stop> last_stop_;
  double theta_ref_ = 0.0;
  std::optional<PoseFrameIntegrator> integrator_;
  SdkPoseHistory sdk_history_{0.7};
  std::optional<std::pair<Vec3, double>> accepted_pose_;
  Vec3 applied_velocity_{0.0, 0.0, 0.0};
  Vec3 last_twist_{0.0, 0.0, 0.0};
  std::optional<double> last_twist_time_;
  std::optional<double> last_scan_time_;
  std::optional<double> scan_stale_since_;
  std::optional<double> prev_tick_time_;
  std::optional<double> tick_first_time_;
  int tick_count_ = 0;
  double tick_dt_sum_ = 0.0;
  int tick_clamp_count_ = 0;
  double tick_max_raw_ = 0.0;
  double tick_window_max_raw_ = 0.0;
  std::optional<double> tick_window_max_at_;
  int tick_window_over_count_ = 0;
  int tick_window_ticks_ = 0;
  std::optional<Vec3> p_slam_prev_;
  std::optional<double> pose_lost_since_;
  std::vector<DriftSample> drift_window_;
  std::vector<Event> events_;
  std::map<std::string, double> repeat_event_at_;
  VelWindow vel_window_;
};

ChassisBridgeCore::ChassisBridgeCore(const ChassisConfig &c,
                                     JointSessionPort &s, PosePort &p,
                                     ClockPort &t)
    : impl_(std::make_unique<Impl>(c, s, p, t)) {}
ChassisBridgeCore::~ChassisBridgeCore() = default;
void ChassisBridgeCore::submit_twist(double x, double y, double z) {
  impl_->submit_twist(x, y, z);
}
void ChassisBridgeCore::submit_scan_seen(std::optional<double> t) {
  impl_->submit_scan_seen(t);
}
const std::string &ChassisBridgeCore::state() const { return impl_->state(); }
std::optional<Vec3> ChassisBridgeCore::pos_cmd() const {
  return impl_->pos_cmd();
}
std::optional<ChassisStop> ChassisBridgeCore::last_stop() const {
  return impl_->last_stop();
}
std::optional<IntegratorState> ChassisBridgeCore::integrator_state() const {
  return impl_->integrator_state();
}
std::pair<bool, std::string> ChassisBridgeCore::enable() {
  return impl_->enable();
}
std::pair<bool, std::string> ChassisBridgeCore::disable() {
  return impl_->disable();
}
std::pair<bool, std::string> ChassisBridgeCore::reset_leash() {
  return impl_->reset_leash();
}
bool ChassisBridgeCore::inner_tick() { return impl_->inner_tick(); }
void ChassisBridgeCore::outer_tick() { return impl_->outer_tick(); }
std::vector<ChassisEvent> ChassisBridgeCore::drain_events() {
  return impl_->drain_events();
}
TickStats ChassisBridgeCore::tick_stats() const { return impl_->tick_stats(); }
TickWindowGap ChassisBridgeCore::consume_tick_window_gap() {
  return impl_->consume_tick_window_gap();
}
void ChassisBridgeCore::reset_tick_stats() { return impl_->reset_tick_stats(); }
VelTrace ChassisBridgeCore::consume_vel_trace() {
  return impl_->consume_vel_trace();
}
} // namespace astribot_trajectory_bridge_native
