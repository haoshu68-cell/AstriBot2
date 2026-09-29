#include "astribot_trajectory_bridge_native/gripper_controller.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace astribot_trajectory_bridge_native {
namespace {

constexpr double kCmdOpen = 0.0;
constexpr double kCmdClosed = 100.0;
constexpr double kRadPerCmd = 0.0093;

std::string repr_double(double value) {
  if (std::isnan(value))
    return "nan";
  if (std::isinf(value))
    return value > 0.0 ? "inf" : "-inf";
  std::ostringstream out;
  out << std::setprecision(17) << value;
  return out.str();
}

double clamp_cmd(double cmd) {
  return std::max(kCmdOpen, std::min(kCmdClosed, cmd));
}

double cmd_to_rad(double cmd) { return clamp_cmd(cmd) * kRadPerCmd; }

std::string describe_cmd(double cmd) {
  const double c = clamp_cmd(cmd);
  std::string state;
  if (c <= 1.0) {
    state = "张开";
  } else if (c >= 99.0) {
    state = "闭合";
  } else {
    std::ostringstream percent;
    percent << std::fixed << std::setprecision(0)
            << ((1.0 - c / 100.0) * 100.0);
    state = "半开(" + percent.str() + "%张开)";
  }
  std::ostringstream out;
  out << state << "（cmd=" << std::fixed << std::setprecision(1) << c << "，"
      << std::setprecision(4) << cmd_to_rad(c) << " rad）";
  return out.str();
}

} // namespace

struct GripperController::Impl {
  Impl(std::vector<std::string> names, bool enable_service,
       double default_duration, double default_max_force, double settle_extra,
       double stream_freq, double stream_tolerance, double stream_timeout,
       GripperSessionPort &session, ClockPort &clock, bool in_simulation)
      : names_(std::move(names)), enable_service_(enable_service),
        default_duration_(default_duration),
        default_max_force_(default_max_force), settle_extra_(settle_extra),
        stream_freq_(stream_freq), stream_tolerance_(stream_tolerance),
        stream_timeout_(stream_timeout), session_(session), clock_(clock),
        in_simulation_(in_simulation) {
    if (names_.empty())
      throw std::invalid_argument("gripper_names 为空");
    if (!(default_duration_ > 0.0) || !std::isfinite(default_duration_))
      throw std::invalid_argument("default_duration_sec 必须为有限正数");
    if (settle_extra_ < 0.0 || !std::isfinite(settle_extra_))
      throw std::invalid_argument("settle_extra_sec 不能为负且必须有限");
    if (!(stream_freq_ > 0.0) || !std::isfinite(stream_freq_))
      throw std::invalid_argument("stream_freq 必须为有限正数");
    if (!(stream_tolerance_ > 0.0) || !std::isfinite(stream_tolerance_))
      throw std::invalid_argument("mid_stream_tolerance 必须为有限正数");
    if (!(stream_timeout_ > 0.0) || !std::isfinite(stream_timeout_))
      throw std::invalid_argument("mid_stream_timeout_sec 必须为有限正数");
  }

  Resolved<Names> resolve_names(const std::string &name) const {
    if (name.empty())
      return {names_, {}};
    if (std::find(names_.begin(), names_.end(), name) == names_.end()) {
      std::ostringstream detail;
      detail << "未知夹爪 '" << name << "'，合法取值：[";
      for (std::size_t i = 0; i < names_.size(); ++i) {
        if (i)
          detail << ", ";
        detail << "'" << names_[i] << "'";
      }
      detail << "]（空字符串表示全部）";
      return {std::nullopt, detail.str()};
    }
    return {Names{name}, {}};
  }

  Resolved<double> resolve_cmd(double opening_fraction, bool use_raw_cmd,
                               double raw_cmd) const {
    if (use_raw_cmd) {
      if (std::isnan(raw_cmd))
        return {std::nullopt, "raw_cmd 是 NaN"};
      if (!(raw_cmd >= 0.0 && raw_cmd <= 100.0)) {
        return {std::nullopt,
                "raw_cmd=" + repr_double(raw_cmd) +
                    " 越出命令范围 [0, 100]。提醒：命令空间是 0=张开、100=闭合"
                    "（与直觉相反），若想表达\"张开程度\"请用 "
                    "opening_fraction_to_cmd()。"};
      }
      return {raw_cmd, {}};
    }
    if (std::isnan(opening_fraction))
      return {std::nullopt, "opening_fraction 是 NaN"};
    if (!(opening_fraction >= 0.0 && opening_fraction <= 1.0)) {
      return {std::nullopt,
              "opening_fraction=" + repr_double(opening_fraction) +
                  " 越出 [0, 1]。提醒：1.0 = 全张开、0.0 = 全闭合；"
                  "若想直接给厂商的 0-100 裸命令值请置 use_raw_cmd=true"
                  "（注意那个空间是 0=张开、100=闭合）。"};
    }
    return {(1.0 - opening_fraction) * 100.0, {}};
  }

  std::vector<GripperEvent> drain_events() {
    std::lock_guard<std::mutex> guard(lock_);
    auto result = std::move(events_);
    events_.clear();
    return result;
  }

  GripperResult execute(const std::string &name, double opening_fraction,
                        double duration, bool use_raw_cmd, double raw_cmd,
                        double max_force, bool write_allowed) {
    if (!enable_service_)
      return result(false, "DISABLED_BY_CONFIG",
                    "夹爪服务未启用（gripper.enable_service=false）。"
                    "显式拒绝而不静默成功 —— 静默成功会让调用方以为夹爪动了。");
    if (!write_allowed)
      return result(false, "WRITE_GATE_DENIED",
                    "写通路准入未通过，拒绝操作夹爪。");

    auto names_result = resolve_names(name);
    if (!names_result.value)
      return result(false, "UNKNOWN_GRIPPER", names_result.error);
    const auto target_names = *names_result.value;

    auto cmd_result = resolve_cmd(opening_fraction, use_raw_cmd, raw_cmd);
    if (!cmd_result.value)
      return result(false, "VALUE_OUT_OF_RANGE", cmd_result.error);
    const double cmd = *cmd_result.value;
    const double dur = duration > 0.0 ? duration : default_duration_;

    {
      std::lock_guard<std::mutex> guard(lock_);
      std::vector<std::string> clash;
      for (const auto &item : target_names)
        if (busy_.count(item))
          clash.push_back(item);
      if (!clash.empty()) {
        std::ostringstream detail;
        detail << "夹爪 [";
        for (std::size_t i = 0; i < clash.size(); ++i) {
          if (i)
            detail << ", ";
          detail << "'" << clash[i] << "'";
        }
        detail << "] 正在执行中。open/close_effector 是阻塞调用，"
                  "并发会让两次动作互相覆盖，所以拒绝而不排队。";
        return result(false, "BUSY", detail.str());
      }
      busy_.insert(target_names.begin(), target_names.end());
    }

    GripperResult answer;
    try {
      answer = do_execute(target_names, cmd, dur, max_force);
    } catch (const std::exception &error) {
      emit("SDK_CALL_FAILED", "夹爪开合失败：" + std::string(error.what()));
      answer = result(false, "SDK_CALL_FAILED",
                      "夹爪开合失败：" + std::string(error.what()), cmd,
                      cmd_to_rad(cmd));
    }

    {
      std::lock_guard<std::mutex> guard(lock_);
      for (const auto &item : target_names)
        busy_.erase(item);
    }
    return answer;
  }

private:
  GripperResult result(bool ok, const std::string &code,
                       const std::string &detail, double dispatched_cmd = 0.0,
                       double dispatched_rad = 0.0, double actual_cmd = 0.0,
                       bool force_applied = false) const {
    return {ok,         code,         detail, dispatched_cmd, dispatched_rad,
            actual_cmd, force_applied};
  }

  void emit(const std::string &code, const std::string &detail) {
    std::lock_guard<std::mutex> guard(lock_);
    events_.emplace_back(code, detail);
  }

  double now() const { return clock_.now(); }

  void sleep(double seconds) const {
    if (seconds > 0.0)
      clock_.sleep(seconds);
  }

  std::vector<double>
  read_positions(const std::vector<std::string> &names) const {
    const auto rows = session_.get_current_joints_position(names);
    std::vector<double> values;
    values.reserve(rows.size());
    for (const auto &row : rows)
      values.push_back(row.empty() ? std::numeric_limits<double>::quiet_NaN()
                                   : row.front());

    return values;
  }

  std::pair<bool, double> stream_to(const std::vector<std::string> &names,
                                    double cmd) {
    const double period = 1.0 / stream_freq_;
    const double deadline = now() + stream_timeout_;
    double last = std::numeric_limits<double>::quiet_NaN();
    const auto positions = std::vector<std::vector<double>>(
        names.size(), std::vector<double>{cmd});
    while (true) {
      session_.set_joints_position(names, positions, "direct", false, false);
      try {
        const auto values = read_positions(names);
        if (!values.empty()) {
          last = *std::max_element(
              values.begin(), values.end(), [cmd](double lhs, double rhs) {
                return std::abs(lhs - cmd) < std::abs(rhs - cmd);
              });
          bool reached = values.size() == names.size();
          for (double value : values)
            reached = reached && std::abs(value - cmd) <= stream_tolerance_;
          if (reached)
            return {true, last};
        }
      } catch (const std::exception &) {
        // Match the Python implementation: a failed read keeps streaming
        // until the bounded deadline, while a failed write escapes to _do.
      }
      if (now() >= deadline)
        return {false, last};
      sleep(period);
    }
  }

  GripperResult do_execute(const std::vector<std::string> &names, double cmd,
                           double duration, double max_force) {
    bool force_applied = false;
    const bool want_force = max_force > 0.0 || default_max_force_ > 0.0;
    const double force = max_force > 0.0 ? max_force : default_max_force_;
    if (want_force) {
      try {
        session_.set_effector_max_force(
            names, std::vector<double>(names.size(), force));
        force_applied = !in_simulation_;
      } catch (const std::exception &error) {
        emit("SDK_CALL_FAILED", "设夹持力失败：" + std::string(error.what()));
        return result(false, "SDK_CALL_FAILED",
                      "设夹持力失败（未执行开合）：" +
                          std::string(error.what()),
                      cmd, cmd_to_rad(cmd), 0.0, force_applied);
      }
    }

    try {
      if (cmd <= kCmdOpen) {
        session_.open_effector(names, duration);
      } else if (cmd >= kCmdClosed) {
        session_.close_effector(names, duration);
      } else {
        const auto stream = stream_to(names, cmd);
        if (!stream.first) {
          std::ostringstream detail;
          detail << "中间开度未到位：目标 " << std::fixed
                 << std::setprecision(2) << cmd << "，" << stream_timeout_
                 << "s 后仍在 " << stream.second;
          emit("SETTLE_TIMEOUT", detail.str());
        }
      }
    } catch (const std::exception &error) {
      emit("SDK_CALL_FAILED", "夹爪开合失败：" + std::string(error.what()));
      return result(false, "SDK_CALL_FAILED",
                    "夹爪开合失败：" + std::string(error.what()), cmd,
                    cmd_to_rad(cmd), 0.0, force_applied);
    }

    sleep(settle_extra_);
    double actual = 0.0;
    try {
      const auto values = read_positions(names);
      if (!values.empty())
        actual = values.front();
    } catch (const std::exception &error) {
      emit("SDK_CALL_FAILED",
           "读夹爪实际位置失败：" + std::string(error.what()));
      return result(true, "SUCCESS",
                    describe_cmd(cmd) + "；但读回实际位置失败：" +
                        std::string(error.what()) + "（actual_cmd 不可信）",
                    cmd, cmd_to_rad(cmd),
                    std::numeric_limits<double>::quiet_NaN(), force_applied);
    }

    std::string detail = describe_cmd(cmd);
    if (want_force && !force_applied)
      detail += "；力限未生效（仿真下 set_effector_max_force 是空操作）";
    return result(true, "SUCCESS", detail, cmd, cmd_to_rad(cmd), actual,
                  force_applied);
  }

  std::vector<std::string> names_;
  bool enable_service_;
  double default_duration_;
  double default_max_force_;
  double settle_extra_;
  double stream_freq_;
  double stream_tolerance_;
  double stream_timeout_;
  GripperSessionPort &session_;
  ClockPort &clock_;
  bool in_simulation_;
  mutable std::mutex lock_;
  std::set<std::string> busy_;
  std::vector<std::pair<std::string, std::string>> events_;
};

GripperController::GripperController(const GripperConfig &c,
                                     GripperSessionPort &session,
                                     ClockPort &clock, bool simulation)
    : impl_(std::make_unique<Impl>(
          c.names, c.enable_service, c.default_duration, c.default_max_force,
          c.settle_extra, c.stream_freq, c.stream_tolerance, c.stream_timeout,
          session, clock, simulation)) {}
GripperController::~GripperController() = default;
Resolved<Names>
GripperController::resolve_names(const std::string &name) const {
  return impl_->resolve_names(name);
}
Resolved<double> GripperController::resolve_cmd(double fraction, bool raw,
                                                double cmd) const {
  return impl_->resolve_cmd(fraction, raw, cmd);
}
GripperResult GripperController::execute(const std::string &name,
                                         double fraction, double duration,
                                         bool raw, double cmd, double force,
                                         bool allowed) {
  return impl_->execute(name, fraction, duration, raw, cmd, force, allowed);
}
std::vector<GripperEvent> GripperController::drain_events() {
  return impl_->drain_events();
}
} // namespace astribot_trajectory_bridge_native
