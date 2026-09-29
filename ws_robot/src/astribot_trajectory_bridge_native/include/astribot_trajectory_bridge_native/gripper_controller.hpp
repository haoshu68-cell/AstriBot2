#pragma once
#include "astribot_trajectory_bridge_native/runtime_ports.hpp"
#include <memory>
#include <optional>
#include <utility>
namespace astribot_trajectory_bridge_native {
struct GripperConfig {
  Names names;
  bool enable_service = false;
  double default_duration = 1.0;
  double default_max_force = 0.0;
  double settle_extra = 0.1;
  double stream_freq = 50.0;
  double stream_tolerance = 1.0;
  double stream_timeout = 2.0;
};
struct GripperResult {
  bool ok;
  std::string code, detail;
  double dispatched_cmd = 0, dispatched_rad = 0, actual_cmd = 0;
  bool force_applied = false;
};
template <class T> struct Resolved {
  std::optional<T> value;
  std::string error;
};
using GripperEvent = std::pair<std::string, std::string>;
class GripperController {
public:
  GripperController(const GripperConfig &, GripperSessionPort &, ClockPort &,
                    bool in_simulation);
  ~GripperController();
  GripperController(const GripperController &) = delete;
  GripperController &operator=(const GripperController &) = delete;
  Resolved<Names> resolve_names(const std::string &) const;
  Resolved<double> resolve_cmd(double opening_fraction, bool use_raw_cmd,
                               double raw_cmd) const;
  GripperResult execute(const std::string &name, double opening_fraction,
                        double duration, bool use_raw_cmd, double raw_cmd,
                        double max_force, bool write_allowed);
  std::vector<GripperEvent> drain_events();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace astribot_trajectory_bridge_native
