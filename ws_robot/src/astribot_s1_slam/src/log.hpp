// Voxel-SLAM diagnostics use the repository ROS -> launch -> spdlog pipeline.
// Keep fmt-style call sites and module tags; never install a private/global sink.
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <spdlog/fmt/fmt.h>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <string>

namespace vxlm_log
{
enum class Module
{
  LIDAR,    // point cloud preprocessing / deskew (feature_point.hpp, motion_blur())
  IMU,      // IMU propagation / preintegration (ekf_imu.hpp, preintegration.hpp)
  EKF,      // state estimation (lio_state_estimation) — this is tightly-coupled
            // LIO, so "fusing" LiDAR+IMU IS the EKF; there is no separate
            // fusion layer today, hence no standalone FUSION tag (see below).
  LOOP,     // BTC loop-closure detection/verification (BTC.cpp, SearchLoop)
  BACKEND,  // sliding-window BA / global BA (voxel_map.hpp LM optimizer, topDownProcess)
  MAP,      // voxel map management (cut_voxel, margi, recut)
  PERF,     // timing/perf lines
  INIT,     // parameter/config/session loading — not in the guideline's base
            // list, added because voxelslam has a distinct, non-trivial
            // startup phase.
  SYS,      // process/session lifecycle: reset, shutdown, mode switches.
  CAMERA,   // synchronized image input and session image recording
  // FRONTEND and FUSION from the guideline's example list are intentionally
  // omitted: there is no VIO-style standalone frontend (that role is split
  // across LIDAR + EKF), and no fusion stage independent of the EKF itself.
  // If a camera gets added and a genuine multi-sensor fusion/weighting layer
  // is written on top of EKF, add FUSION back then — don't reintroduce it
  // speculatively.
};

inline const char *tag(Module m)
{
  switch (m)
  {
    case Module::LIDAR:    return "LIDAR";
    case Module::IMU:      return "IMU";
    case Module::EKF:      return "EKF";
    case Module::LOOP:     return "LOOP";
    case Module::BACKEND:  return "BACKEND";
    case Module::MAP:      return "MAP";
    case Module::PERF:     return "PERF";
    case Module::INIT:     return "INIT";
    case Module::SYS:      return "SYS";
    case Module::CAMERA:   return "CAMERA";
  }
  return "?";
}

// Bind the actual node logger before worker threads start. This respects node
// remapping, named ROS severity overrides and /rosout registration.
inline rclcpp::Logger &logger()
{
  static auto value = rclcpp::get_logger("voxelslam");
  return value;
}
inline void init(const rclcpp::Logger &value) { logger() = value; }

// Diagnostic trajectories are data artifacts, not a second runtime log sink.
inline std::string log_dir()
{
  const auto nonempty = [](const char *key) -> const char * {
    const char *value = std::getenv(key);
    return value && *value ? value : nullptr;
  };
  std::filesystem::path path;
  if (const char *value = nonempty("ASTRIBOT_LOG_DIR")) path = value;
  else if (const char *value = nonempty("ROS_LOG_DIR")) path = value;
  else if (const char *value = nonempty("ROS_HOME")) path = std::filesystem::path(value) / "log/astribot";
  else if (const char *value = nonempty("HOME")) path = std::filesystem::path(value) / ".ros/log/astribot";
  else throw std::runtime_error("Cannot locate SLAM diagnostic directory: HOME is unset");
  return std::filesystem::absolute(path).string();
}
inline int64_t now_monotonic_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace vxlm_log

#define VXLM_TAG(module) vxlm_log::tag(vxlm_log::Module::module)
// ROS checks severity before evaluating fmt::format. DEBUG remains available in
// Release builds and follows the same --ros-args --log-level policy as mapping.
#define LOG_DEBUG(module, pattern, ...) \
  RCLCPP_DEBUG(vxlm_log::logger(), "%s", fmt::format("[{}] " pattern, VXLM_TAG(module), ##__VA_ARGS__).c_str())
#define LOG_INFO(module, pattern, ...) \
  RCLCPP_INFO(vxlm_log::logger(), "%s", fmt::format("[{}] " pattern, VXLM_TAG(module), ##__VA_ARGS__).c_str())
#define LOG_WARN(module, pattern, ...) \
  RCLCPP_WARN(vxlm_log::logger(), "%s", fmt::format("[{}] " pattern, VXLM_TAG(module), ##__VA_ARGS__).c_str())
#define LOG_ERROR(module, pattern, ...) \
  RCLCPP_ERROR(vxlm_log::logger(), "%s", fmt::format("[{}] " pattern, VXLM_TAG(module), ##__VA_ARGS__).c_str())
#define LOG_DECISION(module, pattern, ...) \
  LOG_WARN(module, "[DECISION] " pattern, ##__VA_ARGS__)
#define LOG_STARTUP(module, pattern, ...) \
  LOG_INFO(module, pattern, ##__VA_ARGS__)

// Throttled INFO for steady-state per-frame data (guideline #6): fires on the
// 1st call and every Nth call after that from THIS call site. Uses a
// call-site-local static counter, so each LOG_INFO_THROTTLE(...) location
// tracks its own cadence independently — do not factor a throttled call into
// a shared helper function without being aware the counter would then be shared.
#define LOG_INFO_THROTTLE(module, n, fmt, ...)                      \
  do {                                                              \
    static std::atomic<uint64_t> _vxlm_throttle_ctr{0};             \
    if (_vxlm_throttle_ctr.fetch_add(1, std::memory_order_relaxed) % (n) == 0) \
      LOG_INFO(module, fmt, ##__VA_ARGS__);                          \
  } while (0)

#define LOG_WARN_THROTTLE(module, n, fmt, ...)                      \
  do {                                                              \
    static std::atomic<uint64_t> _vxlm_throttle_ctr{0};             \
    if (_vxlm_throttle_ctr.fetch_add(1, std::memory_order_relaxed) % (n) == 0) \
      LOG_WARN(module, fmt, ##__VA_ARGS__);                          \
  } while (0)
