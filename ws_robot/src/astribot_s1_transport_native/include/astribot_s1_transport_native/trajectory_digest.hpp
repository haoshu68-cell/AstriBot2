#pragma once

#include <nlohmann/json.hpp>
#include <openssl/sha.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <cmath>
#include <stdexcept>
#include <string>

namespace astribot::transport {

// Evidence fingerprint only: equality does not grant permission to execute.
// Arrays retain their order and empty fields; finite doubles use JSON's
// round-trip representation without rounding or time normalization.
inline std::string trajectory_digest(const trajectory_msgs::msg::JointTrajectory& trajectory) {
  nlohmann::ordered_json encoded = {
    {"header", {{"frame_id", trajectory.header.frame_id},
                {"sec", trajectory.header.stamp.sec},
                {"nanosec", trajectory.header.stamp.nanosec}}},
    {"joint_names", trajectory.joint_names},
    {"points", nlohmann::ordered_json::array()}
  };
  for (const auto& point : trajectory.points) {
    // JSON otherwise silently serializes NaN and infinity as null.
    for (const auto* values : {&point.positions, &point.velocities,
                               &point.accelerations, &point.effort}) {
      for (double value : *values) {
        if (!std::isfinite(value)) throw std::runtime_error("TRAJECTORY_DIGEST_NONFINITE");
      }
    }
    encoded["points"].push_back({
      {"positions", point.positions},
      {"velocities", point.velocities},
      {"accelerations", point.accelerations},
      {"effort", point.effort},
      {"time_from_start", {{"sec", point.time_from_start.sec},
                           {"nanosec", point.time_from_start.nanosec}}}
    });
  }
  // Keep the default strict encoding policy: malformed text must throw.
  const std::string bytes = encoded.dump();
  unsigned char digest[SHA256_DIGEST_LENGTH];
  if (!SHA256(reinterpret_cast<const unsigned char*>(bytes.data()), bytes.size(), digest)) {
    throw std::runtime_error("TRAJECTORY_DIGEST_SHA256_FAILED");
  }
  constexpr char hex[] = "0123456789abcdef";
  std::string result;
  result.reserve(2 * SHA256_DIGEST_LENGTH);
  for (unsigned char byte : digest) {
    result.push_back(hex[byte >> 4]);
    result.push_back(hex[byte & 0x0f]);
  }
  return result;
}

}  // namespace astribot::transport
