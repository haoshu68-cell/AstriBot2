#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace astribot::navigation
{
// Experimental opt-in receiver. No Python callbacks, commands, or ACK writers.
// Nonzero query stamps are exact capture times; zero explicitly requests latest.
class NativeTfBuffer
{
public:
  struct Diagnostics
  {
    std::string node_name;
    std::string executor_error;
    std::uint64_t domain_id{0};
    std::uint64_t clock_epoch{0};
    std::int64_t clock_ns{0};
    bool use_sim_time{false};
    bool executor_running{false};
    bool closed{false};
  };

  explicit NativeTfBuffer(bool use_sim_time, const std::string & node_name = "");
  ~NativeTfBuffer();
  NativeTfBuffer(const NativeTfBuffer &) = delete;
  NativeTfBuffer & operator=(const NativeTfBuffer &) = delete;

  std::pair<bool, std::string> can_transform(
    const std::string & target, const std::string & source, std::int64_t stamp_ns) const;
  geometry_msgs::msg::TransformStamped lookup_transform(
    const std::string & target, const std::string & source, std::int64_t stamp_ns) const;
  // tf2 clears dynamic histories while StaticCache::clearList preserves static TF.
  void clear();
  void close();
  Diagnostics diagnostics() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace astribot::navigation
