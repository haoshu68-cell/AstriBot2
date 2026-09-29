#pragma once
#include <stdexcept>
#include <string>
#include <vector>
namespace astribot_trajectory_bridge_native {
// Transport adapters normalize device/pose failures without exposing foreign
// runtimes.
struct PortError : std::runtime_error {
  using std::runtime_error::runtime_error;
};
using Names = std::vector<std::string>;
using JointPositions = std::vector<std::vector<double>>;
// Ports must outlive controllers. The host owns write admission, cancellation
// callback serialization and the single device session. These ports perform no
// transport or device discovery. A production adapter
// must preserve the existing SDK ownership and safety contract.
struct ClockPort {
  virtual ~ClockPort() = default;
  virtual double now() const = 0;
  virtual void sleep(double seconds) = 0;
};
struct JointSessionPort {
  virtual ~JointSessionPort() = default;
  virtual JointPositions get_current_joints_position(const Names &) = 0;
  virtual void set_joints_position(const Names &, const JointPositions &,
                                   const std::string &control_way, bool use_wbc,
                                   bool add_default_torso) = 0;
};
struct GripperSessionPort : JointSessionPort {
  virtual void open_effector(const Names &, double duration) = 0;
  virtual void close_effector(const Names &, double duration) = 0;
  virtual void set_effector_max_force(const Names &,
                                      const std::vector<double> &) = 0;
};
} // namespace astribot_trajectory_bridge_native
