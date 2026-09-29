#include "astribot_trajectory_bridge_native/arm_executor.hpp"
#include "astribot_trajectory_bridge_native/chassis_controller.hpp"
#include <cassert>
#include <stdexcept>
using namespace astribot_trajectory_bridge_native;
struct Clock : ClockPort {
  double now() const override { return 0; }
  void sleep(double) override {}
};
struct Pose : PosePort {
  std::optional<StampedPose> lookup() override { return std::nullopt; }
};
struct NoDevice : ArmSessionPort {
  int calls = 0;
  JointPositions get_current_joints_position(const Names &) override {
    ++calls;
    throw std::logic_error("No device allowed");
  }
  void set_joints_position(const Names &, const JointPositions &,
                           const std::string &, bool, bool) override {
    ++calls;
    throw std::logic_error("No device allowed");
  }
  std::pair<JointPositions, JointPositions>
  get_joints_position_limit(const Names &) override {
    ++calls;
    throw std::logic_error("No device allowed");
  }
  void move_joints_waypoints(const Names &, const std::vector<JointPositions> &,
                             const std::vector<double> &, bool, bool) override {
    ++calls;
    throw std::logic_error("No device allowed");
  }
};
template <class F> void rejects(F make) {
  bool rejected = false;
  try {
    make();
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  assert(rejected);
}
int main() {
  Clock clock;
  Pose pose;
  NoDevice session;
  // A direct C++ caller must not bypass the configuration safety checks that
  // used to run in Python before the binding was constructed.
  for (int which = 0; which < 4; ++which) {
    ArmConfig c;
    if (which == 0)
      c.hold_still_ticks_required = 0;
    if (which == 1)
      c.add_default_torso = true;
    if (which == 2)
      c.settle_timeout_sec = 0;
    if (which == 3)
      c.oob_start_tolerance_rad = .6;
    rejects([&] { ArmTrajExecutor core(c, session, clock); });
  }
  for (int which = 0; which < 6; ++which) {
    ChassisConfig c;
    if (which == 0)
      c.leash_xy_m = 0;
    if (which == 1)
      c.cmd_vel_timeout_sec = 0;
    if (which == 2)
      c.freq = 0;
    if (which == 3)
      c.max_tick_dt_sec = .001;
    if (which == 4)
      c.scan_max_age_sec = 0;
    if (which == 5)
      c.pose_preview_xy_sec = 3;
    rejects([&] { ChassisBridgeCore core(c, session, pose, clock); });
  }
  assert(session.calls == 0);
}
