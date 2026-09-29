#include "astribot_trajectory_bridge_native/arm_executor.hpp"
#include "astribot_trajectory_bridge_native/chassis_controller.hpp"
#include "astribot_trajectory_bridge_native/gripper_controller.hpp"
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
using namespace astribot_trajectory_bridge_native;
struct Clock : ClockPort {
  double now() const override { return 0; }
  void sleep(double) override {}
};
struct BlockingGripper : GripperSessionPort {
  std::mutex mutex;
  std::condition_variable cv;
  bool entered = false, released = false;
  JointPositions get_current_joints_position(const Names &) override {
    return {{0, 0, 0}};
  }
  void set_joints_position(const Names &, const JointPositions &,
                           const std::string &, bool, bool) override {
    assert(false);
  }
  void set_effector_max_force(const Names &,
                              const std::vector<double> &) override {
    assert(false);
  }
  void close_effector(const Names &, double) override { assert(false); }
  void open_effector(const Names &, double) override {
    std::unique_lock<std::mutex> lock(mutex);
    entered = true;
    cv.notify_all();
    cv.wait(lock, [&] { return released; });
  }
};
struct WaypointPort : ArmSessionPort {
  int calls = 0;
  bool fail = false;
  Names names;
  std::vector<JointPositions> points;
  std::vector<double> times;
  bool wbc = true, torso = true;
  JointPositions get_current_joints_position(const Names &) override {
    return {{0, 0, 0}};
  }
  void set_joints_position(const Names &, const JointPositions &,
                           const std::string &, bool, bool) override {
    assert(false);
  }
  std::pair<JointPositions, JointPositions>
  get_joints_position_limit(const Names &) override {
    return {{{-2}}, {{2}}};
  }
  void move_joints_waypoints(const Names &n,
                             const std::vector<JointPositions> &p,
                             const std::vector<double> &t, bool w,
                             bool a) override {
    ++calls;
    if (fail)
      throw PortError("injected waypoint write failure");
    names = n;
    points = p;
    times = t;
    wbc = w;
    torso = a;
  }
};
struct FailingPose : PosePort {
  std::optional<StampedPose> lookup() override {
    throw PortError("injected pose failure");
  }
};
int main() {
  Clock clock;
  BlockingGripper session;
  GripperConfig config;
  config.names = {"left"};
  config.enable_service = true;
  config.settle_extra = 0;
  GripperController gripper(config, session, clock, true);
  GripperResult first;
  std::thread worker(
      [&] { first = gripper.execute("left", 1, 0, false, 0, 0, true); });
  {
    std::unique_lock<std::mutex> lock(session.mutex);
    assert(session.cv.wait_for(lock, std::chrono::seconds(2),
                               [&] { return session.entered; }));
  }
  auto second = gripper.execute("left", 1, 0, false, 0, 0, true);
  assert(!second.ok && second.code == "BUSY");
  {
    std::lock_guard<std::mutex> lock(session.mutex);
    session.released = true;
  }
  session.cv.notify_all();
  worker.join();
  assert(first.ok);
  // Waypoint dispatch remains a separate, explicitly enabled interface.
  WaypointPort waypoints;
  ArmConfig arm;
  WaypointDispatcher dispatcher(arm, waypoints, true);
  auto result =
      dispatcher.dispatch({{0}, {.2}, {.4}}, {0, .2, .4},
                          std::vector<double>{-2}, std::vector<double>{2});
  assert(result.ok && result.sent == 2 && result.dropped == 1);
  assert(waypoints.calls == 1 && waypoints.names == Names{"astribot_arm_left"});
  assert(waypoints.points == std::vector<JointPositions>({{{.2}, {.4}}}));
  assert(waypoints.times == std::vector<double>({.2, .4}) && !waypoints.wbc &&
         !waypoints.torso);
  waypoints.fail = true;
  result = dispatcher.dispatch({{.2}}, {.2});
  assert(!result.ok && result.code == "SDK_CALL_FAILED");
  assert(dispatcher.drain_events().front().code == "SDK_CALL_FAILED");
  WaypointDispatcher disabled(arm, waypoints, false);
  result = disabled.dispatch({{.2}}, {.2});
  assert(!result.ok && result.code == "DISABLED_BY_CONFIG" &&
         waypoints.calls == 2);
  FailingPose pose;
  ChassisConfig chassis;
  chassis.require_slam_to_enable = true;
  ChassisBridgeCore core(chassis, waypoints, pose, clock);
  assert(!core.enable().first);
  auto events = core.drain_events();
  assert(events.front().code == "POSE_PORT_FAILED");
  assert(events.front().detail.find("injected pose failure") !=
         std::string::npos);
}
