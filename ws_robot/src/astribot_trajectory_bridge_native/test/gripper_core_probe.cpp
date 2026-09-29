#include "astribot_trajectory_bridge_native/gripper_controller.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>

using namespace astribot_trajectory_bridge_native;

// Device calls are the observable output of the controller, not real SDK calls.
struct ProbeClock : ClockPort {
  double t = 0;
  double now() const override { return t; }
  void sleep(double dt) override { t += dt; }
};
struct ProbeSession : GripperSessionPort {
  ProbeClock &clock;
  std::string scenario;
  std::map<std::string, double> current{{"left_gripper", 0},
                                        {"right_gripper", 0}};
  ProbeSession(ProbeClock &c, std::string s)
      : clock(c), scenario(std::move(s)) {}
  JointPositions get_current_joints_position(const Names &names) override {
    if (scenario == "read_failure")
      throw std::runtime_error("read failure");
    JointPositions out;
    for (auto &name : names)
      out.push_back({current[name]});
    return out;
  }
  void set_joints_position(const Names &names, const JointPositions &p,
                           const std::string &way, bool wbc,
                           bool torso) override {
    if (scenario == "write_failure")
      throw std::runtime_error("write failure");
    for (std::size_t i = 0; i < names.size(); ++i) {
      std::cout << "WRITE " << clock.t << ' ' << names[i] << ' ' << p[i][0]
                << ' ' << way << ' ' << wbc << ' ' << torso << '\n';
      if (scenario != "stalled")
        current[names[i]] = p[i][0];
    }
  }
  void open_effector(const Names &names, double duration) override {
    if (scenario == "open_failure")
      throw std::runtime_error("open failure");
    for (auto &name : names) {
      std::cout << "OPEN " << clock.t << ' ' << name << ' ' << duration << '\n';
      current[name] = 0;
    }
  }
  void close_effector(const Names &names, double duration) override {
    for (auto &name : names) {
      std::cout << "CLOSE " << clock.t << ' ' << name << ' ' << duration
                << '\n';
      current[name] = 100;
    }
  }
  void set_effector_max_force(const Names &names,
                              const std::vector<double> &force) override {
    if (scenario == "force_failure")
      throw std::runtime_error("force failure");
    for (std::size_t i = 0; i < names.size(); ++i)
      std::cout << "FORCE " << clock.t << ' ' << names[i] << ' ' << force[i]
                << '\n';
  }
};

int main(int argc, char **argv) {
  const std::string scenario = argc > 1 ? argv[1] : "mid";
  std::cout << std::setprecision(17);
  ProbeClock clock;
  ProbeSession session(clock, scenario);
  GripperConfig config;
  config.names = {"left_gripper", "right_gripper"};
  config.enable_service = scenario != "disabled";
  config.default_duration = .2;
  config.settle_extra = 0;
  config.stream_freq = 20;
  config.stream_tolerance = .5;
  config.stream_timeout = .25;
  GripperController controller(config, session, clock,
                               scenario != "force_real");
  double fraction = .5;
  if (scenario == "open" || scenario == "open_failure")
    fraction = 1;
  if (scenario == "close")
    fraction = 0;
  if (scenario == "invalid")
    fraction = 1.2;
  const double force = scenario.rfind("force_", 0) == 0 ? 30 : 0;
  const auto result =
      controller.execute(scenario == "unknown" ? "unknown" : "left_gripper",
                         fraction, 0, false, 0, force, scenario != "denied");
  std::cout << "RESULT " << result.ok << ' ' << result.code << ' '
            << result.dispatched_cmd << ' ' << result.dispatched_rad << ' '
            << result.actual_cmd << ' ' << result.force_applied << ' '
            << clock.t << '\n';
  for (auto &event : controller.drain_events())
    std::cout << "EVENT " << event.first << '\n';
}
