#include "astribot_trajectory_bridge_native/arm_executor.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace astribot_trajectory_bridge_native;
struct Clock : ClockPort {
  double t = 0;
  double now() const override { return t; }
  void sleep(double dt) override { t += dt; }
};
struct Session : ArmSessionPort {
  Clock &clock;
  std::string scenario;
  std::vector<double> actual{0, 0, 0};
  int reads = 0;
  Session(Clock &c, std::string s) : clock(c), scenario(std::move(s)) {}
  std::pair<JointPositions, JointPositions>
  get_joints_position_limit(const Names &) override {
    if (scenario == "limit_failure")
      throw std::runtime_error("limit read injected");
    return {{{-2, -2, -2}}, {{2, 2, 2}}};
  }
  JointPositions get_current_joints_position(const Names &) override {
    ++reads;
    if ((scenario == "read_failure" && reads >= 2) ||
        (scenario == "hold_read_failure" && reads >= 3))
      throw std::runtime_error("position read injected");
    return {actual};
  }
  void set_joints_position(const Names &names, const JointPositions &p,
                           const std::string &way, bool wbc,
                           bool torso) override {
    if (scenario == "write_failure")
      throw std::runtime_error("position write injected");
    std::cout << "WRITE " << clock.t << ' ' << names[0] << ' ' << way << ' '
              << wbc << ' ' << torso;
    for (auto v : p[0])
      std::cout << ' ' << v;
    std::cout << '\n';
    if (scenario != "tracking" && scenario != "settle_timeout")
      for (unsigned i = 0; i < 3; ++i)
        actual[i] += .65 * (p[0][i] - actual[i]);
  }
  void move_joints_waypoints(const Names &, const std::vector<JointPositions> &,
                             const std::vector<double> &, bool, bool) override {
  }
};
int main(int argc, char **argv) {
  std::cout << std::setprecision(17);
  std::string scenario = argc > 1 ? argv[1] : "success";
  Clock clock;
  Session session(clock, scenario);
  ArmConfig c;
  c.joint_names = {"j0", "j1", "j2"};
  c.stream_freq = 50;
  c.settle_timeout_sec = scenario == "settle_timeout" ? .05 : .8;
  c.hold_still_ticks_required = 4;
  c.hold_timeout_sec = .8;
  if (scenario == "tracking")
    c.max_tracking_error_rad = .05;
  if (scenario == "settle_timeout") {
    c.max_tracking_error_rad = 10;
    c.abort_on_tracking_error = false;
  }
  ArmTrajExecutor ex(c, session, clock);
  auto loaded = ex.load_limits();
  std::cout << "LOAD " << loaded.first << '\n';
  auto started = ex.start(c.joint_names, {0, .2, .5},
                          {{0, 0, 0}, {.35, -.20, .15}, {.55, -.30, .25}},
                          JointTrajectory{{0, 0, 0}, {.2, -.1, .1}, {0, 0, 0}});
  std::cout << "START " << started.ok << ' ' << started.code << '\n';
  for (int tick = 0; tick < 300 && started.ok; ++tick) {
    clock.t += .02;
    if ((scenario == "cancel" && clock.t >= .22) ||
        scenario == "hold_read_failure")
      ex.request_cancel();
    ex.step();
    std::cout << "TICK " << clock.t << ' ' << ex.phase() << ' '
              << ex.error_code() << '\n';
    if (ex.phase() == "DONE" || ex.phase() == "CANCELED" ||
        ex.phase() == "ABORTED")
      break;
  }
  for (auto &e : ex.events())
    std::cout << "EVENT " << e.code << ' ' << e.metric_1 << ' ' << e.metric_2
              << '\n';
  for (auto &f : ex.feedbacks()) {
    std::cout << "FB " << f.t << ' ' << f.error;
    for (auto v : f.desired)
      std::cout << ' ' << v;
    for (auto v : f.actual)
      std::cout << ' ' << v;
    std::cout << '\n';
  }
}
