#include "astribot_trajectory_bridge_native/chassis_controller.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace astribot_trajectory_bridge_native;
struct Clock : ClockPort {
  double t = 0;
  double now() const override { return t; }
  void sleep(double dt) override { t += dt; }
};
struct Session : JointSessionPort {
  Clock &clock;
  std::string scenario;
  std::vector<double> actual{0, 0, 0};
  int reads = 0;
  Session(Clock &c, std::string s) : clock(c), scenario(std::move(s)) {}
  JointPositions get_current_joints_position(const Names &) override {
    if (scenario == "read_failure" && ++reads > 1)
      throw std::runtime_error("position read injected");
    return {actual};
  }
  void set_joints_position(const Names &n, const JointPositions &q,
                           const std::string &way, bool wbc,
                           bool torso) override {
    if (scenario == "write_failure")
      throw std::runtime_error("position write injected");
    std::cout << "WRITE " << clock.t << ' ' << n[0] << ' ' << way << ' ' << wbc
              << ' ' << torso;
    for (unsigned i = 0; i < 3; ++i) {
      std::cout << ' ' << q[0][i];
      actual[i] += .87 * (q[0][i] - actual[i]);
    }
    std::cout << '\n';
  }
};
struct Pose : PosePort {
  Clock &clock;
  bool absent = false;
  std::vector<double> p{0, 0, 0};
  explicit Pose(Clock &c) : clock(c) {}
  std::optional<StampedPose> lookup() override {
    if (absent)
      return std::nullopt;
    return StampedPose{p, clock.t};
  }
};
int main(int argc, char **argv) {
  std::cout << std::setprecision(17);
  std::string scenario = argc > 1 ? argv[1] : "replay";
  Clock clock;
  Session session(clock, scenario);
  Pose pose(clock);
  ChassisConfig c;
  c.require_fresh_scan = scenario == "scan";
  c.require_slam_to_enable = scenario == "pose";
  c.scan_max_age_sec = .02;
  c.scan_loss_grace_sec = .05;
  c.slam_loss_grace_sec = .05;
  if (scenario == "timeout")
    c.cmd_vel_timeout_sec = .01;
  if (scenario == "leash") {
    c.leash_xy_m = .005;
    pose.absent = true;
  }
  ChassisBridgeCore core(c, session, pose, clock);
  std::cout << "ENABLE " << core.enable().first << '\n';
  core.submit_scan_seen();
  if (scenario == "pose")
    pose.absent = true;
  for (int tick = 0; tick < (scenario == "replay" ? 700 : 30); ++tick) {
    clock.t += scenario == "replay" ? (tick % 113 == 0 ? .008 : .004) : .01;
    if (scenario == "replay" && (tick == 100 || tick == 101))
      pose.p = {.002 * tick, .0005 * tick, .0002 * tick};
    if (scenario != "timeout" || tick == 0) {
      if (tick == 300)
        core.submit_twist(0, 0, 0);
      else if (tick == 301)
        core.submit_twist(-.08, .02, .03);
      else
        core.submit_twist(.12, -.03, .04);
    }
    if (scenario == "leash")
      session.actual = {.02, 0, 0};
    bool ok = core.inner_tick();
    if (scenario == "pose")
      core.outer_tick();
    std::cout << "TICK " << clock.t << ' ' << ok << ' ' << core.state();
    for (auto v : *core.pos_cmd())
      std::cout << ' ' << v;
    std::cout << '\n';
    for (auto &e : core.drain_events())
      std::cout << "EVENT " << e.code << ' ' << e.metric_1 << ' ' << e.metric_2
                << '\n';
  }
}
