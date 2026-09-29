// Persistent benchmark worker: parses one command per sample, never per tick.
// No ROS, Python embedding, vendor transport or hardware operation.
#include "astribot_trajectory_bridge_native/arm_executor.hpp"
#include "astribot_trajectory_bridge_native/chassis_controller.hpp"
#include "astribot_trajectory_bridge_native/gripper_controller.hpp"
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
using namespace astribot_trajectory_bridge_native;
using Wall = std::chrono::steady_clock;
static double cpu_now() {
  timespec t{};
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t);
  return t.tv_sec + t.tv_nsec * 1e-9;
}
static long status_kib(const std::string &field) {
  std::ifstream f("/proc/self/status");
  std::string line;
  while (std::getline(f, line)) {
    if (line.rfind(field + ":", 0) == 0) {
      std::istringstream value(line.substr(field.size() + 1));
      long kib = 0;
      value >> kib;
      return kib;
    }
  }
  throw std::runtime_error("missing process memory field");
}
static long rss() { return status_kib("VmRSS"); }
static long peak_rss() { return status_kib("VmHWM"); }
struct Clock : ClockPort {
  double t = 0;
  double now() const override { return t; }
  void sleep(double dt) override { t += dt; }
};
struct Record {
  int op;
  double time;
  int part;
  std::vector<double> position;
  int way;
  bool wbc, torso;
  double duration;
};
struct Session : ArmSessionPort, GripperSessionPort {
  Clock &clock;
  int part;
  bool verify;
  std::vector<double> actual;
  std::vector<Record> trace;
  std::size_t writes = 0, opens = 0, closes = 0;
  Session(Clock &c, int p, bool v)
      : clock(c), part(p), verify(v), actual(p == 1 ? 1 : 3, 0) {}
  JointPositions get_current_joints_position(const Names &) override {
    return {actual};
  }
  std::pair<JointPositions, JointPositions>
  get_joints_position_limit(const Names &) override {
    return {{{-2, -2, -2}}, {{2, 2, 2}}};
  }
  void set_joints_position(const Names &names, const JointPositions &q,
                           const std::string &w, bool b, bool t) override {
    ++writes;
    if (verify) {
      const Names expected{part == 1   ? "g"
                           : part == 2 ? "astribot_chassis"
                                       : "astribot_arm_left"};
      if (names != expected || (w != "direct" && w != "filter"))
        throw std::runtime_error("write metadata differs from fixture");
      trace.push_back({0, clock.t, part, q[0], w == "direct" ? 0 : 1, b, t, 0});
    }
    double follow = part == 1 ? 1.0 : part == 2 ? .87 : .65;
    for (std::size_t i = 0; i < actual.size(); ++i)
      actual[i] += follow * (q[0][i] - actual[i]);
  }
  void move_joints_waypoints(const Names &, const std::vector<JointPositions> &,
                             const std::vector<double> &, bool, bool) override {
    throw std::logic_error("not part of benchmark");
  }
  void open_effector(const Names &names, double duration) override {
    ++opens;
    actual[0] = 0;
    if (verify) {
      if (names != Names{"g"})
        throw std::runtime_error("open metadata differs from fixture");
      trace.push_back({1, clock.t, part, actual, 0, false, false, duration});
    }
  }
  void close_effector(const Names &names, double duration) override {
    ++closes;
    actual[0] = 100;
    if (verify) {
      if (names != Names{"g"})
        throw std::runtime_error("close metadata differs from fixture");
      trace.push_back({2, clock.t, part, actual, 0, false, false, duration});
    }
  }
  void set_effector_max_force(const Names &,
                              const std::vector<double> &) override {
    throw std::logic_error("not part of benchmark");
  }
};
struct Pose : PosePort {
  Clock &clock;
  std::vector<double> pose{0, 0, 0};
  explicit Pose(Clock &c) : clock(c) {}
  std::optional<StampedPose> lookup() override {
    return StampedPose{pose, clock.t};
  }
};
struct Result {
  std::string kind, mode, state;
  std::size_t count = 0, writes = 0, opens = 0, closes = 0, events = 0,
              feedbacks = 0;
  double wall = 0, cpu = 0, checksum = 0;
  long entry = 0, ready = 0, after = 0, peak = 0;
  std::vector<double> position;
  std::vector<Record> trace;
};
static Result run(const std::string &kind, const std::string &mode,
                  std::size_t count) {
  Result out;
  out.kind = kind;
  out.mode = mode;
  out.count = count;
  out.entry = rss();
  Clock clock;
  int part = kind == "gripper" ? 1 : kind == "chassis" ? 2 : 3;
  Session session(clock, part, mode == "VERIFY");
  auto measure = [&](auto &&loop) {
    out.ready = rss();
    auto t = Wall::now();
    double c = cpu_now();
    loop();
    out.cpu = cpu_now() - c;
    out.wall = std::chrono::duration<double>(Wall::now() - t).count();
    out.after = rss();
    out.peak = peak_rss();
  };
  if (mode == "BASE") {
    measure([&] {
      for (std::size_t i = 0; i < count; ++i) {
        clock.t += part == 3 ? .002 : .004;
        out.checksum += clock.t;
      }
    });
    out.state = "BASE";
  } else if (kind == "gripper") {
    GripperConfig cfg;
    cfg.names = {"g"};
    cfg.enable_service = true;
    cfg.default_duration = .2;
    cfg.settle_extra = 0;
    cfg.stream_freq = 20;
    cfg.stream_tolerance = .5;
    cfg.stream_timeout = .25;
    GripperController core(cfg, session, clock, true);
    measure([&] {
      for (std::size_t i = 0; i < count; ++i) {
        double command = 25.0 * (i % 5);
        auto result = core.execute("g", 0, .01, true, command, 0, true);
        if (!result.ok)
          throw std::runtime_error(result.code);
        out.checksum += result.dispatched_cmd + result.actual_cmd;
        out.events += core.drain_events().size();
      }
    });
    out.state = "SUCCESS";
  } else if (kind == "arm") {
    ArmConfig cfg;
    cfg.joint_names = {"j0", "j1", "j2"};
    cfg.stream_freq = 500;
    cfg.max_tracking_error_rad = 10;
    cfg.max_traj_duration_sec = 2000;
    ArmTrajExecutor core(cfg, session, clock);
    if (!core.load_limits().first)
      throw std::runtime_error("limit load");
    if (!core.start(cfg.joint_names, {0, count * .002 + 1},
                    {{0, 0, 0}, {1, -1, .5}},
                    JointTrajectory{{0, 0, 0}, {0, 0, 0}})
             .ok)
      throw std::runtime_error("arm start");
    measure([&] {
      for (std::size_t i = 0; i < count; ++i) {
        clock.t += .002;
        core.step();
        out.events += core.drain_events().size();
        out.feedbacks += core.drain_feedbacks().size();
        out.checksum += session.actual[0];
      }
    });
    out.state = core.phase();
  } else if (kind == "chassis") {
    Pose pose(clock);
    ChassisConfig cfg;
    cfg.require_fresh_scan = false;
    cfg.pose_source = "ground_truth";
    ChassisBridgeCore core(cfg, static_cast<ArmSessionPort &>(session), pose,
                           clock);
    if (!core.enable().first)
      throw std::runtime_error("chassis enable");
    // Same host serialization required by the Python facade; timing diagnostics
    // inside the historical Python core remain part of its measured workload.
    std::mutex control;
    measure([&] {
      for (std::size_t i = 0; i < count; ++i) {
        clock.t += i % 113 == 0 ? .008 : .004;
        if (i == 100 || i == 101)
          pose.pose = {.002 * i, .0005 * i, .0002 * i};
        {
          std::lock_guard<std::mutex> guard(control);
          if (i == 300)
            core.submit_twist(0, 0, 0);
          else if (i == 301)
            core.submit_twist(-.08, .02, .03);
          else
            core.submit_twist(.12, -.03, .04);
          if (!core.inner_tick())
            throw std::runtime_error("chassis stopped");
          out.events += core.drain_events().size();
          out.checksum += session.actual[0];
        }
      }
    });
    out.state = core.state();
  } else
    throw std::invalid_argument("unknown core");
  out.position = session.actual;
  out.writes = session.writes;
  out.opens = session.opens;
  out.closes = session.closes;
  out.trace = std::move(session.trace);
  return out;
}
static void print_vector(const std::vector<double> &v) {
  std::cout << '[';
  bool first = true;
  for (auto x : v) {
    if (!first)
      std::cout << ',';
    first = false;
    std::cout << x;
  }
  std::cout << ']';
}
static void print(const Result &r) {
  std::cout << std::setprecision(17) << "{\"kind\":" << std::quoted(r.kind)
            << ",\"mode\":" << std::quoted(r.mode)
            << ",\"iterations\":" << r.count
            << ",\"state\":" << std::quoted(r.state)
            << ",\"wall_seconds\":" << r.wall << ",\"cpu_seconds\":" << r.cpu
            << ",\"rss_entry_kib\":" << r.entry
            << ",\"rss_ready_kib\":" << r.ready
            << ",\"rss_after_kib\":" << r.after
            << ",\"process_peak_rss_kib\":" << r.peak
            << ",\"writes\":" << r.writes << ",\"opens\":" << r.opens
            << ",\"closes\":" << r.closes << ",\"events\":" << r.events
            << ",\"feedbacks\":" << r.feedbacks
            << ",\"checksum\":" << r.checksum << ",\"position\":";
  print_vector(r.position);
  std::cout << ",\"trace\":[";
  bool first = true;
  for (auto &t : r.trace) {
    if (!first)
      std::cout << ',';
    first = false;
    std::cout << '[' << t.op << ',' << t.time << ',' << t.part << ',';
    print_vector(t.position);
    std::cout << ',' << t.way << ',' << int(t.wbc) << ',' << int(t.torso) << ','
              << t.duration << ']';
  }
  std::cout << "]}" << std::endl;
}
int main() {
  std::cout << "{\"ready\":true,\"language\":\"cpp\"}" << std::endl;
  std::string mode, kind;
  std::size_t count;
  while (std::cin >> mode >> kind >> count) {
    try {
      print(run(kind, mode, count));
    } catch (const std::exception &e) {
      std::cout << "{\"error\":" << std::quoted(e.what()) << '}' << std::endl;
      return 1;
    }
  }
}
