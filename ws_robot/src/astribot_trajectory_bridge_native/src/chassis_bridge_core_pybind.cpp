#include "astribot_trajectory_bridge_native/chassis_controller.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;
using namespace astribot_trajectory_bridge_native;
namespace {
template <class Fn> auto python_call(Fn &&fn) -> decltype(fn()) {
  try {
    return fn();
  } catch (const py::error_already_set &e) {
    throw PortError(py::str(e.value()).cast<std::string>());
  }
}
struct PythonSession : JointSessionPort {
  py::object session;
  explicit PythonSession(py::object s) : session(std::move(s)) {}
  JointPositions get_current_joints_position(const Names &n) override {
    return python_call([&] {
      return session.attr("get_current_joints_position")(n)
          .cast<JointPositions>();
    });
  }
  void set_joints_position(const Names &n, const JointPositions &q,
                           const std::string &w, bool b, bool t) override {
    python_call([&] { session.attr("set_joints_position")(n, q, w, b, t); });
  }
};
struct PythonClock : ClockPort {
  py::object clock;
  explicit PythonClock(py::object c) : clock(std::move(c)) {}
  double now() const override {
    return python_call([&] { return clock.attr("now")().cast<double>(); });
  }
  void sleep(double) override {
    throw std::logic_error("Chassis core does not own sleeping");
  }
};
struct PythonPose : PosePort {
  py::object pose;
  explicit PythonPose(py::object p) : pose(std::move(p)) {}
  std::optional<StampedPose> lookup() override {
    return python_call([&]() -> std::optional<StampedPose> {
      auto pair = pose.attr("lookup")().cast<py::sequence>();
      if (pair[0].is_none())
        return std::nullopt;
      return StampedPose{pair[0].cast<std::vector<double>>(),
                         pair[1].cast<double>()};
    });
  }
};
ChassisConfig config_from_python(const py::object &c) {
  ChassisConfig out;
  out.outer_rate = c.attr("outer_rate").cast<double>();
  out.pose_source = c.attr("pose_source").cast<std::string>();
  out.part_name = c.attr("part_name").cast<std::string>();
  out.freq = c.attr("freq").cast<double>();
  out.input_frame = c.attr("input_frame").cast<std::string>();
  out.theta_reference = c.attr("theta_reference").cast<std::string>();
  out.cmd_vel_timeout_sec = c.attr("cmd_vel_timeout_sec").cast<double>();
  out.leash_xy_m = c.attr("leash_xy_m").cast<double>();
  out.leash_theta_rad = c.attr("leash_theta_rad").cast<double>();
  out.require_slam_to_enable = c.attr("require_slam_to_enable").cast<bool>();
  out.slam_max_age_sec = c.attr("slam_max_age_sec").cast<double>();
  out.slam_loss_grace_sec = c.attr("slam_loss_grace_sec").cast<double>();
  out.slam_jump_threshold_m = c.attr("slam_jump_threshold_m").cast<double>();
  out.odom_drift_window_sec = c.attr("odom_drift_window_sec").cast<double>();
  out.odom_drift_warn_m = c.attr("odom_drift_warn_m").cast<double>();
  out.max_tick_dt_sec = c.attr("max_tick_dt_sec").cast<double>();
  out.require_fresh_scan = c.attr("require_fresh_scan").cast<bool>();
  out.scan_max_age_sec = c.attr("scan_max_age_sec").cast<double>();
  out.scan_loss_grace_sec = c.attr("scan_loss_grace_sec").cast<double>();
  out.pose_preview_xy_sec = c.attr("pose_preview_xy_sec").cast<double>();
  out.pose_preview_theta_sec = c.attr("pose_preview_theta_sec").cast<double>();
  out.pose_preview_max_xy_m = c.attr("pose_preview_max_xy_m").cast<double>();
  out.pose_preview_max_theta_rad =
      c.attr("pose_preview_max_theta_rad").cast<double>();
  return out;
}
struct NativeChassisBridgeCore {
  PythonSession session;
  PythonPose pose;
  PythonClock clock;
  ChassisBridgeCore core;
  NativeChassisBridgeCore(py::object c, py::object s, py::object p,
                          py::object t)
      : session(std::move(s)), pose(std::move(p)), clock(std::move(t)),
        core(config_from_python(c), session, pose, clock) {}
  std::string state() const { return core.state(); }
  py::object pos_cmd() const {
    auto p = core.pos_cmd();
    return p ? py::cast(*p) : py::none();
  }
  py::object last_stop() const {
    auto s = core.last_stop();
    return s ? py::make_tuple(s->state, s->reason, s->metric_1, s->metric_2,
                              s->at)
             : py::object(py::none());
  }
  py::tuple integrator_state() const {
    auto s = core.integrator_state();
    if (!s)
      return py::make_tuple(py::none(), py::none(), py::none(), py::none(),
                            py::none());
    return py::make_tuple(s->stamp, s->pose, s->anchor, s->integral,
                          s->velocity);
  }
  void submit_twist(double x, double y, double z) {
    core.submit_twist(x, y, z);
  }
  void submit_scan_seen(std::optional<double> t) { core.submit_scan_seen(t); }
  auto enable() { return core.enable(); }
  auto disable() { return core.disable(); }
  auto reset_leash() { return core.reset_leash(); }
  bool inner_tick() { return core.inner_tick(); }
  void outer_tick() { core.outer_tick(); }
  py::list drain_events() {
    py::list out;
    for (auto &e : core.drain_events())
      out.append(py::make_tuple(e.code, e.detail, e.metric_1, e.metric_2));
    return out;
  }
  py::tuple tick_stats() const {
    auto s = core.tick_stats();
    return py::make_tuple(s.count, s.mean_dt, s.rate_hz, s.clamp_count,
                          s.clamp_ratio, s.live, s.max_dt);
  }
  py::tuple consume_tick_window_gap() {
    auto s = core.consume_tick_window_gap();
    return py::make_tuple(s.max_dt, s.at, s.over_count, s.ticks);
  }
  void reset_tick_stats() { core.reset_tick_stats(); }
  py::tuple consume_vel_trace() {
    auto s = core.consume_vel_trace();
    return py::make_tuple(s.ticks, s.wall, s.in_peak, s.local_peak,
                          s.in_wz_peak, s.local_wz_peak, s.zeroed_ticks,
                          s.cmd_path, s.cmd_net, s.act_net, s.dtheta_cmd,
                          s.dtheta_act, s.corr_path, s.dtheta_integrated,
                          s.pose_rebases, s.frame_dx, s.frame_dy,
                          s.frame_dtheta, s.lead_xy_peak, s.lead_theta_peak);
  }
};
} // namespace
void bind_chassis_bridge_core(py::module_ &module) {
  py::class_<NativeChassisBridgeCore>(module, "ChassisBridgeCore")
      .def(py::init<py::object, py::object, py::object, py::object>(),
           py::arg("config"), py::arg("session"), py::arg("pose"),
           py::arg("clock"))
      .def_property_readonly("state", &NativeChassisBridgeCore::state)
      .def_property_readonly("pos_cmd", &NativeChassisBridgeCore::pos_cmd)
      .def_property_readonly("last_stop", &NativeChassisBridgeCore::last_stop)
      .def("integrator_state", &NativeChassisBridgeCore::integrator_state)
      .def("submit_twist", &NativeChassisBridgeCore::submit_twist)
      .def("submit_scan_seen", &NativeChassisBridgeCore::submit_scan_seen,
           py::arg("stamp") = py::none())
      .def("enable", &NativeChassisBridgeCore::enable)
      .def("disable", &NativeChassisBridgeCore::disable)
      .def("reset_leash", &NativeChassisBridgeCore::reset_leash)
      .def("inner_tick", &NativeChassisBridgeCore::inner_tick)
      .def("outer_tick", &NativeChassisBridgeCore::outer_tick)
      .def("drain_events", &NativeChassisBridgeCore::drain_events)
      .def("tick_stats", &NativeChassisBridgeCore::tick_stats)
      .def("consume_tick_window_gap",
           &NativeChassisBridgeCore::consume_tick_window_gap)
      .def("reset_tick_stats", &NativeChassisBridgeCore::reset_tick_stats)
      .def("consume_vel_trace", &NativeChassisBridgeCore::consume_vel_trace);
}
