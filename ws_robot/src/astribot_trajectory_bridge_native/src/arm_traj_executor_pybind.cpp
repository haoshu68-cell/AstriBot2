#include "astribot_trajectory_bridge_native/arm_executor.hpp"
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
struct PythonArmPort : ArmSessionPort {
  py::object session;
  explicit PythonArmPort(py::object s) : session(std::move(s)) {}
  JointPositions get_current_joints_position(const Names &n) override {
    return python_call([&] {
      return session.attr("get_current_joints_position")(n)
          .cast<JointPositions>();
    });
  }
  std::pair<JointPositions, JointPositions>
  get_joints_position_limit(const Names &n) override {
    return python_call([&] {
      return session.attr("get_joints_position_limit")(n)
          .cast<std::pair<JointPositions, JointPositions>>();
    });
  }
  void set_joints_position(const Names &n, const JointPositions &q,
                           const std::string &way, bool wbc,
                           bool torso) override {
    python_call(
        [&] { session.attr("set_joints_position")(n, q, way, wbc, torso); });
  }
  void move_joints_waypoints(const Names &n,
                             const std::vector<JointPositions> &q,
                             const std::vector<double> &t, bool wbc,
                             bool torso) override {
    python_call(
        [&] { session.attr("move_joints_waypoints")(n, q, t, wbc, torso); });
  }
};
struct PythonClock : ClockPort {
  py::object clock;
  explicit PythonClock(py::object c) : clock(std::move(c)) {}
  double now() const override {
    return python_call([&] { return clock.attr("now")().cast<double>(); });
  }
  void sleep(double) override {
    throw std::logic_error("Arm executor does not own sleeping");
  }
};
ArmConfig config_from_python(const py::object &c) {
  ArmConfig out;
  out.part_name = c.attr("part_name").cast<std::string>();
  out.joint_names = c.attr("joint_names").cast<std::vector<std::string>>();
  out.stream_freq = c.attr("stream_freq").cast<double>();
  out.control_way = c.attr("control_way").cast<std::string>();
  out.use_wbc = c.attr("use_wbc").cast<bool>();
  out.add_default_torso = c.attr("add_default_torso").cast<bool>();
  out.interp = c.attr("interp").cast<std::string>();
  out.max_traj_duration_sec = c.attr("max_traj_duration_sec").cast<double>();
  out.limit_margin_rad = c.attr("limit_margin_rad").cast<double>();
  out.cross_check_urdf = c.attr("cross_check_urdf").cast<bool>();
  out.limit_cross_check_tol_rad =
      c.attr("limit_cross_check_tol_rad").cast<double>();
  out.strict_limit_check = c.attr("strict_limit_check").cast<bool>();
  out.max_tracking_error_rad = c.attr("max_tracking_error_rad").cast<double>();
  out.abort_on_tracking_error = c.attr("abort_on_tracking_error").cast<bool>();
  out.settle_tolerance_rad = c.attr("settle_tolerance_rad").cast<double>();
  out.settle_timeout_sec = c.attr("settle_timeout_sec").cast<double>();
  out.hold_still_epsilon_rad = c.attr("hold_still_epsilon_rad").cast<double>();
  out.hold_still_ticks_required =
      c.attr("hold_still_ticks_required").cast<int>();
  out.hold_timeout_sec = c.attr("hold_timeout_sec").cast<double>();
  out.allow_recovery_from_oob = c.attr("allow_recovery_from_oob").cast<bool>();
  out.oob_start_tolerance_rad =
      c.attr("oob_start_tolerance_rad").cast<double>();
  return out;
}
py::list events_to_python(const std::vector<ArmEvent> &events) {
  py::list out;
  for (auto &e : events)
    out.append(py::make_tuple(e.code, e.detail, e.metric_1, e.metric_2));
  return out;
}
py::list feedbacks_to_python(const std::vector<ArmFeedback> &feedbacks) {
  py::list out;
  for (auto &f : feedbacks)
    out.append(py::make_tuple(f.t, f.desired, f.actual, f.error));
  return out;
}
struct NativeArmTrajExecutor {
  PythonArmPort session;
  PythonClock clock;
  ArmTrajExecutor core;
  NativeArmTrajExecutor(py::object c, py::object s, py::object t)
      : session(std::move(s)), clock(std::move(t)),
        core(config_from_python(c), session, clock) {}
  auto load_limits(std::optional<std::vector<double>> lo,
                   std::optional<std::vector<double>> hi) {
    return core.load_limits(lo, hi);
  }
  py::tuple start(const Names &n, const std::vector<double> &t,
                  const JointTrajectory &q, std::optional<JointTrajectory> v) {
    auto r = core.start(n, t, q, v);
    return py::make_tuple(r.ok, r.code, r.detail);
  }
  void request_cancel() { core.request_cancel(); }
  std::string step() { return core.step(); }
  std::string phase() const { return core.phase(); }
  int error_code() const { return core.error_code(); }
  std::string detail() const { return core.detail(); }
  auto lower() const { return core.lower(); }
  auto upper() const { return core.upper(); }
  py::list events() const { return events_to_python(core.events()); }
  py::list drain_events() { return events_to_python(core.drain_events()); }
  py::list feedbacks() const { return feedbacks_to_python(core.feedbacks()); }
  py::list drain_feedbacks() {
    return feedbacks_to_python(core.drain_feedbacks());
  }
};
struct NativeWaypointDispatcher {
  PythonArmPort session;
  WaypointDispatcher core;
  NativeWaypointDispatcher(py::object c, py::object s, bool enabled)
      : session(std::move(s)), core(config_from_python(c), session, enabled) {}
  py::tuple dispatch(const JointTrajectory &q, const std::vector<double> &t,
                     std::optional<std::vector<double>> lo,
                     std::optional<std::vector<double>> hi) {
    auto r = core.dispatch(q, t, lo, hi);
    return py::make_tuple(r.ok, r.code, r.detail, r.sent, r.dropped);
  }
  py::list events() const { return events_to_python(core.events()); }
  py::list drain_events() { return events_to_python(core.drain_events()); }
};
} // namespace
void bind_arm_traj_executor(py::module_ &module) {
  py::class_<NativeArmTrajExecutor>(module, "ArmTrajExecutor")
      .def(py::init<py::object, py::object, py::object>(), py::arg("config"),
           py::arg("session"), py::arg("clock"))
      .def("load_limits", &NativeArmTrajExecutor::load_limits,
           py::arg("urdf_lower") = py::none(),
           py::arg("urdf_upper") = py::none())
      .def("start", &NativeArmTrajExecutor::start, py::arg("joint_names"),
           py::arg("times"), py::arg("positions"),
           py::arg("velocities") = py::none())
      .def("request_cancel", &NativeArmTrajExecutor::request_cancel)
      .def("step", &NativeArmTrajExecutor::step)
      .def_property_readonly("phase", &NativeArmTrajExecutor::phase)
      .def_property_readonly("error_code", &NativeArmTrajExecutor::error_code)
      .def_property_readonly("detail", &NativeArmTrajExecutor::detail)
      .def_property_readonly("lower", &NativeArmTrajExecutor::lower)
      .def_property_readonly("upper", &NativeArmTrajExecutor::upper)
      .def_property_readonly("events", &NativeArmTrajExecutor::events)
      .def_property_readonly("feedbacks", &NativeArmTrajExecutor::feedbacks)
      .def("drain_events", &NativeArmTrajExecutor::drain_events)
      .def("drain_feedbacks", &NativeArmTrajExecutor::drain_feedbacks);
  py::class_<NativeWaypointDispatcher>(module, "WaypointDispatcher")
      .def(py::init<py::object, py::object, bool>(), py::arg("config"),
           py::arg("session"), py::arg("enabled"))
      .def("dispatch", &NativeWaypointDispatcher::dispatch,
           py::arg("waypoints"), py::arg("time_list"),
           py::arg("lower") = py::none(), py::arg("upper") = py::none())
      .def_property_readonly("events", &NativeWaypointDispatcher::events)
      .def("drain_events", &NativeWaypointDispatcher::drain_events);
}
