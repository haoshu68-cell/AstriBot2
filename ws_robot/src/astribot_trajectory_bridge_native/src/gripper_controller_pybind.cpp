#include "astribot_trajectory_bridge_native/gripper_controller.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <stdexcept>

namespace py = pybind11;
using namespace astribot_trajectory_bridge_native;
namespace {
// Migration adapter only: all validation, streaming, timeouts and results live
// in the independently linkable C++ GripperController.
template <class Fn> auto python_call(Fn &&fn) -> decltype(fn()) {
  try {
    return fn();
  } catch (const py::error_already_set &error) {
    throw PortError(py::str(error.value()).cast<std::string>());
  }
}
struct PythonGripperPort : GripperSessionPort {
  py::object session;
  explicit PythonGripperPort(py::object value) : session(std::move(value)) {}
  JointPositions get_current_joints_position(const Names &names) override {
    return python_call([&] {
      auto got = session.attr("get_current_joints_position")(names)
                     .cast<py::sequence>();
      JointPositions rows;
      for (py::handle item : got) {
        if (PySequence_Check(item.ptr()) && !PyUnicode_Check(item.ptr()) &&
            !PyBytes_Check(item.ptr())) {
          const auto row = py::reinterpret_borrow<py::sequence>(item);
          rows.push_back(row.size() == 0
                             ? std::vector<double>{}
                             : std::vector<double>{py::cast<double>(row[0])});
        } else
          rows.push_back({py::cast<double>(item)});
      }
      return rows;
    });
  }
  void set_joints_position(const Names &names, const JointPositions &positions,
                           const std::string &way, bool wbc,
                           bool torso) override {
    python_call([&] {
      session.attr("set_joints_position")(names, positions, way, wbc, torso);
    });
  }
  void open_effector(const Names &names, double duration) override {
    python_call([&] { session.attr("open_effector")(names, duration); });
  }
  void close_effector(const Names &names, double duration) override {
    python_call([&] { session.attr("close_effector")(names, duration); });
  }
  void set_effector_max_force(const Names &names,
                              const std::vector<double> &force) override {
    python_call([&] { session.attr("set_effector_max_force")(names, force); });
  }
};
struct PythonClock : ClockPort {
  py::object sleep_fn, clock_fn;
  PythonClock(py::object s, py::object c)
      : sleep_fn(std::move(s)), clock_fn(std::move(c)) {}
  double now() const override {
    return python_call([&] { return clock_fn().cast<double>(); });
  }
  void sleep(double seconds) override {
    python_call([&] { sleep_fn(seconds); });
  }
};
struct NativeGripperController {
  PythonGripperPort session;
  PythonClock clock;
  GripperController core;
  NativeGripperController(Names names, bool enable, double duration,
                          double force, double settle, double frequency,
                          double tolerance, double timeout, py::object port,
                          py::object sleep, py::object now, bool simulation)
      : session(std::move(port)), clock(std::move(sleep), std::move(now)),
        core(GripperConfig{std::move(names), enable, duration, force, settle,
                           frequency, tolerance, timeout},
             session, clock, simulation) {}
  py::tuple resolve_names(const std::string &name) {
    auto r = core.resolve_names(name);
    return py::make_tuple(r.value ? py::cast(*r.value) : py::none(),
                          r.value ? py::none() : py::cast(r.error));
  }
  py::tuple resolve_cmd(double fraction, bool raw, double command) {
    auto r = core.resolve_cmd(fraction, raw, command);
    return py::make_tuple(r.value ? py::cast(*r.value) : py::none(),
                          r.value ? py::none() : py::cast(r.error));
  }
  py::tuple execute(const std::string &name, double fraction, double duration,
                    bool raw, double command, double force, bool allowed) {
    auto r =
        core.execute(name, fraction, duration, raw, command, force, allowed);
    return py::make_tuple(r.ok, r.code, r.detail, r.dispatched_cmd,
                          r.dispatched_rad, r.actual_cmd, r.force_applied);
  }
  std::vector<GripperEvent> drain_events() { return core.drain_events(); }
};
} // namespace
void bind_gripper_controller(py::module_ &module) {
  py::class_<NativeGripperController>(module, "GripperController")
      .def(py::init<Names, bool, double, double, double, double, double, double,
                    py::object, py::object, py::object, bool>(),
           py::arg("names"), py::arg("enable_service"),
           py::arg("default_duration"), py::arg("default_max_force"),
           py::arg("settle_extra"), py::arg("stream_freq"),
           py::arg("stream_tolerance"), py::arg("stream_timeout"),
           py::arg("session"), py::arg("sleep_fn"), py::arg("clock_fn"),
           py::arg("in_simulation"))
      .def("resolve_names", &NativeGripperController::resolve_names)
      .def("resolve_cmd", &NativeGripperController::resolve_cmd)
      .def("execute", &NativeGripperController::execute, py::arg("name"),
           py::arg("opening_fraction"), py::arg("duration"),
           py::arg("use_raw_cmd"), py::arg("raw_cmd"), py::arg("max_force"),
           py::arg("write_allowed"))
      .def("drain_events", &NativeGripperController::drain_events);
}
