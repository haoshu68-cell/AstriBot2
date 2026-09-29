#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "astribot_trajectory_bridge_native/chassis_math.hpp"
#include "astribot_trajectory_bridge_native/arm_math.hpp"

namespace py = pybind11;
using namespace astribot_trajectory_bridge_native;

void bind_gripper_controller(py::module_ &module);
void bind_arm_traj_executor(py::module_ &module);
void bind_chassis_bridge_core(py::module_ &module);

PYBIND11_MODULE(_chassis_math_native, module) {
  module.doc() = "Native kernels for chassis_integrator.py";
  module.def("wrap_angle", &wrap_angle);
  module.def("rotate_vec2_transposed", [](double yaw, const Vec2 &value) {
    return rotate_vec2_transposed(yaw, value);
  });
  module.def("to_local_velocity", &to_local_velocity);
  module.def("local_pose_displacement", &local_pose_displacement);
  module.def("integrate_step_dt", &integrate_step_dt);
  module.def("integrate_step", &integrate_step);
  module.def("pose_error", &pose_error);
  module.def("error_magnitude", &error_magnitude);
  module.def("leash_error", &leash_error);
  module.def("pose_jump_distance", &pose_jump_distance);
  module.def("odom_drift", &odom_drift);
  module.def("measure_tick_dt", [](double now, py::object previous,
                                    double nominal_dt, double max_dt) {
    std::optional<double> previous_value;
    if (!previous.is_none()) {
      previous_value = previous.cast<double>();
    }
    const auto result = measure_tick_dt(now, previous_value, nominal_dt, max_dt);
    py::object raw = result.has_raw ? py::cast(result.raw) : py::none();
    return py::make_tuple(result.dt, result.clamped, result.reason, raw);
  });
  py::class_<PoseFrameIntegrator>(module, "PoseFrameIntegrator")
      .def(py::init<const Vec3 &, double, const Vec3 &>())
      .def("observe", &PoseFrameIntegrator::observe)
      .def("reanchor_axis", &PoseFrameIntegrator::reanchor_axis)
      .def("target", &PoseFrameIntegrator::target)
      .def("commit", &PoseFrameIntegrator::commit)
      .def("preview_target", &PoseFrameIntegrator::preview_target)
      .def("commit_preview", &PoseFrameIntegrator::commit_preview)
      .def("stamp", &PoseFrameIntegrator::stamp)
      .def("pose", &PoseFrameIntegrator::pose)
      .def("anchor", &PoseFrameIntegrator::anchor)
      .def("integral", &PoseFrameIntegrator::integral)
      .def("velocity", &PoseFrameIntegrator::velocity)
      .def("set_integral", &PoseFrameIntegrator::set_integral);
  py::class_<SdkPoseHistory>(module, "SdkPoseHistory")
      .def(py::init<double>())
      .def("append", &SdkPoseHistory::append)
      .def("at", &SdkPoseHistory::at);
  py::class_<ChassisOdomSource>(module, "ChassisOdomSource")
      .def(py::init<double, const std::string &>(), py::arg("jump_threshold_m"),
           py::arg("velocity_frame"))
      .def("sample", [](ChassisOdomSource &source, const Vec3 &pos,
                         const Vec3 &vel) {
        const auto result = source.sample(pos, vel);
        return py::make_tuple(result.x, result.y, result.theta,
                              result.vx_body, result.vy_body, result.wz,
                              result.jump_m, result.jumped);
      })
      .def("stats", [](const ChassisOdomSource &source) {
        const auto result = source.stats();
        return py::make_tuple(result.samples, result.jumps, result.max_jump_m,
                              result.travelled_m, result.jump_history);
      })
      .def("jump_ratio", &ChassisOdomSource::jump_ratio);
  module.def("interpolate_trajectory", &interpolate_trajectory,
             py::arg("times"), py::arg("positions"), py::arg("velocities"),
             py::arg("t"), py::arg("use_cubic"));
  module.def("max_abs_error", &max_abs_error);
  module.def("clamp_cmd", &clamp_cmd);
  module.def("is_cmd_in_range", &is_cmd_in_range);
  module.def("cmd_to_rad", &cmd_to_rad);
  module.def("rad_to_cmd", &rad_to_cmd);
  module.def("opening_fraction_to_cmd", &opening_fraction_to_cmd);
  module.def("cmd_to_opening_fraction", &cmd_to_opening_fraction);
  module.def("horizontal_reach", &horizontal_reach);
  module.def("reach_activity", &reach_activity);
  module.def("scale_from_activity", &scale_from_activity);
  module.def("is_extended_by_reach", &is_extended_by_reach);
  bind_gripper_controller(module);
  bind_arm_traj_executor(module);
  bind_chassis_bridge_core(module);
}
