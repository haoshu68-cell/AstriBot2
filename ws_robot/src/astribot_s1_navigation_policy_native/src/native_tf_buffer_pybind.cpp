#include "astribot_s1_navigation_policy_native/native_tf_buffer.hpp"

#include <pybind11/pybind11.h>
#include <tf2/exceptions.hpp>

namespace py = pybind11;
using astribot::navigation::NativeTfBuffer;

namespace
{
void translate_tf_exception(std::exception_ptr failure)
{
  if (!failure) {return;}
  try {std::rethrow_exception(failure);}
  catch (const tf2::LookupException & error) {
    PyErr_SetString(py::module_::import("tf2_ros").attr("LookupException").ptr(), error.what());
  } catch (const tf2::ConnectivityException & error) {
    PyErr_SetString(py::module_::import("tf2_ros").attr("ConnectivityException").ptr(), error.what());
  } catch (const tf2::ExtrapolationException & error) {
    PyErr_SetString(py::module_::import("tf2_ros").attr("ExtrapolationException").ptr(), error.what());
  } catch (const tf2::InvalidArgumentException & error) {
    PyErr_SetString(py::module_::import("tf2_ros").attr("InvalidArgumentException").ptr(), error.what());
  } catch (const tf2::TransformException & error) {
    PyErr_SetString(py::module_::import("tf2_ros").attr("TransformException").ptr(), error.what());
  }
}

py::object ros_message(const geometry_msgs::msg::TransformStamped & value)
{
  auto result = py::module_::import("geometry_msgs.msg").attr("TransformStamped")();
  result.attr("header").attr("frame_id") = value.header.frame_id;
  result.attr("header").attr("stamp").attr("sec") = value.header.stamp.sec;
  result.attr("header").attr("stamp").attr("nanosec") = value.header.stamp.nanosec;
  result.attr("child_frame_id") = value.child_frame_id;
  auto translation = result.attr("transform").attr("translation");
  translation.attr("x") = value.transform.translation.x;
  translation.attr("y") = value.transform.translation.y;
  translation.attr("z") = value.transform.translation.z;
  auto rotation = result.attr("transform").attr("rotation");
  rotation.attr("x") = value.transform.rotation.x;
  rotation.attr("y") = value.transform.rotation.y;
  rotation.attr("z") = value.transform.rotation.z;
  rotation.attr("w") = value.transform.rotation.w;
  return result;
}
}  // namespace

PYBIND11_MODULE(_native_tf_buffer, module)
{
  module.doc() = "Opt-in independent C++ TF receiver; exact-time, nonblocking queries";
  py::register_local_exception_translator(translate_tf_exception);
  py::class_<NativeTfBuffer>(module, "NativeTfBuffer")
    .def(py::init<bool, const std::string &>(), py::arg("use_sim_time"),
      py::arg("node_name") = "", py::call_guard<py::gil_scoped_release>())
    .def("can_transform", [](const NativeTfBuffer & self, const std::string & target,
      const std::string & source, const py::object & when, bool debug) -> py::object {
        const auto stamp = when.attr("nanoseconds").cast<std::int64_t>();
        std::pair<bool, std::string> result;
        {py::gil_scoped_release release; result = self.can_transform(target, source, stamp);}
        if (debug) {return py::make_tuple(result.first, result.second);}
        return py::bool_(result.first);
      }, py::arg("target_frame"), py::arg("source_frame"), py::arg("time"),
      py::arg("return_debug_tuple") = false)
    .def("lookup_transform", [](const NativeTfBuffer & self, const std::string & target,
      const std::string & source, const py::object & when) {
        const auto stamp = when.attr("nanoseconds").cast<std::int64_t>();
        geometry_msgs::msg::TransformStamped result;
        {py::gil_scoped_release release; result = self.lookup_transform(target, source, stamp);}
        return ros_message(result);
      }, py::arg("target_frame"), py::arg("source_frame"), py::arg("time"))
    .def("clear", &NativeTfBuffer::clear, py::call_guard<py::gil_scoped_release>())
    .def("close", &NativeTfBuffer::close, py::call_guard<py::gil_scoped_release>())
    .def("diagnostics", [](const NativeTfBuffer & self) {
        NativeTfBuffer::Diagnostics state;
        {py::gil_scoped_release release; state = self.diagnostics();}
        py::dict result;
        result["node_name"] = state.node_name;
        result["domain_id"] = state.domain_id;
        result["clock_ns"] = state.clock_ns;
        result["clock_epoch"] = state.clock_epoch;
        result["use_sim_time"] = state.use_sim_time;
        result["executor_running"] = state.executor_running;
        result["executor_error"] = state.executor_error;
        result["closed"] = state.closed;
        return result;
      });
}
