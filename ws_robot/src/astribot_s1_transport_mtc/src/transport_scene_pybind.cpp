#include "astribot_s1_transport_mtc/canonical_octomap.hpp"
#include <pybind11/pybind11.h>
namespace py=pybind11;
PYBIND11_MODULE(_transport_scene_native, module) {
  module.def("canonical_octomap",[](py::bytes data, bool binary, double resolution,
                                     const std::string& tree_id) {
    const std::string input=data;
    std::string output;
    { py::gil_scoped_release release;
      output=astribot::transport::canonical_octomap(input,binary,resolution,tree_id); }
    return py::bytes(output);
  },py::arg("data"),py::arg("binary"),py::arg("resolution"),py::arg("tree_id"));
}
