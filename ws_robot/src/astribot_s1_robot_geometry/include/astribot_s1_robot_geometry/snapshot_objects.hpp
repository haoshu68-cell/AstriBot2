#pragma once
// Optional compatibility fast path. Static occupied cells retain their exact
// geometry and capture-time contracts; dynamic derivation stays in Python.
#include <pybind11/pybind11.h>
#include <algorithm>
#include <cmath>
#include <vector>

namespace astribot_s1_robot_geometry {
namespace snapshot_objects_detail {
namespace py = pybind11;

inline py::object snapshotObjects(const py::object & fusion, const py::object & now,
                                 const py::object & region, const py::object & fallback) {
  const py::object epoch = fusion.attr("epoch");
  const py::object current_epoch = py::make_tuple(now.attr("clock"), now.attr("epoch"));
  if (!epoch.is_none() && !epoch.equal(current_epoch)) return fallback(fusion, now, region);
  const auto module = py::module_::import("astribot_s1_navigation_policy.fusion");
  const auto track_class = module.attr("TrackedObstacle");
  const py::str dictionary_name("__dict__");
  const auto model = module.attr("prediction_model")(
    module.attr("ZERO_VELOCITY"), 0., fusion.attr("prediction_steps"));
  const auto frame = fusion.attr("frame_id");
  const auto tracks = fusion.attr("tracks").cast<py::dict>();
  py::list output;
  py::dict dynamic;
  std::vector<py::ssize_t> dynamic_positions;
  const auto since = now.attr("since");
  const auto hypot = py::module_::import("math").attr("hypot");
  double x = 0., y = 0., travel = 0., fixed_radius = 0.;
  if (!region.is_none()) {
    const auto r = region.cast<py::tuple>();
    if (r.size() != 3) throw py::value_error("region must contain x, y, travel");
    x = r[0].cast<double>(); y = r[1].cast<double>(); travel = r[2].cast<double>();
    const auto p = fusion.attr("profile");
    fixed_radius = travel + p.attr("half_length_m").cast<double>() +
      p.attr("half_width_m").cast<double>() + p.attr("clearance_margin_m").cast<double>() +
      p.attr("payload_extra_margin_m").cast<double>();
  }
  for (auto item : tracks) {
    const auto track = py::reinterpret_borrow<py::object>(item.second);
    const auto obs = track.attr("observation");
    const auto velocity = track.attr("velocity");
    if (!obs.attr("spatial_occupancy").cast<bool>() ||
        velocity.attr("x").cast<double>() != 0. ||
        velocity.attr("y").cast<double>() != 0. ||
        velocity.attr("z").cast<double>() != 0.) {
      dynamic[item.first] = track;
      dynamic_positions.push_back(py::len(output));
      output.append(py::none());
      continue;
    }
    // Preserve the original per-track capture clock/epoch validation, including
    // arbitrary-precision integer stamps. No timestamp is relabelled.
    since(obs.attr("capture_stamp"));
    const auto geometry = obs.attr("geometry");
    bool relevant = true;
    if (!region.is_none()) {
      const auto size = geometry.attr("size_m");
      const auto center = geometry.attr("center_m");
      const auto covariance = geometry.attr("position_covariance_m2").attr("values");
      const auto radius = fixed_radius + hypot(size.attr("x"), size.attr("y")).cast<double>() / 2. +
        2. * std::sqrt(2. * std::max(covariance[py::int_(0)].cast<double>(),
                                   covariance[py::int_(4)].cast<double>()));
      // Use Python's correctly rounded hypot, matching the reference boundary.
      relevant = hypot(center.attr("x").cast<double>() - x,
                       center.attr("y").cast<double>() - y).cast<double>() <= radius;
    }
    py::dict fields;
    fields["fused_track_id"] = track.attr("identifier"); fields["frame_id"] = frame;
    fields["stamp"] = now; fields["geometry"] = geometry; fields["predictions"] = py::tuple();
    fields["provenance"] = obs.attr("provenance");
    fields["prediction_model"] = relevant ? py::object(model) : py::none();
    // C-API equivalent of the existing _new_frozen reconstruction.
    auto result = py::reinterpret_steal<py::object>(
      PyType_GenericAlloc(reinterpret_cast<PyTypeObject *>(py::object(track_class).ptr()), 0));
    if (!result) throw py::error_already_set();
    if (PyObject_GenericSetAttr(result.ptr(), dictionary_name.ptr(), fields.ptr()) != 0)
      throw py::error_already_set();
    output.append(result);
  }
  if (!dynamic.empty()) {
    // Never swap the original fusion's tracks, even temporarily: the fallback
    // receives an independent shallow copy and cannot alter the source map.
    auto subset = py::module_::import("copy").attr("copy")(fusion);
    subset.attr("tracks") = dynamic;
    const auto derived = fallback(subset, now, region).attr("tracks").cast<py::tuple>();
    for (std::size_t i = 0; i < dynamic_positions.size(); ++i)
      output[dynamic_positions[i]] = derived[i];
  }
  // Keep the original world validation (metadata, duplicates, frames, stamps).
  return module.attr("WorldSnapshot")(
    fusion.attr("version"), now, frame, py::tuple(output),
    py::tuple(fusion.attr("unassociated").attr("values")()),
    fusion.attr("sensors"), fusion.attr("sequence"));
}
}  // namespace snapshot_objects_detail
}  // namespace astribot_s1_robot_geometry
