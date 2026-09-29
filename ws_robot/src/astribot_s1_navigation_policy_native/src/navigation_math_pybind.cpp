#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "astribot_s1_navigation_policy_native/navigation_math.hpp"
#include "astribot_s1_navigation_policy_native/scan_timing.hpp"
#include "astribot_s1_navigation_policy_native/publication_lease.hpp"

namespace py = pybind11;
using astribot::navigation::body_pose;
using astribot::navigation::body_to_world_xy;
using astribot::navigation::footprint_axes;
using astribot::navigation::sampling_margin;
using astribot::navigation::stopping_horizon;
using astribot::navigation::CommandRestriction;
using astribot::navigation::scan_usable;
using astribot::navigation::occupied_cells;
using astribot::navigation::angular_box_free;
using astribot::navigation::posture_out_of_bounds;
using astribot::navigation::is_degenerate_attitude_source;
using astribot::navigation::describe_monitor_state;
using astribot::navigation::evaluate_posture;
using astribot::navigation::ControlTime;
using astribot::navigation::lateral_variants;
using astribot::navigation::scan_coverage;
using astribot::navigation::movement_directions;
using astribot::navigation::coverage_allows_motion;
using astribot::navigation::assess_path;
using astribot::navigation::PathAssessmentResult;
using astribot::navigation::yield_requires_stop;
using astribot::navigation::YieldPolicy;
using astribot::navigation::SelectionResult;
using astribot::navigation::ExecutionContext;
using astribot::navigation::PlanningSessionState;
using astribot::navigation::costmap_clearing_ranges;
using astribot::navigation::clearance_many_rect;
using astribot::navigation::motion_clearance_rect;
using astribot::navigation::path_position_batch;
using astribot::navigation::remaining_path;
using astribot::navigation::path_samples;

PYBIND11_MODULE(_navigation_math_native, module) {
  module.doc() = "Native navigation prediction math";
  module.def("evaluate_publication_lease", [](
      std::int64_t now_ns, std::int64_t decision_ns,
      std::uint64_t decision_epoch, std::uint64_t current_epoch,
      double lease_cap_s,
      const std::vector<std::array<std::int64_t, 2>>& required_intervals,
      bool evidence_valid) {
        const auto lease = astribot::navigation::evaluate_publication_lease(
          now_ns, decision_ns, decision_epoch, current_epoch, lease_cap_s,
          required_intervals, evidence_valid);
        py::dict result;
        result["allowed"] = lease.allowed;
        result["lease_s"] = lease.lease_s;
        result["deadline_ns"] = lease.deadline_ns;
        result["reason"] = lease.reason;
        return result;
      }, py::arg("now_ns"), py::arg("decision_ns"), py::arg("decision_epoch"),
      py::arg("current_epoch"), py::arg("lease_cap_s"), py::arg("required_intervals"),
      py::arg("evidence_valid"));
  using astribot::navigation::ScanTiming;
  using astribot::navigation::ScanTimes;
  using astribot::navigation::ScanTimingRecord;
  py::class_<ScanTiming>(module, "ScanTiming")
      .def(py::init<>())
      .def("receive", &ScanTiming::receive)
      .def("tf_check", &ScanTiming::tf_check)
      .def("select", &ScanTiming::select)
      .def("finish", &ScanTiming::finish)
      .def("drop", &ScanTiming::drop)
      .def("defer", &ScanTiming::defer)
      .def("reset", &ScanTiming::reset)
      .def("snapshot", [](const ScanTiming& trace) {
        // Copy under the C++ mutex; build Python values after releasing it.
        const auto snapshot=trace.snapshot();
        auto times=[](const std::optional<ScanTimes>& t)->py::object {
          if(!t)return py::none();
          py::dict d;d["ros_ns"]=t->ros_ns;d["steady_ns"]=t->steady_ns;return d;
        };
        auto record=[&](const std::optional<ScanTimingRecord>& r)->py::object {
          if(!r)return py::none();
          py::dict d;
          d["sequence"]=r->sequence;d["clock_epoch"]=r->clock_epoch;
          d["frame_id"]=r->frame_id;d["capture_ros_ns"]=r->capture_ros_ns;
          d["receive"]=times(r->receive);d["first_tf_ready"]=times(r->first_tf_ready);
          d["selected"]=times(r->selected);d["finished"]=times(r->finished);
          d["last_tf_check"]=times(r->last_tf_check);d["deferred_at"]=times(r->deferred_at);d["tf_checks"]=r->tf_checks;
          d["tracking_tf"]=r->tracking_tf;d["map_tf"]=r->map_tf;
          d["processing_success"]=r->processing_success;d["reason"]=r->reason;
          return d;
        };
        py::dict d;
        d["clock_epoch"]=snapshot.clock_epoch;d["retained"]=snapshot.retained;
        d["evictions"]=snapshot.evictions;d["missing_updates"]=snapshot.missing_updates;
        d["resets"]=snapshot.resets;d["drop_counts"]=snapshot.drop_counts;d["defer_counts"]=snapshot.defer_counts;
        d["received"]=record(snapshot.received);d["waiting"]=record(snapshot.waiting);
        d["selected"]=record(snapshot.selected);d["finished"]=record(snapshot.finished);
        d["successful"]=record(snapshot.successful);d["dropped"]=record(snapshot.dropped);d["deferred"]=record(snapshot.deferred);
        return d;
      });
  module.def("stopping_horizon", &stopping_horizon);
  module.def("body_pose", &body_pose);
  module.def("body_to_world_xy", &body_to_world_xy);
  module.def("sampling_margin", &sampling_margin);
  module.def("footprint_axes", &footprint_axes);
  module.def("scan_usable", &scan_usable);
  module.def("occupied_cells", &occupied_cells);
  module.def("angular_box_free", &angular_box_free);
  module.def("posture_out_of_bounds", &posture_out_of_bounds);
  module.def("is_degenerate_attitude_source", &is_degenerate_attitude_source,
             py::arg("samples"), py::arg("eps"), py::arg("min_samples"));
  module.def("describe_monitor_state", &describe_monitor_state);
  module.def("evaluate_posture", &evaluate_posture,
             py::arg("enabled"), py::arg("samples"), py::arg("z"),
             py::arg("roll"), py::arg("pitch"), py::arg("normal_height"),
             py::arg("max_height_deviation"), py::arg("max_tilt_rad"),
             py::arg("min_samples"));
  module.def("lateral_variants", &lateral_variants);
  module.def("scan_coverage", &scan_coverage,
             py::arg("ranges"), py::arg("range_min"), py::arg("range_max"),
             py::arg("angle_min"), py::arg("angle_increment"),
             py::arg("body_yaw"));
  module.def("movement_directions", &movement_directions,
             py::arg("vx"), py::arg("vy"), py::arg("wz"));
  module.def("coverage_allows_motion", &coverage_allows_motion,
             py::arg("cones_xy_half"), py::arg("vx"), py::arg("vy"),
             py::arg("wz"));
  py::class_<PathAssessmentResult>(module, "PathAssessmentResult")
      .def_readonly("blocked", &PathAssessmentResult::blocked)
      .def_readonly("conflict_time_s", &PathAssessmentResult::conflict_time_s)
      .def_readonly("status", &PathAssessmentResult::status)
      .def_readonly("distance_m", &PathAssessmentResult::distance_m);
  module.def("assess_path", &assess_path,
             py::arg("known"), py::arg("blocked"), py::arg("stamp_s"),
             py::arg("received_wall_s"), py::arg("distance_m"),
             py::arg("now_s"), py::arg("wall_s"),
             py::arg("path_risk_timeout_s"), py::arg("max_speed_m_s"),
             py::arg("clearance_margin_m"),
             py::arg("payload_extra_margin_m"), py::arg("legacy_blocked"));
  module.def("yield_requires_stop", &yield_requires_stop,
             py::arg("immediate"), py::arg("uncertain"),
             py::arg("blocked"), py::arg("conflict_time_s"),
             py::arg("max_speed_m_s"), py::arg("reaction_time_s"),
             py::arg("brake_deceleration_m_s2"),
             py::arg("linear_stop_delay_s"));
  py::class_<SelectionResult>(module, "SelectionResult")
      .def_readonly("motion", &SelectionResult::motion)
      .def_readonly("speed", &SelectionResult::speed)
      .def_readonly("reason", &SelectionResult::reason)
      .def_readonly("episode", &SelectionResult::episode);
  py::class_<YieldPolicy>(module, "YieldPolicy")
      .def(py::init<double, double, double, double, double, double, double>(),
           py::arg("max_speed_m_s"), py::arg("narrow_speed_m_s"),
           py::arg("reaction_time_s"),
           py::arg("brake_deceleration_m_s2"),
           py::arg("linear_stop_delay_s"), py::arg("wait_budget_s"),
           py::arg("clear_hold_s"))
      .def("select", &YieldPolicy::select,
           py::arg("immediate"), py::arg("blocked"), py::arg("uncertain"),
           py::arg("conflict_time_s"), py::arg("valid"), py::arg("now"))
      .def("state", &YieldPolicy::state);
  py::class_<ExecutionContext>(module, "ExecutionContext")
      .def(py::init<>())
      .def("task", &ExecutionContext::task)
      .def("map", &ExecutionContext::map)
      .def("localization", &ExecutionContext::localization)
      .def("path", &ExecutionContext::path)
      .def("set_version", &ExecutionContext::set_version)
      .def("version", &ExecutionContext::version);
  py::class_<PlanningSessionState>(module, "PlanningSessionState")
      .def(py::init<std::string, std::int64_t, std::int64_t, std::int64_t>(),
           py::arg("session_id"), py::arg("request_timeout_ns"),
           py::arg("episode_timeout_ns"),
           py::arg("max_requests_per_goal"))
      .def("activate", &PlanningSessionState::activate,
           py::arg("goal_id"), py::arg("path_revision"),
           py::arg("map_epoch"), py::arg("envelope_epoch"),
           py::arg("localization_epoch"), py::arg("clock_epoch"),
           py::arg("now_ns"))
      .def("finish", &PlanningSessionState::finish)
      .def("clear_blockage", &PlanningSessionState::clear_blockage,
           py::arg("now_ns"), py::arg("clock_epoch"))
      .def("request", &PlanningSessionState::request,
           py::arg("goal_id"), py::arg("path_revision"),
           py::arg("map_epoch"), py::arg("envelope_epoch"),
           py::arg("localization_epoch"), py::arg("clock_epoch"),
           py::arg("observation_seq"), py::arg("now_ns"))
      .def("response_current", &PlanningSessionState::response_current,
           py::arg("request_id"), py::arg("goal_id"),
           py::arg("path_revision"), py::arg("map_epoch"),
           py::arg("envelope_epoch"), py::arg("localization_epoch"),
           py::arg("clock_epoch"), py::arg("now_ns"))
      .def("failure_reason", &PlanningSessionState::failure_reason,
           py::arg("now_ns"), py::arg("clock_epoch"))
      .def("retire", &PlanningSessionState::retire);
  py::class_<ControlTime>(module, "ControlTime")
      .def(py::init<bool, double>(), py::arg("simulated"),
           py::arg("stall_timeout_s"))
      .def("advance", [](ControlTime &clock, double ros, double wall) {
        const auto result = clock.advance(ros, wall);
        return py::make_tuple(result.now, result.dt, result.wall_dt,
                              result.running, result.reset,
                              result.stop_commands);
      })
      .def("accepts", &ControlTime::accepts)
      .def("fresh", &ControlTime::fresh)
      .def("command_fresh", &ControlTime::command_fresh)
      .def("state", &ControlTime::state)
      .def_property_readonly("simulated", &ControlTime::simulated);
  module.def("costmap_clearing_ranges", &costmap_clearing_ranges);
  module.def("clearance_many_rect", &clearance_many_rect);
  module.def("motion_clearance_rect", &motion_clearance_rect);
  module.def("path_position_batch", &path_position_batch);
  module.def("remaining_path", &remaining_path);
  module.def("path_samples", &path_samples);
  py::class_<CommandRestriction>(module, "CommandRestriction")
      .def(py::init<>())
      .def("apply", &CommandRestriction::apply,
           py::arg("command"), py::arg("cap"), py::arg("angular_cap"),
           py::arg("stop"), py::arg("dt"), py::arg("max_acceleration"),
           py::arg("max_angular_acceleration"), py::arg("allow_zero_dt"))
      .def("reset", &CommandRestriction::reset)
      .def("set_state", &CommandRestriction::set_state)
      .def("output", &CommandRestriction::output)
      .def("recovering", &CommandRestriction::recovering);
}
