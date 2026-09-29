#include "astribot_s1_navigation_policy_native/policy_profile.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>

namespace astribot::navigation {
namespace {
using json = nlohmann::json;

void require(bool ok, const std::string& field) {
  if (!ok) throw std::invalid_argument("INVALID_INPUT: " + field);
}

bool truthy(const json& value) {
  if (value.is_null()) return false;
  if (value.is_boolean()) return value.get<bool>();
  if (value.is_number()) return value.get<double>() != 0.0;
  return !value.empty() && (!value.is_string() || !value.get_ref<const std::string&>().empty());
}

bool equals_one(const json& value) {
  return (value.is_number() && value.get<double>() == 1.0) ||
         (value.is_boolean() && value.get<bool>());
}

double number(const json& object, const char* name, bool positive = true,
              const std::string& prefix = "") {
  require(object.contains(name) && object.at(name).is_number(), prefix + name);
  const double value = object.at(name).get<double>();
  require(std::isfinite(value) && (positive ? value > 0.0 : value >= 0.0), prefix + name);
  return value;
}

json read_object(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) throw std::invalid_argument("unable to open profile: " + path.string());
  auto result = json::parse(input);
  require(result.is_object(), "profile object");
  return result;
}

double reference_number(const json& value) {
  // stop_reference.py uses Python arithmetic (including bool as 0/1), unlike
  // Profile's strict finite() validator. Preserve that existing file contract.
  require(value.is_number() || value.is_boolean(), "stopping reference number");
  return value.is_boolean() ? (value.get<bool>() ? 1.0 : 0.0) : value.get<double>();
}

void finite_json(const json& value) {
  if (value.is_number()) require(std::isfinite(value.get<double>()), "nonfinite profile value");
  if (value.is_structured()) for (const auto& child : value) finite_json(child);
}

void apply_stop_reference(json& data, const json& reference) {
  require(data.at("environment") == "simulation" &&
              !truthy(reference.at("hardware_validated")),
          "historical stop reference is simulation-only");
  require(equals_one(reference.at("schema_version")) &&
              reference.at("quantity") == "peak_excursion", "unsupported stop reference");
  const auto& fit = reference.at("polynomial");
  const auto positive = [&fit](const char* key) {
    const auto value = reference_number(fit.at(key));
    require(std::isfinite(value) && value > 0.0, key);
    return value;
  };
  const double linear = positive("nominal_linear_s");
  const double quadratic = positive("nominal_quadratic_s2_per_m");
  const double scale = positive("engineering_scale");
  const double margin = positive("additive_margin_m");
  const auto& range = reference.at("actual_speed_range_m_s");
  require(range.is_array() && range.size() == 2,
          "invalid measured speed range");
  const double lo = reference_number(range[0]), hi = reference_number(range[1]);
  require(0.0 < lo && lo < hi && std::isfinite(hi), "invalid measured speed range");
  data["linear_stop_delay_s"] = scale * linear;
  const double denominator = 2.0 * scale * quadratic;
  require(denominator > 0.0, "stopping reference denominator underflow");
  const double bound = 1.0 / denominator;
  if (bound < reference_number(data.at("brake_deceleration_m_s2")))
    data["brake_deceleration_m_s2"] = bound;
  data["clearance_margin_m"] = reference_number(data.at("clearance_margin_m")) + margin;
  data["stopping_reference"] = reference;
  require(data.at("sources").is_object(), "stop reference sources object");
  data.at("sources")["braking"] = reference.at("model_id").get<std::string>() +
      ": historical peak-envelope prior; simulation engineering bound";
}

void validate(json& data, bool simulated) {
  // Also cover overflow in derived values, matching json.dumps(allow_nan=False).
  finite_json(data);
  auto def = [&data](const char* key, const json& value) {
    if (!data.contains(key)) data[key] = value;
  };
  def("scan_occupancy_resolution_m", .05);
  def("scan_obstacle_padding_m", .04);
  number(data, "scan_obstacle_padding_m", false);
  def("narrow_angular_speed_rad_s", std::min(.2, data.value("max_angular_speed_rad_s", .2)));
  def("narrow_centering_speed_m_s", .05);
  def("narrow_centering_tolerance_m", .01);
  def("narrow_centering_max_offset_m", .3);
  require(equals_one(data.at("schema_version")), "profile.schema_version");
  require(data.at("environment") == "simulation" || data.at("environment") == "hardware", "profile.environment");
  require(data.at("hardware_validated").is_boolean(), "profile.hardware_validated");
  for (const auto* key : {"half_length_m", "half_width_m", "height_m", "clearance_margin_m",
       "scan_occupancy_resolution_m", "max_speed_m_s", "max_acceleration_m_s2", "brake_deceleration_m_s2",
       "reaction_time_s", "sensor_timeout_s", "track_memory_s", "prediction_horizon_s", "prediction_step_s",
       "association_distance_m", "max_obstacle_speed_m_s", "clear_hold_s", "blocked_confirm_s", "wait_budget_s",
       "narrow_speed_m_s", "narrow_heading_limit_rad", "narrow_angular_speed_rad_s", "command_timeout_s",
       "scan_min_valid_fraction", "velocity_confirmation_s", "velocity_fit_window_s", "velocity_fit_max_residual_m",
       "min_tracked_speed_m_s", "stationary_velocity_variance_m2_s2", "max_angular_speed_rad_s",
       "max_angular_acceleration_rad_s2", "angular_brake_deceleration_rad_s2", "constraint_lease_s",
       "input_command_timeout_s", "path_risk_timeout_s", "local_rejoin_distance_m", "local_max_deviation_m",
       "narrow_centering_speed_m_s", "narrow_centering_tolerance_m", "narrow_centering_max_offset_m"}) {
    number(data, key);
  }
  number(data, "payload_mass_kg", false);
  number(data, "payload_extra_margin_m", false);
  const auto v = [&data](const char* key) { return data.at(key).get<double>(); };
  require(v("constraint_lease_s") <= .5, "constraint_wire_lease_limit");
  require(v("path_risk_timeout_s") <= v("reaction_time_s"), "path_evidence_age_budget");
  require(v("reaction_time_s") >= v("input_command_timeout_s"), "input_stop_budget");
  require(v("narrow_speed_m_s") <= v("max_speed_m_s"), "narrow_speed");
  require(v("narrow_angular_speed_rad_s") <= v("max_angular_speed_rad_s"), "narrow_angular_speed");
  require(v("narrow_centering_speed_m_s") <= std::min(.05, v("narrow_speed_m_s")), "narrow_centering_speed");
  require(v("narrow_centering_tolerance_m") < v("narrow_centering_max_offset_m") &&
              v("narrow_centering_max_offset_m") <= .3, "narrow_centering_bounds");
  require(v("prediction_step_s") <= v("prediction_horizon_s"), "prediction_step");
  require(v("blocked_confirm_s") < v("wait_budget_s"), "wait_budget");
  require(v("reaction_time_s") >= v("command_timeout_s"), "watchdog_stop_budget");
  require(v("scan_min_valid_fraction") <= 1.0, "scan_min_valid_fraction");
  require(v("velocity_confirmation_s") <= v("velocity_fit_window_s"), "velocity_fit_window");
  for (const auto* key : {"position_tolerance_m", "linear_speed_m_s", "angular_speed_rad_s",
                        "terminal_exclusion_m", "context_timeout_s"}) {
    number(data.at("planning_takeover"), key, true, "planning_takeover.");
  }
  if (data.at("environment") == "simulation") {
    require(simulated, "simulation_profile_requires_sim_time");
  } else {
    require(data.at("hardware_validated").get<bool>(), "hardware_evidence_required");
    const auto& evidence = data.at("hardware_evidence");
    for (const auto* key : {"transport_envelope", "payload", "braking", "latency", "sensor_coverage"}) {
      require(evidence.contains(key) && truthy(evidence.at(key)), "hardware_evidence_required");
    }
  }
}
}  // namespace

nlohmann::json load_policy_profile(const std::filesystem::path& path, bool use_sim_time) {
  auto data = read_object(path);
  if (data.contains("base_profile")) {
    auto base = read_object(path.parent_path() / data.at("base_profile").get<std::string>());
    require(!base.contains("base_profile"), "nested base profiles are unsupported");
    data.erase("base_profile");
    base.update(data);
    data = std::move(base);
  }
  if (data.contains("stop_reference_file")) {
    const auto reference = read_object(path.parent_path() / data.at("stop_reference_file").get<std::string>());
    data.erase("stop_reference_file");
    apply_stop_reference(data, reference);
  }
  validate(data, use_sim_time);
  return data;
}
}  // namespace astribot::navigation
