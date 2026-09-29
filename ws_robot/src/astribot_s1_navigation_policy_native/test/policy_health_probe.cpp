// Standalone JSON process boundary, used only by the frozen-oracle tests.
#include "astribot_s1_navigation_policy_native/policy_health.hpp"

#include "astribot_s1_navigation_policy_native/policy_integer_json.hpp"
#include <array>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace p = astribot::navigation::policy;
using Json = p::IntegerJson;

namespace {
double number(const Json& input) {
  if (input.is_number()) return input.get<double>();
  const auto text = input.get<std::string>();
  if (text == "nan") return std::numeric_limits<double>::quiet_NaN();
  if (text == "inf") return std::numeric_limits<double>::infinity();
  if (text == "-inf") return -std::numeric_limits<double>::infinity();
  throw std::invalid_argument("invalid test number");
}

p::Stamp stamp(const Json& input) {
  return p::Stamp(input.at("ns"), input.at("clock"), input.at("epoch"));
}
Json json(const p::Stamp& input) {
  return {{"ns", input.ns}, {"clock", input.clock}, {"epoch", input.epoch}};
}
Json json(const p::BearingCone& input) {
  return {{"direction", {{"x", input.direction.x}, {"y", input.direction.y}, {"z", input.direction.z}}},
          {"half_angle_rad", input.half_angle_rad}};
}
Json json(const std::vector<p::BearingCone>& input) {
  auto out = Json::array();
  for (const auto& cone : input) out.push_back(json(cone));
  return out;
}
Json json(const p::SensorHealth& input) {
  return {{"sensor_id", input.sensor_id}, {"health", p::to_string(input.health)},
          {"stamp", json(input.stamp)}, {"valid_until", json(input.valid_until)},
          {"frame_id", input.frame_id}, {"coverage", json(input.coverage)},
          {"depth_available", input.depth_available}, {"calibration_epoch", input.calibration_epoch},
          {"reason", input.reason}};
}
Json json(const p::CameraCalibration& input) {
  return {{"camera_id", input.camera_id}, {"optical_frame", input.optical_frame},
          {"calibration_epoch", input.calibration_epoch}, {"image_width_px", input.image_width_px},
          {"image_height_px", input.image_height_px}, {"intrinsic_matrix", input.intrinsic_matrix},
          {"distortion_model", input.distortion_model},
          {"distortion_coefficients", input.distortion_coefficients},
          {"depth_unit_m", input.depth_unit_m ? Json(*input.depth_unit_m) : Json(nullptr)}};
}

std::vector<p::BearingCone> cones(const Json& input) {
  std::vector<p::BearingCone> result;
  for (const auto& cone : input) {
    const auto& direction = cone.at("direction");
    result.emplace_back(p::Vec3(number(direction.at("x")), number(direction.at("y")), number(direction.at("z"))),
                        number(cone.at("half_angle_rad")));
  }
  return result;
}

p::CameraCalibration camera(const Json& input) {
  // Shape is a property of this test wire format. The production C++ API is a
  // fixed-size array, so it cannot receive a wrong-size intrinsic matrix.
  if (input.at("intrinsic_matrix").size() != 9) throw std::invalid_argument("invalid test matrix shape");
  std::array<double, 9> intrinsic{};
  for (std::size_t i = 0; i < intrinsic.size(); ++i) intrinsic[i] = number(input.at("intrinsic_matrix")[i]);
  std::vector<double> distortion;
  for (const auto& n : input.at("distortion_coefficients")) distortion.push_back(number(n));
  std::optional<double> depth;
  if (!input.at("depth_unit_m").is_null()) depth = number(input.at("depth_unit_m"));
  return p::CameraCalibration(input.at("camera_id"), input.at("optical_frame"),
      p::json_integer(input.at("calibration_epoch")), p::json_integer(input.at("image_width_px")), p::json_integer(input.at("image_height_px")),
      intrinsic, input.at("distortion_model"), distortion, depth);
}

Json execute(const Json& input) {
  p::SensorHealthRegistry registry(input.value("timeout", 1e-7),
      input.value("required", std::set<std::string>{"scan"}));
  p::CameraCalibrationRegistry cameras;
  auto output = Json::array();
  for (const auto& op : input.at("ops")) {
    try {
      const auto name = op.at("op").get<std::string>();
      Json result;
      if (name == "record") {
        // Match oracle construction order before the mutating registry call.
        const auto coverage = cones(op.at("coverage"));
        const auto capture = stamp(op.at("capture"));
        const auto now = stamp(op.at("now"));
        result = registry.record(op.at("sensor"), capture, now, op.at("frame"),
                                 coverage, op.at("depth"), p::json_integer(op.at("calibration_epoch")));
      } else if (name == "health") {
        result = Json::array();
        for (const auto& h : registry.health(stamp(op.at("now")))) result.push_back(json(h));
      } else if (name == "required_valid") {
        result = registry.required_valid(stamp(op.at("now")));
      } else if (name == "allows") {
        result = registry.allows(stamp(op.at("now")), op.at("directions").get<std::vector<double>>());
      } else if (name == "allows_motion") {
        const auto& velocity = op.at("velocity");
        result = registry.allows_motion(stamp(op.at("now")), velocity[0], velocity[1], velocity[2]);
      } else if (name == "directions") {
        const auto& velocity = op.at("velocity");
        result = p::movement_directions(velocity[0], velocity[1], velocity[2]);
      } else if (name == "coverage_motion") {
        const auto& velocity = op.at("velocity");
        result = p::coverage_allows_motion(cones(op.at("coverage")), velocity[0], velocity[1], velocity[2]);
      } else if (name == "scan") {
        const auto& params = op.at("parameters");
        std::vector<double> ranges;
        for (const auto& r : op.at("ranges")) ranges.push_back(number(r));
        result = json(p::scan_coverage(ranges, params[0], params[1], params[2], params[3], params[4]));
      } else if (name == "calibrate") {
        cameras.register_calibration(camera(op.at("calibration")));
      } else if (name == "calibration") {
        result = json(cameras.calibration(op.at("camera_id"), p::json_integer(op.at("epoch"))));
      } else {
        throw std::invalid_argument("unknown test operation");
      }
      output.push_back({{"value", result}});
    } catch (const std::out_of_range& error) {
      output.push_back({{"error", "'" + std::string(error.what()) + "'"}});
    } catch (const std::exception& error) {
      output.push_back({{"error", error.what()}});
    }
  }
  return output;
}
}  // namespace

int main() {
  std::string line;
  while (std::getline(std::cin, line)) {
    try {
      std::cout << p::dump_integer_json(execute(p::parse_integer_json(line))) << '\n';
    } catch (const std::exception& error) {
      std::cerr << error.what() << '\n';
      return 1;
    }
  }
}
