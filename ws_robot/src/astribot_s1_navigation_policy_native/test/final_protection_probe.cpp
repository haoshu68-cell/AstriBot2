// Test-only line-oriented process boundary; the production core has no JSON RPC.
#include "astribot_s1_navigation_policy_native/final_protection_core.hpp"
#include "astribot_s1_navigation_policy_native/policy_profile.hpp"
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  const auto baseline = astribot::navigation::load_policy_profile(argv[1], true);
  std::string line;
  while (std::getline(std::cin, line)) {
    try {
      const auto input = nlohmann::json::parse(line);
      astribot::navigation::ProtectionProfile profile(baseline, input.contains("polygon"));
      if (input.contains("polygon")) {
        astribot::navigation::ProtectionEnvelopeV2 message;
        message.header.frame_id = baseline.at("base_frame"); message.header.stamp.sec = 10;
        message.valid_until.sec = 10; message.valid_until.nanosec = 300000000;
        message.coordinator_session_id = "probe"; message.hold_id = "hold";
        message.mode = message.FIXED_POSTURE; message.epoch = 1; message.navigation_allowed = true;
        auto& e = message.limits;
        e.stamp = message.header.stamp; e.frame_id = message.header.frame_id;
        e.posture_id = "probe"; e.epoch = 1; e.lease_s = .3; e.transport_ready = true;
        e.half_length_m = baseline.at("half_length_m"); e.half_width_m = baseline.at("half_width_m");
        e.height_m = baseline.at("height_m"); e.payload_mass_kg = baseline.at("payload_mass_kg");
        e.max_speed_m_s = baseline.at("max_speed_m_s"); e.max_angular_speed_rad_s = baseline.at("max_angular_speed_rad_s");
        e.max_acceleration_m_s2 = baseline.at("max_acceleration_m_s2"); e.brake_deceleration_m_s2 = baseline.at("brake_deceleration_m_s2");
        message.clearance_m = baseline.at("clearance_margin_m");
        astribot_s1_robot_geometry::Polygon2 polygon;
        for (const auto& row : input.at("polygon")) {
          geometry_msgs::msg::Point32 point; point.x = row[0]; point.y = row[1];
          message.reserved_footprint.points.push_back(point); polygon.push_back({point.x, point.y});
        }
        auto inflated = astribot_s1_robot_geometry::inflatePolygon(polygon, message.clearance_m + 2e-6);
        for (const auto& row : inflated) {
          geometry_msgs::msg::Point32 point; point.x = row[0]; point.y = row[1];
          message.installed_footprint.points.push_back(point);
        }
        inflated.clear();
        for (const auto& point : message.installed_footprint.points) inflated.push_back({point.x, point.y});
        message.installed_geometry_hash = astribot_s1_robot_geometry::geometryHash(inflated, message.header.frame_id, message.clearance_m);
        if (!profile.accept(message, 10000000000LL, 1)) return 3;
      }
      const auto command = input.at("command").get<std::vector<double>>();
      const auto points = input.at("points").get<std::vector<std::array<double, 2>>>();
      std::cout << nlohmann::json{{"collision", astribot::navigation::protection_swept_collision(points, command, profile)}}.dump() << std::endl;
    } catch (const std::exception& error) {
      std::cout << nlohmann::json{{"error", error.what()}}.dump() << std::endl;
    }
  }
}
