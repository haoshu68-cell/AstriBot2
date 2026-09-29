#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "astribot_s1_navigation_policy_native/policy_profile.hpp"
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <astribot_navigation_msgs/msg/robot_envelope.hpp>
#include <astribot_navigation_msgs/srv/set_robot_envelope.hpp>

namespace astribot::navigation {
namespace {

using Envelope = astribot_navigation_msgs::msg::RobotEnvelope;

constexpr double kLeaseSeconds = 0.3;
constexpr int64_t kFootprintFreshNs = 500000000LL;
constexpr int64_t kOdomFreshNs = 300000000LL;
constexpr double kLinearStop = 0.02;
constexpr double kAngularStop = 0.03;

struct Profile {
  std::string environment;
  std::string base_frame;
  bool hardware_validated{false};
  double half_length_m{0.0};
  double half_width_m{0.0};
  double height_m{0.0};
  double payload_mass_kg{0.0};
  double max_speed_m_s{0.0};
  double max_angular_speed_rad_s{0.0};
  double max_acceleration_m_s2{0.0};
  double brake_deceleration_m_s2{0.0};
};

Profile load_profile(const std::string& filename, bool use_sim_time) {
  const auto data = load_policy_profile(filename, use_sim_time);
  Profile profile;
  profile.environment = data.at("environment").get<std::string>();
  profile.base_frame = data.at("base_frame").get<std::string>();
  profile.hardware_validated = data.at("hardware_validated").get<bool>();
  profile.half_length_m = data.at("half_length_m").get<double>();
  profile.half_width_m = data.at("half_width_m").get<double>();
  profile.height_m = data.at("height_m").get<double>();
  profile.payload_mass_kg = data.at("payload_mass_kg").get<double>();
  profile.max_speed_m_s = data.at("max_speed_m_s").get<double>();
  profile.max_angular_speed_rad_s = data.at("max_angular_speed_rad_s").get<double>();
  profile.max_acceleration_m_s2 = data.at("max_acceleration_m_s2").get<double>();
  profile.brake_deceleration_m_s2 = data.at("brake_deceleration_m_s2").get<double>();
  return profile;
}

int64_t stamp_ns(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<int64_t>(stamp.sec) * 1000000000LL +
         static_cast<int64_t>(stamp.nanosec);
}

builtin_interfaces::msg::Time now_msg(rclcpp::Clock& clock) {
  const int64_t nanoseconds = clock.now().nanoseconds();
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}

double edge_length(const geometry_msgs::msg::Point32& a,
                   const geometry_msgs::msg::Point32& b) {
  return std::hypot(static_cast<double>(b.x) - a.x,
                    static_cast<double>(b.y) - a.y);
}

class EnvelopeCoordinatorNode final : public rclcpp::Node {
public:
  EnvelopeCoordinatorNode() : Node("robot_envelope_coordinator") {
    declare_parameter<std::string>("profile", "");
    declare_parameter<std::string>("navigation_geometry_mode", "legacy");
    if (!has_parameter("use_sim_time")) declare_parameter("use_sim_time", false);
    const auto mode = get_parameter("navigation_geometry_mode").as_string();
    if (mode != "legacy") {
      throw std::invalid_argument(
          "envelope_coordinator_cpp supports navigation_geometry_mode:=legacy only");
    }
    profile_ = load_profile(get_parameter("profile").as_string(),
                            get_parameter("use_sim_time").as_bool());
    envelope_.epoch = static_cast<uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
    envelope_.lease_s = kLeaseSeconds;
    envelope_.frame_id = profile_.base_frame;
    envelope_.posture_id = profile_.environment == "simulation"
                               ? "simulation_transport"
                               : "validated_transport";
    envelope_.half_length_m = profile_.half_length_m;
    envelope_.half_width_m = profile_.half_width_m;
    envelope_.height_m = profile_.height_m;
    envelope_.payload_mass_kg = profile_.payload_mass_kg;
    envelope_.max_speed_m_s = profile_.max_speed_m_s;
    envelope_.max_angular_speed_rad_s = profile_.max_angular_speed_rad_s;
    envelope_.max_acceleration_m_s2 = profile_.max_acceleration_m_s2;
    envelope_.brake_deceleration_m_s2 = profile_.brake_deceleration_m_s2;
    desired_ready_ = true;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, false);
    publisher_ = create_publisher<Envelope>("/navigation/robot_envelope", 10);
    for (const auto& name : {std::string("global_costmap"), std::string("local_costmap")}) {
      footprints_.emplace(name, create_publisher<geometry_msgs::msg::Polygon>(
                                  "/" + name + "/footprint", 1));
      footprint_subscriptions_.push_back(create_subscription<geometry_msgs::msg::PolygonStamped>(
          "/" + name + "/published_footprint", 10,
          [this, name](geometry_msgs::msg::PolygonStamped::SharedPtr message) {
            acknowledge(name, message);
          }));
    }
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(),
        std::bind(&EnvelopeCoordinatorNode::on_odom, this, std::placeholders::_1));
    service_ = create_service<astribot_navigation_msgs::srv::SetRobotEnvelope>(
        "/navigation/set_robot_envelope",
        std::bind(&EnvelopeCoordinatorNode::propose, this, std::placeholders::_1,
                  std::placeholders::_2));
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.1),
                                  std::bind(&EnvelopeCoordinatorNode::tick, this));
    RCLCPP_INFO(get_logger(),
                "envelope_coordinator_cpp started in legacy mode: frame=%s, profile=%s",
                profile_.base_frame.c_str(), profile_.environment.c_str());
  }

private:
  bool stopped() const {
    return linear_speed_ <= kLinearStop && angular_speed_ <= kAngularStop;
  }

  bool stopped_with_fresh_odom(int64_t now_ns) const {
    return odom_ns_ >= 0 && now_ns >= odom_ns_ && now_ns - odom_ns_ <= kOdomFreshNs &&
           stopped();
  }

  static bool validate_envelope(const Envelope& candidate, const Profile& baseline,
                                std::string& reason) {
    if (candidate.posture_id.empty() || candidate.frame_id != baseline.base_frame) {
      reason = "invalid posture/frame";
      return false;
    }
    if (!std::isfinite(candidate.lease_s) || candidate.lease_s <= 0.0 ||
        candidate.lease_s > 0.5) {
      reason = "invalid envelope lease";
      return false;
    }
    const std::vector<std::pair<const char*, double>> fields = {
        {"half_length_m", candidate.half_length_m},
        {"half_width_m", candidate.half_width_m},
        {"height_m", candidate.height_m},
        {"payload_mass_kg", candidate.payload_mass_kg},
        {"max_speed_m_s", candidate.max_speed_m_s},
        {"max_angular_speed_rad_s", candidate.max_angular_speed_rad_s},
        {"max_acceleration_m_s2", candidate.max_acceleration_m_s2},
        {"brake_deceleration_m_s2", candidate.brake_deceleration_m_s2}};
    for (const auto& [name, value] : fields) {
      if (!std::isfinite(value) || value < 0.0 ||
          (std::string(name) != "payload_mass_kg" && value == 0.0)) {
        reason = name;
        return false;
      }
    }
    if (candidate.half_length_m < baseline.half_length_m) {
      reason = "envelope cannot undercut baseline: half_length_m";
      return false;
    }
    if (candidate.half_width_m < baseline.half_width_m) {
      reason = "envelope cannot undercut baseline: half_width_m";
      return false;
    }
    if (candidate.height_m < baseline.height_m) {
      reason = "envelope cannot undercut baseline: height_m";
      return false;
    }
    if (candidate.max_speed_m_s > baseline.max_speed_m_s) {
      reason = "unvalidated limit increase: max_speed_m_s";
      return false;
    }
    if (candidate.max_angular_speed_rad_s > baseline.max_angular_speed_rad_s) {
      reason = "unvalidated limit increase: max_angular_speed_rad_s";
      return false;
    }
    if (candidate.max_acceleration_m_s2 > baseline.max_acceleration_m_s2) {
      reason = "unvalidated limit increase: max_acceleration_m_s2";
      return false;
    }
    if (candidate.brake_deceleration_m_s2 > baseline.brake_deceleration_m_s2) {
      reason = "unvalidated limit increase: brake_deceleration_m_s2";
      return false;
    }
    return true;
  }

  geometry_msgs::msg::Polygon polygon() const {
    geometry_msgs::msg::Polygon result;
    const float x = static_cast<float>(envelope_.half_length_m);
    const float y = static_cast<float>(envelope_.half_width_m);
    for (const auto& pair : std::vector<std::pair<float, float>>{{x, y}, {-x, y},
                                                                   {-x, -y}, {x, -y}}) {
      geometry_msgs::msg::Point32 point;
      point.x = pair.first;
      point.y = pair.second;
      point.z = 0.0F;
      result.points.push_back(point);
    }
    return result;
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr message) {
    const auto& twist = message->twist.twist;
    linear_speed_ = std::hypot(twist.linear.x, twist.linear.y);
    angular_speed_ = std::abs(twist.angular.z);
    odom_ns_ = stamp_ns(message->header.stamp);
  }

  void acknowledge(const std::string& name,
                   const geometry_msgs::msg::PolygonStamped::SharedPtr message) {
    const int64_t now_ns = get_clock()->now().nanoseconds();
    const int64_t source_ns = stamp_ns(message->header.stamp);
    if (source_ns < changed_ns_ || now_ns < source_ns ||
        now_ns - source_ns > kFootprintFreshNs || message->polygon.points.size() != 4) {
      return;
    }
    const auto& points = message->polygon.points;
    std::vector<double> edges;
    edges.reserve(4);
    for (size_t i = 0; i < points.size(); ++i) {
      edges.push_back(edge_length(points[i], points[(i + 1) % points.size()]));
    }
    // Reject before sorting: NaN has no strict weak ordering and must revoke
    // an earlier acknowledgement just as the reference interval check does.
    if (!std::all_of(edges.begin(), edges.end(), [](double edge) { return std::isfinite(edge); })) {
      acks_.erase(name);
      return;
    }
    std::sort(edges.begin(), edges.end());
    std::vector<double> expected = {2.0 * envelope_.half_length_m,
                                    2.0 * envelope_.half_length_m,
                                    2.0 * envelope_.half_width_m,
                                    2.0 * envelope_.half_width_m};
    std::sort(expected.begin(), expected.end());
    for (size_t i = 0; i < edges.size(); ++i) {
      if (edges[i] < expected[i] - 0.002 || edges[i] > expected[i] + 0.08) {
        acks_.erase(name);
        return;
      }
    }
    // Match the legacy coordinator: acknowledgement requires a stopped sample;
    // the service path additionally requires fresh odometry.
    if (acks_.count(name) != 0U || !stopped()) return;
    try {
      const auto transform = tf_buffer_->lookupTransform(
          envelope_.frame_id, message->header.frame_id,
          rclcpp::Time(message->header.stamp));
      std::vector<geometry_msgs::msg::Point> transformed;
      transformed.reserve(points.size());
      for (const auto& point : points) {
        geometry_msgs::msg::PointStamped source;
        source.header = message->header;
        source.point.x = point.x;
        source.point.y = point.y;
        source.point.z = point.z;
        geometry_msgs::msg::PointStamped target;
        tf2::doTransform(source, target, transform);
        transformed.push_back(target.point);
      }
      const auto target = polygon().points;
      for (const auto& corner : target) {
        bool found = false;
        for (const auto& point : transformed) {
          if (std::abs(point.x - corner.x) < 0.04 &&
              std::abs(point.y - corner.y) < 0.04) {
            found = true;
            break;
          }
        }
        if (!found) {
          acks_.erase(name);
          return;
        }
      }
      const auto minmax_x = std::minmax_element(
          transformed.begin(), transformed.end(),
          [](const auto& a, const auto& b) { return a.x < b.x; });
      const auto minmax_y = std::minmax_element(
          transformed.begin(), transformed.end(),
          [](const auto& a, const auto& b) { return a.y < b.y; });
      if (minmax_x.first->x > -envelope_.half_length_m + 0.001 ||
          minmax_x.second->x < envelope_.half_length_m - 0.001 ||
          minmax_y.first->y > -envelope_.half_width_m + 0.001 ||
          minmax_y.second->y < envelope_.half_width_m - 0.001) {
        acks_.erase(name);
        return;
      }
      acks_.insert(name);
    } catch (const std::exception&) {
      // Time conversion can also throw (e.g. a negative sec field). Such input,
      // like unavailable TF, must leave this side pending, not exit the node.
      acks_.erase(name);
    }
  }

  void propose(
      const std::shared_ptr<astribot_navigation_msgs::srv::SetRobotEnvelope::Request> request,
      std::shared_ptr<astribot_navigation_msgs::srv::SetRobotEnvelope::Response> response) {
    const int64_t now_ns = get_clock()->now().nanoseconds();
    std::string reason;
    if (!validate_envelope(request->envelope, profile_, reason)) {
      response->accepted = false;
      response->reason = reason;
      return;
    }
    if (!stopped_with_fresh_odom(now_ns)) {
      response->accepted = false;
      response->reason = "ROBOT_MUST_BE_STOPPED_WITH_FRESH_ODOMETRY";
      return;
    }
    const uint64_t next_epoch = envelope_.epoch + 1U;
    envelope_ = request->envelope;
    envelope_.epoch = next_epoch;
    desired_ready_ = request->envelope.transport_ready;
    envelope_.transport_ready = false;
    acks_.clear();
    changed_ns_ = now_ns;
    response->accepted = true;
    response->epoch = envelope_.epoch;
    response->reason = "WAITING_FOR_BOTH_COSTMAPS";
    tick();
  }

  void tick() {
    envelope_.stamp = now_msg(*get_clock());
    const bool ready = desired_ready_ && acks_.size() == 2U;
    envelope_.transport_ready = ready;
    envelope_.reason = ready ? "TRANSPORT_READY" : "POSTURE_OR_FOOTPRINT_PENDING";
    publisher_->publish(envelope_);
    if (acks_.size() < 2U) {
      const auto shape = polygon();
      for (const auto& [name, publisher] : footprints_) {
        (void)name;
        publisher->publish(shape);
      }
    }
  }

  Profile profile_;
  Envelope envelope_;
  bool desired_ready_{false};
  int64_t changed_ns_{0};
  int64_t odom_ns_{-1};
  double linear_speed_{std::numeric_limits<double>::infinity()};
  double angular_speed_{std::numeric_limits<double>::infinity()};
  std::set<std::string> acks_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<Envelope>::SharedPtr publisher_;
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr> footprints_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr>
      footprint_subscriptions_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Service<astribot_navigation_msgs::srv::SetRobotEnvelope>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace astribot::navigation

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<astribot::navigation::EnvelopeCoordinatorNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("robot_envelope_coordinator"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
