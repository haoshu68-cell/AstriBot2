#include "astribot_navigation_zones/client.hpp"
#include "astribot_s1_navigation_policy_native/final_protection_core.hpp"
#include "astribot_s1_navigation_policy_native/navigation_math.hpp"
#include "astribot_s1_navigation_policy_native/policy_profile.hpp"
#include "astribot_s1_navigation_policy_native/projection_capability.hpp"
#include "astribot_s1_navigation_policy_native/publication_lease.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/msg/speed_limit.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <astribot_navigation_msgs/msg/motion_constraint.hpp>
#include <astribot_perception_msgs/msg/camera_health.hpp>
#include <astribot_perception_msgs/msg/projection_health.hpp>

namespace astribot::navigation {
namespace {
using Constraint = astribot_navigation_msgs::msg::MotionConstraint;
using Scan = sensor_msgs::msg::LaserScan;
using Odom = nav_msgs::msg::Odometry;
using Twist = geometry_msgs::msg::Twist;
using ArmSpeedLimit = nav2_msgs::msg::SpeedLimit;
constexpr double arm_limit_timeout_s = .5;
constexpr double absent = -std::numeric_limits<double>::infinity();
double steady_seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::uint64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
double seconds(const builtin_interfaces::msg::Time& time) {
  return time.sec + time.nanosec * 1e-9;
}
int64_t valid_ns(const builtin_interfaces::msg::Time& t) {
  return t.sec<0||t.nanosec>=1000000000u?0:int64_t(t.sec)*1000000000LL+t.nanosec;
}
std::vector<double> velocity(const Twist& message) {
  return {message.linear.x, message.linear.y, message.angular.z};
}
nlohmann::json age(double value) {
  return std::isfinite(value) ? nlohmann::json(value) : nlohmann::json(nullptr);
}
}  // namespace

class NavigationConstraintNode final : public rclcpp::Node {
public:
  explicit NavigationConstraintNode(const rclcpp::NodeOptions& options=rclcpp::NodeOptions()) : Node("navigation_constraint",options) {
    const char* timing_diagnostics = std::getenv("ASTRIBOT_SCAN_TIMING_DIAGNOSTICS");
    timing_diagnostics_ = timing_diagnostics && std::string(timing_diagnostics) == "1";
    const auto profile_path = declare_parameter<std::string>("profile",
      ament_index_cpp::get_package_share_directory("astribot_s1_navigation_policy") + "/config/simulation.json");
    const auto scan_topic = declare_parameter<std::string>("scan_topic", "/scan_from_cloud");
    rcl_interfaces::msg::ParameterDescriptor command_descriptor;command_descriptor.read_only=true;
    const auto command_topic=declare_parameter<std::string>("command_topic","/cmd_vel_nav_body_raw",command_descriptor);
    rcl_interfaces::msg::ParameterDescriptor arm_descriptor;arm_descriptor.read_only=true;
    require_arm_speed_limit_=declare_parameter("require_arm_speed_limit",false,arm_descriptor);
    const auto mode = declare_parameter<std::string>("navigation_geometry_mode", "legacy");
    if (mode != "legacy" && mode != "fixed_v2") throw std::invalid_argument("invalid navigation_geometry_mode");
    profile_ = std::make_unique<ProtectionProfile>(load_policy_profile(profile_path,
      get_parameter("use_sim_time").as_bool()), mode == "fixed_v2");
    clock_ = std::make_unique<ControlTime>(profile_->simulated(), profile_->value("command_timeout_s"));
    clock_->advance(now().seconds(), steady_seconds());
    tf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, true);
    require_zones_=declare_parameter("require_navigation_zones",false);if(require_zones_){zones_.init(*this);zone_ack_=create_publisher<std_msgs::msg::String>("/navigation_zones/applied",10);}
    constraint_ = create_publisher<Constraint>("/navigation_policy/constraint", 10);
    diagnostics_ = create_publisher<std_msgs::msg::String>("/navigation_policy/constraint_state", 10);
    if(require_arm_speed_limit_) {
      arm_limit_sub_=create_subscription<ArmSpeedLimit>("/navigation_policy/arm_speed_limit",10,
        [this](ArmSpeedLimit::ConstSharedPtr message) {
          const auto stamp=valid_ns(message->header.stamp);
          if(stamp<arm_limit_stamp_ns_)return;
          if(message->header.frame_id!=profile_->base_frame()||!message->percentage||!std::isfinite(message->speed_limit)||
              message->speed_limit<0.||message->speed_limit>100.||stamp<=0) {
            arm_limit_valid_=false;arm_limit_reason_="ARM_SPEED_LIMIT_INVALID";return;
          }
          // Replaying a source sample must not refresh its steady receipt age
          // or clear an invalid update. Recovery needs a newer valid sample.
          if(stamp==arm_limit_stamp_ns_) {
            if(message->speed_limit!=arm_limit_percent_) {
              arm_limit_valid_=false;arm_limit_reason_="ARM_SPEED_LIMIT_INVALID";
            }
            return;
          }
          arm_limit_stamp_ns_=stamp;arm_limit_received_wall_=steady_seconds();
          arm_limit_percent_=message->speed_limit;arm_limit_valid_=true;
          arm_limit_reason_="ARM_SPEED_LIMIT_VALID";
        });
    }
    rcl_interfaces::msg::ParameterDescriptor camera_descriptor;camera_descriptor.read_only=true;
    const auto required_cameras=nlohmann::json::parse(declare_parameter<std::string>("required_projection_cameras","[]",camera_descriptor)).get<std::vector<std::string>>();
    camera_gate_=std::make_unique<ProjectionCapabilityGate>(required_cameras);
    for(const auto& camera:required_cameras) {
      using Raw=astribot_perception_msgs::msg::CameraHealth;
      using Processing=astribot_perception_msgs::msg::ProjectionHealth;
      camera_subscriptions_.push_back(create_subscription<Raw>("/perception/camera_health/"+camera,rclcpp::QoS(4).reliable().transient_local(),
        [this,camera](Raw::ConstSharedPtr h) {
          if(h->camera_id!=camera)return;
          camera_gate_->raw({h->camera_id,h->frame_id,h->source_epoch,h->valid&&h->header.frame_id==h->frame_id,
            valid_ns(h->header.stamp),valid_ns(h->capture_stamp),valid_ns(h->valid_until)},now().nanoseconds(),steady_seconds());
        }));
      camera_subscriptions_.push_back(create_subscription<Processing>("/perception/projection_health/"+camera,rclcpp::QoS(4).reliable().transient_local(),
        [this,camera](Processing::ConstSharedPtr h) {
          if(h->camera_id!=camera)return;
          camera_gate_->processing({h->camera_id,h->header.frame_id,h->processing_epoch,h->valid,
            valid_ns(h->header.stamp),valid_ns(h->capture_stamp),valid_ns(h->valid_until)},now().nanoseconds(),steady_seconds());
        }));
    }
    if (mode == "legacy") {
      envelope_ = create_subscription<ProtectionEnvelope>("/navigation/robot_envelope", 10,
        [this](ProtectionEnvelope::ConstSharedPtr message) {
          try { profile_->accept(*message, now().nanoseconds(), epoch_); }
          catch (const std::invalid_argument& error) { RCLCPP_WARN(get_logger(), "%s", error.what()); }
        });
    } else {
      v2_ = create_subscription<ProtectionEnvelopeV2>("/navigation/envelope_v2", 10,
        [this](ProtectionEnvelopeV2::ConstSharedPtr message) {
          try { profile_->accept(*message, now().nanoseconds(), epoch_); }
          catch (const std::invalid_argument& error) { RCLCPP_WARN(get_logger(), "%s", error.what()); }
        });
    }
    input_ = create_subscription<Twist>(command_topic, 10,
      [this](Twist::ConstSharedPtr message) {
        command_ = velocity(*message); command_at_ = steady_seconds(); command_ros_at_ = now().seconds();
        tick();
      });
    odom_ = create_subscription<Odom>("/odom", rclcpp::SensorDataQoS(),
      [this](Odom::ConstSharedPtr message) {
        if (message->header.stamp.sec < 0 || message->header.stamp.nanosec >= 1000000000u) return;
        const double capture = seconds(message->header.stamp);
        if (!clock_->accepts(capture) || capture<odom_stamp_) return;
        measured_ = velocity(message->twist.twist); odom_at_ = steady_seconds(); odom_stamp_ = capture;
        odom_stamp_ns_ = valid_ns(message->header.stamp);
      });
    scan_ = create_subscription<Scan>(scan_topic, rclcpp::SensorDataQoS(),
      [this](Scan::ConstSharedPtr message) {
        ++scan_received_;
        if (message->header.stamp.sec < 0 || message->header.stamp.nanosec >= 1000000000u) {
          ++scan_invalid_; return;
        }
        const std::vector<double> ranges(message->ranges.begin(), message->ranges.end());
        if (!scan_usable(ranges, message->range_min, message->range_max, message->angle_min,
            message->angle_increment, profile_->value("scan_min_valid_fraction"))) {
          ++scan_invalid_; return;
        }
        pending_.push_back(message);
        if (pending_.size() > 5) pending_.erase(pending_.begin());
        process_scans(now().seconds());
      });
    proposal_sub_ = create_subscription<Constraint>("/navigation_policy/proposed_constraint", 10,
      [this](Constraint::ConstSharedPtr message) {
        if (!clock_->accepts(seconds(message->stamp))) return;
        if (proposal_ && message->epoch == proposal_->epoch && message->sequence <= proposal_->sequence) return;
        if (!std::isfinite(message->lease_s) || !std::isfinite(message->max_linear_speed) ||
            !std::isfinite(message->max_angular_speed) || message->lease_s <= 0. || message->lease_s > .5 ||
            message->max_linear_speed < 0. || message->max_angular_speed < 0.) return;
        proposal_ = message; proposal_at_ = steady_seconds();
      });
    // Steady timer expires navigation evidence even while ROS time is frozen.
    // This node publishes constraints only; it never writes a velocity command.
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { tick(); });
  }

private:
  void process_scans(double ros) {
    std::vector<Scan::ConstSharedPtr> pending;
    for (const auto& message : pending_) {
      const double capture = seconds(message->header.stamp);
      if (!clock_->accepts(capture) || capture <= scan_stamp_) continue;
      try {
        const auto transform = tf_->lookupTransform(profile_->base_frame(), message->header.frame_id,
          tf2::TimePointZero);
        const auto& q = transform.transform.rotation;
        const auto& t = transform.transform.translation;
        std::vector<std::array<double, 2>> points;
        for (std::size_t i = 0; i < message->ranges.size(); ++i) {
          const double range = message->ranges[i];
          if (!std::isfinite(range) || range < message->range_min || range >= message->range_max) continue;
          const double a = message->angle_min + i * static_cast<double>(message->angle_increment);
          const double x = range * std::cos(a), y = range * std::sin(a);
          // Same quaternion rotation as observer.rotate, including full roll/pitch.
          const double tx = -2. * q.z * y, ty = 2. * q.z * x, tz = 2. * (q.x * y - q.y * x);
          points.push_back({x + q.w * tx + q.y * tz - q.z * ty + t.x,
                            y + q.w * ty + q.z * tx - q.x * tz + t.y});
        }
        const double yaw = std::atan2(2. * (q.w * q.z + q.x * q.y), 1. - 2. * (q.y * q.y + q.z * q.z));
        const auto cones = scan_coverage(std::vector<double>(message->ranges.begin(), message->ranges.end()),
          message->range_min, message->range_max, message->angle_min, message->angle_increment, yaw);
        // scan_coverage returns XYZ + half-angle; motion admission consumes XY
        // + half-angle, matching sensor_health.py's BearingCone adapter.
        coverage_.clear();
        for (std::size_t i = 0; i < cones.size(); i += 4) {
          coverage_.insert(coverage_.end(), {cones[i], cones[i + 1], cones[i + 3]});
        }
        points_ = std::move(points); scan_at_ = steady_seconds(); scan_stamp_ = capture;
        scan_stamp_ns_ = valid_ns(message->header.stamp);
        ++scan_transformed_; last_scan_error_.clear();
      } catch (const std::exception& error) {
        ++scan_tf_waits_; last_scan_error_ = error.what(); pending.push_back(message);
      }
    }
    pending_.clear();
    for (const auto& message : pending) if (seconds(message->header.stamp) > scan_stamp_) pending_.push_back(message);
  }

  void tick() {
    const double wall = steady_seconds(); const auto ros = now(); const double seconds_now = ros.seconds();
    const auto timing = clock_->advance(seconds_now, wall);
    process_scans(seconds_now);
    const auto camera_status=camera_gate_->evaluate(ros.nanoseconds(),wall);
    if(camera_gate_->enabled()&&camera_status.revision!=camera_revision_) {
      // Latched faults and recovery invalidate cached navigation evidence.
      // Requested body-frame Twist is only input to sweep prediction, never an output.
      camera_revision_=camera_status.revision;proposal_.reset();command_={0.,0.,0.};
      command_at_=command_ros_at_=absent;clear_at_.reset();
    }
    const auto& p = *profile_;
    const bool requested_command_fresh=clock_->command_fresh(command_ros_at_,command_at_,p.value("input_command_timeout_s"),seconds_now,wall);
    const auto prediction_command=requested_command_fresh?command_:std::vector<double>{0.,0.,0.};
    const bool fresh = clock_->fresh(scan_stamp_, scan_at_, p.value("sensor_timeout_s"), seconds_now, wall) &&
      clock_->fresh(odom_stamp_, odom_at_, p.value("sensor_timeout_s"), seconds_now, wall);
    const bool lease = proposal_ &&
      clock_->fresh(seconds(proposal_->stamp), proposal_at_, proposal_->lease_s, seconds_now, wall);
    const bool workstation_alignment=lease && proposal_->workstation_alignment;
    std::string reason = !fresh ? "INPUT_UNAVAILABLE" : (!lease ? "POLICY_UNAVAILABLE" : proposal_->reason);
    // Clearance is evidence from the independent sensors, not the lifetime of
    // a policy message. A proposal gap requests HOLD without erasing continuous
    // clear observations; actual environment/input faults still restart it.
    bool independent_stop = !fresh;
    if (!p.ready(ros.nanoseconds(), epoch_)) { independent_stop = true; reason = "ROBOT_ENVELOPE_UNAVAILABLE"; }
    if (fresh && (!coverage_allows_motion(coverage_, prediction_command[0], prediction_command[1], prediction_command[2]) ||
                  !coverage_allows_motion(coverage_, measured_[0], measured_[1], measured_[2]))) {
      independent_stop = true; reason = "INDEPENDENT_COVERAGE_UNAVAILABLE";
    }
    // A committed workstation controller checks current layered occupancy and
    // its complete stopping sweep. Only this ordinary 2-D sweep is replaced.
    if (!workstation_alignment && fresh && (protection_swept_collision(points_, prediction_command, p) || protection_swept_collision(points_, measured_, p))) {
      independent_stop = true; reason = "INDEPENDENT_SWEEP_RISK";
    }
    if(require_zones_){
      auto snapshot=zones_.constraints.get();bool installed=bool(snapshot);std::string zone_reason="ZONES_UNAVAILABLE";
      if(snapshot)try{
        auto transform=tf_->lookupTransform("map",p.base_frame(),tf2::TimePointZero);
        const auto&q=transform.transform.rotation;double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
        double radius=std::hypot(p.value("half_length_m"),p.value("half_width_m"));
        for(const auto&point:p.polygon())radius=std::max(radius,std::hypot(point[0],point[1]));
        radius+=p.value("clearance_margin_m")+p.value("payload_extra_margin_m");
        bool risk=false;for(const auto&v:{prediction_command,measured_}){
          double horizon=stopping_horizon(v,p.value("reaction_time_s"),p.value("brake_deceleration_m_s2"),p.value("angular_brake_deceleration_rad_s2"),p.value("linear_stop_delay_s"));
          risk=risk||astribot_navigation_zones::sweptCircleBlocked(snapshot->regions,transform.transform.translation.x,transform.transform.translation.y,yaw,v[0],v[1],v[2],radius,horizon);
        }
        if(risk){independent_stop=true;reason="ZONES_SWEEP_RISK";}zone_reason="ZONES_APPLIED";
      }catch(...){installed=false;zone_reason="ZONES_TF_OR_GEOMETRY_UNAVAILABLE";}
      if(snapshot)zone_ack_->publish(astribot_navigation_zones::acknowledgement(*snapshot,"navigation_constraint",installed,zone_reason));
      if(!installed||!zones_.ready()){independent_stop=true;reason=installed?"ZONES_WAIT_APPLICATION":zone_reason;}
    }
    if(!camera_status.ready){independent_stop=true;reason=camera_status.reason;}
    if (independent_stop) clear_at_.reset();
    else if (!clear_at_) clear_at_ = wall;
    bool stop = independent_stop || !lease || (proposal_ && proposal_->hold);
    if (!stop && wall - *clear_at_ < p.value("clear_hold_s")) { stop = true; reason = "NAVIGATION_CLEAR_CONFIRMATION"; }
    const double cap = lease ? std::min(p.value("max_speed_m_s"), proposal_->max_linear_speed) : 0.;
    const double angular_cap = lease ? std::min(p.value("max_angular_speed_rad_s"),proposal_->max_angular_speed) : 0.;
    // Re-anchor only after the expensive protection/zone calculations. A new
    // wire stamp must not renew the policy evidence behind an older proposal.
    const auto publication_anchor = now();
    const auto publication_wall = steady_seconds();
    const bool arm_fresh=!require_arm_speed_limit_||arm_limit_valid_;
    // This dedicated percentage topic uses zero as HOLD, not the generic
    // Nav2 SpeedLimit convention where zero can mean no limit.
    if(require_arm_speed_limit_&&(!arm_fresh||arm_limit_percent_==0.)&&!stop) {
      stop=true;
      reason=!arm_limit_valid_?arm_limit_reason_:(!arm_fresh?"ARM_SPEED_LIMIT_STALE":"ARM_SPEED_LIMIT_ZERO");
    }
    const double arm_factor=require_arm_speed_limit_?arm_limit_percent_/100.:1.;
    const auto proposal_stamp_ns = proposal_ ? valid_ns(proposal_->stamp) : 0;
    ++sequence_;
    Constraint c; c.stamp = publication_anchor; c.epoch = epoch_; c.sequence = sequence_;
    c.lease_s = p.value("constraint_lease_s");
    c.hold = stop; c.reason = reason; c.max_linear_speed = stop ? 0. : cap*arm_factor;
    c.max_angular_speed = stop ? 0. : angular_cap*arm_factor; c.planning = !stop && lease ? proposal_->planning : 0;
    c.alignment_required = !stop && lease && proposal_->alignment_required;
    c.centering_required = !stop && lease && proposal_->centering_required;
    c.corridor_tracking_required = !stop && lease && proposal_->corridor_tracking_required;
    c.workstation_alignment = workstation_alignment;
    const auto constraint_publish_start_ns = now().nanoseconds();
    constraint_->publish(c);
    if (timing_diagnostics_ || sequence_ % 5 == 0) {
      nlohmann::json data{{"reason", reason}, {"hold", stop},
        {"workstation_alignment",workstation_alignment},
        {"constraint_epoch",c.epoch},{"constraint_sequence",c.sequence},
        {"proposal_epoch",proposal_ ? nlohmann::json(proposal_->epoch) : nlohmann::json(nullptr)},
        {"proposal_sequence",proposal_ ? nlohmann::json(proposal_->sequence) : nlohmann::json(nullptr)},
        {"proposal_stamp_ns",proposal_ ? nlohmann::json(proposal_stamp_ns) : nlohmann::json(nullptr)},
        {"publication_stamp_ns",publication_anchor.nanoseconds()},
        {"publication_wall_s",publication_wall},{"publication_inputs_available",fresh},
        {"scan_capture_ns",scan_stamp_ns_},{"odom_capture_ns",odom_stamp_ns_},
        {"effective_lease_s",c.lease_s},{"time_contract","latest_observation"},
        {"constraint_publish_start_ns",constraint_publish_start_ns},
        {"arm_speed_limit_required",require_arm_speed_limit_},{"arm_speed_limit_fresh",arm_fresh},
        {"arm_speed_limit_percent",arm_limit_valid_?nlohmann::json(arm_limit_percent_):nlohmann::json(nullptr)},
        {"arm_speed_limit_stamp_ns",arm_limit_stamp_ns_},
        {"projection_ready",camera_status.ready},{"projection_reason",camera_status.reason},
        {"projection_revision",camera_status.revision},{"projection_released_ns",camera_status.released_ns},
        {"projection_future_samples_rejected",camera_status.future_samples_rejected},
        {"scan_age_s", age(seconds_now - scan_stamp_)}, {"scan_wall_age_s", age(wall - scan_at_)},
        {"odom_age_s", age(seconds_now - odom_stamp_)}, {"odom_wall_age_s", age(wall - odom_at_)},
        {"policy_wall_age_s", age(wall - proposal_at_)}, {"loop_wall_dt_s", timing.wall_dt},
        {"loop_control_dt_s", timing.dt}, {"time_domain", clock_->simulated() ? "simulation" : "wall"},
        {"clock_running", timing.running}, {"points", points_.size()}, {"scan_received", scan_received_},
        {"scan_transformed", scan_transformed_}, {"scan_invalid", scan_invalid_}, {"scan_tf_waits", scan_tf_waits_},
        {"scan_expired", scan_expired_}, {"pending_scans", pending_.size()}, {"last_scan_error", last_scan_error_},
        {"requested_command_fresh",requested_command_fresh},
        {"requested_command_speed_m_s",age(std::hypot(command_[0],command_[1]))},
        {"measured_speed_m_s",age(std::hypot(measured_[0],measured_[1]))},
        {"constraint_max_linear_speed",c.max_linear_speed},{"constraint_max_angular_speed",c.max_angular_speed}};
      std_msgs::msg::String diagnostic; diagnostic.data = data.dump(); diagnostics_->publish(diagnostic);
    }
  }

  bool timing_diagnostics_{false};
  bool require_arm_speed_limit_{false},arm_limit_valid_{false};
  std::int64_t arm_limit_stamp_ns_{0};double arm_limit_received_wall_{absent},arm_limit_percent_{0.};
  std::string arm_limit_reason_{"ARM_SPEED_LIMIT_MISSING"};
  rclcpp::Subscription<ArmSpeedLimit>::SharedPtr arm_limit_sub_;
  bool require_zones_{false};astribot_navigation_zones::Gate zones_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr zone_ack_;
  std::unique_ptr<ProjectionCapabilityGate> camera_gate_;uint64_t camera_revision_{0};
  std::vector<rclcpp::SubscriptionBase::SharedPtr> camera_subscriptions_;
  std::unique_ptr<ProtectionProfile> profile_;
  std::unique_ptr<ControlTime> clock_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::vector<double> command_{0., 0., 0.}, measured_{0., 0., 0.}, coverage_;
  std::vector<std::array<double, 2>> points_;
  double command_at_{absent}, command_ros_at_{absent}, odom_at_{absent}, odom_stamp_{absent};
  double scan_at_{absent}, scan_stamp_{absent}, proposal_at_{absent};
  std::int64_t scan_stamp_ns_{-1}, odom_stamp_ns_{-1};
  Constraint::ConstSharedPtr proposal_;
  std::optional<double> clear_at_;
  std::uint64_t sequence_{0}, epoch_{steady_ns()};
  std::uint64_t scan_received_{0}, scan_transformed_{0}, scan_invalid_{0}, scan_tf_waits_{0}, scan_expired_{0};
  std::string last_scan_error_;
  std::vector<Scan::ConstSharedPtr> pending_;
  rclcpp::Publisher<Constraint>::SharedPtr constraint_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_;
  rclcpp::Subscription<Twist>::SharedPtr input_;
  rclcpp::Subscription<Odom>::SharedPtr odom_;
  rclcpp::Subscription<Scan>::SharedPtr scan_;
  rclcpp::Subscription<Constraint>::SharedPtr proposal_sub_;
  rclcpp::Subscription<ProtectionEnvelope>::SharedPtr envelope_;
  rclcpp::Subscription<ProtectionEnvelopeV2>::SharedPtr v2_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace astribot::navigation

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<astribot::navigation::NavigationConstraintNode>();
    rclcpp::spin(node); node.reset(); rclcpp::shutdown();
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("navigation_constraint"), "%s", error.what());
    rclcpp::shutdown(); return 1;
  }
  return 0;
}
