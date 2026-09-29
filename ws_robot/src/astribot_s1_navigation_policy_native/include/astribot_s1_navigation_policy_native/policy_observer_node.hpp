#pragma once
#include "astribot_s1_navigation_policy_native/policy_observer_core.hpp"
#include "astribot_s1_navigation_policy_native/policy_envelope.hpp"
#include "astribot_s1_navigation_policy_native/policy_fusion.hpp"
#include "astribot_s1_navigation_policy_native/policy_health.hpp"
#include "astribot_s1_navigation_policy_native/policy_observation_adapters.hpp"
#include "astribot_s1_navigation_policy_native/policy_risk.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <astribot_navigation_msgs/msg/sensor_health_array.hpp>
#include <astribot_navigation_msgs/msg/envelope_apply_status.hpp>
#include <atomic>
#include <deque>
#include <mutex>

namespace astribot::navigation::policy {
class PolicyObserverNode : public rclcpp::Node {
 public:
  explicit PolicyObserverNode(const rclcpp::NodeOptions& options=rclcpp::NodeOptions(),
                              bool observation_only=true);
 protected:
  using Scan=sensor_msgs::msg::LaserScan;
  using Envelope=PolicyEnvelopeProfile::Envelope;
  using EnvelopeV2=PolicyEnvelopeProfile::EnvelopeV2;
  struct Source {
    std::unique_ptr<ObservationAdapter> adapter;
    bool requires_calibration=false;
    AdapterJson configuration=AdapterJson::object();
  };
  virtual void tick();
  virtual void process_plan();
  Stamp stamp();
  RiskProfile risk_profile() const;
  void process_envelope();
  void process_scans(const Stamp& now);
  void process_scan(const Scan& message);
  void scan(const Scan::ConstSharedPtr& message);
  void odom(const nav_msgs::msg::Odometry& message);
  void mapping(const nav_msgs::msg::OccupancyGrid& message);
  void accept_observations(Source& source,const std::function<std::vector<Observation>()>& normalize);
  void camera_info(Source& source,const sensor_msgs::msg::CameraInfo& message);
  void envelope(const Envelope::ConstSharedPtr& message);
  void envelope(const EnvelopeV2::ConstSharedPtr& message);
  void ack(const EnvelopeV2& message);
  AdapterTransform adapter_transform(const std::string& target,const std::string& source,std::int64_t capture_ns);
  const bool observation_only_;
  std::string clock_id_,base_frame_;
  std::unique_ptr<PolicyEnvelopeProfile> profile_;
  std::unique_ptr<ConservativeFusion> fusion_;
  std::unique_ptr<SensorHealthRegistry> health_registry_;
  CameraCalibrationRegistry calibrations_;
  navigation::ExecutionContext execution_;
  ObserverMap map_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::CallbackGroup::SharedPtr processing_group_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr observation_pub_;
  rclcpp::Publisher<astribot_navigation_msgs::msg::SensorHealthArray>::SharedPtr health_pub_;
  rclcpp::Publisher<astribot_navigation_msgs::msg::EnvelopeApplyStatus>::SharedPtr ack_pub_;
  std::vector<std::unique_ptr<Source>> sources_;
  std::unique_ptr<Source> vision_source_;
  std::mutex scan_mutex_,odom_mutex_,plan_mutex_,envelope_mutex_,profile_mutex_;
  std::deque<Scan::ConstSharedPtr> pending_scans_;
  nav_msgs::msg::Path::ConstSharedPtr pending_plan_;
  std::variant<std::monostate,Envelope::ConstSharedPtr,EnvelopeV2::ConstSharedPtr> pending_envelope_;
  std::shared_ptr<const RobotState> robot_,last_robot_;
  std::shared_ptr<const WorldSnapshot> last_world_;
  std::optional<Risk> last_risk_;
  std::optional<double> odom_at_,scan_at_;
  Polygon path_;
  std::atomic<std::int64_t> epoch_{0};
  std::optional<std::int64_t> last_time_;
  std::int64_t errors_=0,scan_count_=0,vision_count_=0,path_revision_=0,last_evaluation_epoch_=0;
  bool last_inputs_valid_=false;
  std::string last_error_;
};
}  // namespace astribot::navigation::policy
