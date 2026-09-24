#pragma once
#include <astribot_s1_manipulation_perception/pick_planning_client.hpp>
#include <astribot_perception_msgs/msg/camera_health.hpp>
#include <astribot_perception_msgs/msg/projection_health.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>

namespace astribot::perception_planning {
// M1 owns this assertion about an exclusive, single-instance orange BOX station.
// A connected color region is not persistent object identification.
struct SingleBoxFixture {
  std::string station_id, object_instance, identity_revision;
  uint32_t confirmed_instances{};
  builtin_interfaces::msg::Time issued_at, valid_until;
  // Authoritative workstation volume, not an object pose or estimated box ROI.
  std::string station_frame, region_revision;
  geometry_msgs::msg::Point region_min_m, region_max_m;
};

struct SensorReceipt {
  int64_t ros_ns{};
  std::chrono::steady_clock::time_point steady{};
};
struct RgbdFrame {
  sensor_msgs::msg::Image::ConstSharedPtr color;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr points;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
  astribot_perception_msgs::msg::CameraHealth camera;
  astribot_perception_msgs::msg::ProjectionHealth projection;
  // Both clocks at receipt. Late delivery consumes the original source budget;
  // pausing ROS time cannot restore that consumed budget.
  SensorReceipt color_received, points_received, info_received;
  SensorReceipt camera_received, camera_capture_received;
  SensorReceipt projection_received, projection_capture_received;
};

std::string camera_info_revision(const sensor_msgs::msg::CameraInfo&);
// Existing rgbd_pointcloud_node provider ONLY: decimation=1, flattened full
// image with NaNs preserved, xyz FLOAT32 at offsets 0/4/8, point_step=16.
// No simulation entity pose, 6D seed, learned detector, or motion call is used.
Request single_box_request(Request task, const SingleBoxFixture&, const RgbdFrame&,
                           const geometry_msgs::msg::TransformStamped& capture_tf,
                           const geometry_msgs::msg::TransformStamped& station_from_camera,
                           int64_t now_ns);

struct SingleBoxTopics {
  std::string camera_id{"head_rgbd"};
  std::string color{"/camera/raw/head_rgbd/image"};
  std::string info{"/camera/raw/head_rgbd/camera_info"};
  std::string points{"/manipulation/single_box/head_rgbd/points"};
  std::string camera_health{"/perception/camera_health/head_rgbd"};
  std::string projection_health{"/perception/projection_health/single_box/head_rgbd"};
};

// Attach to M1's already-spinning node. capture() runs on its worker thread;
// the executor must continue delivering sensor/TF/context updates. The owner
// supplies the existing TF buffer and all task/scene/geometry/mapping fields.
// Destroy the source after quiescing that executor's callbacks.
class SingleBoxRequestSource {
public:
  SingleBoxRequestSource(rclcpp::Node::SharedPtr, tf2_ros::Buffer&, SingleBoxTopics = {});
  ~SingleBoxRequestSource();
  SingleBoxRequestSource(const SingleBoxRequestSource&) = delete;
  SingleBoxRequestSource& operator=(const SingleBoxRequestSource&) = delete;
  // Fill actual sensor fields; require the owner's calibration version to match.
  // Use this in PickPlanningClient::plan's current callback as well.
  Context bind_context(Context owner_context);
  Request capture(Request task, const SingleBoxFixture&,
                  const std::function<Context()>& current);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace astribot::perception_planning
