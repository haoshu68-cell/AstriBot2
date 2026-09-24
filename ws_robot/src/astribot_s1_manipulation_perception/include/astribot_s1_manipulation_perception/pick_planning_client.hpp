#pragma once

#include <astribot_perception_msgs/action/compute_grasps.hpp>
#include <astribot_perception_msgs/action/estimate_object_pose.hpp>
#include <astribot_transport_msgs/action/plan_manipulation.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <functional>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

namespace astribot::perception_planning {
using Grasps = astribot_perception_msgs::action::ComputeGrasps;
using Pose = astribot_perception_msgs::action::EstimateObjectPose;
using Plan = astribot_transport_msgs::action::PlanManipulation;
using Candidate = astribot_perception_msgs::msg::GraspCandidate;

// Identity is assigned by the caller's object inventory, NEVER from a YOLO
// per-frame hypothesis or a class label. Revision changes invalidate this job.
struct Context {
  std::string object_instance, identity_revision, camera_id, optical_frame;
  std::string source_epoch, processing_epoch, model_id;
  std::string pose_model_revision, grasp_model_revision;
  uint64_t calibration_revision{}, scene_revision{}, envelope_epoch{}, clock_epoch{};
  // M1's canonical signature of the actual complete scene, refreshed by current().
  std::string scene_signature;
  // Actual CameraInfo content, excluding its changing capture stamp. The RGB-D
  // source sets this; older explicitly supplied cloud providers leave it empty.
  std::string camera_info_revision;
  std::string station_region_revision;
};

struct GraspMapping {
  // T_grasp_tcp for THIS candidate's depth and actual gripper closure.
  geometry_msgs::msg::Transform grasp_from_tcp;
  double physical_object_width_m{}; // excludes GraspNet's opening margin
  double commanded_joint_angle_rad{}; // registration evidence; MTC independently computes the same q
};

struct Request {
  Context context;
  std::string task_id, context_id, detection_id;
  // First supported input is an explicitly isolated single-object fixture.
  // The caller supplies its segmented cloud; this library does not segment boxes.
  std::string segmentation_source;
  uint32_t visible_instances{};
  sensor_msgs::msg::PointCloud2 object_cloud;
  sensor_msgs::msg::CameraInfo capture_camera_info;
  geometry_msgs::msg::TransformStamped capture_station_from_camera;
  builtin_interfaces::msg::Time valid_until;
  // Anchored by the original sensor receiver, never reset by a later caller.
  std::chrono::steady_clock::time_point admission_deadline_steady{}, result_deadline_steady{};
  geometry_msgs::msg::TransformStamped base_from_camera; // exact capture stamp
  // Registered CAD geometry in model_id; shape poses relative to geometry.pose.
  moveit_msgs::msg::CollisionObject model_geometry;
  std::string geometry_revision;
  moveit_msgs::msg::PlanningScene scene; // full, authoritative M1 snapshot
  // Caller-owned registered geometry adapter. Native grasp origin depends on
  // candidate depth; TCP offset depends on closure. Never assume a constant
  // transform or send GraspNet's margin-expanded opening as object width.
  std::function<GraspMapping(const Candidate &, const Pose::Result &)> map_grasp;
  std::string grasp_registration_revision, grasp_registration_evidence;
  double minimum_width_m{}, maximum_width_m{}, approach_m{}, lift_m{};
  std::vector<std::string> touch_links;
  uint32_t max_candidates{8};
  double inference_timeout_s{4.}, planning_timeout_s{4.};
};

struct InferenceGoals { Pose::Goal pose; Grasps::Goal grasps; };
// Boundary functions throw std::runtime_error with an explicit reason code.
void check_context(const Request &, const Context &current, int64_t now_ns,
                   bool admission = false);
InferenceGoals inference_goals(const Request &, const Context &current, int64_t now_ns);
Plan::Goal planning_goal(const Request &, const Pose::Result &, const Candidate &,
                         const Context &current, int64_t now_ns, GraspMapping *mapping_used = nullptr);
void check_plan(const Plan::Goal &, const Plan::Result &);

struct PlannedPick {
  Context context;
  std_msgs::msg::Header capture_header;
  builtin_interfaces::msg::Time valid_until;
  std::chrono::steady_clock::time_point result_deadline_steady{};
  Candidate candidate;
  Pose::Result object_pose;
  Plan::Goal request;
  Plan::Result plan;
  GraspMapping mapping;
  std::string geometry_revision, grasp_registration_revision, grasp_registration_evidence;
};
struct Outcome {
  bool success{false};
  // True only when every dispatched Action has a terminal result/rejection.
  // Cancel acceptance alone does not count. False forbids client reuse.
  bool terminal_confirmed{true};
  std::string reason;
  std::vector<std::string> candidate_rejections;
  std::optional<PlannedPick> pick;
};

// Blocking planning-only client. Use a dedicated, otherwise unspun node, on an
// M1 worker thread. M1's executor must keep context/clock/health updates running.
// Endpoints must be separate pose-only and grasp-only service instances.
// No ExecuteTrajectory, controller, attach/detach or scene-apply calls exist here.
class PickPlanningClient {
public:
  explicit PickPlanningClient(rclcpp::Node::SharedPtr dedicated_node);
  ~PickPlanningClient();
  PickPlanningClient(const PickPlanningClient &) = delete;
  PickPlanningClient &operator=(const PickPlanningClient &) = delete;
  bool ready() const;
  Outcome plan(const Request &, const std::function<Context()> &current,
               const std::function<bool()> &cancel_requested);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace astribot::perception_planning
