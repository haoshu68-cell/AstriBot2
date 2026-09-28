#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include "astribot_perception_msgs/msg/grasp_candidate_array.hpp"
#include "astribot_perception_msgs/msg/grasp_candidate.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

builtin_interfaces::msg::Time time_message(const rclcpp::Time & time)
{
  builtin_interfaces::msg::Time message;
  const auto nanoseconds = time.nanoseconds();
  message.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  message.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  return message;
}

rclcpp::Time ros_time(const builtin_interfaces::msg::Time & message)
{
  const auto nanoseconds = static_cast<int64_t>(message.sec) * 1000000000LL +
    static_cast<int64_t>(message.nanosec);
  return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
}

bool is_zero_time(const builtin_interfaces::msg::Time & message)
{
  return message.sec == 0 && message.nanosec == 0;
}

bool finite_pose(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  const double quaternion_norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
    std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
    std::isfinite(quaternion_norm) && std::abs(quaternion_norm - 1.0) <= 1.0e-3;
}

}  // namespace

class GraspCandidateGate final : public rclcpp::Node
{
public:
  explicit GraspCandidateGate(const rclcpp::NodeOptions & options=rclcpp::NodeOptions())
  : Node("grasp_candidate_gate",options)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/perception/grasp_candidates");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/manipulation/valid_grasp_candidates");
    min_score_ = declare_parameter<double>("min_score", 0.20);
    max_candidate_age_sec_ = declare_parameter<double>("max_candidate_age_sec", 0.50);
    min_gripper_width_m_ = declare_parameter<double>("min_gripper_width_m", 0.0);
    max_gripper_width_m_ = declare_parameter<double>("max_gripper_width_m", 0.12);
    rcl_interfaces::msg::ParameterDescriptor context_descriptor;
    context_descriptor.read_only=true;
    context_descriptor.description="Pinned screening context; restart with new context. MTC must revalidate before execution.";
    const auto calibration = declare_parameter<int64_t>("required_calibration_revision", 0, context_descriptor);
    required_calibration_revision_ = calibration > 0 ? static_cast<uint64_t>(calibration) : 0;
    required_planning_scene_revision_ = static_cast<uint64_t>(declare_parameter<int64_t>(
      "required_planning_scene_revision", 0, context_descriptor));
    required_envelope_epoch_ = static_cast<uint64_t>(declare_parameter<int64_t>(
      "required_envelope_epoch", 0, context_descriptor));
    require_collision_checked_ = declare_parameter<bool>("require_collision_checked", true);
    require_collision_free_ = declare_parameter<bool>("require_collision_free", true);
    require_geometry_valid_ = declare_parameter<bool>("require_geometry_valid", true);

    if (!std::isfinite(min_score_) || min_score_<0 || min_score_>1 ||
        !std::isfinite(max_candidate_age_sec_) || max_candidate_age_sec_<=0 ||
        !std::isfinite(min_gripper_width_m_) || !std::isfinite(max_gripper_width_m_) ||
        min_gripper_width_m_<0 || max_gripper_width_m_<=min_gripper_width_m_ ||
        get_parameter("required_planning_scene_revision").as_int()<0 ||
        get_parameter("required_envelope_epoch").as_int()<0 || calibration<0 ||
        !require_collision_checked_ || !require_collision_free_ || !require_geometry_valid_)
      throw std::invalid_argument("invalid grasp candidate gate configuration");
    // Volatile: replaying a previously accepted candidate must never be a start signal.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    publisher_ = create_publisher<astribot_perception_msgs::msg::GraspCandidateArray>(
      output_topic_, qos);
    subscription_ = create_subscription<astribot_perception_msgs::msg::GraspCandidateArray>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&GraspCandidateGate::on_candidates, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Grasp candidate gate listening on %s and publishing %s (score >= %.3f, age <= %.3fs)",
      input_topic_.c_str(), output_topic_.c_str(), min_score_, max_candidate_age_sec_);
  }

private:
  std::string validate(
    const astribot_perception_msgs::msg::GraspCandidate & candidate,
    const rclcpp::Time & now) const
  {
    if (!required_planning_scene_revision_ || !required_envelope_epoch_ || !required_calibration_revision_)
      return "VALIDATION_CONTEXT_MISSING";
    if (!candidate.calibration_revision || candidate.calibration_revision!=required_calibration_revision_)
      return "CALIBRATION_REVISION_MISMATCH";
    if (candidate.camera_id.empty() || candidate.model_name.empty() || candidate.model_revision.empty())
      return "PROVENANCE_MISSING";
    if (candidate.header.frame_id.empty() || candidate.header.frame_id != candidate.grasp_pose.header.frame_id ||
        candidate.header.stamp != candidate.grasp_pose.header.stamp)
      return "POSE_CONTEXT_MISMATCH";
    if (candidate.candidate_id.empty() || candidate.object_id.empty() || candidate.arm_id.empty()) {
      return "IDENTITY_MISSING";
    }
    if (candidate.grasp_pose.header.frame_id.empty() || !finite_pose(candidate.grasp_pose)) {
      return "POSE_INVALID";
    }
    if (!std::isfinite(candidate.score) || candidate.score < min_score_ || candidate.score > 1.) {
      return "SCORE_LOW";
    }
    if (!std::isfinite(candidate.gripper_width_m) ||
      candidate.gripper_width_m < min_gripper_width_m_ ||
      candidate.gripper_width_m > max_gripper_width_m_)
    {
      return "GRIPPER_WIDTH_INVALID";
    }
    if (!std::isfinite(candidate.gripper_height_m) || candidate.gripper_height_m <= 0.0 ||
      !std::isfinite(candidate.gripper_depth_m) || candidate.gripper_depth_m <= 0.0)
    {
      return "GRIPPER_GEOMETRY_INVALID";
    }
    if (is_zero_time(candidate.valid_until)) {
      return "VALID_UNTIL_MISSING";
    }
    if (candidate.header.stamp.sec<0 || candidate.header.stamp.nanosec>=1000000000u ||
        candidate.valid_until.sec<0 || candidate.valid_until.nanosec>=1000000000u ||
        is_zero_time(candidate.header.stamp)) return "STAMP_INVALID";
    const auto valid_until = ros_time(candidate.valid_until);
    const auto age = (now - rclcpp::Time(candidate.header.stamp, RCL_ROS_TIME)).seconds();
    if (!std::isfinite(age) || age < 0.0 || age > max_candidate_age_sec_) {
      return "CANDIDATE_STALE";
    }
    if (valid_until <= now) {
      return "CANDIDATE_EXPIRED";
    }
    if (require_geometry_valid_ && !candidate.geometry_valid) {
      return "GEOMETRY_UNCONFIRMED";
    }
    if (require_collision_checked_ && !candidate.collision_checked) {
      return "COLLISION_UNCHECKED";
    }
    if (require_collision_free_ && !candidate.collision_free) {
      return "COLLISION_BLOCKED";
    }
    if (required_planning_scene_revision_ != 0 &&
      candidate.planning_scene_revision != required_planning_scene_revision_)
    {
      return "PLANNING_SCENE_MISMATCH";
    }
    if (required_envelope_epoch_ != 0 && candidate.envelope_epoch != required_envelope_epoch_) {
      return "ENVELOPE_EPOCH_MISMATCH";
    }
    return "";
  }

  void on_candidates(
    const astribot_perception_msgs::msg::GraspCandidateArray::SharedPtr message)
  {
    const auto now = get_clock()->now();
    astribot_perception_msgs::msg::GraspCandidateArray output;
    output.header.stamp = time_message(now);
    output.header.frame_id = message->header.frame_id;
    output.task_id = message->task_id;
    output.source_epoch = message->source_epoch;
    output.source_frame = message->source_frame;

    std::string first_rejection;
    output.candidates.reserve(message->candidates.size());
    for (const auto & candidate : message->candidates) {
      const auto rejection = message->task_id.empty() || message->source_epoch.empty() ?
        std::string("TASK_CONTEXT_MISSING") :
        (message->source_frame.empty() || message->header.frame_id!=message->source_frame ||
        candidate.header.frame_id!=message->source_frame) ? std::string("ARRAY_FRAME_MISMATCH") : validate(candidate, now);
      if (rejection.empty()) {
        output.candidates.push_back(candidate);
      } else if (first_rejection.empty()) {
        first_rejection = rejection;
      }
    }

    if (output.candidates.empty()) {
      output.reason_code = first_rejection.empty() ? "NO_CANDIDATE" : first_rejection;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No grasp candidate passed the safety gate: %s", output.reason_code.c_str());
    } else {
      output.reason_code = "CANDIDATES_SCREENED_REQUIRES_MTC_VALIDATION";
    }
    publisher_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  double min_score_{0.20};
  double max_candidate_age_sec_{0.50};
  double min_gripper_width_m_{0.0};
  double max_gripper_width_m_{0.12};
  uint64_t required_calibration_revision_{0};
  uint64_t required_planning_scene_revision_{0};
  uint64_t required_envelope_epoch_{0};
  bool require_collision_checked_{true};
  bool require_collision_free_{true};
  bool require_geometry_valid_{true};
  rclcpp::Subscription<astribot_perception_msgs::msg::GraspCandidateArray>::SharedPtr subscription_;
  rclcpp::Publisher<astribot_perception_msgs::msg::GraspCandidateArray>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GraspCandidateGate>());
  rclcpp::shutdown();
  return 0;
}
