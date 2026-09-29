#include "astribot_s1_transport_native/fixed_station_scene.hpp"
#include "astribot_s1_transport_native/scene_binding.hpp"
#include <astribot_s1_payload_state/ledger.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace astribot::transport {
namespace {
constexpr const char *base = "astribot_torso_base";
void require(bool okay, const char *reason) {
  if (!okay) throw std::runtime_error(reason);
}
const moveit_msgs::msg::CollisionObject &unique_object(
  const moveit_msgs::msg::PlanningScene &scene, const std::string &id) {
  const auto &objects = scene.world.collision_objects;
  require(std::count_if(objects.begin(), objects.end(), [&](const auto &o) {return o.id == id;}) == 1,
          "STATION_WORLD_OBJECT_NOT_UNIQUE");
  require(std::none_of(scene.robot_state.attached_collision_objects.begin(),
                      scene.robot_state.attached_collision_objects.end(),
                      [&](const auto &o) {return o.object.id == id;}), "STATION_IS_ATTACHED");
  return *std::find_if(objects.begin(), objects.end(), [&](const auto &o) {return o.id == id;});
}
}

geometry_msgs::msg::Pose fixed_station_target_in_base(
  const geometry_msgs::msg::Pose &world_target, const Eigen::Isometry3d &world_from_base) {
  const auto &p = world_target.position;
  const auto &q = world_target.orientation;
  Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
  require(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
          rotation.coeffs().allFinite() && std::abs(rotation.norm() - 1.) <= 1e-6,
          "STATION_WORLD_POSE_INVALID");
  Eigen::Isometry3d world_from_target = Eigen::Isometry3d::Identity();
  world_from_target.translation() = Eigen::Vector3d(p.x, p.y, p.z);
  world_from_target.linear() = rotation.normalized().toRotationMatrix();
  const Eigen::Isometry3d base_from_target = world_from_base.inverse() * world_from_target;
  geometry_msgs::msg::Pose result;
  result.position.x = base_from_target.translation().x();
  result.position.y = base_from_target.translation().y();
  result.position.z = base_from_target.translation().z();
  const Eigen::Quaterniond orientation(base_from_target.linear());
  result.orientation.x = orientation.x();result.orientation.y = orientation.y();
  result.orientation.z = orientation.z();result.orientation.w = orientation.w();
  return result;
}

moveit_msgs::msg::PlanningScene fixed_station_scene_diff(
  const moveit_msgs::msg::PlanningScene &before, const FixedStations &stations,
  const Eigen::Isometry3d &world_from_base, const std::set<std::string> &known_links) {
  (void)bind_scene(before, known_links);
  require(!stations[0].id.empty() && !stations[1].id.empty() && stations[0].id != stations[1].id,
          "STATION_REGISTRATION_REQUIRED");
  moveit_msgs::msg::PlanningScene diff;
  diff.is_diff = true;diff.robot_state.is_diff = true;
  for (const auto &station : stations) {
    auto object = unique_object(before, station.id);
    require(object.primitives.size() == 1 && object.primitive_poses.size() == 1 &&
            object.primitives[0].type == shape_msgs::msg::SolidPrimitive::BOX &&
            object.primitives[0].dimensions.size() == 3 && object.meshes.empty() && object.planes.empty(),
            "STATION_REGISTERED_BOX_REQUIRED");
    for (size_t i = 0; i < 3; ++i)
      require(std::isfinite(station.size_xyz[i]) && station.size_xyz[i] > 0 &&
              object.primitives[0].dimensions[i] == station.size_xyz[i], "STATION_SIZE_CHANGED");
    // The registered BOX is centered at the physical model origin. MoveIt may
    // store its previous world pose in either object.pose or primitive_poses.
    object.primitive_poses[0] = geometry_msgs::msg::Pose();
    object.primitive_poses[0].orientation.w = 1.;
    object.header.frame_id = base;object.header.stamp = builtin_interfaces::msg::Time();
    object.pose = fixed_station_target_in_base(station.world_pose, world_from_base);
    object.operation = object.ADD;
    diff.world.collision_objects.push_back(std::move(object));
  }
  return diff;
}

void validate_fixed_station_scene(
  const moveit_msgs::msg::PlanningScene &before, const moveit_msgs::msg::PlanningScene &after,
  const moveit_msgs::msg::PlanningScene &diff, const std::set<std::string> &known_links) {
  (void)bind_scene(before, known_links);(void)bind_scene(after, known_links);
  require(diff.is_diff && diff.robot_state.is_diff && diff.robot_state.attached_collision_objects.empty() &&
          diff.world.collision_objects.size() == 2, "STATION_DIFF_REQUIRED");
  auto old_other = before, new_other = after;
  std::set<std::string> ids;
  for (const auto &expected : diff.world.collision_objects) {
    require(!expected.id.empty() && ids.insert(expected.id).second && expected.header.frame_id == base &&
            expected.operation == expected.ADD, "STATION_DIFF_INVALID");
    (void)unique_object(before, expected.id);
    const auto &actual = unique_object(after, expected.id);
    // Reuse the existing geometry comparison, including quaternion sign and
    // MoveIt normalization tolerance. These wrappers never enter PlanningScene.
    moveit_msgs::msg::AttachedCollisionObject e, a;
    e.link_name = a.link_name = "astribot_arm_left_tcp_link";
    e.touch_links = a.touch_links = {e.link_name};e.object = expected;a.object = actual;
    e.object.header.frame_id = a.object.header.frame_id = e.link_name;
    require(actual.header.frame_id == base && actual.type == expected.type &&
            actual.subframe_names == expected.subframe_names && actual.subframe_poses == expected.subframe_poses &&
            astribot::payload::scene_matches({e}, {a}, {e.link_name}), "STATION_READBACK_MISMATCH");
    for (auto *scene : {&old_other, &new_other}) {
      auto &objects = scene->world.collision_objects;
      objects.erase(std::remove_if(objects.begin(), objects.end(),
                                  [&](const auto &o) {return o.id == expected.id;}), objects.end());
    }
  }
  // Joint samples and unconsumed fixed TF can advance during the independent
  // service read. Preserve unrelated objects, attachments, ACM and occupancy.
  require(bind_scene(old_other, known_links) == bind_scene(new_other, known_links),
          "STATION_UNRELATED_SCENE_CHANGED");
}
}
