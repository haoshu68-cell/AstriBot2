#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

namespace astribot_s1_transport_mtc {
// Select an existing support for a supplied PLACE target. Conservative geometry
// proximity is not proof of nominal physical contact; execution still requires
// the existing physical placement/readback checks.
inline std::string selectPlaceSupport(const moveit_msgs::msg::PlanningScene& scene,
    const geometry_msgs::msg::PoseStamped& target, const std::string& object_id,
    double placement_tolerance_m) {
  if(!std::isfinite(placement_tolerance_m) || placement_tolerance_m<=0. || placement_tolerance_m>.025)
    throw std::runtime_error("PLACE_SUPPORT_TOLERANCE_INVALID");
  if(target.header.frame_id!="astribot_torso_base")
    throw std::runtime_error("PLACE_SUPPORT_TARGET_FRAME_INVALID");
  const auto pose=[](const geometry_msgs::msg::Pose& value) {
    Eigen::Isometry3d result=Eigen::Isometry3d::Identity();
    const auto& q=value.orientation;
    const Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
    result.translation()=Eigen::Vector3d(value.position.x,value.position.y,value.position.z);
    if(!result.translation().allFinite() || !rotation.coeffs().allFinite() ||
       std::abs(rotation.squaredNorm()-1.)>.001)
      throw std::runtime_error("PLACE_SUPPORT_POSE_INVALID");
    result.linear()=rotation.normalized().toRotationMatrix();return result;
  };
  const auto half=[](const moveit_msgs::msg::CollisionObject& object) {
    if(object.primitives.size()!=1 || object.primitive_poses.size()!=1 ||
       object.primitives.front().type!=shape_msgs::msg::SolidPrimitive::BOX ||
       object.primitives.front().dimensions.size()!=3 || !object.meshes.empty() || !object.planes.empty())
      throw std::runtime_error("PLACE_SUPPORT_BOX_REQUIRED");
    const auto& dimensions=object.primitives.front().dimensions;
    Eigen::Vector3d result(dimensions[0]/2.,dimensions[1]/2.,dimensions[2]/2.);
    if(!result.allFinite() || (result.array()<=0.).any())
      throw std::runtime_error("PLACE_SUPPORT_BOX_INVALID");
    return result;
  };
  const auto& bodies=scene.robot_state.attached_collision_objects;
  const auto body=std::find_if(bodies.begin(),bodies.end(),[&](const auto& value){return value.object.id==object_id;});
  if(body==bodies.end() || body->link_name!="astribot_arm_left_tcp_link" ||
     body->object.header.frame_id!=body->link_name)
    throw std::runtime_error("PLACE_SUPPORT_ATTACHMENT_REQUIRED");
  const auto object_half=half(body->object);
  const Eigen::Isometry3d base_object=pose(target.pose)*pose(body->object.pose)*pose(body->object.primitive_poses.front());
  std::string selected;
  for(const auto& id:std::array<std::string,2>{object_id+"_pick_station",object_id+"_place_station"}) {
    const auto& objects=scene.world.collision_objects;
    const auto station=std::find_if(objects.begin(),objects.end(),[&](const auto& value){return value.id==id;});
    if(station==objects.end())continue;
    if(station->header.frame_id!=target.header.frame_id)
      throw std::runtime_error("PLACE_SUPPORT_STATION_FRAME_INVALID");
    const auto station_half=half(*station);
    const Eigen::Isometry3d local=(pose(station->pose)*pose(station->primitive_poses.front())).inverse()*base_object;
    const Eigen::Vector3d extent=local.linear().cwiseAbs()*object_half;
    const auto& center=local.translation();
    const double supported_center_z=station_half.z()+extent.z();
    if(std::abs(center.x())+extent.x()>station_half.x() ||
       std::abs(center.y())+extent.y()>station_half.y() || center.z()<=station_half.z() ||
       center.z()<supported_center_z-placement_tolerance_m ||
       center.z()>supported_center_z+placement_tolerance_m)continue;
    if(!selected.empty())throw std::runtime_error("PLACE_SUPPORT_AMBIGUOUS");
    selected=id;
  }
  if(selected.empty())throw std::runtime_error("PLACE_SUPPORT_NOT_FOUND");
  return selected;
}
}  // namespace astribot_s1_transport_mtc
