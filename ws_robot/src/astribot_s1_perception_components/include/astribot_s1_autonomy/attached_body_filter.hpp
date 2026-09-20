#pragma once
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>
#include <vector>
#include "geometry_msgs/msg/pose.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace astribot_s1_autonomy {
inline Eigen::Isometry3d rigidPose(const geometry_msgs::msg::Pose & p) {
  Eigen::Quaterniond q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
  if(!q.coeffs().allFinite() || std::abs(q.norm()-1.)>1e-3)
    throw std::invalid_argument("invalid attached-body quaternion");
  Eigen::Isometry3d t=Eigen::Isometry3d::Identity();t.linear()=q.normalized().toRotationMatrix();
  t.translation()<<p.position.x,p.position.y,p.position.z;
  if(!t.matrix().allFinite()) throw std::invalid_argument("invalid attached-body position");
  return t;
}
struct AttachedBody {
  shape_msgs::msg::SolidPrimitive primitive;
  Eigen::Isometry3d from_base;
  static void validate(const shape_msgs::msg::SolidPrimitive & p) {
    const auto n=p.type==p.BOX?3U:p.type==p.SPHERE?1U:p.type==p.CYLINDER?2U:0U;
    if(!n || p.dimensions.size()!=n) throw std::invalid_argument("unsupported attached-body primitive");
    for(double d:p.dimensions) if(!std::isfinite(d)||d<=0)
      throw std::invalid_argument("invalid attached-body dimension");
  }
  bool contains(double x,double y,double z) const {
    const Eigen::Vector3d p=from_base*Eigen::Vector3d(x,y,z);
    // Simulation lidar range resolution is 1 cm. Reserve half a range cell
    // plus 1 mm numerical allowance per entity, never the whole robot hull.
    constexpr double e=.006;
    const auto & d=primitive.dimensions;
    if(primitive.type==primitive.BOX)
      return std::abs(p.x())<=d[0]/2+e && std::abs(p.y())<=d[1]/2+e && std::abs(p.z())<=d[2]/2+e;
    if(primitive.type==primitive.SPHERE)return p.norm()<=d[0]+e;
    return std::hypot(p.x(),p.y())<=d[1]+e && std::abs(p.z())<=d[0]/2+e;
  }
};
}
