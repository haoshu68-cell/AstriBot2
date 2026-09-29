#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <vector>

namespace astribot::slam {

// A measured map<-chassis pose at estimator startup, not a spawn command or
// an ongoing replacement for LiDAR/IMU localization. Empty starts a new map.
inline std::optional<Eigen::Isometry3d> initialMapChassisPose(
  const std::vector<double>& values, bool loading_map)
{
  if(values.empty()) return std::nullopt;
  if(loading_map) throw std::invalid_argument(
    "General.initial_chassis_pose cannot be combined with General.previous_map");
  if(values.size() != 7) throw std::invalid_argument(
    "General.initial_chassis_pose requires x,y,z,qx,qy,qz,qw");
  for(double value : values) if(!std::isfinite(value))
    throw std::invalid_argument("General.initial_chassis_pose must be finite");
  const Eigen::Quaterniond q(values[6], values[3], values[4], values[5]);
  if(std::abs(q.squaredNorm() - 1.) > 1e-6)
    throw std::invalid_argument("General.initial_chassis_pose requires a unit quaternion");
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = q.normalized().toRotationMatrix();
  pose.translation() = Eigen::Vector3d(values[0], values[1], values[2]);
  return pose;
}

// Gravity initialization may correct roll/pitch, but its world-frame gauge
// must retain the measured chassis heading. Rotation is applied around the
// first chassis origin by the estimator, preserving its map translation too.
inline Eigen::Matrix3d gravityRotationPreservingYaw(
  const Eigen::Vector3d& gravity, const Eigen::Matrix3d& chassis_rotation)
{
  const Eigen::Vector3d vertical(0., 0., gravity.z() < 0. ? -1. : 1.);
  const Eigen::Matrix3d tilt =
    Eigen::Quaterniond::FromTwoVectors(gravity, vertical).toRotationMatrix();
  const Eigen::Matrix3d tilted_chassis = tilt * chassis_rotation;
  const double delta_yaw = std::atan2(chassis_rotation(1,0), chassis_rotation(0,0)) -
    std::atan2(tilted_chassis(1,0), tilted_chassis(0,0));
  return Eigen::AngleAxisd(delta_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * tilt;
}

} // namespace astribot::slam
