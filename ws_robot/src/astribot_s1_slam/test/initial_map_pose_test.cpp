#include "initial_map_pose.hpp"
#include <gtest/gtest.h>
#include <limits>

using astribot::slam::initialMapChassisPose;
using astribot::slam::gravityRotationPreservingYaw;
namespace {
double yaw(const Eigen::Matrix3d& r) {return std::atan2(r(1,0), r(0,0));}
double angle_difference(double a, double b) {return std::atan2(std::sin(a-b), std::cos(a-b));}
}

TEST(InitialMapPose, MeasuredChassisRegistersPoseAndWorldPointsTogether)
{
  const double q = std::sqrt(.5);
  const auto map_chassis = initialMapChassisPose({.25,.15,.1292,0.,0.,q,q}, false);
  ASSERT_TRUE(map_chassis);
  Eigen::Isometry3d chassis_imu = Eigen::Isometry3d::Identity();
  chassis_imu.linear() = Eigen::AngleAxisd(-.8, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  chassis_imu.translation() = Eigen::Vector3d(.17393,.16893,.082);
  const Eigen::Isometry3d map_imu = *map_chassis * chassis_imu;
  EXPECT_TRUE((map_imu * chassis_imu.inverse()).matrix().isApprox(map_chassis->matrix(), 1e-12));
  const Eigen::Vector3d imu_point(.7,-.2,.9);
  EXPECT_TRUE((map_imu * imu_point).isApprox(*map_chassis * (chassis_imu * imu_point),1e-12));
  EXPECT_NEAR(yaw(map_imu.linear() * chassis_imu.linear().transpose()),std::acos(-1.)/2.,1e-12);
  EXPECT_NEAR((map_imu * chassis_imu.inverse()).translation().z(),.1292,1e-12);
}

TEST(InitialMapPose, GravityAlignmentKeepsRegisteredChassisOriginAndHeading)
{
  const Eigen::Matrix3d chassis_rotation =
    (Eigen::AngleAxisd(1.5708,Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(.03,Eigen::Vector3d::UnitY()) *
     Eigen::AngleAxisd(-.02,Eigen::Vector3d::UnitX())).toRotationMatrix();
  const Eigen::Vector3d gravity(.15,-.21,-9.79);
  const Eigen::Matrix3d correction = gravityRotationPreservingYaw(gravity,chassis_rotation);
  const Eigen::Vector3d aligned_gravity = correction * gravity;
  EXPECT_NEAR(aligned_gravity.x(),0.,1e-12);
  EXPECT_NEAR(aligned_gravity.y(),0.,1e-12);
  EXPECT_NEAR(angle_difference(yaw(correction * chassis_rotation),yaw(chassis_rotation)),0.,1e-12);
  const Eigen::Vector3d imu_position(.4,.3,.21), imu_chassis_offset(-.15,-.1,-.08);
  const Eigen::Vector3d chassis_origin = imu_position + chassis_rotation * imu_chassis_offset;
  const Eigen::Vector3d corrected_imu = correction * (imu_position-chassis_origin) + chassis_origin;
  EXPECT_TRUE((corrected_imu + correction*chassis_rotation*imu_chassis_offset).isApprox(chassis_origin,1e-12));
  const Eigen::Vector3d local_point(.3,.2,1.);
  EXPECT_TRUE((corrected_imu + correction*chassis_rotation*local_point).isApprox(
    correction*(imu_position+chassis_rotation*local_point-chassis_origin)+chassis_origin,1e-12));
}

TEST(InitialMapPose, LevelGravityAndPiHeadingRemainFinite)
{
  for(double heading : {0.,3.141592653589793,-3.141592653589793}) {
    const Eigen::Matrix3d r=Eigen::AngleAxisd(heading,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const auto correction=gravityRotationPreservingYaw(Eigen::Vector3d(0,0,-9.81),r);
    EXPECT_TRUE(correction.allFinite());
    EXPECT_TRUE(correction.isApprox(Eigen::Matrix3d::Identity(),1e-12));
  }
}

TEST(InitialMapPose, LocalMappingUnchangedAndLoadedMapCannotDoubleRegister)
{
  EXPECT_FALSE(initialMapChassisPose({},false));
  EXPECT_FALSE(initialMapChassisPose({},true));
  EXPECT_THROW(initialMapChassisPose({0,0,0,0,0,0,1},true),std::invalid_argument);
  EXPECT_THROW(initialMapChassisPose({0,0,0,0},false),std::invalid_argument);
  EXPECT_THROW(initialMapChassisPose({0,0,0,0,0,0,2},false),std::invalid_argument);
  EXPECT_THROW(initialMapChassisPose({std::numeric_limits<double>::quiet_NaN(),0,0,0,0,0,1},false),std::invalid_argument);
}
