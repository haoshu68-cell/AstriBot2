#include <gtest/gtest.h>
#include "astribot_s1_transport_native/trajectory_digest.hpp"
#include <cmath>
#include <limits>
#include <utility>

using astribot::transport::trajectory_digest;

namespace {
trajectory_msgs::msg::JointTrajectory trajectory() {
  trajectory_msgs::msg::JointTrajectory result;
  result.header.frame_id = "astribot_torso_base";
  result.header.stamp.sec = 17;
  result.header.stamp.nanosec = 29;
  result.joint_names = {"joint_a", "joint_b"};
  trajectory_msgs::msg::JointTrajectoryPoint first;
  first.positions = {.1, -.2};
  first.velocities = {.3, -.4};
  first.accelerations = {.5, -.6};
  first.effort = {.7, -.8};
  first.time_from_start.sec = 1;
  first.time_from_start.nanosec = 23;
  auto second = first;
  second.positions = {.2, -.3};
  second.time_from_start.sec = 2;
  result.points = {first, second};
  return result;
}
}  // namespace

TEST(TrajectoryDigest, SameMessageIsDeterministicAndUnmodified) {
  const auto original = trajectory();
  auto copy = original;
  const auto digest = trajectory_digest(copy);
  EXPECT_EQ(digest.size(), 64u);
  EXPECT_EQ(digest.find_first_not_of("0123456789abcdef"), std::string::npos);
  EXPECT_EQ(digest, trajectory_digest(original));
  EXPECT_EQ(digest, trajectory_digest(copy));
  EXPECT_EQ(copy, original);
}

TEST(TrajectoryDigest, EveryPointFieldAndAdjacentDoubleChangeDigest) {
  const auto original = trajectory();
  const auto digest = trajectory_digest(original);
  auto changed = original;
  changed.points[0].positions[0] = std::nextafter(changed.points[0].positions[0], 1.);
  EXPECT_NE(digest, trajectory_digest(changed));
  changed = original;
  changed.points[1].velocities[0] += .01;
  EXPECT_NE(digest, trajectory_digest(changed));
  changed = original;
  changed.points[1].accelerations[0] += .01;
  EXPECT_NE(digest, trajectory_digest(changed));
  changed = original;
  changed.points[1].effort[0] += .01;
  EXPECT_NE(digest, trajectory_digest(changed));
}

TEST(TrajectoryDigest, IntegerDurationAndEachHeaderFieldChangeDigest) {
  const auto original = trajectory();
  const auto digest = trajectory_digest(original);
  auto changed = original;
  ++changed.points[1].time_from_start.sec;
  EXPECT_NE(digest, trajectory_digest(changed));
  changed = original;
  ++changed.points[1].time_from_start.nanosec;
  EXPECT_NE(digest, trajectory_digest(changed));
  changed = original;
  changed.header.frame_id += "_other";
  EXPECT_NE(digest, trajectory_digest(changed));
  changed = original;
  ++changed.header.stamp.sec;
  EXPECT_NE(digest, trajectory_digest(changed));
  changed = original;
  ++changed.header.stamp.nanosec;
  EXPECT_NE(digest, trajectory_digest(changed));
}

TEST(TrajectoryDigest, JointPointAndValueOrderArePreserved) {
  const auto original = trajectory();
  const auto digest = trajectory_digest(original);
  auto changed = original;
  std::swap(changed.joint_names[0], changed.joint_names[1]);
  EXPECT_NE(digest, trajectory_digest(changed));
  changed = original;
  std::swap(changed.points[0], changed.points[1]);
  EXPECT_NE(digest, trajectory_digest(changed));
  changed = original;
  std::swap(changed.points[0].positions[0], changed.points[0].positions[1]);
  EXPECT_NE(digest, trajectory_digest(changed));
}

TEST(TrajectoryDigest, EmptyFieldsAreDistinctFromExplicitZeros) {
  auto empty = trajectory();
  empty.points[0].velocities.clear();
  auto zeros = empty;
  zeros.points[0].velocities = {0., 0.};
  EXPECT_NE(trajectory_digest(empty), trajectory_digest(zeros));
  empty.points[0].accelerations.clear();
  zeros = empty;
  zeros.points[0].accelerations = {0., 0.};
  EXPECT_NE(trajectory_digest(empty), trajectory_digest(zeros));
  empty.points[0].effort.clear();
  zeros = empty;
  zeros.points[0].effort = {0., 0.};
  EXPECT_NE(trajectory_digest(empty), trajectory_digest(zeros));
}

TEST(TrajectoryDigest, InvalidNumericOrTextEncodingThrows) {
  auto changed = trajectory();
  changed.points[0].positions[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(trajectory_digest(changed), std::runtime_error);
  changed = trajectory();
  changed.points[0].effort[0] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(trajectory_digest(changed), std::runtime_error);
  changed = trajectory();
  changed.header.frame_id = std::string(1, static_cast<char>(0xff));
  EXPECT_THROW(trajectory_digest(changed), nlohmann::json::type_error);
}
