#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "astribot_s1_transport_native/fixed_station_scene.hpp"
using namespace astribot::transport;
namespace {
const std::set<std::string> links = {"astribot_arm_left_tcp_link"};
geometry_msgs::msg::Pose pose(double x, double y, double z) {
  geometry_msgs::msg::Pose p;p.position.x=x;p.position.y=y;p.position.z=z;p.orientation.w=1.;return p;
}
const FixedStations stations = {{{"pick_station", pose(.1,.7,.5175), {.1,.1,1.035}},
                                {"place_station", pose(1.15,.72,.5175), {.1,.1,1.035}}}};
moveit_msgs::msg::CollisionObject box(const std::string &id, const geometry_msgs::msg::Pose &p,
                                    const std::vector<double> &size) {
  moveit_msgs::msg::CollisionObject o;o.id=id;o.header.frame_id="astribot_torso_base";o.pose=p;
  shape_msgs::msg::SolidPrimitive shape;shape.type=shape.BOX;shape.dimensions.assign(size.begin(),size.end());
  o.primitives={shape};o.primitive_poses={pose(0,0,0)};return o;
}
moveit_msgs::msg::PlanningScene scene() {
  moveit_msgs::msg::PlanningScene s;s.world.octomap.header.frame_id="astribot_torso_base";
  for(const auto &station:stations)s.world.collision_objects.push_back(box(station.id,station.world_pose,{.1,.1,1.035}));
  s.world.collision_objects.push_back(box("unrelated",pose(3,2,1),{.3,.4,.5}));
  moveit_msgs::msg::AttachedCollisionObject a;a.link_name="astribot_arm_left_tcp_link";a.touch_links={a.link_name};a.weight=.2;
  a.object=box("transport_box_01",pose(0,0,.1),{.071,.071,.131});a.object.header.frame_id=a.link_name;
  s.robot_state.attached_collision_objects={a};return s;
}
moveit_msgs::msg::PlanningScene apply_station_diff(const moveit_msgs::msg::PlanningScene &before,
                                    const moveit_msgs::msg::PlanningScene &diff) {
  auto after=before;
  for(const auto &o:diff.world.collision_objects)for(auto &target:after.world.collision_objects)if(target.id==o.id)target=o;
  return after;
}
}

TEST(FixedStationScene, UsesActualTranslatedAndRotatedBaseForStationsAndTarget) {
  Eigen::Isometry3d world_base=Eigen::Isometry3d::Identity();world_base.translation()=Eigen::Vector3d(1.1,0.,.129);
  world_base.linear()=Eigen::AngleAxisd(M_PI/2.,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto diff=fixed_station_scene_diff(scene(),stations,world_base,links);
  const auto &p=diff.world.collision_objects[1].pose.position;
  EXPECT_NEAR(p.x,.72,1e-12);EXPECT_NEAR(p.y,-.05,1e-12);EXPECT_NEAR(p.z,.3885,1e-12);
  const auto target=fixed_station_target_in_base(pose(1.15,.72,1.095),world_base);
  EXPECT_NEAR(target.position.x,.72,1e-12);EXPECT_NEAR(target.position.y,-.05,1e-12);
  EXPECT_NEAR(target.position.z,.966,1e-12);
  EXPECT_NEAR(std::abs(target.orientation.z),std::sqrt(.5),1e-12);
}

TEST(FixedStationScene, ChangesOnlyStationsAndPreservesRealAttachment) {
  auto before=scene();
  for (size_t i=0;i<2;++i) {
    auto &object=before.world.collision_objects[i];
    object.primitive_poses[0]=object.pose;object.pose=pose(0,0,0);
  }
  Eigen::Isometry3d world_base=Eigen::Isometry3d::Identity();world_base.translation().x()=1.1;
  const auto diff=fixed_station_scene_diff(before,stations,world_base,links);
  EXPECT_TRUE(diff.is_diff);EXPECT_TRUE(diff.robot_state.is_diff);
  EXPECT_TRUE(diff.robot_state.attached_collision_objects.empty());ASSERT_EQ(diff.world.collision_objects.size(),2u);
  EXPECT_EQ(diff.world.collision_objects[1].primitive_poses[0],pose(0,0,0));
  EXPECT_NEAR(diff.world.collision_objects[1].pose.position.x,.05,1e-12);
  const auto after=apply_station_diff(before,diff);
  EXPECT_EQ(after.robot_state.attached_collision_objects,before.robot_state.attached_collision_objects);
  EXPECT_EQ(after.world.collision_objects[2],before.world.collision_objects[2]);
  EXPECT_NO_THROW(validate_fixed_station_scene(before,after,diff,links));
}

TEST(FixedStationScene, RejectsOccupancyChangesWithUnchangedMetadata) {
  auto before=scene();auto &map=before.world.octomap.octomap;
  map.id="OcTree";map.binary=true;map.resolution=.05;
  // Octomap binary root with one leaf; changing its state preserves metadata.
  map.data={1,0};
  const auto diff=fixed_station_scene_diff(before,stations,Eigen::Isometry3d::Identity(),links);
  auto after=apply_station_diff(before,diff);
  EXPECT_NO_THROW(validate_fixed_station_scene(before,after,diff,links));
  after.world.octomap.octomap.data.clear();
  EXPECT_THROW(validate_fixed_station_scene(before,after,diff,links),std::runtime_error);
  after=apply_station_diff(before,diff);after.world.octomap.octomap.data={2,0};
  EXPECT_THROW(validate_fixed_station_scene(before,after,diff,links),std::runtime_error);
}

TEST(FixedStationScene, RejectsOldStationReadbackAndUnrelatedChanges) {
  const auto before=scene();Eigen::Isometry3d world_base=Eigen::Isometry3d::Identity();world_base.translation().x()=1.1;
  const auto diff=fixed_station_scene_diff(before,stations,world_base,links);
  EXPECT_THROW(validate_fixed_station_scene(before,before,diff,links),std::runtime_error);
  auto after=apply_station_diff(before,diff);after.robot_state.attached_collision_objects.clear();
  EXPECT_THROW(validate_fixed_station_scene(before,after,diff,links),std::runtime_error);
  after=apply_station_diff(before,diff);after.world.collision_objects[2].pose.position.x+=.01;
  EXPECT_THROW(validate_fixed_station_scene(before,after,diff,links),std::runtime_error);
  after=apply_station_diff(before,diff);after.world.collision_objects.pop_back();
  EXPECT_THROW(validate_fixed_station_scene(before,after,diff,links),std::runtime_error);
  after=apply_station_diff(before,diff);after.allowed_collision_matrix.default_entry_names={"unrelated"};after.allowed_collision_matrix.default_entry_values={true};
  EXPECT_THROW(validate_fixed_station_scene(before,after,diff,links),std::runtime_error);
}
