#include "astribot_s1_transport_mtc/support_selection.hpp"
#include <gtest/gtest.h>
#include <moveit/collision_detection/collision_matrix.h>

using astribot_s1_transport_mtc::selectPlaceSupport;
class PlaceSupport : public ::testing::Test {
protected:
  moveit_msgs::msg::PlanningScene scene;
  geometry_msgs::msg::PoseStamped target;
  static moveit_msgs::msg::CollisionObject box(const std::string& id,
      const std::vector<double>& size,double x,double y,double z) {
    moveit_msgs::msg::CollisionObject object;object.id=id;
    object.header.frame_id="astribot_torso_base";object.pose.orientation.w=1.;
    object.pose.position.x=x;object.pose.position.y=y;object.pose.position.z=z;
    shape_msgs::msg::SolidPrimitive shape;shape.type=shape.BOX;shape.dimensions.assign(size.begin(),size.end());
    object.primitives={shape};geometry_msgs::msg::Pose origin;origin.orientation.w=1.;
    object.primitive_poses={origin};return object;
  }
  void SetUp() override {
    scene.world.collision_objects={box("box_pick_station",{.1,.1,1.035},.1,.7,.5175),
      box("box_place_station",{.1,.1,1.035},1.15,.72,.5175)};
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name="astribot_arm_left_tcp_link";
    attached.object=box("box",{.07146969384566991,.07146969384566991,.1314696938456699},
                        .005967979170712483,-.10000050807826605,.0002866474461655103);
    attached.object.header.frame_id=attached.link_name;
    auto& q=attached.object.pose.orientation;
    q.x=.49898550996387264;q.y=.5010107681221758;
    q.z=.5010129100213471;q.w=-.49898670830360325;
    scene.robot_state.attached_collision_objects={attached};
    target.header.frame_id="astribot_torso_base";
    target.pose.position.x=.1;target.pose.position.y=.7;target.pose.position.z=1.195;
    auto& t=target.pose.orientation;t.x=t.y=t.z=t.w=.5;
  }
  void expectReason(const std::string& reason) {
    try {(void)selectPlaceSupport(scene,target,"box",.025);FAIL()<<"unexpected support";}
    catch(const std::runtime_error& error){EXPECT_EQ(error.what(),reason);}
  }
};
TEST_F(PlaceSupport, ActualInflatedAttachmentSelectsOriginalPickOrPlaceTable) {
  EXPECT_EQ(selectPlaceSupport(scene,target,"box",.025),"box_pick_station");
  target.pose.position.x=1.15;target.pose.position.y=.72;
  EXPECT_EQ(selectPlaceSupport(scene,target,"box",.025),"box_place_station");
}
TEST_F(PlaceSupport, OnlySelectedPairReceivesTemporaryPermissionAndOriginalRestores) {
  collision_detection::AllowedCollisionMatrix original(scene.allowed_collision_matrix),temporary=original;
  const auto id=selectPlaceSupport(scene,target,"box",.025);
  temporary.setEntry("box",id,true);
  collision_detection::AllowedCollision::Type permission;
  ASSERT_TRUE(temporary.getEntry("box","box_pick_station",permission));
  EXPECT_EQ(permission,collision_detection::AllowedCollision::ALWAYS);
  EXPECT_FALSE(temporary.getEntry("box","box_place_station",permission));
  EXPECT_FALSE(temporary.getEntry("box","astribot_torso_link_4",permission));
  temporary=original;
  EXPECT_FALSE(temporary.getEntry("box","box_pick_station",permission));
}
TEST_F(PlaceSupport, RejectsNoMatchAndMissingSupportWithoutFallback) {
  target.pose.position.x=.4;expectReason("PLACE_SUPPORT_NOT_FOUND");
  target.pose.position.x=.1;scene.world.collision_objects.erase(scene.world.collision_objects.begin());
  expectReason("PLACE_SUPPORT_NOT_FOUND");
}
TEST_F(PlaceSupport, RejectsAmbiguousCoincidentRegisteredTables) {
  scene.world.collision_objects[1].pose=scene.world.collision_objects[0].pose;
  expectReason("PLACE_SUPPORT_AMBIGUOUS");
}
TEST_F(PlaceSupport, IgnoresUnrelatedTableAtTheTarget) {
  scene.world.collision_objects.front().id="unrelated_table";
  expectReason("PLACE_SUPPORT_NOT_FOUND");
}
TEST_F(PlaceSupport, RequiresWholeProjectedBoxOnSupportAndCenterAboveTop) {
  target.pose.position.x+=.03;expectReason("PLACE_SUPPORT_NOT_FOUND");
  target.pose.position.x=.1;target.pose.position.z=.9;
  expectReason("PLACE_SUPPORT_NOT_FOUND");
}
TEST_F(PlaceSupport, VerticalSelectionUsesExistingTwentyFiveMillimeterBound) {
  auto& object=scene.robot_state.attached_collision_objects.front().object;
  object.pose=geometry_msgs::msg::Pose();object.pose.orientation.w=1.;
  object.primitives.front().dimensions={.05,.05,.125};
  target.pose.orientation=geometry_msgs::msg::Quaternion();target.pose.orientation.w=1.;
  const double center=1.035+.0625;
  for(double sign:{-1.,1.}) {
    target.pose.position.z=center+sign*.025;
    EXPECT_EQ(selectPlaceSupport(scene,target,"box",.025),"box_pick_station");
    target.pose.position.z=center+sign*(.025+1e-8);
    expectReason("PLACE_SUPPORT_NOT_FOUND");
  }
}
TEST_F(PlaceSupport, AppliesStationAndPrimitiveRotationsBeforeProjection) {
  auto& station=scene.world.collision_objects.front();
  station.primitives.front().dimensions={.1,.08,1.035};
  const double angle=.6;
  station.pose.orientation.z=std::sin(angle/2.);station.pose.orientation.w=std::cos(angle/2.);
  auto& object=scene.robot_state.attached_collision_objects.front().object;
  object.pose=geometry_msgs::msg::Pose();object.pose.orientation.w=1.;
  object.primitives.front().dimensions={.08,.06,.12};
  target.pose.position.z=1.095;target.pose.orientation=station.pose.orientation;
  EXPECT_EQ(selectPlaceSupport(scene,target,"box",.025),"box_pick_station");
  object.primitive_poses.front().orientation.z=std::sin(.7853981633974483/2.);
  object.primitive_poses.front().orientation.w=std::cos(.7853981633974483/2.);
  expectReason("PLACE_SUPPORT_NOT_FOUND");
}
