#include <gtest/gtest.h>
#include "astribot_s1_gazebo_bringup/inventory_gate.hpp"
using namespace astribot::simulation;
InventorySnapshot clean() {InventorySnapshot s;s.world_present=true;s.robot_count=1;s.world_metadata=true;s.background_matches=true;s.robot_structure_matches=true;
  s.models.push_back({2,true,true});s.models.push_back({3,false,true});
  s.plugins.push_back({1,"ignition::gazebo::systems::Physics","ignition-gazebo-physics-system"});return s;}
TEST(EmptyInventory, RequiresWorldRobotAndWorldPluginMetadata) {
  auto s=clean();EXPECT_EQ(empty_inventory_reason(s),"EMPTY_INVENTORY_OBSERVED");
  s.world_present=false;EXPECT_EQ(empty_inventory_reason(s),"WORLD_UNAVAILABLE");s=clean();s.robot_count=0;
  EXPECT_EQ(empty_inventory_reason(s),"ROBOT_IDENTITY_AMBIGUOUS");s.robot_count=2;EXPECT_NE(empty_inventory_reason(s),"EMPTY_INVENTORY_OBSERVED");
  s=clean();s.world_metadata=false;EXPECT_EQ(empty_inventory_reason(s),"WORLD_PLUGIN_INVENTORY_UNAVAILABLE");
}
TEST(EmptyInventory, RejectsEveryUnsupportedMechanismEvenWhenClaimingDetached) {
  for(const auto &name:{"astribot::KinematicPayload","ignition::gazebo::systems::DetachableJoint","custom::Attachment"}) {
    auto s=clean();s.plugins.push_back({3,name,"payload.so"});EXPECT_NE(empty_inventory_reason(s),"EMPTY_INVENTORY_OBSERVED");
  }
  auto s=clean();s.detachable_joints=1;EXPECT_EQ(empty_inventory_reason(s),"UNSUPPORTED_DETACHABLE_JOINT");
  s=clean();s.external_joints=1;EXPECT_EQ(empty_inventory_reason(s),"UNSUPPORTED_EXTERNAL_JOINT");
}
TEST(EmptyInventory, DynamicOrMissingStaticExternalBodiesBlockEmpty) {
  auto s=clean();s.models[1].is_static=false;EXPECT_EQ(empty_inventory_reason(s),"UNSUPPORTED_DYNAMIC_MODEL");
  s=clean();s.model_metadata_missing=true;EXPECT_EQ(empty_inventory_reason(s),"MODEL_METADATA_UNAVAILABLE");
}
TEST(EmptyInventory, PluginNameAndLibraryMustBothMatch) {
  auto s=clean();s.plugins[0].filename="/opt/lib/libignition-gazebo-physics-system.so";
  EXPECT_EQ(empty_inventory_reason(s),"EMPTY_INVENTORY_OBSERVED");
  s.plugins[0].filename="custom-physics-system.so";EXPECT_EQ(empty_inventory_reason(s),"UNSUPPORTED_SYSTEM_PLUGIN");
  s=clean();s.plugins[0].name="custom::Physics";EXPECT_EQ(empty_inventory_reason(s),"UNSUPPORTED_SYSTEM_PLUGIN");
}
TEST(EmptyInventory, WholeSnapshotBoundedAndInvalidEntitiesRejected) {
  auto s=clean();s.models.resize(4097);EXPECT_EQ(empty_inventory_reason(s),"INVENTORY_LIMIT_EXCEEDED");
  s=clean();s.plugins.resize(257);EXPECT_EQ(empty_inventory_reason(s),"INVENTORY_LIMIT_EXCEEDED");
  s=clean();s.models.push_back(s.models.back());EXPECT_EQ(empty_inventory_reason(s),"DUPLICATE_MODEL_ID");
}
TEST(EmptyInventory, RevisionChangesAcrossUnknownAndBackAndClockReset) {
  InventoryVersion v;auto a=v.sample(100,"empty"),b=v.sample(200,"empty"),c=v.sample(300,"unknown"),d=v.sample(400,"empty");
  EXPECT_EQ(a.revision,b.revision);EXPECT_GT(b.sequence,a.sequence);EXPECT_GT(c.revision,b.revision);EXPECT_GT(d.revision,c.revision);
  auto reset=v.sample(50,"empty");EXPECT_GT(reset.clock_epoch,d.clock_epoch);EXPECT_GT(reset.revision,d.revision);
  EXPECT_THROW(v.sample(50,"empty"),std::invalid_argument);
}

TEST(EmptyInventory, UnknownStaticPayloadCannotMasqueradeAsBackground) {
  auto s=clean();s.background_matches=false;
  EXPECT_EQ(empty_inventory_reason(s),"BACKGROUND_MANIFEST_MISMATCH");
}

TEST(EmptyInventory, RobotStructureMustMatchIndependentDescription) {
  auto s=clean();s.robot_structure_matches=false;
  EXPECT_EQ(empty_inventory_reason(s),"ROBOT_STRUCTURE_MISMATCH");
  s=clean();s.models.push_back({4,true,false});
  EXPECT_EQ(empty_inventory_reason(s),"ROBOT_STRUCTURE_MISMATCH");
}

TEST(EmptyInventory, AddedReplacedLinksAndChangedJointEndpointsReject) {
  const std::set<std::string> links={"arm","gripper"};const JointConnections joints={{"wrist",{"arm","gripper"}}};
  EXPECT_TRUE(robot_structure_matches(links,joints,links,joints));
  auto extra=links;extra.insert("payload");EXPECT_FALSE(robot_structure_matches(links,joints,extra,joints));
  auto replacement=links;replacement.erase("arm");replacement.insert("payload");
  EXPECT_FALSE(robot_structure_matches(links,joints,replacement,joints));
  auto changed=joints;changed["wrist"]={"arm","payload"};
  EXPECT_FALSE(robot_structure_matches(links,joints,links,changed));
  EXPECT_FALSE(robot_structure_matches({}, {}, {}, {}));
}
TEST(EmptyInventory, BufferedCaptureNeverRenewsLeaseOrCrossesRevision) {
  Version a{0,1,1},b{0,2,1};
  EXPECT_TRUE(publication_window(a,b,100,150,400,1000,1050));
  EXPECT_FALSE(publication_window(a,b,100,400,400,1000,1050));
  EXPECT_FALSE(publication_window(a,b,100,150,400,1000,1300));
  EXPECT_FALSE(publication_window(a,b,100,100,400,1000,1050));
  b.revision=2;EXPECT_FALSE(publication_window(a,b,100,150,400,1000,1050));
  b={1,2,1};EXPECT_FALSE(publication_window(a,b,100,150,400,1000,1050));
}

TEST(InventoryVersion, ChangedPositiveRevisionRequiresImmediateRevocation) {
  const Version old{1,10,4},same{1,11,4},changed{1,11,5},reset{2,11,5};
  EXPECT_FALSE(requires_revocation(old,same));
  EXPECT_TRUE(requires_revocation(old,changed));
  EXPECT_TRUE(requires_revocation(old,reset));
}
