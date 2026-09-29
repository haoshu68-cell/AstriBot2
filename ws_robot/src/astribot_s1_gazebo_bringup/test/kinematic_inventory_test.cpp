#include <gtest/gtest.h>
#include "astribot_s1_gazebo_bringup/kinematic_inventory.hpp"
using namespace astribot::simulation;
namespace {
InventorySnapshot inventory() {
  InventorySnapshot s;s.world_present=s.world_metadata=s.background_matches=s.robot_structure_matches=true;
  s.world_entity=99;
  s.robot_count=1;s.models={{1,true,false},{2,false,true},{3,false,false}};
  s.plugins={{99,"ignition::gazebo::systems::Physics","ignition-gazebo-physics-system"},
    {3,"astribot::KinematicPayload","libastribot_kinematic_payload.so"}};return s;
}
PayloadExecution state() {PayloadExecution p;p.epoch="epoch";p.parent_model="robot";p.parent_link="wrist";p.parent=10;p.capture=100;return p;}
}
TEST(KinematicInventory,ExplicitDetachedAndAttachedRequireCurrentExecution) {
  auto s=inventory();auto p=state();
  EXPECT_EQ(kinematic_inventory_reason(s,{{3,p}},100),"EMPTY_INVENTORY_OBSERVED");
  p.attached=true;p.accepted=p.applied=1;
  EXPECT_EQ(kinematic_inventory_reason(s,{{3,p}},100),"ATTACHED_INVENTORY_OBSERVED");
  EXPECT_NE(empty_inventory_reason(s),"EMPTY_INVENTORY_OBSERVED");
}
TEST(KinematicInventory,ReadOnlyContactEvidencePreservesPayloadInventory) {
  auto s=inventory();auto p=state();
  s.plugins.push_back({99,"astribot::ContactEvidence","libastribot_contact_evidence.so"});
  for(const auto attached:{false,true}) {
    p.attached=attached;p.accepted=p.applied=attached?1:0;
    EXPECT_EQ(kinematic_inventory_reason(s,{{3,p}},100),
      attached?"ATTACHED_INVENTORY_OBSERVED":"EMPTY_INVENTORY_OBSERVED");
    s.plugins.push_back({99,"custom::Attachment","libcustom_attachment.so"});
    EXPECT_EQ(kinematic_inventory_reason(s,{{3,p}},100),"UNSUPPORTED_SYSTEM_PLUGIN");
    s.plugins.pop_back();
  }
}
TEST(KinematicInventory,HeightSliceMapOnWorldPreservesDetachedAndAttachedEvidence) {
  auto s=inventory();auto p=state();
  s.plugins.push_back({99,"astribot::HeightSliceMap","/opt/astribot/lib/libastribot_height_slice_map.so"});
  for(const auto attached:{false,true}) {
    p.attached=attached;p.accepted=p.applied=attached?1:0;s.plugins.back().entity=99;
    EXPECT_EQ(kinematic_inventory_reason(s,{{3,p}},100),
      attached?"ATTACHED_INVENTORY_OBSERVED":"EMPTY_INVENTORY_OBSERVED");
    for(const auto entity:{0u,1u,2u,3u,10u,100u}) {
      s.plugins.back().entity=entity;
      EXPECT_EQ(kinematic_inventory_reason(s,{{3,p}},100),"UNSUPPORTED_SYSTEM_PLUGIN")<<entity;
    }
  }
}
TEST(KinematicInventory,MissingStalePendingAndFailedEvidenceNeverConfirm) {
  auto s=inventory();auto p=state();
  EXPECT_NE(kinematic_inventory_reason(s,{},100),"EMPTY_INVENTORY_OBSERVED");
  for(int i=0;i<7;++i) {
    auto q=p;
    if(i==0)q.capture=99;if(i==1)q.pending=true;if(i==2)q.accepted=1;
    if(i==3)q.error="PARENT_LOST";if(i==4)q.epoch.clear();
    if(i==5){q.attached=true;q.accepted=q.applied=1;q.parent=0;}
    if(i==6)q.offset.Pos().X(std::numeric_limits<double>::quiet_NaN());
    auto why=kinematic_inventory_reason(s,{{3,q}},100);
    EXPECT_NE(why,"EMPTY_INVENTORY_OBSERVED");EXPECT_NE(why,"ATTACHED_INVENTORY_OBSERVED");
  }
}
TEST(KinematicInventory,UnknownModelPluginOrDuplicateExecutorNeverConfirms) {
  auto p=state();auto s=inventory();s.models.push_back({4,false,true});s.background_matches=false;
  EXPECT_NE(kinematic_inventory_reason(s,{{3,p}},100),"EMPTY_INVENTORY_OBSERVED");
  s=inventory();s.plugins.push_back(s.plugins.back());
  EXPECT_NE(kinematic_inventory_reason(s,{{3,p}},100),"EMPTY_INVENTORY_OBSERVED");
  s=inventory();s.plugins.back().filename="different.so";
  EXPECT_NE(kinematic_inventory_reason(s,{{3,p}},100),"EMPTY_INVENTORY_OBSERVED");
  s=inventory();s.plugins.clear();
  EXPECT_NE(kinematic_inventory_reason(s,{{3,p}},100),"EMPTY_INVENTORY_OBSERVED");
}
