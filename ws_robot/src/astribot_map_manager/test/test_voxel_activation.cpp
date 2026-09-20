#include "astribot_map_manager/voxel_activation.hpp"
#include "fixture.hpp"
#include <gtest/gtest.h>
#include <unistd.h>
using namespace astribot_map_manager;
TEST(VoxelActivation, RestoresOriginalSessionNameAndRejectsCorruption){
 auto root=fs::temp_directory_path()/("voxel_activation_"+std::to_string(getpid()));fs::remove_all(root);makeMap(root/"imports/floor1");
 Catalog catalog(root/"catalog",root/"imports");
 auto target=catalog.command("import","map_import",{{"expected_revision",0},{"map_id","floor1"},{"floor",1},{"directory",(root/"imports/floor1").string()}},"test");
 auto plan=materializeSession(target,root/"catalog/assets",root/"runtime","attempt1");
 EXPECT_EQ(plan.session,"floor1");EXPECT_TRUE(fs::exists(plan.root/"floor1/floor1.yaml"));
 EXPECT_EQ(astribot_s1_autonomy::inspectSession(plan.root/plan.session).at("sha256"),target.at("sha256"));
 EXPECT_THROW(materializeSession(target,root/"catalog/assets",root/"runtime","attempt1"),std::runtime_error);
 EXPECT_THROW(materializeSession(target,root/"imports",root/"runtime","outside"),std::runtime_error);
 std::ofstream(fs::path(target.at("directory").get<std::string>())/"map.pgm",std::ios::app)<<'x';
 EXPECT_THROW(materializeSession(target,root/"catalog/assets",root/"runtime","bad"),std::runtime_error);EXPECT_FALSE(fs::exists(root/"runtime/bad"));
 EXPECT_TRUE(fs::exists(plan.root/"floor1/floor1.yaml"));fs::remove_all(root);
}
TEST(VoxelActivation, EveryEvidenceGateIsRequired){
 ActivationEvidence e;EXPECT_FALSE(e.ready());
 for(auto member:{&ActivationEvidence::nav_reset,&ActivationEvidence::nav_started,&ActivationEvidence::tracking,&ActivationEvidence::tf,&ActivationEvidence::map,&ActivationEvidence::global_costmap,&ActivationEvidence::local_costmap})e.*member=true;
 EXPECT_TRUE(e.ready());
 for(auto member:{&ActivationEvidence::nav_reset,&ActivationEvidence::nav_started,&ActivationEvidence::tracking,&ActivationEvidence::tf,&ActivationEvidence::map,&ActivationEvidence::global_costmap,&ActivationEvidence::local_costmap}){e.*member=false;EXPECT_FALSE(e.ready());e.*member=true;}
}
