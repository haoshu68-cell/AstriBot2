#include "astribot_map_manager/catalog.hpp"
#include "fixture.hpp"
#include "astribot_map_manager/storage_path.hpp"
#include <gtest/gtest.h>
#include <unistd.h>
using namespace astribot_map_manager;
class CatalogTest:public ::testing::Test{
protected:
 fs::path root;std::unique_ptr<Catalog> c;
 void SetUp()override{root=fs::temp_directory_path()/("catalog_test_"+std::to_string(getpid()));fs::remove_all(root);makeMap(root/"imports/floor1");c=std::make_unique<Catalog>(root/"store",root/"imports");}
 void TearDown()override{c.reset();fs::remove_all(root);}
 Json import(){return c->command("import","map_import",{{"expected_revision",c->snapshot().at("revision")},{"map_id","floor1"},{"floor",1},{"directory",(root/"imports/floor1").string()}},"alice");}
 Json request(const Json & map){return {{"expected_revision",c->snapshot().at("revision")},{"map_id","floor1"},{"map_version",map.at("version")},{"manual_transfer",true}};}
};
TEST_F(CatalogTest, ImmutableStationVersionsAndRestart){
 auto map=import();Json p={{"expected_revision",1},{"map_id","floor1"},{"map_version",map.at("version")},{"station_id","shelf"},{"expected_station_version",0},{"kind","shelf"},{"dock_pose",{{"frame","map"},{"x",1},{"y",2},{"yaw",0}}}};
 auto station=c->command("put","station_put",p,"alice");for(int i=0;i<100;++i)EXPECT_EQ(c->command("put","station_put",p,"alice"),station);
 EXPECT_EQ(c->snapshot().at("revision"),2);p["dock_pose"]["x"]=2;EXPECT_THROW(c->command("put","station_put",p,"alice"),std::runtime_error);
 p["expected_revision"]=2;EXPECT_THROW(c->command("put2","station_put",p,"alice"),std::runtime_error);
 p["expected_station_version"]=1;p["dock_pose"]["x"]=999;EXPECT_THROW(c->command("outside","station_put",p,"alice"),std::runtime_error);p["dock_pose"]["x"]=2;c->command("put2","station_put",p,"alice");
 EXPECT_EQ(c->snapshot().at("stations").at("floor1/shelf").at(0).at("dock_pose").at("x"),1);
 c.reset();c=std::make_unique<Catalog>(root/"store",root/"imports");EXPECT_EQ(c->snapshot().at("stations").at("floor1/shelf").size(),2u);
 EXPECT_THROW(Catalog(root/"store",root/"imports"),std::runtime_error);
}
TEST_F(CatalogTest, CorruptionAndPathEscapeRejected){
 auto p=Json{{"expected_revision",0},{"map_id","bad"},{"floor",1},{"directory",(root/"imports/floor1").string()}};
 std::ofstream(root/"imports/floor1/map.pgm",std::ios::app)<<'x';
 EXPECT_THROW(c->command("bad","map_import",p,"alice"),std::runtime_error);EXPECT_EQ(c->snapshot().at("revision"),0);
 p["directory"]=root.string();EXPECT_THROW(c->command("outside","map_import",p,"alice"),std::runtime_error);
}
TEST_F(CatalogTest, SwitchCommitRequiresLegalTransitionsAndPreservesOldMap){
 auto map=import();auto p=request(map);c->command("switch","map_switch_begin",p,"alice");
 EXPECT_TRUE(Catalog::blocked(c->snapshot().at("transaction")));EXPECT_TRUE(c->snapshot().at("active_map").is_null());
 EXPECT_THROW(c->transition("switch","COMMITTED","bad"),std::runtime_error);
 c->command("confirm","map_transfer_confirm",{{"expected_revision",2},{"transaction_id","switch"}},"bob");
 EXPECT_THROW(c->command("abort","map_abort",{{"expected_revision",3},{"transaction_id","switch"}},"bob"),std::runtime_error);
 c->transition("switch","LOADING","intent");c->transition("switch","COMMITTED","verified");
 EXPECT_EQ(c->snapshot().at("active_map").at("version"),map.at("version"));
 EXPECT_FALSE(Catalog::blocked(c->snapshot().at("transaction")));
}
TEST_F(CatalogTest, RestartDoesNotReplayPendingTransfer){
 auto map=import();c->command("switch","map_switch_begin",request(map),"alice");c.reset();
 c=std::make_unique<Catalog>(root/"store",root/"imports");
 EXPECT_EQ(c->snapshot().at("transaction").at("state"),"RECOVERY_REQUIRED");
 EXPECT_TRUE(c->snapshot().at("active_map").is_null());
 EXPECT_THROW(c->command("old","map_transfer_confirm",{{"expected_revision",2},{"transaction_id","switch"}},"alice"),std::runtime_error);
}
TEST_F(CatalogTest, ImportedSnapshotSurvivesSourceDeletionButDetectsTamper){
 auto map=import();fs::remove_all(root/"imports");EXPECT_NO_THROW(c->verifiedMap("floor1"));
 std::ofstream(fs::path(map.at("directory").get<std::string>())/"map.pgm",std::ios::app)<<'x';
 EXPECT_THROW(c->verifiedMap("floor1"),std::runtime_error);
}

TEST_F(CatalogTest, DevicesAndScenesPersistWithVersionedReferences) {
 auto map=import();
 auto call=[&](std::string id,std::string op,Json p){p["expected_revision"]=c->snapshot().at("revision");return c->command(id,op,p,"alice");};
 Json pos={{"frame","map"},{"x",1.0},{"y",2.0},{"yaw",0.0}};
 Json device={{"device_id","reader01"},{"name","酶标仪01"},{"type","plate_reader"},{"map_id","floor1"},{"map_version",map.at("version")},{"expected_device_version",0},{"pose",pos},{"dock_pose",pos},{"wait_pose",pos},{"width",0.8},{"depth",0.6},{"reviewed",false},{"enabled",true}};
 auto d=call("device1","device_put",device);EXPECT_EQ(d.at("version"),1);
 Json scene={{"scene_id","reader_scene"},{"name","酶标仪场景"},{"type","plate_reader"},{"map_id","floor1"},{"map_version",map.at("version")},{"expected_scene_version",0},{"devices",Json::array({{{"device_id","reader01"},{"version",1}}})}};
 call("scene1","scene_put",scene);
 c.reset();c=std::make_unique<Catalog>(root/"store",root/"imports");
 EXPECT_EQ(c->snapshot().at("devices").at("reader01").at(0).at("name"),"酶标仪01");
 EXPECT_EQ(c->snapshot().at("scenes").at("reader_scene").size(),1u);
 EXPECT_THROW(call("stale","device_put",device),std::runtime_error);
 device["expected_device_version"]=1;device["pose"]["x"]=3;call("device2","device_put",device);
 EXPECT_THROW(call("load_stale","scene_load",{{"scene_id","reader_scene"},{"scene_version",1}}),std::runtime_error);
 scene["expected_scene_version"]=1;scene["devices"][0]["version"]=2;call("scene2","scene_put",scene);
 call("load","scene_load",{{"scene_id","reader_scene"},{"scene_version",2},{"manual_transfer",false}});
 EXPECT_TRUE(c->snapshot().at("active_scene").is_null());
 c->transition("load","LOADING","intent");c->transition("load","COMMITTED","verified");
 EXPECT_EQ(c->snapshot().at("active_scene").at("scene_id"),"reader_scene");
 EXPECT_EQ(c->snapshot().at("active_scene").at("version"),2);EXPECT_TRUE(c->sceneReady());
 scene["scene_id"]="reader_other";scene["expected_scene_version"]=0;call("other","scene_put",scene);
 auto tx=c->snapshot().at("transaction");auto selected=call("reuse","scene_load",{{"scene_id","reader_other"},{"scene_version",1},{"reuse_current_map",true}});
 EXPECT_TRUE(selected.at("reused_map"));EXPECT_EQ(c->snapshot().at("transaction"),tx);EXPECT_EQ(c->snapshot().at("active_scene").at("scene_id"),"reader_other");
 device["expected_device_version"]=2;device["enabled"]=false;call("disable","device_put",device);EXPECT_FALSE(c->sceneReady());
 EXPECT_THROW(call("load-disabled","scene_load",{{"scene_id","reader_other"},{"scene_version",1}}),std::runtime_error);
 c.reset();c=std::make_unique<Catalog>(root/"store",root/"imports");EXPECT_FALSE(c->sceneReady());
}
TEST_F(CatalogTest, SceneRejectsInvalidMapAndDeviceMetadata) {
 auto map=import();Json p={{"expected_revision",c->snapshot().at("revision")},{"device_id","box"},{"expected_device_version",0},{"name","box"},{"type","incubator_door"},{"map_id","floor1"},{"map_version",map.at("version")},{"pose",{{"frame","odom"},{"x",1},{"y",2},{"yaw",0}}},{"width",1},{"depth",1},{"reviewed",false},{"enabled",true}};
 EXPECT_THROW(c->command("invalid","device_put",p,"alice"),std::runtime_error);
 p["pose"]["frame"]="map";p["width"]=-1;EXPECT_THROW(c->command("invalid2","device_put",p,"alice"),std::runtime_error);
 p["width"]=1;p["map_version"]="wrong";EXPECT_THROW(c->command("invalid3","device_put",p,"alice"),std::runtime_error);
 p["map_version"]=map.at("version");p["dock_pose"]=p["pose"];p["dock_pose"]["x"]=999;
 EXPECT_THROW(c->command("outside","device_put",p,"alice"),std::runtime_error);
 p["dock_pose"]["x"]=1;EXPECT_NO_THROW(c->command("valid","device_put",p,"alice"));
 EXPECT_EQ(c->snapshot().at("revision"),2);
}

TEST_F(CatalogTest, IndependentSceneMapAndInterruptedActivation) {
 auto one=import();makeMap(root/"imports/floor2");
 auto call=[&](std::string id,std::string op,Json p){p["expected_revision"]=c->snapshot().at("revision");return c->command(id,op,p,"alice");};
 auto two=call("map2","map_import",{{"map_id","floor2"},{"floor",2},{"directory",(root/"imports/floor2").string()}});
 Json s={{"scene_id","freeze"},{"name","冻干"},{"type","freeze_dryer"},{"map_id","floor1"},{"map_version",one.at("version")},{"expected_scene_version",0},{"devices",Json::array()}};
 call("s1","scene_put",s);call("l1","scene_load",{{"scene_id","freeze"},{"scene_version",1}});c->transition("l1","LOADING","intent");c->transition("l1","COMMITTED","verified");
 s["scene_id"]="incubator";s["type"]="incubator_door";s["map_id"]="floor2";s["map_version"]=two.at("version");call("s2","scene_put",s);
 EXPECT_THROW(call("unsafe","scene_load",{{"scene_id","incubator"},{"scene_version",1}}),std::runtime_error);
 call("l2","scene_load",{{"scene_id","incubator"},{"scene_version",1},{"manual_transfer",true}});
 EXPECT_EQ(c->snapshot().at("active_scene").at("scene_id"),"freeze");
 c.reset();c=std::make_unique<Catalog>(root/"store",root/"imports");EXPECT_EQ(c->snapshot().at("transaction").at("state"),"RECOVERY_REQUIRED");
 EXPECT_EQ(c->snapshot().at("active_scene").at("scene_id"),"freeze");EXPECT_EQ(c->snapshot().at("transaction").at("scene_target").at("scene_id"),"incubator");
}
TEST(StoragePath, DurableDefaultsAndExplicitOverride) {
 EXPECT_NE(catalogStorageRoot(""),"/tmp/astribot_map_catalog");
 EXPECT_TRUE(fs::path(catalogStorageRoot("")).is_absolute());
 EXPECT_EQ(catalogAssetRoot(""),(fs::path(catalogStorageRoot(""))/"assets").string());
 EXPECT_EQ(catalogStorageRoot("/tmp/explicit_catalog"),"/tmp/explicit_catalog");
 EXPECT_EQ(catalogAssetRoot("/tmp/explicit_catalog/assets"),"/tmp/explicit_catalog/assets");
}
