#include "astribot_map_manager/catalog.hpp"
#include "fixture.hpp"
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
