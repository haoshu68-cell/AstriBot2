#include "astribot_navigation_zones/store.hpp"
#include "astribot_navigation_zones/snapshot.hpp"
#include <gtest/gtest.h>
#include <unistd.h>
using namespace astribot_navigation_zones;
TEST(ZoneState, SourceClockAndReceiptTimeouts) {
 SnapshotCache cache;Json wire={{"schema_version",1},{"frame","map"},{"boot_id","b"},{"context_id","mapping:s"},{"revision",0},{"sequence",1},{"stamp",10.},{"valid",true},{"regions",Json::array()}};
 EXPECT_TRUE(cache.receive(wire,10.,2.));EXPECT_TRUE(cache.get(10.,2.));EXPECT_FALSE(cache.get(10.,4.1));EXPECT_FALSE(cache.get(8.,2.));EXPECT_FALSE(cache.get(10.,2.,2));
 EXPECT_FALSE(cache.receive(wire,10.,3.));EXPECT_FALSE(cache.get(10.,4.1));wire["sequence"]=2;wire["stamp"]=1.;EXPECT_FALSE(cache.receive(wire,10.,3.));EXPECT_FALSE(cache.get(10.,3.));
}
TEST(ZoneStore, PersistenceConflictAndReviewedMapInheritance) {
 auto dir=std::filesystem::temp_directory_path()/("zones_test_"+std::to_string(getpid()));std::filesystem::remove_all(dir);
 Json zones=Json::array({{{"id","a"},{"name","A"},{"type","wall"},{"enabled",true},{"points",{{0,0},{1,0}}},{"width_m",.1},{"margin_m",.1}}});
 {Store store(dir);store.ensure("mapping:s","/maps/s");auto result=store.replace("one","mapping:s",0,zones,"alice",true);EXPECT_EQ(result.at("revision"),1);EXPECT_EQ(store.replace("one","mapping:s",0,zones,"alice",true),result);
 EXPECT_THROW(store.replace("two","mapping:s",0,zones,"alice",true),std::exception);
 EXPECT_THROW(Store{dir},std::exception);}
 {Store store(dir);EXPECT_EQ(store.context("mapping:s").at("regions").size(),1u);store.ensure("map:m:v","","/maps/s");EXPECT_TRUE(store.context("map:m:v").at("review_required"));store.replace("review","map:m:v",1,zones,"alice",true);EXPECT_FALSE(store.context("map:m:v").at("review_required"));}
 std::filesystem::remove_all(dir);
}

TEST(ZoneState, InvalidInputDoesNotEraseReplayFence) {
 SnapshotCache cache;Json w={{"schema_version",1},{"frame","map"},{"boot_id","b"},{"context_id","mapping:s"},{"revision",2},{"sequence",5},{"stamp",10.},{"valid",true},{"regions",Json::array()}};
 ASSERT_TRUE(cache.receive(w,10.,2.));cache.invalidate();
 EXPECT_FALSE(cache.receive(w,10.,2.1));EXPECT_FALSE(cache.get(10.,2.1));
 w["sequence"]=6;w["revision"]=1;EXPECT_FALSE(cache.receive(w,10.,2.2));
 w["sequence"]=7;w["revision"]=2;EXPECT_TRUE(cache.receive(w,10.,2.3));
}

TEST(ZoneStore, PortableMapManifestPreservesGeometryAndRequiresReview) {
 auto dir=std::filesystem::temp_directory_path()/("zones_archive_"+std::to_string(getpid()));std::filesystem::remove_all(dir);
 Json regions=Json::array({{{"id","a"},{"name","A"},{"enabled",true},{"type","wall"},{"points",{{0,0},{1,0}}},{"width_m",.1}}});
 {Store store(dir);Json archive={{"schema_version",1},{"frame","map"},{"context_id","mapping:original"},{"revision",9},{"regions",regions}};store.ensure("map:relocated:v2","","/different/computer",archive);auto saved=store.context("map:relocated:v2");EXPECT_EQ(saved.at("source_context"),"mapping:original");EXPECT_TRUE(saved.at("review_required"));EXPECT_EQ(saved.at("regions").size(),1u);}
 std::filesystem::remove_all(dir);
}
