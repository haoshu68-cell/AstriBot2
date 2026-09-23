#include <gtest/gtest.h>
#include "astribot_s1_transport_native/resource_journal.hpp"
#include <filesystem>
#include <fstream>
#include <unistd.h>
using astribot::transport::ResourceJournal;
TEST(ResourceDomain, OctalAliasesCannotBypassDomainLock){
 EXPECT_EQ(astribot::transport::canonical_domain("20"),"20");
 EXPECT_EQ(astribot::transport::canonical_domain("0"),"0");
 for(const auto &value:{"024","094","0x14","20 ","-1","233",""})EXPECT_THROW(astribot::transport::canonical_domain(value),std::invalid_argument);
}
class JournalTest:public ::testing::Test {
protected:
 std::string path;
 void SetUp()override{char pattern[]="/tmp/astribot_hold_test_XXXXXX";auto p=mkdtemp(pattern);ASSERT_NE(p,nullptr);path=p;}
 void TearDown()override{std::filesystem::remove_all(path);}
};
TEST_F(JournalTest, LockAndFullRecordSurviveRestart){
 {ResourceJournal j(path+"/lock",path+"/state");EXPECT_FALSE(j.restored());j.append({{"phase",2}});
 EXPECT_THROW(ResourceJournal(path+"/lock",path+"/other"),std::runtime_error);}
 ResourceJournal j(path+"/lock",path+"/state");ASSERT_TRUE(j.restored());EXPECT_EQ(j.restored()->at("phase"),2);
}
TEST_F(JournalTest, IncompleteOrCorruptHistoryIsNotEmpty){
 {std::ofstream f(path+"/state");f<<"{\"phase\":2}";}
 EXPECT_THROW(ResourceJournal(path+"/lock",path+"/state"),std::runtime_error);
 {std::ofstream f(path+"/state");f<<"not json\n";}
 EXPECT_THROW(ResourceJournal(path+"/lock",path+"/state"),nlohmann::json::parse_error);
}
TEST_F(JournalTest, SymlinkAndLegacyCheckpointDenyStartup){
 {std::ofstream f(path+"/target");}symlink((path+"/target").c_str(),(path+"/lock").c_str());
 EXPECT_THROW(ResourceJournal(path+"/lock",path+"/state"),std::runtime_error);
 std::filesystem::remove(path+"/lock");{std::ofstream f(path+"/lock");f<<"{\"unconfirmed_executor\":true}";}
 EXPECT_THROW(ResourceJournal(path+"/lock",path+"/state"),std::runtime_error);
}
TEST_F(JournalTest, LaterLegacyCrashCannotHideBehindOldNativeRelease){
 {ResourceJournal j(path+"/lock",path+"/state");j.append({{"phase",0},{"epoch","old"},{"lease_id","old_1"}});}
 {std::ofstream f(path+"/lock");f<<"{\"unconfirmed_executor\":true}";}
 EXPECT_THROW(ResourceJournal(path+"/lock",path+"/state"),std::runtime_error);
}
TEST_F(JournalTest, EmptyMarkerCannotOverrideDurableNativeHistory){
 {ResourceJournal j(path+"/lock",path+"/state");j.append({{"phase",0},{"epoch","old"},{"lease_id","old_1"}});}
 {std::ofstream f(path+"/lock");}
 EXPECT_THROW(ResourceJournal(path+"/lock",path+"/state"),std::runtime_error);
}
TEST_F(JournalTest, NativeUnresolvedSetsLegacyMarkerAndRestoresQuarantine){
 {ResourceJournal j(path+"/lock",path+"/state");j.append({{"phase",2},{"epoch","boot"},{"lease_id","boot_1"}});}
 {std::ifstream f(path+"/lock");nlohmann::json marker;f>>marker;EXPECT_TRUE(marker.at("unconfirmed_executor").get<bool>());}
 ResourceJournal j(path+"/lock",path+"/state");EXPECT_EQ(j.restored()->at("phase"),2);
}
