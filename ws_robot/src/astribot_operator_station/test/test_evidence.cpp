#include <gtest/gtest.h>
#include "astribot_operator_station/evidence.hpp"
#include <fstream>
#include <unistd.h>
using namespace astribot_operator_station;
TEST(Evidence, TimelineAndDeletion) {
 std::vector<Json> e={
 {{"sequence",1},{"kind","parameter_snapshot"},{"node","/n"},{"values",{{"speed",parameter_value(3,0.3)},{"axes",parameter_value(7,{1,2})}}}},
 {{"sequence",2},{"kind","parameter_event"},{"node","/n"},{"values",{{"speed",parameter_value(3,0.1)}}}},
 {{"sequence",3},{"kind","parameter_event"},{"node","/n"},{"values",{{"speed",parameter_value(0,nullptr)}}}}
 };
 EXPECT_EQ(parameters_at(e,1)["/n"]["values"]["speed"]["value"],0.3);
 EXPECT_EQ(parameters_at(e,2)["/n"]["values"]["speed"]["value"],0.1);
 EXPECT_EQ(parameters_at(e,3)["/n"]["values"]["speed"]["available"],false);
 EXPECT_EQ(parameters_at(e,3)["/n"]["effective_confirmed"],false);
 EXPECT_TRUE(parameters_at(e,0).empty());
}
TEST(Evidence, GapDoesNotCarryOldValue) {
 std::vector<Json> e={
 {{"sequence",1},{"kind","parameter_snapshot"},{"node","/n"},{"values",{{"x",parameter_value(2,12)}}}},
 {{"sequence",2},{"kind","parameter_gap"},{"node","/n"},{"reason","timeout"}}};
 auto s=parameters_at(e,2); EXPECT_EQ(s["/n"]["quality"],"unknown");EXPECT_FALSE(s["/n"].contains("values"));
 e.push_back({{"sequence",3},{"kind","parameter_snapshot"},{"node","/n"},{"values",{{"x",parameter_value(2,15)}}}});
 EXPECT_EQ(parameters_at(e,3)["/n"]["values"]["x"]["value"],15);
}
TEST(Evidence, InvalidSequenceAndPartialLineRejected) {
 auto path=std::filesystem::temp_directory_path()/("evidence_test_"+std::to_string(getpid()));
 {std::ofstream f(path);f<<"{\"sequence\":2}\n{\"sequence\":1}\n";}
 EXPECT_THROW(read_events(path),std::runtime_error);
 {std::ofstream f(path);f<<"{\"sequence\":";}
 EXPECT_THROW(read_events(path),Json::parse_error);
 std::filesystem::remove(path);
}

TEST(Evidence, EndpointChangeDoesNotCarryOldParameters) {
 std::vector<Json> e={
 {{"sequence",1},{"kind","parameter_snapshot"},{"node","/n"},{"node_instance_id","A"},{"values",{{"x",parameter_value(2,1)}}}},
 {{"sequence",2},{"kind","parameter_event"},{"node","/n"},{"node_instance_id","B"},{"values",{{"y",parameter_value(2,2)}}}}};
 auto state=parameters_at(e,2);EXPECT_FALSE(state["/n"]["values"].contains("x"));
 EXPECT_EQ(state["/n"]["node_instance_id"],"B");
}
TEST(Evidence, DiscoveredGapInvalidatesUncertainPastInterval) {
 std::vector<Json> e={
 {{"sequence",1},{"kind","parameter_snapshot"},{"node","/n"},{"values",{{"x",parameter_value(2,1)}}}},
 {{"sequence",2},{"kind","operator_mark"}},
 {{"sequence",3},{"kind","parameter_gap"},{"node","/n"},{"uncertain_after_sequence",1},{"reason","readback_differs_without_event"}},
 {{"sequence",4},{"kind","parameter_snapshot"},{"node","/n"},{"values",{{"x",parameter_value(2,2)}}}}};
 EXPECT_EQ(parameters_at(e,1)["/n"]["quality"],"observed");
 EXPECT_EQ(parameters_at(e,2)["/n"]["quality"],"unknown");
 EXPECT_EQ(parameters_at(e,4)["/n"]["values"]["x"]["value"],2);
}
