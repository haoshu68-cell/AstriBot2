#include "astribot_operator_backend/arm_preview.hpp"
#include <gtest/gtest.h>
#include <thread>
TEST(ArmPreview, ValidDriftMalformedAndLateResponse) {
 rclcpp::init(0,nullptr);
 auto node=std::make_shared<rclcpp::Node>("arm_preview_test",rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("arm_preview_ttl_sec",.5)}));
 ArmPreview preview(*node);EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("arm_preview_ttl_sec",1.0)).successful);using Plan=astribot_transport_msgs::srv::PlanSkill;
 int calls=0;bool malformed=false;double position=0;
 auto server=node->create_service<Plan>("/transport/plan_skill",[&](Plan::Request::SharedPtr q,Plan::Response::SharedPtr r){
   ++calls;EXPECT_EQ(q->group,"arm_left");EXPECT_EQ(q->operation,"named");r->success=true;
   auto & t=r->trajectory.joint_trajectory;t.joint_names={"joint"};trajectory_msgs::msg::JointTrajectoryPoint p;
   p.positions={position};t.points.push_back(p);p.positions={malformed?NAN:position+.1};p.time_from_start.sec=1;t.points.push_back(p);
 });
 auto joints=node->create_publisher<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS());
 int displays=0;auto sub=node->create_subscription<moveit_msgs::msg::DisplayTrajectory>("/operator/arm_preview",rclcpp::QoS(1).transient_local(),[&](moveit_msgs::msg::DisplayTrajectory::ConstSharedPtr m){if(!m->trajectory.empty())++displays;});
 rclcpp::executors::SingleThreadedExecutor exec;exec.add_node(node);
 auto spin=[&](int ms){auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);while(std::chrono::steady_clock::now()<until){exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));}};
 auto publish=[&]{sensor_msgs::msg::JointState j;j.header.stamp=node->now();j.name={"joint"};j.position={position};joints->publish(j);spin(30);};
 spin(300);publish();std::string state,reason;int completions=0;
 auto done=[&](std::string s,std::string r){state=s;reason=r;++completions;};
 nlohmann::json target={{"group","arm_left"},{"named_target","transport_compact"}};
 preview.plan("one",target,"map-a",done);spin(100);EXPECT_EQ(state,"SUCCEEDED");EXPECT_EQ(preview.status().at("state"),"READY");EXPECT_EQ(displays,1);EXPECT_FALSE(preview.status().at("can_execute").get<bool>());
 position=.04;publish();preview.tick("map-a");EXPECT_EQ(preview.status().at("reason_code"),"ARM.START_STATE_CHANGED");
 malformed=true;preview.plan("bad",target,"map-a",done);spin(100);EXPECT_EQ(state,"FAILED");EXPECT_EQ(displays,1);
 malformed=false;publish();preview.plan("cancel",target,"map-a",done);preview.invalidate("ARM.USER_DISCARDED");spin(100);EXPECT_EQ(state,"CANCELED");EXPECT_EQ(displays,1);EXPECT_EQ(completions,3);EXPECT_EQ(calls,3);
 publish();preview.plan("context",target,"map-a",done);spin(100);preview.tick("map-b");EXPECT_EQ(preview.status().at("reason_code"),"ARM.CONTEXT_CHANGED");
 publish();preview.plan("lease",target,"map-a",done);preview.invalidate("ARM.CONTROL_LOST");spin(100);EXPECT_EQ(preview.status().at("reason_code"),"ARM.CONTROL_LOST");
 publish();preview.plan("expire",target,"map-a",done);spin(650);publish();preview.tick("map-a");EXPECT_EQ(preview.status().at("reason_code"),"ARM.PLAN_EXPIRED");
 sensor_msgs::msg::JointState invalid;invalid.header.stamp.sec=-1;invalid.name={"joint"};invalid.position={0};joints->publish(invalid);spin(30);
 try {preview.plan("invalid-time",target,"map-a",done);FAIL()<<"negative source stamp accepted";}
 catch(const std::runtime_error & error){EXPECT_STREQ(error.what(),"ARM.JOINT_STATE_UNAVAILABLE");}
 EXPECT_EQ(node->count_publishers("/cmd_vel"),0u);
 exec.remove_node(node);rclcpp::shutdown();
}
