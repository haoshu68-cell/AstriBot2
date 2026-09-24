// Synthetic ROS Action peers. No inference, Gazebo, hardware or motion endpoints.
#include <astribot_s1_manipulation_perception/pick_planning_client.hpp>
#include <gtest/gtest.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <thread>

namespace pp=astribot::perception_planning;
using namespace std::chrono_literals;
class PickProtocol : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"),"100");
    ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"),"1");
    rclcpp::init(0,nullptr);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}
  template<class A> using Handle=rclcpp_action::ServerGoalHandle<A>;
  rclcpp::Node::SharedPtr peers,client_node;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread server_thread;
  rclcpp_action::Server<pp::Pose>::SharedPtr pose_server;
  rclcpp_action::Server<pp::Grasps>::SharedPtr grasp_server;
  rclcpp_action::Server<pp::Plan>::SharedPtr plan_server;
  std::shared_ptr<Handle<pp::Pose>> ph;
  std::shared_ptr<Handle<pp::Grasps>> gh;
  std::shared_ptr<Handle<pp::Plan>> mh;
  rclcpp::TimerBase::SharedPtr timer;
  std::unique_ptr<pp::PickPlanningClient> client;
  pp::Request request;
  std::string behavior;
  std::atomic<int> pose_cancels{0},grasp_cancels{0},plan_calls{0};
  std::atomic<bool> scene_changed{false};
  void setup(const std::string &mode) {
    behavior=mode;peers=std::make_shared<rclcpp::Node>("m3_synthetic_action_peers");
    client_node=std::make_shared<rclcpp::Node>("m3_owned_planning_client");
    auto accept=[](auto,auto){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;};
    auto cancel=[](auto){return rclcpp_action::CancelResponse::ACCEPT;};
    pose_server=rclcpp_action::create_server<pp::Pose>(peers,"/perception/estimate_object_pose",accept,cancel,
      [this](auto h){ph=h;});
    grasp_server=rclcpp_action::create_server<pp::Grasps>(peers,"/perception/compute_grasps",accept,cancel,
      [this](auto h){gh=h;});
    plan_server=rclcpp_action::create_server<pp::Plan>(peers,"/transport/plan_manipulation",accept,cancel,
      [this](auto h){mh=h;++plan_calls;});
    timer=peers->create_wall_timer(2ms,[this]{serve();});
    executor.add_node(peers);server_thread=std::thread([this]{executor.spin();});
    client=std::make_unique<pp::PickPlanningClient>(client_node);
    const auto deadline=std::chrono::steady_clock::now()+3s;
    while(!client->ready()&&std::chrono::steady_clock::now()<deadline)std::this_thread::sleep_for(5ms);
    ASSERT_TRUE(client->ready());
    request.context={"instance_a","identity_v2","camera","optical","source","processing",
      "cad","cad_hash","grasp_hash",2,3,4,1,"scene_signature","",""};
    request.task_id="task";request.context_id="owner:scene";request.detection_id="frame:0";
    request.segmentation_source="single_instance_fixture";request.visible_instances=1;
    request.object_cloud.header.frame_id="optical";request.object_cloud.header.stamp=client_node->now();
    request.valid_until=client_node->now()+rclcpp::Duration::from_seconds(5);
    // Keep validity exactly capture+5s, not a second acquisition of now().
    request.valid_until=rclcpp::Time(request.object_cloud.header.stamp)+rclcpp::Duration::from_seconds(5);
    request.admission_deadline_steady=std::chrono::steady_clock::now()+std::chrono::milliseconds(500);
    request.result_deadline_steady=request.admission_deadline_steady+std::chrono::milliseconds(4500);
    request.base_from_camera.header=request.object_cloud.header;
    request.base_from_camera.header.frame_id="astribot_torso_base";
    request.base_from_camera.child_frame_id="optical";request.base_from_camera.transform.rotation.w=1;
    request.model_geometry.id="instance_a";request.model_geometry.header.frame_id="cad";
    request.model_geometry.pose.orientation.w=1;
    shape_msgs::msg::SolidPrimitive box;box.type=box.BOX;box.dimensions={.02,.03,.04};
    request.model_geometry.primitives={box};geometry_msgs::msg::Pose p;p.orientation.w=1;
    request.model_geometry.primitive_poses={p};request.scene.world.collision_objects={request.model_geometry};
    request.geometry_revision="synthetic_geometry";request.grasp_registration_revision="synthetic_registration";
    request.grasp_registration_evidence="pick_planning_protocol_test.cpp";
    request.map_grasp=[](const auto &,const auto &) {pp::GraspMapping m;m.grasp_from_tcp.rotation.w=1;m.physical_object_width_m=.03;return m;};
    request.minimum_width_m=.01;request.maximum_width_m=.08;request.approach_m=.03;request.lift_m=.1;
    request.touch_links={"left_pad","right_pad"};
  }
  void serve() {
    const bool hold=behavior=="hold"||behavior=="ignore_cancel";
    if(ph) {
      auto result=std::make_shared<pp::Pose::Result>();
      if(ph->is_canceling()&&behavior!="ignore_cancel") {++pose_cancels;ph->canceled(result);ph.reset();}
      else if(behavior=="pose_fail") {result->reason_code="POSE_FIXTURE_FAILURE";ph->abort(result);ph.reset();}
      else if(!hold&&behavior!="grasp_fail") {
        const auto &g=*ph->get_goal();auto &o=result->observation;
        result->success=true;o.header=g.header;o.object_id=g.object_id;o.source_camera_id=g.camera_id;
        o.source_epoch=g.source_epoch;o.source_model=g.model_id;o.model_revision="cad_hash";
        o.calibration_revision=g.calibration_revision;o.planning_scene_revision=g.planning_scene_revision;
        o.envelope_epoch=g.envelope_epoch;o.valid_until=g.valid_until;o.position_valid=o.orientation_valid=true;
        o.pose.pose.orientation.w=1;o.pose.pose.position.z=1;ph->succeed(result);ph.reset();
      }
    }
    if(gh) {
      auto result=std::make_shared<pp::Grasps::Result>();
      if(gh->is_canceling()&&behavior!="ignore_cancel") {++grasp_cancels;gh->canceled(result);gh.reset();}
      else if(behavior=="grasp_fail") {result->reason_code="GRASP_FIXTURE_FAILURE";gh->abort(result);gh.reset();}
      else if(!hold&&behavior!="pose_fail") {
        const auto &g=*gh->get_goal();pp::Candidate c;
        c.header=c.grasp_pose.header=g.header;c.source_epoch=g.source_epoch;c.object_id=g.object_id;
        c.camera_id=g.camera_id;c.arm_id=g.arm_id;c.model_name="graspnet_baseline_torchscript";
        c.model_revision="grasp_hash";c.calibration_revision=g.calibration_revision;
        c.planning_scene_revision=g.planning_scene_revision;c.envelope_epoch=g.envelope_epoch;
        c.valid_until=g.valid_until;c.geometry_valid=true;c.gripper_width_m=.04;
        c.grasp_pose.pose.orientation.w=1;c.grasp_pose.pose.position.z=1;
        c.candidate_id="first";result->candidates.push_back(c);c.candidate_id="second";result->candidates.push_back(c);
        result->success=true;gh->succeed(result);gh.reset();
      }
    }
    if(mh) {
      auto result=std::make_shared<pp::Plan::Result>();result->context_id=mh->get_goal()->context_id;
      if(behavior=="scene_change")scene_changed=true;
      if(plan_calls==1&&behavior=="success") {result->reason="MTC_NO_SOLUTION:fixture";mh->abort(result);}
      else {
        for(const auto &[name,kind]:std::vector<std::pair<std::string,std::string>>{
          {"PREGRASP","ARM"},{"GRASP_APPROACH","ARM"},{"GRASP_CONFIRM","GRIPPER"},
          {"ATTACH_CONFIRM","ATTACH"},{"LIFT","ARM"},{"TRANSPORT_POSTURE","ARM"}}) {
          astribot_transport_msgs::msg::ManipulationStage s;s.stage_id=name;s.kind=kind;result->stages.push_back(s);
        }
        result->success=true;mh->succeed(result);
      }
      mh.reset();
    }
  }
  pp::Outcome run(std::function<bool()> canceled=[] {return false;}) {
    return client->plan(request,[this]{auto c=request.context;if(scene_changed)c.scene_signature="changed";return c;},canceled);
  }
  void TearDown() override {
    executor.cancel();if(server_thread.joinable())server_thread.join();
    client.reset();timer.reset();ph.reset();gh.reset();mh.reset();
  }
};
TEST_F(PickProtocol, CallsBothInferenceActionsAndUsesNextMtcCandidate) {
  setup("success");const auto out=run();ASSERT_TRUE(out.success)<<out.reason;
  ASSERT_TRUE(out.pick);EXPECT_EQ(out.pick->candidate.candidate_id,"second");
  EXPECT_EQ(plan_calls,2);ASSERT_EQ(out.candidate_rejections.size(),1u);EXPECT_TRUE(out.terminal_confirmed);
}
TEST_F(PickProtocol, PoseFailureCancelsOtherEndpointWithoutHidingOriginal) {
  setup("pose_fail");const auto out=run();EXPECT_FALSE(out.success);EXPECT_TRUE(out.terminal_confirmed);
  EXPECT_EQ(out.reason,"POSE_FAILED:POSE_FIXTURE_FAILURE");EXPECT_EQ(grasp_cancels,1);EXPECT_EQ(plan_calls,0);
}
TEST_F(PickProtocol, GraspFailureCancelsPoseEndpoint) {
  setup("grasp_fail");const auto out=run();EXPECT_FALSE(out.success);EXPECT_TRUE(out.terminal_confirmed);
  EXPECT_EQ(out.reason,"GRASP_FAILED:GRASP_FIXTURE_FAILURE");EXPECT_EQ(pose_cancels,1);
}
TEST_F(PickProtocol, CancellationWaitsForBothTerminalResults) {
  setup("hold");const auto start=std::chrono::steady_clock::now();
  const auto out=run([&]{return std::chrono::steady_clock::now()-start>50ms;});
  EXPECT_EQ(out.reason,"CANCELED");EXPECT_TRUE(out.terminal_confirmed);
  EXPECT_EQ(pose_cancels,1);EXPECT_EQ(grasp_cancels,1);EXPECT_FALSE(out.pick);
}
TEST_F(PickProtocol, SceneChangesDuringPlanningDiscardSuccessfulResult) {
  setup("scene_change");const auto out=run();EXPECT_EQ(out.reason,"CONTEXT_CHANGED");
  EXPECT_FALSE(out.success);EXPECT_FALSE(out.pick);EXPECT_TRUE(out.terminal_confirmed);
}
TEST_F(PickProtocol, CancelAcceptanceWithoutTerminalBlocksReuse) {
  setup("ignore_cancel");const auto start=std::chrono::steady_clock::now();
  auto out=run([&]{return std::chrono::steady_clock::now()-start>50ms;});
  EXPECT_EQ(out.reason,"CANCELED");EXPECT_FALSE(out.terminal_confirmed);EXPECT_FALSE(out.pick);
  out=run();EXPECT_EQ(out.reason,"PREVIOUS_ACTION_TERMINAL_UNCONFIRMED");
}
