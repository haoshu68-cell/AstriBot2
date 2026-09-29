#include <gtest/gtest.h>
#include <astribot_perception_msgs/msg/camera_health.hpp>
#include <astribot_perception_msgs/msg/projection_health.hpp>
#define main constraint_program_main
#include "../src/navigation_constraint_node.cpp"
#undef main
#include <thread>
using namespace std::chrono_literals;
class NavigationConstraintRos:public ::testing::Test {
 protected:
  static void SetUpTestSuite(){rclcpp::init(0,nullptr);}
  static void TearDownTestSuite(){rclcpp::shutdown();}
  void SetUp()override {
    rclcpp::NodeOptions options;options.parameter_overrides({rclcpp::Parameter("use_sim_time",true),
      rclcpp::Parameter("require_arm_speed_limit",require_arm_limit),
      rclcpp::Parameter("required_projection_cameras","[\"head_rgbd\"]")});
    node=std::make_shared<astribot::navigation::NavigationConstraintNode>(options);
    probe=std::make_shared<rclcpp::Node>("navigation_constraint_probe",options);
    profile=astribot::navigation::load_policy_profile(node->get_parameter("profile").as_string(),true);
    raw=probe->create_publisher<astribot_perception_msgs::msg::CameraHealth>("/perception/camera_health/head_rgbd",rclcpp::QoS(1).transient_local());
    processing=probe->create_publisher<astribot_perception_msgs::msg::ProjectionHealth>("/perception/projection_health/head_rgbd",rclcpp::QoS(1).transient_local());
    commands=probe->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_nav_body_raw",10);
    proposals=probe->create_publisher<astribot_navigation_msgs::msg::MotionConstraint>("/navigation_policy/proposed_constraint",10);
    scans=probe->create_publisher<sensor_msgs::msg::LaserScan>("/scan_from_cloud",rclcpp::SensorDataQoS());
    odometry=probe->create_publisher<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS());
    envelopes=probe->create_publisher<astribot_navigation_msgs::msg::RobotEnvelope>("/navigation/robot_envelope",10);
    if(require_arm_limit)arm_limits=probe->create_publisher<nav2_msgs::msg::SpeedLimit>("/navigation_policy/arm_speed_limit",10);
    output=probe->create_subscription<astribot_navigation_msgs::msg::MotionConstraint>("/navigation_policy/constraint",10,[this](astribot_navigation_msgs::msg::MotionConstraint::ConstSharedPtr m){last_constraint=*m;received_constraint=true;});
    diagnostics=probe->create_subscription<std_msgs::msg::String>("/navigation_policy/constraint_state",10,[this](std_msgs::msg::String::ConstSharedPtr m){diagnostic=nlohmann::json::parse(m->data);});
    started=std::chrono::steady_clock::now();exec.add_node(node);exec.add_node(probe);pump(.15);
  }
  void pump(double seconds) {
    const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
    while(std::chrono::steady_clock::now()<end) {
      const auto t=10000000000LL+std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-started).count();
      for(auto n:{std::static_pointer_cast<rclcpp::Node>(node),probe}) {
        auto* c=n->get_clock()->get_clock_handle();ASSERT_EQ(rcl_enable_ros_time_override(c),RCL_RET_OK);ASSERT_EQ(rcl_set_ros_time_override(c,t),RCL_RET_OK);
      }
      const rclcpp::Time now(t);astribot_perception_msgs::msg::CameraHealth h;
      h.camera_id="head_rgbd";h.header.frame_id=h.frame_id="head_optical";h.header.stamp=h.capture_stamp=now;
      h.valid_until=now+rclcpp::Duration::from_seconds(.25);h.source_epoch="raw1";h.valid=true;h.calibration_revision=1;raw->publish(h);
      if(publish_projection) {
        astribot_perception_msgs::msg::ProjectionHealth p;p.header=h.header;p.camera_id=h.camera_id;
        p.processing_epoch=processing_epoch;p.capture_stamp=h.capture_stamp;p.valid_until=h.valid_until;
        p.valid=processing_valid;p.sequence=++health_sequence;processing->publish(p);
      }
      nav_msgs::msg::Odometry o;o.header.stamp=now;odometry->publish(o);
      sensor_msgs::msg::LaserScan scan;scan.header.frame_id=profile.at("base_frame");scan.header.stamp=now;
      scan.range_min=.1;scan.range_max=10.;scan.angle_min=-3.14159265359;scan.angle_increment=2*3.14159265359/720;
      scan.ranges.assign(721,std::numeric_limits<float>::infinity());
      if(obstacle)scan.ranges[360]=.3f;
      scans->publish(scan);
      astribot_navigation_msgs::msg::RobotEnvelope e;e.stamp=now;e.frame_id=profile.at("base_frame");e.epoch=1;
      e.posture_id="fixture";e.transport_ready=true;e.lease_s=.3;
      e.half_length_m=profile.at("half_length_m");e.half_width_m=profile.at("half_width_m");e.height_m=profile.at("height_m");
      e.payload_mass_kg=profile.at("payload_mass_kg");e.max_speed_m_s=profile.at("max_speed_m_s");
      e.max_angular_speed_rad_s=profile.at("max_angular_speed_rad_s");e.max_acceleration_m_s2=profile.at("max_acceleration_m_s2");
      e.brake_deceleration_m_s2=profile.at("brake_deceleration_m_s2");envelopes->publish(e);
      if(publish_proposal) {
        astribot_navigation_msgs::msg::MotionConstraint p;p.stamp=now;p.epoch=1;p.sequence=++proposal_sequence;
        p.lease_s=.3;p.max_linear_speed=.2;p.max_angular_speed=.5;p.reason="FIXTURE_CLEAR";
        p.workstation_alignment=workstation_alignment;p.hold=proposal_hold;proposals->publish(p);
      }
      if(publish_command){geometry_msgs::msg::Twist c;c.linear.x=.1;commands->publish(c);}
      if(publish_arm_limit) {
        nav2_msgs::msg::SpeedLimit limit;
        limit.header.frame_id=arm_frame_matches?profile.at("base_frame").get<std::string>():"wrong_frame";
        last_arm_stamp_ns=arm_stamp_override.value_or(t+arm_stamp_offset_ns);
        limit.header.stamp=rclcpp::Time(last_arm_stamp_ns);limit.percentage=arm_percentage;
        limit.speed_limit=arm_percent;arm_limits->publish(limit);
      }
      exec.spin_some();std::this_thread::sleep_for(10ms);
    }
  }
  void TearDown()override{exec.remove_node(node);exec.remove_node(probe);node.reset();probe.reset();}
  bool publish_projection=false,processing_valid=true,publish_proposal=true,publish_command=true,obstacle=false;
  bool workstation_alignment=false,proposal_hold=false;
  bool require_arm_limit=false,publish_arm_limit=false,arm_percentage=true,arm_frame_matches=true;
  double arm_percent=25.;int64_t last_arm_stamp_ns=0,arm_stamp_offset_ns=0;
  std::optional<int64_t> arm_stamp_override;
  std::string processing_epoch="processing1";uint64_t health_sequence=0,proposal_sequence=0;
  astribot_navigation_msgs::msg::MotionConstraint last_constraint;bool received_constraint=false;
  nlohmann::json profile,diagnostic;std::chrono::steady_clock::time_point started;
  std::shared_ptr<astribot::navigation::NavigationConstraintNode> node;rclcpp::Node::SharedPtr probe;
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::Publisher<astribot_perception_msgs::msg::CameraHealth>::SharedPtr raw;
  rclcpp::Publisher<astribot_perception_msgs::msg::ProjectionHealth>::SharedPtr processing;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commands;
  rclcpp::Publisher<astribot_navigation_msgs::msg::MotionConstraint>::SharedPtr proposals;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scans;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry;
  rclcpp::Publisher<astribot_navigation_msgs::msg::RobotEnvelope>::SharedPtr envelopes;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr arm_limits;
  rclcpp::Subscription<astribot_navigation_msgs::msg::MotionConstraint>::SharedPtr output;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr diagnostics;
};
TEST_F(NavigationConstraintRos, ProcessingFaultHoldsAndRequiresFreshProposalAfterRecovery) {
  pump(.8);ASSERT_TRUE(received_constraint);EXPECT_TRUE(last_constraint.hold);
  EXPECT_DOUBLE_EQ(last_constraint.max_linear_speed,0.);
  publish_projection=true;pump(1.);ASSERT_FALSE(last_constraint.hold);
  EXPECT_DOUBLE_EQ(last_constraint.max_linear_speed,.2);
  processing_valid=false;pump(.15);EXPECT_TRUE(last_constraint.hold);
  EXPECT_DOUBLE_EQ(last_constraint.max_linear_speed,0.);
  publish_command=false;publish_proposal=false;processing_valid=true;processing_epoch="processing2";
  pump(.8);EXPECT_TRUE(last_constraint.hold);
  publish_proposal=true;publish_command=true;pump(1.);EXPECT_FALSE(last_constraint.hold);
  EXPECT_DOUBLE_EQ(last_constraint.max_linear_speed,.2);
  ASSERT_TRUE(diagnostic.contains("projection_ready"));EXPECT_TRUE(diagnostic.at("projection_ready").get<bool>());
  EXPECT_TRUE(diagnostic.contains("requested_command_speed_m_s"));
  EXPECT_FALSE(diagnostic.contains("output_speed_m_s"));
  EXPECT_FALSE(diagnostic.contains("output_publish_start_ns"));
  const auto publishers=node->get_node_graph_interface()->get_publisher_names_and_types_by_node(node->get_name(),node->get_namespace());
  EXPECT_EQ(publishers.count("/navigation_policy/constraint"),1u);
  EXPECT_EQ(publishers.count("/navigation_policy/constraint_state"),1u);
  for(const auto& topic:publishers)for(const auto& type:topic.second) {
    EXPECT_NE(type,"geometry_msgs/msg/Twist") << topic.first;
    EXPECT_NE(type,"astribot_navigation_msgs/msg/EnvelopeApplyStatus") << topic.first;
  }
}

TEST_F(NavigationConstraintRos, ProposalGapHoldsButDoesNotEraseContinuousClearance) {
  publish_projection=true;pump(1.1);ASSERT_TRUE(received_constraint);ASSERT_FALSE(last_constraint.hold);
  publish_proposal=false;pump(.4);ASSERT_TRUE(last_constraint.hold);
  EXPECT_DOUBLE_EQ(last_constraint.max_linear_speed,0.);
  ASSERT_EQ(diagnostic.at("reason"),"POLICY_UNAVAILABLE");
  publish_proposal=true;pump(.2);
  EXPECT_FALSE(last_constraint.hold) << "Fresh proposal must not restart a completed independent clearance interval";
  EXPECT_DOUBLE_EQ(last_constraint.max_linear_speed,.2);
  ASSERT_TRUE(diagnostic.contains("effective_deadline_ns"));
  ASSERT_FALSE(diagnostic.at("hold").get<bool>());
  const auto effective=diagnostic.at("effective_deadline_ns").get<int64_t>();
  EXPECT_LE(effective,diagnostic.at("proposal_deadline_ns").get<int64_t>());
  EXPECT_LE(effective,diagnostic.at("scan_deadline_ns").get<int64_t>());
  EXPECT_LE(effective,diagnostic.at("odom_deadline_ns").get<int64_t>());
}

TEST_F(NavigationConstraintRos, RealObstacleDuringProposalGapRestartsClearance) {
  publish_projection=true;pump(1.1);ASSERT_TRUE(received_constraint);ASSERT_FALSE(last_constraint.hold);
  publish_proposal=false;obstacle=true;pump(.4);ASSERT_TRUE(last_constraint.hold);
  ASSERT_EQ(diagnostic.at("reason"),"INDEPENDENT_SWEEP_RISK");
  publish_proposal=true;obstacle=false;pump(.2);
  EXPECT_TRUE(last_constraint.hold);EXPECT_DOUBLE_EQ(last_constraint.max_linear_speed,0.);
  EXPECT_EQ(diagnostic.at("reason"),"NAVIGATION_CLEAR_CONFIRMATION");
  pump(.6);EXPECT_FALSE(last_constraint.hold);
}

TEST_F(NavigationConstraintRos, FixedGeometryHasNoCommandOrEnvelopeAcknowledgementPublisher) {
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args","-r","__node:=navigation_constraint_fixed_fixture"});
  options.parameter_overrides({rclcpp::Parameter("use_sim_time",true),
    rclcpp::Parameter("navigation_geometry_mode","fixed_v2")});
  auto fixed=std::make_shared<astribot::navigation::NavigationConstraintNode>(options);
  const auto publishers=fixed->get_node_graph_interface()->get_publisher_names_and_types_by_node(fixed->get_name(),fixed->get_namespace());
  EXPECT_EQ(publishers.count("/navigation_policy/constraint"),1u);
  EXPECT_EQ(publishers.count("/navigation_policy/constraint_state"),1u);
  for(const auto& topic:publishers)for(const auto& type:topic.second) {
    EXPECT_NE(type,"geometry_msgs/msg/Twist") << topic.first;
    EXPECT_NE(type,"astribot_navigation_msgs/msg/EnvelopeApplyStatus") << topic.first;
  }
  const auto subscriptions=fixed->get_node_graph_interface()->get_subscriber_names_and_types_by_node(fixed->get_name(),fixed->get_namespace());
  EXPECT_EQ(subscriptions.count("/navigation/envelope_v2"),1u);
  EXPECT_EQ(subscriptions.count("/cmd_vel_nav_body_raw"),1u);
}

class NavigationConstraintArmLimitRos:public NavigationConstraintRos {
 protected:
  void SetUp()override {require_arm_limit=true;NavigationConstraintRos::SetUp();}
};

TEST_F(NavigationConstraintRos, WorkstationReplacesOnlyOrdinarySweep) {
  publish_projection=true;obstacle=true;pump(1.1);
  ASSERT_TRUE(received_constraint);ASSERT_TRUE(last_constraint.hold);
  EXPECT_EQ(last_constraint.reason,"INDEPENDENT_SWEEP_RISK");
  workstation_alignment=true;pump(1.1);
  ASSERT_TRUE(last_constraint.workstation_alignment);ASSERT_FALSE(last_constraint.hold);
  EXPECT_DOUBLE_EQ(last_constraint.max_linear_speed,.2);
  EXPECT_DOUBLE_EQ(last_constraint.max_angular_speed,.5);
  proposal_hold=true;pump(.15);EXPECT_TRUE(last_constraint.hold);
  EXPECT_TRUE(last_constraint.workstation_alignment);EXPECT_DOUBLE_EQ(last_constraint.max_linear_speed,0.);
  proposal_hold=false;processing_valid=false;pump(.15);EXPECT_TRUE(last_constraint.hold);
  EXPECT_DOUBLE_EQ(last_constraint.max_linear_speed,0.);
  processing_valid=true;pump(1.1);ASSERT_FALSE(last_constraint.hold);
  workstation_alignment=false;pump(.15);
  EXPECT_FALSE(last_constraint.workstation_alignment);EXPECT_TRUE(last_constraint.hold);
  EXPECT_EQ(last_constraint.reason,"INDEPENDENT_SWEEP_RISK");
}

TEST_F(NavigationConstraintArmLimitRos, WorkstationRetainsArmCapsAndZeroStop) {
  workstation_alignment=true;publish_projection=true;obstacle=true;pump(1.1);
  ASSERT_TRUE(last_constraint.workstation_alignment);ASSERT_TRUE(last_constraint.hold);
  EXPECT_EQ(last_constraint.reason,"ARM_SPEED_LIMIT_MISSING");
  publish_arm_limit=true;pump(.2);ASSERT_FALSE(last_constraint.hold);
  EXPECT_DOUBLE_EQ(last_constraint.max_linear_speed,.05);
  EXPECT_DOUBLE_EQ(last_constraint.max_angular_speed,.125);
  arm_percent=0.;pump(.15);EXPECT_TRUE(last_constraint.hold);
  EXPECT_EQ(last_constraint.reason,"ARM_SPEED_LIMIT_ZERO");
  EXPECT_TRUE(last_constraint.workstation_alignment);
}

TEST_F(NavigationConstraintArmLimitRos, PercentageTightensCapsAndMissingStaleInvalidSourcesHold) {
  publish_projection=true;pump(1.1);ASSERT_TRUE(received_constraint);
  ASSERT_TRUE(last_constraint.hold);EXPECT_EQ(last_constraint.reason,"ARM_SPEED_LIMIT_MISSING");
  publish_arm_limit=true;pump(.2);ASSERT_FALSE(last_constraint.hold);
  EXPECT_DOUBLE_EQ(last_constraint.max_linear_speed,.05);
  EXPECT_DOUBLE_EQ(last_constraint.max_angular_speed,.125);
  EXPECT_LE(diagnostic.at("effective_deadline_ns").get<int64_t>(),
    diagnostic.at("arm_speed_limit_deadline_ns").get<int64_t>());

  // Repeated old source stamps cannot keep the percentage alive.
  const auto replay_stamp=last_arm_stamp_ns;arm_stamp_override=replay_stamp;
  pump(.6);EXPECT_TRUE(last_constraint.hold);EXPECT_EQ(last_constraint.reason,"ARM_SPEED_LIMIT_STALE");
  arm_stamp_override.reset();pump(.2);ASSERT_FALSE(last_constraint.hold);
  publish_arm_limit=false;pump(.6);
  EXPECT_TRUE(last_constraint.hold);EXPECT_EQ(last_constraint.reason,"ARM_SPEED_LIMIT_STALE");

  publish_arm_limit=true;arm_percent=0.;pump(.1);
  EXPECT_TRUE(last_constraint.hold);EXPECT_EQ(last_constraint.reason,"ARM_SPEED_LIMIT_ZERO");
  for(double invalid:{std::numeric_limits<double>::quiet_NaN(),-1.,101.}) {
    arm_percent=invalid;pump(.1);
    EXPECT_TRUE(last_constraint.hold);EXPECT_EQ(last_constraint.reason,"ARM_SPEED_LIMIT_INVALID");
  }
  arm_percent=25.;arm_percentage=false;pump(.1);
  EXPECT_TRUE(last_constraint.hold);EXPECT_EQ(last_constraint.reason,"ARM_SPEED_LIMIT_INVALID");
  arm_percentage=true;arm_frame_matches=false;pump(.1);
  EXPECT_TRUE(last_constraint.hold);EXPECT_EQ(last_constraint.reason,"ARM_SPEED_LIMIT_INVALID");
  arm_frame_matches=true;arm_stamp_offset_ns=1000000000LL;pump(.1);
  EXPECT_TRUE(last_constraint.hold);EXPECT_EQ(last_constraint.reason,"ARM_SPEED_LIMIT_INVALID");
  arm_stamp_offset_ns=0;arm_stamp_override=replay_stamp;pump(.1);
  EXPECT_TRUE(last_constraint.hold);EXPECT_EQ(last_constraint.reason,"ARM_SPEED_LIMIT_INVALID");
  arm_stamp_override.reset();pump(.2);ASSERT_FALSE(last_constraint.hold);
  EXPECT_DOUBLE_EQ(last_constraint.max_linear_speed,.05);
  arm_percent=101.;obstacle=true;pump(.1);
  EXPECT_TRUE(last_constraint.hold);EXPECT_EQ(last_constraint.reason,"INDEPENDENT_SWEEP_RISK");
}
