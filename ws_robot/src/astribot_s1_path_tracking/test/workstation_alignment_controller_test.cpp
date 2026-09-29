#include "workstation_alignment_fixture.hpp"
#include "astribot_s1_path_tracking/workstation_alignment_controller.hpp"
#include "astribot_s1_path_tracking/arrival_controller.hpp"
#include "astribot_navigation_msgs/msg/motion_constraint.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pluginlib/class_loader.hpp"
#include <cassert>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <thread>
using namespace astribot_s1_path_tracking;
namespace {
using Steady=std::chrono::steady_clock;
geometry_msgs::msg::PoseStamped pose(double x,double y,double yaw) {
  geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.pose.position.x=x;p.pose.position.y=y;
  p.pose.orientation.z=std::sin(yaw*.5);p.pose.orientation.w=std::cos(yaw*.5);return p;
}
struct Fixture {
  workstation_test::Geometry geometry;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  rclcpp::Node::SharedPtr peer;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr poses;
  rclcpp::Publisher<astribot_navigation_msgs::msg::MotionConstraint>::SharedPtr constraints;
  rclcpp::Publisher<workstation_test::Snapshot::Maps>::SharedPtr maps;
  rclcpp::Publisher<workstation_test::Snapshot::Envelope>::SharedPtr envelopes;
  pluginlib::ClassLoader<nav2_core::Controller> loader{"nav2_core","nav2_core::Controller"};
  std::shared_ptr<nav2_core::Controller> controller;
  ArrivalGoalChecker checker;
  int64_t source{100000000000000};uint64_t sequence{};
  Fixture() {
    map=std::make_shared<nav2_costmap_2d::Costmap2DROS>(rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides({
      rclcpp::Parameter("plugins",std::vector<std::string>{}),rclcpp::Parameter("global_frame",std::string("map")),
      rclcpp::Parameter("robot_base_frame",std::string("base")),rclcpp::Parameter("track_unknown_space",false)}));
    assert(map->on_configure(rclcpp_lifecycle::State())==nav2_util::CallbackReturn::SUCCESS);
    node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("workstation_controller_fixture",rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides({
      rclcpp::Parameter("precise_goal_checker.xy_goal_tolerance",.002),
      rclcpp::Parameter("precise_goal_checker.yaw_goal_tolerance",.0017453292519943296),
      rclcpp::Parameter("precise_goal_checker.stopped_linear_velocity",.001),
      rclcpp::Parameter("precise_goal_checker.stopped_angular_velocity",.001),
      rclcpp::Parameter("WorkstationAlign.min_linear_speed",.008),
      rclcpp::Parameter("WorkstationAlign.min_angular_speed",.02)}));
    peer=std::make_shared<rclcpp::Node>("workstation_controller_peer");
    executor.add_node(node->get_node_base_interface());executor.add_node(peer);
    checker.initialize(node,"precise_goal_checker",map);
    controller=loader.createSharedInstance("astribot_s1_path_tracking::WorkstationAlignmentController");
    assert(dynamic_cast<ThreePhaseController*>(controller.get())==nullptr);
    controller->configure(node,"WorkstationAlign",map->getTfBuffer(),map);controller->activate();
    poses=peer->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/slam/pose",rclcpp::SensorDataQoS());
    constraints=peer->create_publisher<astribot_navigation_msgs::msg::MotionConstraint>("/navigation_policy/constraint",10);
    maps=peer->create_publisher<workstation_test::Snapshot::Maps>("/height_maps/snapshot",rclcpp::QoS(1).reliable().transient_local());
    envelopes=peer->create_publisher<workstation_test::Snapshot::Envelope>("/navigation/envelope_v2",10);
    const auto deadline=Steady::now()+std::chrono::seconds(3);
    while(poses->get_subscription_count()<1||constraints->get_subscription_count()<1||maps->get_subscription_count()<1||envelopes->get_subscription_count()<1) {
      assert(Steady::now()<deadline);spin();
    }
    publishGeometry();permission(true,false);observe(pose(.25,.15,M_PI/2.));
  }
  ~Fixture() {controller->cleanup();controller.reset();map->on_cleanup(rclcpp_lifecycle::State());}
  void spin(int count=5) {for(int i=0;i<count;++i) {executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}}
  void publishGeometry() {
    geometry.maps->header.stamp=peer->now();for(auto &grid:geometry.maps->grids)grid.header.stamp=geometry.maps->header.stamp;
    geometry.envelope->header.stamp=peer->now();geometry.envelope->limits.stamp=geometry.envelope->header.stamp;
    envelopes->publish(*geometry.envelope);maps->publish(*geometry.maps);spin();
  }
  void permission(bool mode,bool hold,double speed=.06,double angular=.15) {
    astribot_navigation_msgs::msg::MotionConstraint msg;msg.stamp=peer->now();msg.epoch=1;msg.sequence=++sequence;
    msg.workstation_alignment=mode;msg.hold=hold;msg.max_linear_speed=speed;msg.max_angular_speed=angular;
    constraints->publish(msg);spin();
  }
  void observe(const geometry_msgs::msg::PoseStamped &p,bool newer=true) {
    if(newer)source+=10000000;
    geometry_msgs::msg::PoseWithCovarianceStamped msg;msg.header=p.header;msg.header.stamp=rclcpp::Time(source);
    msg.pose.pose=p.pose;poses->publish(msg);spin();
  }
  void plan(const geometry_msgs::msg::PoseStamped &start,const geometry_msgs::msg::PoseStamped &goal) {
    nav_msgs::msg::Path path;path.header.frame_id="map";path.poses={start,goal};checker.reset();controller->setPlan(path);
  }
  geometry_msgs::msg::TwistStamped command() {
    // Deliberately contradictory Nav2 pose and odometry: neither is a station
    // control nor completion input. Only the /slam/pose observation is used.
    auto other=pose(-10.,-10.,0.);geometry_msgs::msg::Twist odom;odom.linear.x=3.;odom.angular.z=2.;
    return controller->computeVelocityCommands(other,odom,&checker);
  }
};
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  {
    auto node=std::make_shared<rclcpp::Node>("workstation_minimum_config_fixture");
    const auto zero=loadWorkstationConfig(node);
    assert(zero.min_linear_speed==0.&&zero.min_angular_speed==0.);
    node->set_parameter(rclcpp::Parameter("WorkstationAlign.min_linear_speed",-.001));
    bool rejected=false;
    try {loadWorkstationConfig(node);}catch(const std::invalid_argument &error) {
      rejected=std::string(error.what())=="WORKSTATION_INVALID_PARAMETER:WorkstationAlign.min_linear_speed";
    }
    assert(rejected);
    node->set_parameter(rclcpp::Parameter("WorkstationAlign.min_linear_speed",.008));
    node->set_parameter(rclcpp::Parameter("WorkstationAlign.min_angular_speed",.16));rejected=false;
    try {loadWorkstationConfig(node);}catch(const std::invalid_argument &error) {
      rejected=std::string(error.what())=="WORKSTATION_MINIMUM_SPEED_EXCEEDS_MAXIMUM";
    }
    assert(rejected);
  }
  {
    Fixture f;const auto start=pose(.25,.15,M_PI/2.);f.plan(start,pose(.45,.15,M_PI/2.));
    auto command=f.command();assert(command.twist.linear.y<0.&&std::abs(command.twist.linear.x)<1e-8);
    f.observe(pose(-.5,-.5,0.),false); // duplicate cannot replace the future-stamped latest observation.
    command=f.command();assert(command.twist.linear.y<0.);
    for(int i=0;i<10;++i) {std::this_thread::sleep_for(std::chrono::milliseconds(30));f.command();}
    f.permission(true,false,.001,.002);command=f.command();
    assert(std::hypot(command.twist.linear.x,command.twist.linear.y)<=.001+1e-12);
    assert(std::abs(command.twist.angular.z)<=.002+1e-12);
    f.permission(true,true);command=f.command();assert(command.twist==geometry_msgs::msg::Twist());
    f.permission(true,false,.01);command=f.command();
    std::cerr<<std::setprecision(17)<<"resumed_policy_cap speed="
      <<std::hypot(command.twist.linear.x,command.twist.linear.y)<<" cap="<<.01
      <<" excess="<<std::hypot(command.twist.linear.x,command.twist.linear.y)-.01<<std::endl;
    assert(std::hypot(command.twist.linear.x,command.twist.linear.y)<=.01+1e-12);

    f.plan(start,pose(.250488,.153701,M_PI/2.+.006222));f.permission(true,false);
    for(int i=0;i<100;++i) {command=f.command();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    assert(std::abs(std::hypot(command.twist.linear.x,command.twist.linear.y)-.008)<1e-8);
    assert(std::abs(command.twist.angular.z-.02)<1e-8);
    f.permission(true,false,.001,.002);command=f.command();
    assert(std::hypot(command.twist.linear.x,command.twist.linear.y)<=.001+1e-12);
    assert(std::abs(command.twist.angular.z)<=.002+1e-12);

    const auto target=pose(.25,.15,M_PI-.00001);f.observe(target);f.plan(target,target);
    f.permission(false,true);assert(f.command().twist==geometry_msgs::msg::Twist());
    f.permission(true,false);assert(f.command().twist==geometry_msgs::msg::Twist());
    std::this_thread::sleep_for(std::chrono::milliseconds(650));f.command();
    assert(!f.checker.isGoalReached({}, {}, {})); // Time alone is not two distinct SLAM samples.
    for(int i=0;i<50;++i) {
      const double e=i%2?.00008:-.00008;
      f.observe(pose(.25+e,.15-e,std::remainder(M_PI-.00001+e*.25,2.*M_PI)));
      f.command();std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    assert(f.checker.isGoalReached({}, {}, {}));

    f.observe(start);f.plan(start,pose(.45,.15,M_PI/2.));f.command();
    f.geometry.obstacle(0,.25,.15);f.publishGeometry();
    bool rejected=false;
    try {f.command();}catch(const nav2_core::PlannerException &error) {
      rejected=std::string(error.what())=="WORKSTATION_EXECUTION_SWEEP_COLLISION_OR_UNKNOWN";
    }
    assert(rejected&&!f.checker.isGoalReached({}, {}, {}));
  }
  rclcpp::shutdown();
  std::cout<<"workstation controller: actual plugin, SLAM-only pose, latest stamps, committed mode/caps, signed settling and new obstacle passed\n";
}
