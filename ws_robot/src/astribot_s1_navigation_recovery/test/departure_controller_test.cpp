#include "astribot_s1_navigation_recovery/departure_controller.hpp"
#include "astribot_s1_navigation_recovery/navigation_start_assessment.hpp"
#include "astribot_s1_path_tracking/arrival_controller.hpp"
#include "astribot_s1_robot_geometry/layered_envelope.hpp"
#include <rcl/time.h>
#include <chrono>
#include <functional>
#include <iostream>
#include <limits>
#include <thread>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace {
using namespace astribot_s1_path_tracking;
using namespace astribot_s1_navigation_recovery;
using Envelope=astribot_navigation_msgs::msg::NavigationEnvelopeV2;
using Maps=astribot_slam_msgs::msg::HeightSliceMaps;
using Steady=std::chrono::steady_clock;
void check(bool value,const char *why) {if(!value)throw std::runtime_error(why);}
void rejects(const std::function<void()> &call,const char *reason) {
  try {call();}catch(const nav2_core::PlannerException &error) {
    check(std::string(error.what()).find(reason)!=std::string::npos,error.what());return;
  }
  throw std::runtime_error(std::string("missing rejection: ")+reason);
}
geometry_msgs::msg::Polygon rectangle(double x,double y) {
  geometry_msgs::msg::Polygon result;
  for(auto pair:std::vector<std::pair<double,double>>{{-x,-y},{x,-y},{x,y},{-x,y}}) {
    geometry_msgs::msg::Point32 p;p.x=pair.first;p.y=pair.second;result.points.push_back(p);
  }
  return result;
}
void setTime(const rclcpp::Clock::SharedPtr &clock,double seconds) {
  check(rcl_enable_ros_time_override(clock->get_clock_handle())==RCL_RET_OK,"enable ROS clock");
  check(rcl_set_ros_time_override(clock->get_clock_handle(),std::llround(seconds*1e9))==RCL_RET_OK,"set ROS clock");
}
struct Fixture {
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  rclcpp::Node::SharedPtr publisher;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Publisher<Envelope>::SharedPtr envelope_pub;
  rclcpp::Publisher<Maps>::SharedPtr maps_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_pub;
  rclcpp::Publisher<PolicyLease::Message>::SharedPtr policy_pub;
  Envelope envelope;Maps maps;PolicyLease::Message policy;
  DepartureController controller;ArrivalGoalChecker checker;
  EnvelopeGuard assessment_guard;LayeredCollisionReader assessment_reader;
  std::unique_ptr<NavigationStartAssessment> assessment;
  rclcpp::Client<NavigationStartAssessment::Service>::SharedPtr assessment_client;
  double now{100.},slam_stamp{1000.};
  Fixture() {
    map=std::make_shared<nav2_costmap_2d::Costmap2DROS>(rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides({
      rclcpp::Parameter("plugins",std::vector<std::string>{}),rclcpp::Parameter("global_frame",std::string("map")),
      rclcpp::Parameter("robot_base_frame",std::string("base_link")),rclcpp::Parameter("track_unknown_space",false),
      rclcpp::Parameter("footprint_padding",0.),
      rclcpp::Parameter("transform_tolerance",0.),
      rclcpp::Parameter("use_sim_time",true)}));
    check(map->on_configure(rclcpp_lifecycle::State())==nav2_util::CallbackReturn::SUCCESS,"costmap configure");
    map->getCostmap()->resizeMap(100,100,.05,-2.5,-2.5);
    map->setRobotFootprintPolygon(std::make_shared<geometry_msgs::msg::Polygon>(rectangle(.6,.3)));
    unsigned mx,my;check(map->getCostmap()->worldToMap(.4,.25,mx,my),"table cell");
    map->getCostmap()->setCost(mx,my,254);
    node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("departure_controller_fixture",rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides({
      rclcpp::Parameter("use_sim_time",true),rclcpp::Parameter("navigation_geometry_mode",std::string("fixed_v2")),
      rclcpp::Parameter("navigation_policy_enabled",true)}));
    setTime(node->get_clock(),now);setTime(map->get_clock(),now);
    checker.initialize(node,"precise_goal_checker",map);
    controller.configure(node,"Departure",map->getTfBuffer(),map);controller.activate();
    assessment_guard.configure(node,map,"planner");assessment_reader.configure(node,map);
    assessment=std::make_unique<NavigationStartAssessment>(node,map,assessment_guard,assessment_reader);
    publisher=std::make_shared<rclcpp::Node>("departure_evidence_fixture");
    envelope_pub=publisher->create_publisher<Envelope>("/navigation/envelope_v2",10);
    slam_pub=publisher->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/slam/pose",rclcpp::SensorDataQoS());
    maps_pub=publisher->create_publisher<Maps>("/height_maps/snapshot",rclcpp::QoS(1).reliable().transient_local());
    policy_pub=publisher->create_publisher<PolicyLease::Message>("/navigation_policy/constraint",10);
    assessment_client=publisher->create_client<NavigationStartAssessment::Service>("/navigation/assess_start");
    executor.add_node(node->get_node_base_interface());executor.add_node(publisher);
    envelope.header.frame_id="base_link";envelope.mode=Envelope::FIXED_POSTURE;
    envelope.coordinator_session_id="departure-test";envelope.epoch=1;envelope.clock_epoch=1;
    envelope.hold_id="hold";envelope.request_id="request";envelope.attachment_revision="attachment";envelope.model_revision="model";
    envelope.navigation_allowed=envelope.limits.transport_ready=true;envelope.limits.lease_s=.3;envelope.limits.height_m=1.4;
    envelope.installed_footprint=rectangle(.6,.3);envelope.installed_geometry_hash="departure-installed";
    maps.header.frame_id="map";maps.map_revision="clear";maps.profile_revision=std::string(64,'a');
    maps.ground_reference="world_horizontal_ground";maps.evidence_kind="gazebo_collision_geometry";
    maps.height_edges={.05,.5,1.5};maps.layer_names={"base","arm"};
    nav_msgs::msg::OccupancyGrid grid;grid.header=maps.header;grid.info.width=grid.info.height=100;grid.info.resolution=.05;
    grid.info.origin.position.x=grid.info.origin.position.y=-2.5;grid.info.origin.orientation.w=1.;grid.data.assign(10000,0);
    maps.grids={grid,grid};envelope.height_profile_revision=maps.profile_revision;envelope.ground_in_base_m=-.1;
    for(size_t i=0;i<2;++i) {
      astribot_navigation_msgs::msg::EnvelopeSlice slice;slice.z_min_m=maps.height_edges[i]-.1;slice.z_max_m=maps.height_edges[i+1]-.1;
      slice.footprint=i?rectangle(.6,.08):rectangle(.2,.2);envelope.height_slices.push_back(slice);
    }
    envelope.height_geometry_hash=astribot_s1_robot_geometry::layeredGeometryHash(
      astribot_s1_robot_geometry::envelopePolygonPoints(envelope.installed_footprint),envelope.height_slices,"base_link",0.,maps.profile_revision,-.1);
    policy.epoch=1;policy.lease_s=.3;policy.max_linear_speed=.35;policy.max_angular_speed=1.5;
    block(0,.4,.25,100);
    const auto deadline=Steady::now()+std::chrono::seconds(5);
    while((envelope_pub->get_subscription_count()<2||maps_pub->get_subscription_count()<1)&&Steady::now()<deadline) {
      executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    check(envelope_pub->get_subscription_count()>=2&&maps_pub->get_subscription_count()>=1,"reader discovery");
    check(policy_pub->get_subscription_count()==0,"short-segment controller does not subscribe to policy constraints");
    publish();
  }
  ~Fixture() {assessment.reset();assessment_reader.cleanup();assessment_guard.cleanup();controller.cleanup();map->on_cleanup(rclcpp_lifecycle::State());}
  void block(size_t layer,double x,double y,int8_t value) {
    auto &g=maps.grids.at(layer);const unsigned ix=(x-g.info.origin.position.x)/g.info.resolution,iy=(y-g.info.origin.position.y)/g.info.resolution;
    g.data.at(size_t(iy)*g.info.width+ix)=value;maps.map_revision+="x";
  }
  void publish(bool refresh_map=true) {
    setTime(node->get_clock(),now);setTime(map->get_clock(),now);
    envelope.header.stamp=node->now();envelope.limits.stamp=node->now();envelope.valid_until=node->now()+rclcpp::Duration::from_seconds(.3);
    policy.stamp=node->now();++policy.sequence;
    if(refresh_map){maps.header.stamp=node->now();for(auto &g:maps.grids)g.header.stamp=maps.header.stamp;}
    for(int i=0;i<12;++i) {
      envelope_pub->publish(envelope);maps_pub->publish(maps);policy_pub->publish(policy);
      executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  geometry_msgs::msg::PoseStamped pose(double x=0.,double y=0.,double yaw=0.) {
    geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.header.stamp=node->now();p.pose.position.x=x;p.pose.position.y=y;
    p.pose.orientation.z=std::sin(yaw/2);p.pose.orientation.w=std::cos(yaw/2);return p;
  }
  nav_msgs::msg::Path path(double distance=.2) {
    nav_msgs::msg::Path p;p.header=pose().header;p.poses={pose()};if(distance!=0.)p.poses.push_back(pose(-distance));return p;
  }
  geometry_msgs::msg::Twist tick(double x=0.,double y=0.,double yaw=0.,double vx=0.) {
    slam(x,y,yaw);slam(x,y,yaw);
    geometry_msgs::msg::Twist v;v.linear.x=vx;return controller.computeVelocityCommands(pose(x,y,yaw),v,&checker).twist;
  }
  void slam(double x=0.,double y=0.,double yaw=0.) {
    auto p=pose(x,y,yaw);slam_stamp+=.05;p.header.stamp=rclcpp::Time(static_cast<int64_t>(slam_stamp*1e9));
    geometry_msgs::msg::PoseWithCovarianceStamped m;m.header=p.header;m.pose.pose=p.pose;
    slam_pub->publish(m);
    const auto until=Steady::now()+std::chrono::milliseconds(4);
    while(Steady::now()<until){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  }
  void plan(const nav_msgs::msg::Path &path) {
    const auto &p=path.poses.front().pose;
    const auto until=Steady::now()+std::chrono::milliseconds(670);
    while(Steady::now()<until) {slam(p.position.x,p.position.y,tf2::getYaw(p.orientation));
      std::this_thread::sleep_for(std::chrono::milliseconds(20));}
    controller.setPlan(path);
  }
  bool ready() {return checker.isGoalReached({}, {}, {});}
  void actualTransform(double x,double y=0.) {
    geometry_msgs::msg::TransformStamped transform;transform.header.frame_id="map";transform.child_frame_id="base_link";
    transform.header.stamp=node->now();transform.transform.translation.x=x;transform.transform.translation.y=y;transform.transform.rotation.w=1.;
    check(map->getTfBuffer()->setTransform(transform,"departure_fixture",false),"set actual fixture TF");
  }
  NavigationStartAssessment::Service::Response::SharedPtr assess() {
    check(assessment_client->wait_for_service(std::chrono::seconds(2)),"assessment service discovery");
    auto request=std::make_shared<NavigationStartAssessment::Service::Request>();request->execution_id="departure-fixture";request->goal=pose(2.);
    auto future=assessment_client->async_send_request(request);
    const auto deadline=Steady::now()+std::chrono::seconds(2);
    while(future.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready&&Steady::now()<deadline) {
      executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    check(future.wait_for(std::chrono::milliseconds(0))==std::future_status::ready,"assessment service response timeout");
    return future.get();
  }
};
void run() {
  for(const auto &configuration:std::vector<std::pair<bool,std::string>>{{true,"legacy"},{false,"fixed_v2"}}) {
    auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("unsupported_departure_fixture",rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides({
      rclcpp::Parameter("use_sim_time",configuration.first),rclcpp::Parameter("navigation_geometry_mode",configuration.second)}));
    DepartureController disabled;disabled.configure(node,"Departure",nullptr,nullptr);disabled.activate();
    rejects([&]{disabled.setPlan({});},"DEPARTURE_REQUIRES_SIM_FIXED_V2");
    rejects([&]{disabled.computeVelocityCommands({}, {}, nullptr);},"DEPARTURE_REQUIRES_SIM_FIXED_V2");
    disabled.cleanup();
  }
  Fixture f;
  std::cout<<"[scenario] start assessment and actual TF"<<std::endl;
  using Assessment=NavigationStartAssessment::Service::Response;
  check(f.assess()->state==Assessment::UNAVAILABLE,"missing actual TF cannot authorize start");
  f.actualTransform(0.);
  auto result=f.assess();check(result->state==Assessment::RECOVERY_REQUIRED,"2D projection overlap with clear height layers requires recovery");
  check(result->costmap_revision.size()==64&&!result->height_map_revision.empty()&&result->geometry_hash==f.envelope.height_geometry_hash&&result->envelope_epoch==f.envelope.epoch,"assessment evidence identity");
  f.block(0,.1,.1,100);f.now+=.05;f.publish();f.actualTransform(0.);
  check(f.assess()->state==Assessment::BLOCKED,"actual same-layer start intersection is blocked");
  f.block(0,.1,.1,0);f.now+=1.;f.publish();
  check(f.assess()->state==Assessment::RECOVERY_REQUIRED,"latest TF remains valid with fresh geometry");
  f.actualTransform(-.3);result=f.assess();
  check(result->state==Assessment::READY&&std::abs(result->evaluated_start.pose.position.x+.3)<1e-9&&
    result->costmap_revision.size()==64&&result->geometry_hash==f.envelope.height_geometry_hash&&result->envelope_epoch==f.envelope.epoch,
    "actual retreat TF yields READY with geometry evidence");
  std::cout<<"[scenario] reverse limits and measured stop"<<std::endl;
  f.actualTransform(0.);f.plan(f.path());
  auto command=f.tick();check(command.linear.x<0.&&std::hypot(command.linear.x,command.linear.y)<=.05&&command.angular.z==0.,"reverse despite only unrelated-height 2D projection");
  f.controller.setSpeedLimit(.02,false);check(std::abs(f.tick().linear.x)<=.020000001,"absolute speed limit");
  f.controller.setSpeedLimit(50.,true);check(std::abs(f.tick().linear.x)<=.025000001,"percentage speed limit");
  f.controller.setSpeedLimit(0.,false);f.now+=.05;f.publish();
  f.tick(-.195);check(!f.ready(),"arrival requires stop evidence");
  for(int i=0;i<7;++i){f.now+=.1;f.publish();f.tick(-.195);}
  check(!f.ready(),"advancing source time alone cannot complete steady stop window");
  const auto until=Steady::now()+std::chrono::milliseconds(650);
  while(Steady::now()<until){f.now+=.05;f.publish();f.tick(-.195);std::this_thread::sleep_for(std::chrono::milliseconds(25));}
  check(f.ready(),"source plus steady stopped endpoint should complete");
  f.checker.reset();f.tick(-.195);check(!f.ready(),"a new FollowPath goal cannot reuse old stop evidence");
  f.plan(f.path(.15));f.tick();
  command=f.tick(0.,.04);check(command.linear.x<0.&&command.linear.y<0.,"lateral error is controlled without deviation rejection");
  f.plan(f.path());f.tick();
  check(f.tick(0.,0.,.04).linear.x<0.,"heading error does not reject the admitted segment");
  f.plan(f.path(.15));f.tick();
  command=f.tick(-.16);check(command.linear.x>0.&&command.linear.x<=.05,"small overshoot is controlled back to endpoint without rejection");
  rejects([&]{f.plan(f.path(.3));},"SHORT_TRANSLATION_SAME_YAW");
  std::cout<<"[scenario] isolated clear-space translation directions"<<std::endl;
  // The earlier table cell intentionally blocks part of the forward sweep.
  // Direction/heading behavior starts from a separately admitted clear path;
  // the next section changes inputs only after a short segment is accepted.
  f.block(0,.4,.25,0);f.now+=.05;f.publish();
  auto left=f.path();left.poses.back()=f.pose(0.,.2);f.plan(left);
  command=f.tick();check(command.linear.y>0.&&std::abs(command.linear.x)<1e-9&&command.angular.z==0.,"left translation keeps robot heading");
  auto right=f.path();right.poses.back()=f.pose(0.,-.2);f.plan(right);
  command=f.tick();check(command.linear.y<0.&&std::abs(command.linear.x)<1e-9,"right translation");
  f.plan(f.path(-.2));command=f.tick();check(command.linear.x>0.&&std::abs(command.linear.y)<1e-9,"forward translation");
  auto turn=f.path();turn.poses.back()=f.pose(-.2,0.,.1);
  rejects([&]{f.plan(turn);},"SHORT_TRANSLATION_SAME_YAW");
  std::cout<<"[scenario] feedback validity, initial stop and cancellation"<<std::endl;
  f.plan(f.path());
  check(f.tick(0.,0.,0.,.04).linear.x<0.,"SLAM stationary start ignores odom twist");
  f.plan(f.path(.15));
  rejects([&]{f.tick(0.,0.,0.,std::numeric_limits<double>::infinity());},"DEPARTURE_INVALID_FEEDBACK");
  f.plan(f.path());f.tick();f.controller.deactivate();
  rejects([&]{f.tick();},"DEPARTURE_NO_ACTIVE_PATH");
  f.controller.activate();f.plan(f.path());
  auto stale=f.pose();f.now+=1.;f.publish();
  check(f.controller.computeVelocityCommands(stale,{},&f.checker).twist.linear.x<0.,"latest feedback is not rejected due to age");
  stale.header.stamp=f.node->now()+rclcpp::Duration::from_seconds(100.);
  check(f.controller.computeVelocityCommands(stale,{},&f.checker).twist.linear.x<0.,"future feedback is not rejected by local clock");

  std::cout<<"[scenario] SLAM stop evidence is independent of the navigation map origin"<<std::endl;
  f.plan(f.path());
  geometry_msgs::msg::Twist contradictory;contradictory.linear.x=.4;
  command=f.controller.computeVelocityCommands(f.pose(0.,.04),contradictory,&f.checker).twist;
  check(command.linear.x<0.,"control feedback does not replace SLAM initial stop evidence");
  f.slam(0.,.04);check(f.controller.computeVelocityCommands(f.pose(),{},&f.checker).twist.linear.x<0.,
    "SLAM displacement is not compared to the admitted path");
  auto shifted=f.path(.1);shifted.poses={f.pose(.25,.15,M_PI/2),f.pose(.25,.05,M_PI/2)};
  // Exactly the front66 contract: navigation in the static-map world while
  // the independent SLAM monitor has a local zero origin and heading.
  f.plan(f.path(.15));f.controller.setPlan(shifted);
  command=f.controller.computeVelocityCommands(shifted.poses.front(),{},&f.checker).twist;
  check(command.linear.x<0.&&std::abs(command.linear.y)<1e-9,"nonzero map origin and ninety-degree heading allow reverse");
  f.slam(0.,0.,0.);f.controller.computeVelocityCommands(shifted.poses.back(),{},&f.checker);
  check(!f.ready(),"different map origin cannot bypass final stop window");
  const auto shifted_deadline=Steady::now()+std::chrono::seconds(2);
  while(!f.ready()&&Steady::now()<shifted_deadline) {
    f.slam(0.,0.,0.);f.controller.computeVelocityCommands(shifted.poses.back(),{},&f.checker);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }
  check(f.ready(),"shifted navigation endpoint completes with independent SLAM stop evidence");
  std::cout<<"[scenario] admitted short segment does not repeat environment or permit checks"<<std::endl;
  f.block(0,.4,.25,100);f.now+=.05;f.publish();
  f.plan(f.path());check(f.tick().linear.x<0.,"admitted segment starts");
  // These later inputs belong to the stop/recheck boundary, not a second
  // in-motion admission. The executed short path and controller limits remain.
  f.block(0,-.3,0.,100);++f.envelope.epoch;
  f.envelope.navigation_allowed=f.envelope.limits.transport_ready=false;
  f.policy.hold=true;f.policy.max_linear_speed=0.;
  f.now+=2.;f.publish(false);
  command=f.tick(-.05);check(command.linear.x<0.&&std::abs(command.linear.x)<=.05,
    "map age, geometry epoch and policy changes do not readmit the accepted segment");
  f.tick(-.195);check(!f.ready(),"short segment still requires a measured stop");
  const auto deadline=Steady::now()+std::chrono::seconds(2);
  while(!f.ready()&&Steady::now()<deadline) {
    f.now+=.05;f.publish(false);f.tick(-.195);std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }
  check(f.ready(),"accepted short segment completes only with source and steady stopped evidence");
  f.now+=.05;f.publish();f.actualTransform(-.195);
  check(f.assess()->state==Assessment::BLOCKED,"fresh post-stop assessment catches the new actual obstacle");
  f.block(0,-.3,0.,0);f.now+=.05;f.publish();f.actualTransform(0.);
  check(f.assess()->state==Assessment::RECOVERY_REQUIRED,"2D-blocked actual start still requires another recovery step");
  f.actualTransform(-.3);
  check(f.assess()->state==Assessment::READY,"only fresh actual-pose assessment can admit normal navigation");
  f.controller.deactivate();
  std::cout<<"departure controller: admitted translation limits, valid feedback, initial/final stop, cancellation, refreshed post-stop assessment passed\n";
}
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try{run();rclcpp::shutdown();return 0;}
  catch(const std::exception &error){std::cerr<<error.what()<<'\n';rclcpp::shutdown();return 1;}
}
