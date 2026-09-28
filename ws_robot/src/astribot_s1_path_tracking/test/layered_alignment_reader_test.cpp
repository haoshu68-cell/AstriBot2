#include "astribot_s1_path_tracking/three_phase_controller.hpp"
#include "astribot_s1_robot_geometry/layered_envelope.hpp"
#include "nav2_core/exceptions.hpp"
#include <rcl/time.h>
#include <chrono>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace astribot_s1_path_tracking {
class AlignmentRecordingInner : public nav2_core::Controller {
public:
  std::size_t calls{0};
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &,std::string,
    std::shared_ptr<tf2_ros::Buffer>,std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}
  void setPlan(const nav_msgs::msg::Path &) override {}
  void setSpeedLimit(const double &,const bool &) override {}
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *) override {
    ++calls;geometry_msgs::msg::TwistStamped command;command.header=pose.header;
    command.twist.linear.x=.08;command.twist.linear.y=.03;command.twist.angular.z=.02;
    return command;
  }
};

// Use the existing test friendship; no production accessors or virtual hooks.
class ThreePhaseControllerTestPeer {
public:
  static void init(ThreePhaseController &controller,
    const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap) {
    controller.clock_=node->get_clock();controller.costmap_ros_=costmap;
    controller.tf_=costmap->getTfBuffer();controller.corner_turn_enabled_=true;
    controller.zero_vy_in_follow_=false;controller.approach_enabled_=false;
    controller.inner_=std::make_shared<AlignmentRecordingInner>();
    controller.alignment_collision_.configure(node,costmap);
  }
  static void select(ThreePhaseController &controller,Phase phase,double error=.5) {
    controller.phase_=phase;controller.phase_started_=controller.budget_clock_.now();
    controller.start_heading_valid_=true;controller.start_heading_=error;
    controller.start_alignment_engaged_=false;controller.align_coasting_=false;
    controller.corner_active_=phase==Phase::kAlignCorner;
    controller.corner_position_={0.,0.};controller.corner_heading_=error;
    controller.corner_stop_.reset();controller.has_corner_pose_=false;
    controller.corner_reanchor_pending_=false;controller.corner_position_recovering_=false;
    controller.plan_.header.frame_id="map";controller.plan_.poses.resize(1);
    auto &goal=controller.plan_.poses.front();goal.header=controller.plan_.header;
    goal.pose.position.x=phase==Phase::kAlignGoal?0.:3.;
    goal.pose.orientation.z=std::sin(error/2.);goal.pose.orientation.w=std::cos(error/2.);
  }
  static void require(ThreePhaseController &controller,
    const geometry_msgs::msg::PoseStamped &pose,double target) {
    controller.alignment_collision_.requireRotationClear(pose,target);
  }
  static std::size_t innerCalls(const ThreePhaseController &controller) {
    return std::static_pointer_cast<AlignmentRecordingInner>(controller.inner_)->calls;
  }
};
}  // namespace astribot_s1_path_tracking

namespace {
using Controller=astribot_s1_path_tracking::ThreePhaseController;
using Peer=astribot_s1_path_tracking::ThreePhaseControllerTestPeer;
using Phase=astribot_s1_path_tracking::Phase;
using Envelope=astribot_navigation_msgs::msg::NavigationEnvelopeV2;
using Maps=astribot_slam_msgs::msg::HeightSliceMaps;
using Steady=std::chrono::steady_clock;

void check(bool condition,const char *reason) {
  if(!condition)throw std::runtime_error(reason);
}
geometry_msgs::msg::Polygon rectangle(double x,double y) {
  geometry_msgs::msg::Polygon polygon;
  for(const auto &xy:std::vector<std::pair<double,double>>{{-x,-y},{x,-y},{x,y},{-x,y}}) {
    geometry_msgs::msg::Point32 point;point.x=xy.first;point.y=xy.second;
    polygon.points.push_back(point);
  }
  return polygon;
}
void setTime(const rclcpp::Clock::SharedPtr &clock,double seconds) {
  check(rcl_enable_ros_time_override(clock->get_clock_handle())==RCL_RET_OK,"enable test ROS clock");
  check(rcl_set_ros_time_override(clock->get_clock_handle(),
    static_cast<rcl_time_point_value_t>(std::llround(seconds*1e9)))==RCL_RET_OK,"set test ROS clock");
}
std::string resultOf(const std::function<void()> &operation) {
  try {operation();return {};}
  catch(const nav2_core::PlannerException &error) {return error.what();}
}
void expectReason(const std::function<void()> &operation,const std::string &reason) {
  const auto actual=resultOf(operation);
  if(actual.find(reason)==std::string::npos) {
    throw std::runtime_error("expected "+reason+", got "+(actual.empty()?"success":actual));
  }
}

struct FrozenFixture {
  Envelope envelope;
  Maps maps;
  explicit FrozenFixture(const std::string &base) {
    maps.header.frame_id="map";maps.map_revision="reader-clear";
    maps.profile_revision=std::string(64,'a');maps.ground_reference="world_horizontal_ground";
    maps.evidence_kind="gazebo_collision_geometry";
    maps.height_edges={.05,.5,1.5,2.};maps.layer_names={"base","arm","empty"};
    nav_msgs::msg::OccupancyGrid grid;grid.header=maps.header;
    grid.info.width=grid.info.height=100;grid.info.resolution=.05;
    grid.info.origin.position.x=grid.info.origin.position.y=-2.5;grid.info.origin.orientation.w=1.;
    grid.data.assign(10000,0);maps.grids={grid,grid,grid};
    envelope.header.frame_id=base;envelope.mode=Envelope::FIXED_POSTURE;
    envelope.coordinator_session_id="alignment-reader-fixture";envelope.epoch=7;
    envelope.hold_id="hold-7";envelope.request_id="request-7";
    envelope.attachment_revision="empty-7";envelope.model_revision="model-7";
    envelope.installed_geometry_hash="fixture-installed-footprint";
    envelope.navigation_allowed=envelope.limits.transport_ready=true;
    envelope.limits.lease_s=.3;envelope.limits.height_m=1.4;
    envelope.height_profile_revision=maps.profile_revision;envelope.ground_in_base_m=-.1;
    envelope.installed_footprint=rectangle(.9,.2);
    for(std::size_t i=0;i<3;++i) {
      astribot_navigation_msgs::msg::EnvelopeSlice slice;
      slice.z_min_m=maps.height_edges[i]-.1;slice.z_max_m=maps.height_edges[i+1]-.1;
      if(i==0)slice.footprint=rectangle(.2,.2);
      if(i==1)slice.footprint=rectangle(.9,.08);
      envelope.height_slices.push_back(slice);
    }
    envelope.height_geometry_hash=astribot_s1_robot_geometry::layeredGeometryHash(
      astribot_s1_robot_geometry::envelopePolygonPoints(envelope.installed_footprint),
      envelope.height_slices,base,0.,envelope.height_profile_revision,envelope.ground_in_base_m);
  }
  void stamp(const rclcpp::Time &now,bool stamp_map) {
    envelope.header.stamp=now;envelope.limits.stamp=now;
    envelope.valid_until=now+rclcpp::Duration::from_seconds(.3);
    if(stamp_map) {
      maps.header.stamp=now;
      for(auto &grid:maps.grids)grid.header.stamp=now;
    }
  }
  void blockRotation() {
    maps.map_revision="reader-blocked";
    auto &grid=maps.grids[1];
    const auto x=static_cast<unsigned>((.5-grid.info.origin.position.x)/grid.info.resolution);
    const auto y=static_cast<unsigned>((.5-grid.info.origin.position.y)/grid.info.resolution);
    grid.data.at(static_cast<std::size_t>(y)*grid.info.width+x)=100;
  }
};

void run() {
  auto costmap=std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides({
      rclcpp::Parameter("plugins",std::vector<std::string>{}),
      rclcpp::Parameter("global_frame",std::string("map")),
      rclcpp::Parameter("robot_base_frame",std::string("base_link")),
      rclcpp::Parameter("track_unknown_space",false),
      rclcpp::Parameter("use_sim_time",true)}));
  check(costmap->on_configure(rclcpp_lifecycle::State())==nav2_util::CallbackReturn::SUCCESS,
    "empty costmap configure failed");
  costmap->getCostmap()->resizeMap(100,100,.05,-2.5,-2.5);
  costmap->setRobotFootprintPolygon(std::make_shared<geometry_msgs::msg::Polygon>(rectangle(.2,.2)));
  check(costmap->isCurrent(),"empty configured costmap must be current");

  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("alignment_reader_fixture",
    rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides({rclcpp::Parameter("use_sim_time",true)}));
  auto publisher=std::make_shared<rclcpp::Node>("alignment_fixture_publisher",
    rclcpp::NodeOptions().use_global_arguments(false));
  setTime(node->get_clock(),10.);
  Controller controller;Peer::init(controller,node,costmap);
  auto envelope_pub=publisher->create_publisher<Envelope>("/navigation/envelope_v2",10);
  auto map_pub=publisher->create_publisher<Maps>("/height_maps/snapshot",
    rclcpp::QoS(1).reliable().transient_local());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());executor.add_node(publisher);
  FrozenFixture fixture(costmap->getBaseFrameID());fixture.stamp(node->now(),true);
  geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="map";
  pose.header.stamp=node->now();pose.pose.orientation.w=1.;
  const auto require=[&] {Peer::require(controller,pose,M_PI/2.);};
  // Bounded loopback delivery. No fixture writes the reader's private inbox.
  const auto deliver=[&](bool with_map,const std::string &expected) {
    const auto deadline=Steady::now()+std::chrono::seconds(5);
    std::string observed;
    while(Steady::now()<deadline) {
      envelope_pub->publish(fixture.envelope);
      if(with_map)map_pub->publish(fixture.maps);
      executor.spin_some();observed=resultOf(require);
      if(expected.empty()?observed.empty():observed.find(expected)!=std::string::npos)return;
      if(!observed.empty() && observed.find("ALIGNMENT_ENVELOPE_MISSING")==std::string::npos &&
        observed.find("ALIGNMENT_HEIGHT_MAP_MISSING")==std::string::npos &&
        observed.find("ALIGNMENT_ENVELOPE_STALE")==std::string::npos &&
        observed.find("ALIGNMENT_LAYER_ROTATION_COLLISION")==std::string::npos) {
        throw std::runtime_error("unexpected loopback result: "+observed);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    throw std::runtime_error("loopback delivery timeout; last result: "+observed);
  };

  deliver(false,"ALIGNMENT_HEIGHT_MAP_MISSING");
  for(const auto phase:{Phase::kAlignStart,Phase::kAlignGoal}) {
    Peer::select(controller,phase);
    expectReason([&] {controller.computeVelocityCommands(pose,{},nullptr);},"ALIGNMENT_HEIGHT_MAP_MISSING");
  }
  Peer::select(controller,Phase::kFollow);
  const auto calls=Peer::innerCalls(controller);
  const auto follow=controller.computeVelocityCommands(pose,{},nullptr);
  check(follow.twist.linear.x==.08 && follow.twist.linear.y==.03 && follow.twist.angular.z==.02 &&
    Peer::innerCalls(controller)==calls+1,"FOLLOW changed or read missing height map");
  Peer::select(controller,Phase::kAlignCorner);
  const auto corner=controller.computeVelocityCommands(pose,{},nullptr);
  check(corner.twist.angular.z>0. && corner.twist.linear.x==0. && corner.twist.linear.y==0. &&
    controller.phase()==Phase::kAlignCorner,"CORNER did not retain its existing free-2D rotation path");
  Peer::select(controller,Phase::kAlignGoal,0.);
  const auto settled=controller.computeVelocityCommands(pose,{},nullptr);
  check(settled.twist.angular.z==0. && controller.phase()==Phase::kDone,
    "settled goal must exit without waiting for a height map");

  deliver(true,"");
  check(resultOf(require).empty(),"fresh clear height map must allow rotation");
  setTime(node->get_clock(),10.05);fixture.stamp(node->now(),true);fixture.blockRotation();
  pose.header.stamp=node->now();deliver(true,"ALIGNMENT_LAYER_ROTATION_COLLISION");
  expectReason(require,"ALIGNMENT_LAYER_ROTATION_COLLISION");

  // Keep the newly published envelope fresh while the actual received map expires.
  setTime(node->get_clock(),12.);fixture.stamp(node->now(),false);pose.header.stamp=node->now();
  deliver(false,"ALIGNMENT_LAYER_ROTATION_COLLISION");
  expectReason(require,"ALIGNMENT_LAYER_ROTATION_COLLISION");

  controller.cleanup();executor.remove_node(node->get_node_base_interface());executor.remove_node(publisher);
  check(costmap->on_cleanup(rclcpp_lifecycle::State())==nav2_util::CallbackReturn::SUCCESS,
    "costmap cleanup failed");
  std::cout<<"reader loopback: missing/clear/blocked/expired; START/GOAL gated; FOLLOW/CORNER unchanged; settled exit passed\n";
}
}  // namespace

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  int status=0;
  try {run();}
  catch(const std::exception &error) {std::cerr<<error.what()<<'\n';status=1;}
  rclcpp::shutdown();return status;
}
