#include "astribot_s1_path_tracking/whole_body_collision_critic.hpp"
#include "astribot_s1_path_tracking/arrival_controller.hpp"
#include "pluginlib/class_loader.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

namespace astribot_s1_path_tracking {
class ArrivalControllerTestPeer {
public:
  static void setup(ArrivalController &controller,const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &map) {
    controller.clock_=node->get_clock();controller.tf_=map->getTfBuffer();controller.costmap_=map;
    controller.geometry_guard_.configure(node,map,"controller");
    controller.alignment_collision_.configure(node,map);
  }
  static bool safe(ArrivalController &controller,const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &command,const geometry_msgs::msg::Twist &measured) {
    return controller.safeCommand(pose,command,measured);
  }
};
}

namespace {
using Envelope=astribot_navigation_msgs::msg::NavigationEnvelopeV2;
using Maps=astribot_slam_msgs::msg::HeightSliceMaps;
using Steady=std::chrono::steady_clock;
void check(bool value,const char *why) {if(!value)throw std::runtime_error(why);}
geometry_msgs::msg::Polygon rectangle(float x,float y) {
  geometry_msgs::msg::Polygon result;
  for(auto xy:std::vector<std::pair<float,float>>{{-x,-y},{x,-y},{x,y},{-x,y}}) {
    geometry_msgs::msg::Point32 p;p.x=xy.first;p.y=xy.second;result.points.push_back(p);
  }
  return result;
}
void run() {
  auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides({
      rclcpp::Parameter("plugins",std::vector<std::string>{}),
      rclcpp::Parameter("global_frame",std::string("map")),
      rclcpp::Parameter("robot_base_frame",std::string("base_link")),
      rclcpp::Parameter("footprint_padding",0.),
      rclcpp::Parameter("track_unknown_space",false)}));
  check(map->on_configure(rclcpp_lifecycle::State())==nav2_util::CallbackReturn::SUCCESS,"configure map");
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("whole_body_critic_test",
    rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("navigation_geometry_mode",std::string("fixed_v2"))}));
  auto publisher=std::make_shared<rclcpp::Node>("whole_body_evidence_test");
  mppi::ParametersHandler parameters(node);
  pluginlib::ClassLoader<mppi::critics::CriticFunction> loader("nav2_mppi_controller","mppi::critics::CriticFunction");
  auto critic=loader.createSharedInstance("mppi::critics::WholeBodyCollisionCritic");
  critic->on_configure(node,"FollowPath.inner","FollowPath.inner.WholeBodyCollisionCritic",map,&parameters);
  auto env_pub=publisher->create_publisher<Envelope>("/navigation/envelope_v2",10);
  auto map_pub=publisher->create_publisher<Maps>("/height_maps/snapshot",rclcpp::QoS(1).reliable().transient_local());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());executor.add_node(publisher);
  Maps maps;maps.header.frame_id="map";maps.map_revision="clear";
  maps.profile_revision=std::string(64,'a');maps.ground_reference="horizontal_ground";
  maps.evidence_kind="gazebo_collision_geometry";
  maps.height_edges={.05,.5,1.5};maps.layer_names={"base","arm"};
  nav_msgs::msg::OccupancyGrid grid;grid.header=maps.header;
  grid.info.width=grid.info.height=200;grid.info.resolution=.05;
  grid.info.origin.position.x=grid.info.origin.position.y=-5.;grid.info.origin.orientation.w=1.;
  grid.data.assign(40000,0);maps.grids={grid,grid};
  Envelope envelope;envelope.header.frame_id="base_link";envelope.mode=Envelope::FIXED_POSTURE;
  envelope.coordinator_session_id="test";envelope.epoch=1;envelope.hold_id="hold";
  envelope.installed_geometry_hash="fixed";envelope.height_profile_revision=maps.profile_revision;
  envelope.navigation_allowed=envelope.limits.transport_ready=true;envelope.limits.height_m=1.4;
  envelope.installed_footprint=rectangle(.9,.2);
  for(std::size_t i=0;i<2;++i) {
    astribot_navigation_msgs::msg::EnvelopeSlice slice;
    slice.z_min_m=maps.height_edges[i];slice.z_max_m=maps.height_edges[i+1];
    slice.footprint=i?rectangle(.9,.08):rectangle(.2,.2);envelope.height_slices.push_back(slice);
  }
  envelope.height_geometry_hash=astribot_s1_robot_geometry::layeredGeometryHash(
    astribot_s1_robot_geometry::envelopePolygonPoints(envelope.installed_footprint),
    envelope.height_slices,"base_link",0.,maps.profile_revision,0.);
  const auto discovery=Steady::now()+std::chrono::seconds(5);
  while((env_pub->get_subscription_count()!=1||map_pub->get_subscription_count()!=1)&&Steady::now()<discovery) {
    executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  check(env_pub->get_subscription_count()==1&&map_pub->get_subscription_count()==1,"critic discovery");
  double lease_seconds=.3;
  const auto publish=[&] {
    envelope.header.stamp=maps.header.stamp=node->now();
    envelope.valid_until=node->now()+rclcpp::Duration::from_seconds(lease_seconds);
    for(int i=0;i<10;++i) {
      env_pub->publish(envelope);map_pub->publish(maps);executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  };
  mppi::models::State state;state.reset(2,1);state.pose.header.frame_id="map";state.pose.pose.orientation.w=1.;
  mppi::models::Trajectories trajectories;trajectories.reset(2,1);
  mppi::models::Path path;path.reset(1);
  xt::xtensor<float,1> costs=xt::zeros<float>({2});float dt=.05;
  mppi::CriticData data{state,trajectories,path,costs,dt,false,nullptr,nullptr,std::nullopt,std::nullopt};
  const auto score=[&] {costs.fill(0);data.fail_flag=false;critic->score(data);};
  publish();score();check(costs(0)==0&&costs(1)==0&&!data.fail_flag,"clear rollout");
  // Both endpoint poses are clear. Only the arm's intermediate rotation hits.
  const auto block=[&](std::size_t layer,double x,double y,int8_t value) {
    const unsigned ix=(x+5.)/.05,iy=(y+5.)/.05;
    maps.grids[layer].data[iy*200+ix]=value;maps.map_revision+="x";
  };
  block(1,.5,.5,100);publish();trajectories.yaws(0,0)=M_PI/2;
  score();check(std::isinf(costs(0))&&costs(1)==0&&!data.fail_flag,"arm sweep must reject only rotating rollout");
  check(std::exp(-(costs(0)-costs(1))/.3f)==0.f,"colliding rollout has zero MPPI weight");
  trajectories.yaws(1,0)=M_PI/2;score();
  check(data.fail_flag&&std::isfinite(costs(0))&&std::isfinite(costs(1)),"all rejected has finite costs and failure flag");
  block(1,.5,.5,0);block(0,.5,.5,100);publish();score();
  check(!data.fail_flag&&costs(0)==0&&costs(1)==0,"low obstacle outside base does not hit high arm");
  block(1,.5,.5,-1);publish();score();check(data.fail_flag,"unknown arm space is blocked");
  envelope.navigation_allowed=false;publish();bool revoked=false;
  try {score();}catch(const nav2_core::PlannerException &error) {
    revoked=std::string(error.what()).find("ENVELOPE_REVOKED")!=std::string::npos;
  }
  check(revoked,"cached geometry must not retain revoked motion authority");
  envelope.navigation_allowed=true;block(1,.5,.5,0);block(0,.5,.5,0);publish();
  lease_seconds=-1.;publish();bool expired=false;
  try {score();}catch(const nav2_core::PlannerException &error) {
    expired=std::string(error.what()).find("ENVELOPE_EXPIRED")!=std::string::npos;
  }
  check(expired,"expired envelope must reject motion even with cached geometry");
  lease_seconds=.3;publish();
  {
    using Peer=astribot_s1_path_tracking::ArrivalControllerTestPeer;
    astribot_s1_path_tracking::ArrivalController controller;
    map->getCostmap()->resizeMap(200,200,.05,-5.,-5.);
    map->setRobotFootprintPolygon(std::make_shared<geometry_msgs::msg::Polygon>(envelope.installed_footprint));
    Peer::setup(controller,node,map);
    const auto until=Steady::now()+std::chrono::seconds(5);
    while((env_pub->get_subscription_count()<3||map_pub->get_subscription_count()<2)&&Steady::now()<until) {
      executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    check(env_pub->get_subscription_count()==3&&map_pub->get_subscription_count()==2,"final command reader discovery");
    publish();geometry_msgs::msg::Twist command;command.linear.x=.1;
    check(Peer::safe(controller,state.pose,command,{}),"clear final command");
    block(1,.5,.5,100);publish();command={};command.angular.z=M_PI/2;
    check(!Peer::safe(controller,state.pose,command,{}),"final command must reject the arm arc");
    check(!Peer::safe(controller,state.pose,{},command),"zero command must still check measured motion");
    block(1,.5,.5,0);publish();controller.cleanup();
  }
  // Full configured layer count, all occupied by robot geometry, for timing.
  maps.height_edges={.05,.25,.68,1.18,1.63,2.30};
  maps.layer_names={"low_obstacle","main_nav","torso","head","upper_extension"};
  maps.grids.assign(5,grid);maps.map_revision="five-layer-benchmark";
  envelope.height_slices.clear();envelope.limits.height_m=2.2;++envelope.epoch;
  for(std::size_t i=0;i<5;++i) {
    astribot_navigation_msgs::msg::EnvelopeSlice slice;
    slice.z_min_m=maps.height_edges[i];slice.z_max_m=maps.height_edges[i+1];
    slice.footprint=envelope.installed_footprint;envelope.height_slices.push_back(slice);
  }
  envelope.height_geometry_hash=astribot_s1_robot_geometry::layeredGeometryHash(
    astribot_s1_robot_geometry::envelopePolygonPoints(envelope.installed_footprint),
    envelope.height_slices,"base_link",0.,maps.profile_revision,0.);
  state.reset(2000,56);trajectories.reset(2000,56);costs=xt::zeros<float>({2000});
  for(unsigned i=0;i<2000;++i)for(unsigned k=0;k<56;++k) {
    trajectories.x(i,k)=.01f*(k+1);trajectories.y(i,k)=(int(i%21)-10)*.0005f*(k+1);
    trajectories.yaws(i,k)=(int(i%17)-8)*.001f*(k+1);
  }
  std::vector<double> times;
  for(unsigned i=0;i<105;++i) {
    publish();
    const auto before=Steady::now();score();
    const auto ms=std::chrono::duration<double,std::milli>(Steady::now()-before).count();
    check(!data.fail_flag,"benchmark candidates clear");if(i>=5)times.push_back(ms);
  }
  std::sort(times.begin(),times.end());
  std::cout<<"layers=5 rollouts=2000 steps=56 iterations=100 p50_ms="<<times[49]
    <<" p99_ms="<<times[98]<<" max_ms="<<times.back()<<'\n';
  critic.reset();executor.remove_node(node->get_node_base_interface());executor.remove_node(publisher);
  map->on_cleanup(rclcpp_lifecycle::State());
  std::cout<<"whole body critic: plugin, clear/arm/height/unknown/sweep/all-rejected/revoked passed\n";
}
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {run();rclcpp::shutdown();return 0;}
  catch(const std::exception &error) {std::cerr<<error.what()<<'\n';rclcpp::shutdown();return 1;}
}
