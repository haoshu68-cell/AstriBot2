#include "astribot_s1_navigation_recovery/departure_path.hpp"
#include "astribot_navigation_msgs/srv/plan_start_recovery.hpp"
#include "astribot_navigation_msgs/srv/get_recovery_obstacles.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>
#include <atomic>
#include <iostream>
#include <thread>

namespace {
using namespace astribot_s1_navigation_recovery;
using Snapshot=astribot_s1_robot_geometry::LayeredCollisionSnapshot;
using Service=astribot_navigation_msgs::srv::PlanStartRecovery;
using Clock=std::chrono::steady_clock;
void require(bool condition,const char *why) {if(!condition)throw std::runtime_error(why);}
geometry_msgs::msg::Polygon rectangle(double x,double y) {
  geometry_msgs::msg::Polygon p;
  for(const auto &xy:std::vector<std::pair<double,double>>{{-x,-y},{x,-y},{x,y},{-x,y}}) {
    geometry_msgs::msg::Point32 v;v.x=xy.first;v.y=xy.second;p.points.push_back(v);
  }
  return p;
}
void run() {
  auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>(rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides({
    rclcpp::Parameter("plugins",std::vector<std::string>{}),rclcpp::Parameter("global_frame",std::string("map")),
    rclcpp::Parameter("robot_base_frame",std::string("base")),rclcpp::Parameter("footprint_padding",0.),
    rclcpp::Parameter("track_unknown_space",false)}));
  require(map->on_configure(rclcpp_lifecycle::State())==nav2_util::CallbackReturn::SUCCESS,"map configure");
  map->getCostmap()->resizeMap(160,160,.025,-2.,-2.);
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("connection_recovery_fixture",rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides({
    rclcpp::Parameter("navigation_geometry_mode",std::string("fixed_v2"))}));
  auto peer=std::make_shared<rclcpp::Node>("connection_recovery_peer");
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node->get_node_base_interface());executor.add_node(peer);
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core","nav2_core::GlobalPlanner");
  auto planner=loader.createSharedInstance("astribot_s1_navigation_recovery::DeparturePlanner");
  planner->configure(node,"Departure",map->getTfBuffer(),map);planner->activate();
  auto client=peer->create_client<Service>("/navigation/plan_start_recovery");
  auto envelopes=peer->create_publisher<Snapshot::Envelope>("/navigation/envelope_v2",10);
  auto height_maps=peer->create_publisher<Snapshot::Maps>("/height_maps/snapshot",rclcpp::QoS(1).reliable().transient_local());
  auto setPose=[&](double x,double y,double yaw) {
    geometry_msgs::msg::TransformStamped tf;tf.header.frame_id="map";tf.child_frame_id="base";tf.header.stamp=node->now();
    tf.transform.translation.x=x;tf.transform.translation.y=y;tf.transform.rotation.z=std::sin(yaw/2);tf.transform.rotation.w=std::cos(yaw/2);
    require(map->getTfBuffer()->setTransform(tf,"connection_fixture",false),"actual TF");
  };
  auto query=[&](double heading,const std::string &frame="map",bool use_policy_obstacles=false) {
    require(client->wait_for_service(std::chrono::seconds(2)),"recovery service discovery");
    auto req=std::make_shared<Service::Request>();req->header.frame_id=frame;req->execution_id="actual-candidate-failure";req->required_heading=heading;
    req->use_policy_obstacles=use_policy_obstacles;
    auto future=client->async_send_request(req);const auto end=Clock::now()+std::chrono::seconds(3);
    while(future.wait_for(std::chrono::seconds(0))!=std::future_status::ready&&Clock::now()<end) {
      executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    require(future.wait_for(std::chrono::seconds(0))==std::future_status::ready,"recovery service timeout");return future.get();
  };
  setPose(0.,0.,0.);auto response=query(M_PI/2);
  require(!response->success&&response->path.poses.empty(),"missing geometry must fail without a path");
  auto maps=std::make_shared<Snapshot::Maps>();auto envelope=std::make_shared<Snapshot::Envelope>();
  maps->header.frame_id="map";maps->map_revision="connection-map";maps->profile_revision=std::string(64,'a');
  maps->ground_reference="horizontal_world";maps->evidence_kind="gazebo_collision_geometry";
  maps->height_edges={.05,.5,1.5};maps->layer_names={"base","arm"};
  nav_msgs::msg::OccupancyGrid grid;grid.header=maps->header;grid.info.width=grid.info.height=160;grid.info.resolution=.025;
  grid.info.origin.position.x=grid.info.origin.position.y=-2.;grid.info.origin.orientation.w=1.;grid.data.assign(25600,0);maps->grids={grid,grid};
  envelope->header.frame_id="base";envelope->mode=envelope->FIXED_POSTURE;envelope->epoch=1;envelope->coordinator_session_id="fixture";
  envelope->installed_geometry_hash="fixture";envelope->installed_footprint=rectangle(.6,.2);envelope->height_profile_revision=maps->profile_revision;
  envelope->limits.height_m=1.4;envelope->navigation_allowed=envelope->limits.transport_ready=true;
  for(size_t i=0;i<2;++i) {astribot_navigation_msgs::msg::EnvelopeSlice slice;slice.z_min_m=maps->height_edges[i];slice.z_max_m=maps->height_edges[i+1];slice.footprint=rectangle(i?.6:.2,.2);envelope->height_slices.push_back(slice);}
  envelope->height_geometry_hash=astribot_s1_robot_geometry::layeredGeometryHash(
    astribot_s1_robot_geometry::envelopePolygonPoints(envelope->installed_footprint),envelope->height_slices,
    "base",0.,maps->profile_revision,0.);
  map->setRobotFootprintPolygon(std::make_shared<geometry_msgs::msg::Polygon>(envelope->installed_footprint));
  unsigned ix,iy;require(map->getCostmap()->worldToMap(0.,.45,ix,iy),"obstacle cell");
  map->getCostmap()->setCost(ix,iy,254);maps->grids[0].data[iy*160+ix]=100;
  const auto discovered=Clock::now()+std::chrono::seconds(3);
  while((envelopes->get_subscription_count()<2||height_maps->get_subscription_count()<1)&&Clock::now()<discovered) {
    executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  require(envelopes->get_subscription_count()>=2&&height_maps->get_subscription_count()>=1,"geometry discovery");
  maps->header.stamp=envelope->header.stamp=node->now();for(auto &g:maps->grids)g.header.stamp=maps->header.stamp;
  for(int i=0;i<15;++i) {envelopes->publish(*envelope);height_maps->publish(*maps);executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}
  response=query(M_PI/2);require(response->success&&response->path.poses.size()==2,"known failed turn selects short translation");
  const auto &a=response->path.poses.front().pose;const auto &b=response->path.poses.back().pose;
  require(a.position.x==0.&&a.position.y==0.,"path starts at actual pose");
  require(std::hypot(b.position.x-a.position.x,b.position.y-a.position.y)<=.2+1e-9,"short recovery bound");
  require(a.orientation==b.orientation,"recovery keeps actual orientation");
  Snapshot snapshot(maps,envelope,"map");require(!snapshot.edgeCollision(a.position.x,a.position.y,0.,b.position.x,b.position.y,0.),"selected translation is layered safe");
  setPose(-1.,-1.,.1);response=query(.1);require(response->success&&response->path.poses.size()==1,"clear actual connection has stationary response");
  require(response->path.poses.front().pose.position.x==-1.&&response->path.poses.front().pose.position.y==-1.,"next query uses new actual TF");
  require(!query(.1,"odom")->success,"wrong frame rejected");
  require(!query(std::numeric_limits<double>::quiet_NaN())->success,"invalid heading rejected");

  // The policy service must run independently of the executor currently
  // dispatching PlanStartRecovery; otherwise this fixture deadlocks itself.
  using Obstacles=astribot_navigation_msgs::srv::GetRecoveryObstacles;
  auto policy_node=std::make_shared<rclcpp::Node>("recovery_policy_fixture");
  rclcpp::executors::SingleThreadedExecutor policy_executor;policy_executor.add_node(policy_node);
  std::atomic<int> policy_mode{0},policy_requests{0};
  auto policy_service=policy_node->create_service<Obstacles>("/navigation_policy/recovery_obstacles",
    [&](Obstacles::Request::ConstSharedPtr,Obstacles::Response::SharedPtr result) {
      ++policy_requests;result->header.frame_id="map";result->header.stamp=policy_node->now();
      result->valid=policy_mode.load()!=1;result->reason=result->valid?"READY":"fixture policy observation unavailable";
      if(policy_mode.load()==2) {
        auto obstacle=rectangle(.025,.025);
        for(auto &point:obstacle.points) {point.x+=.45;point.y+=.35;}
        result->obstacles.push_back(obstacle);
      }
    });
  map->getCostmap()->setCost(ix,iy,0);maps->grids[0].data[iy*160+ix]=0;
  maps->map_revision="policy-only-map";maps->header.stamp=node->now();
  for(auto &grid:maps->grids)grid.header.stamp=maps->header.stamp;
  for(int i=0;i<15;++i) {height_maps->publish(*maps);executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}
  setPose(0.,0.,0.);
  std::atomic<bool> running{true};
  std::thread policy_worker([&] {
    while(running.load()) {policy_executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  });
  try {
    response=query(M_PI/2,"map",true);
    require(response->success&&response->path.poses.size()==1,"valid empty policy world permits clear connection");
    require(policy_requests.load()==1,"policy flag makes a real obstacle RPC");
    policy_mode=2;response=query(M_PI/2,"map",true);
    require(response->success&&response->path.poses.size()==2,"policy-only obstacle selects recovery on empty maps");
    require(response->path.poses.back().pose.position.x<0.,"policy-only recovery selects safe backward step");
    require(policy_requests.load()==2,"each recovery plan refreshes policy obstacles");
    policy_mode=1;response=query(M_PI/2,"map",true);
    require(!response->success&&response->path.poses.empty(),"invalid policy snapshot cannot authorize movement");
    require(response->reason=="DEPARTURE_POLICY_UNAVAILABLE: fixture policy observation unavailable","policy failure reason propagates");
    require(policy_requests.load()==3,"invalid snapshot is received through real RPC");
    response=query(M_PI/2,"map",false);
    require(response->success&&response->path.poses.size()==1,"ordinary connection recovery keeps existing map contract");
    require(policy_requests.load()==3,"false policy flag never queries policy service");
  } catch(...) {running=false;policy_worker.join();throw;}
  running=false;policy_worker.join();
  planner->deactivate();planner->cleanup();planner.reset();map->on_cleanup(rclcpp_lifecycle::State());
  std::cout<<"start connection recovery: actual plugin service, missing input, typed heading, bounded layered-safe translation, latest TF, stationary response, invalid requests and real policy-obstacle RPC passed\n";
}
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);try {run();rclcpp::shutdown();return 0;}
  catch(const std::exception &e) {std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
}
