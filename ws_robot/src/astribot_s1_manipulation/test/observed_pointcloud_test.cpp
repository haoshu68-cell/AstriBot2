#include <gtest/gtest.h>
#include <astribot_s1_manipulation/observed_pointcloud_updater.hpp>
#include <moveit/pointcloud_octomap_updater/pointcloud_octomap_updater.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pluginlib/class_loader.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <thread>
#include <sstream>
#include <cmath>

using Observation = astribot_transport_msgs::msg::ObservedOctomap;
using Updater = occupancy_map_monitor::OccupancyMapUpdater;
using Monitor = occupancy_map_monitor::OccupancyMapMonitor;
using namespace std::chrono_literals;

class ObservedCloud : public ::testing::Test {
protected:
  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::unique_ptr<Monitor> monitor;
  std::unique_ptr<Monitor> baseline_monitor;
  std::vector<std::shared_ptr<Updater>> updaters;
  rclcpp::Subscription<Observation>::SharedPtr subscription;
  std::vector<Observation> received;

  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    node = std::make_shared<rclcpp::Node>("observed_cloud_test");
    node->declare_parameter("sensors", std::vector<std::string>{});
    monitor = std::make_unique<Monitor>(node, std::shared_ptr<tf2_ros::Buffer>{}, "base", .1);
    subscription = node->create_subscription<Observation>("/moveit/observed_octomap", rclcpp::QoS(20),
      [this](Observation::ConstSharedPtr value) { received.push_back(*value); });
    executor.add_node(node);
  }
  void TearDown() override {
    // Remove while subscription callback groups are still alive.
    executor.remove_node(node);
    for (auto& updater : updaters) updater->stop();
    updaters.clear();
    baseline_monitor.reset();
    monitor.reset();
    subscription.reset();
    node.reset();
  }
  void spin(std::chrono::milliseconds duration = 100ms) {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor.spin_some();
      std::this_thread::sleep_for(1ms);
    }
  }
  std::shared_ptr<Updater> add(const std::string& name, bool upstream = false, Monitor* target = nullptr,
                             const std::string& sensor_frame = "base") {
    node->declare_parameter(name + ".point_cloud_topic", "/cloud_" + name);
    node->declare_parameter(name + ".sensor_frame", sensor_frame);
    node->declare_parameter(name + ".max_range", 3.0);
    node->declare_parameter(name + ".padding_offset", 0.0);
    node->declare_parameter(name + ".padding_scale", 1.0);
    node->declare_parameter(name + ".point_subsample", 1);
    node->declare_parameter(name + ".max_update_rate", 0.0);
    node->declare_parameter(name + ".filtered_cloud_topic", "/filtered_" + name);
    std::shared_ptr<Updater> updater;
    if (upstream) updater = std::make_shared<occupancy_map_monitor::PointCloudOctomapUpdater>();
    else updater = std::make_shared<astribot_s1_manipulation::ObservedPointCloudUpdater>();
    updater->setMonitor(target ? target : monitor.get());
    EXPECT_TRUE(updater->initialize(node));
    EXPECT_TRUE(updater->setParams(name));
    updater->setTransformCacheCallback([](const auto&, const auto&, auto&) { return true; });
    updater->start(); updaters.push_back(updater);
    return updater;
  }
  sensor_msgs::msg::PointCloud2 cloud(const std::vector<std::array<float, 3>>& points) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = "base"; msg.header.stamp = node->now();
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz"); modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(msg,"x"), y(msg,"y"), z(msg,"z");
    for (const auto& point : points) { *x=point[0]; *y=point[1]; *z=point[2]; ++x; ++y; ++z; }
    return msg;
  }
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher(const std::string& name) {
    auto pub=node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_"+name, rclcpp::SensorDataQoS());
    const auto end=std::chrono::steady_clock::now()+2s;
    while(pub->get_subscription_count()==0 && std::chrono::steady_clock::now()<end) spin(10ms);
    EXPECT_GT(pub->get_subscription_count(),0u);
    spin(50ms); return pub;
  }
  std::unique_ptr<octomap::OcTree> tree(const Observation& observation) {
    return std::unique_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(observation.octomap)));
  }
};

TEST_F(ObservedCloud, PluginCanBeLoadedByActualMonitorLoader) {
  pluginlib::ClassLoader<Updater> loader("moveit_ros_occupancy_map_monitor", "occupancy_map_monitor::OccupancyMapUpdater");
  EXPECT_TRUE(loader.createSharedInstance("astribot_s1_manipulation/ObservedPointCloudUpdater"));
}

TEST_F(ObservedCloud, FreshFreeVoxelDoesNotCertifyTwentyMmTestRodIsAbsent) {
  monitor.reset();
  monitor = std::make_unique<Monitor>(node, std::shared_ptr<tf2_ros::Buffer>{}, "base", .05);
  add("head"); auto pub = publisher("head");
  // Independent geometry: a 20 mm diameter, 160 mm long cylinder has its
  // axis at (1.025, .035), along z in [0, .160]. This background ray misses
  // it, but crosses the same 50 mm cell as part of that cylinder.
  const float x = 2.025f, y = .025f, z = .025f;
  const double axis_distance = std::abs(x * .035 - y * 1.025) / std::hypot(x, y);
  ASSERT_GT(axis_distance, .010);
  const auto input = cloud({{x, y, z}});
  pub->publish(input); spin();
  ASSERT_EQ(received.size(), 1u);
  EXPECT_EQ(received.back().header.stamp, input.header.stamp);
  auto map = tree(received.back());
  auto* cell = map->search(1.025, .035, .025);
  ASSERT_NE(cell, nullptr);
  EXPECT_FALSE(map->isNodeOccupied(cell));
  const auto key = map->coordToKey(1.025, .035, .025);
  const auto& free = received.back().ray_free_keys;
  bool crossed = false;
  for (std::size_t i = 0; i < free.size(); i += 3)
    crossed = crossed || (free[i] == key[0] && free[i + 1] == key[1] && free[i + 2] == key[2]);
  EXPECT_TRUE(crossed);
  // Fresh ray evidence is correct; certifying the entire cell as observed
  // would be incorrect. This is a coverage counterexample, not a camera test.
}

TEST_F(ObservedCloud, TwentyMmRodReturnsOccupyFiveCmMapAfterFreeObservation) {
  monitor.reset();
  monitor = std::make_unique<Monitor>(node, std::shared_ptr<tf2_ros::Buffer>{}, "base", .05);
  add("head"); auto pub = publisher("head");
  pub->publish(cloud({{2.025f, .025f, .025f}})); spin();
  ASSERT_EQ(received.size(), 1u);
  // Synthetic *measured* surface returns from the same 20 x 160 mm rod.
  // This only tests integration; a real camera still has to produce them.
  std::vector<std::array<float, 3>> returns;
  for (int i = 0; i < 16; ++i) returns.push_back({1.015f, .035f, .005f + .01f * i});
  pub->publish(cloud(returns)); spin();
  ASSERT_EQ(received.size(), 2u);
  auto map = tree(received.back());
  for (const auto& point : returns) {
    auto* cell = map->search(point[0], point[1], point[2]);
    ASSERT_NE(cell, nullptr);
    EXPECT_TRUE(map->isNodeOccupied(cell));
  }
  EXPECT_LT(received.front().map_revision, received.back().map_revision);
}

TEST_F(ObservedCloud, AtomicSnapshotRetainsSourceAndFreeRayEvidence) {
  add("head"); auto pub=publisher("head"); auto msg=cloud({{1.05f,.05f,.05f}});
  pub->publish(msg); spin(); ASSERT_EQ(received.size(),1u);
  const auto& observation=received.back(); auto map=tree(observation); ASSERT_TRUE(map);
  EXPECT_EQ(observation.header,msg.header); EXPECT_EQ(observation.source_id,"head");
  EXPECT_FALSE(observation.map_epoch.empty()); EXPECT_GT(observation.map_revision,0u);
  EXPECT_EQ(observation.octomap.header.frame_id,"base");
  EXPECT_GE(rclcpp::Time(observation.integrated_stamp),rclcpp::Time(observation.callback_stamp));
  EXPECT_DOUBLE_EQ(observation.sensor_to_map.rotation.w,1.0);
  EXPECT_TRUE(map->isNodeOccupied(map->search(1.05,.05,.05)));
  ASSERT_FALSE(observation.ray_free_keys.empty()); ASSERT_EQ(observation.ray_free_keys.size()%3,0u);
  for(size_t i=0;i<observation.ray_free_keys.size();i+=3) {
    octomap::OcTreeKey key(observation.ray_free_keys[i],observation.ray_free_keys[i+1],observation.ray_free_keys[i+2]);
    ASSERT_TRUE(map->search(key)); EXPECT_FALSE(map->isNodeOccupied(map->search(key)));
  }
  octomap_msgs::msg::Octomap current;
  {auto lock=monitor->getOcTreePtr()->reading(); ASSERT_TRUE(octomap_msgs::fullMapToMsg(*monitor->getOcTreePtr(),current));}
  EXPECT_EQ(current.data,observation.octomap.data);
}

TEST_F(ObservedCloud, TwoSourcesProduceOrderedImmutableSnapshots) {
  add("head"); add("torso"); auto a=publisher("head"), b=publisher("torso");
  a->publish(cloud({{1.05f,.05f,.05f}})); spin(); ASSERT_EQ(received.size(),1u);
  const auto first=received.back();
  b->publish(cloud({{.05f,1.05f,.05f}})); spin(); ASSERT_EQ(received.size(),2u);
  const auto& second=received.back();
  EXPECT_EQ(first.map_epoch,second.map_epoch); EXPECT_LT(first.map_revision,second.map_revision);
  EXPECT_EQ(second.source_id,"torso");
  auto old_map=tree(first), new_map=tree(second);
  EXPECT_EQ(old_map->search(.05,1.05,.05),nullptr);
  ASSERT_NE(new_map->search(.05,1.05,.05),nullptr);
  EXPECT_TRUE(new_map->isNodeOccupied(new_map->search(.05,1.05,.05)));
  EXPECT_TRUE(new_map->isNodeOccupied(new_map->search(1.05,.05,.05)));
}

TEST_F(ObservedCloud, ModelClearingCannotManufactureFreeEvidence) {
  auto updater=add("head");
  auto shape=std::make_shared<shapes::Sphere>(.2); const auto handle=updater->excludeShape(shape);
  updater->setTransformCacheCallback([handle](const auto&,const auto&,auto& transforms) {
    Eigen::Isometry3d pose=Eigen::Isometry3d::Identity(); pose.translation()=Eigen::Vector3d(1.05,.05,.05);
    transforms[handle]=pose; return true;
  });
  auto pub=publisher("head"); pub->publish(cloud({{1.05f,.05f,.05f}})); spin();
  ASSERT_EQ(received.size(),1u); auto map=tree(received.back());
  ASSERT_NE(map->search(1.05,.05,.05),nullptr);
  EXPECT_FALSE(map->isNodeOccupied(map->search(1.05,.05,.05)));
  EXPECT_TRUE(received.back().ray_free_keys.empty());
}

TEST_F(ObservedCloud, SameInputMapMatchesUpstreamIncludingSelfFilter) {
  auto other_node=std::make_shared<rclcpp::Node>("upstream_monitor");
  other_node->declare_parameter("sensors",std::vector<std::string>{});
  baseline_monitor=std::make_unique<Monitor>(other_node,std::shared_ptr<tf2_ros::Buffer>{},"base",.1);
  auto current=add("head"), old=add("baseline",true,baseline_monitor.get());
  for(auto updater:{current,old}) {
    const auto h=updater->excludeShape(std::make_shared<shapes::Sphere>(.2));
    updater->setTransformCacheCallback([h](const auto&,const auto&,auto& transforms) {
      auto pose=Eigen::Isometry3d::Identity(); pose.translation()=Eigen::Vector3d(.05,.05,1.05);
      transforms[h]=pose;return true;
    });
  }
  auto a=publisher("head"),b=publisher("baseline");
  for(int i=0;i<3;++i) {
    auto msg=cloud({{1.05f,.05f,.05f},{.05f,1.05f,.05f},{.05f,.05f,1.05f},{4.05f,.05f,.05f}});
    a->publish(msg); b->publish(msg); spin();
    octomap_msgs::msg::Octomap lhs,rhs;
    {auto lock=monitor->getOcTreePtr()->reading(); ASSERT_TRUE(octomap_msgs::fullMapToMsg(*monitor->getOcTreePtr(),lhs));}
    {auto lock=baseline_monitor->getOcTreePtr()->reading(); ASSERT_TRUE(octomap_msgs::fullMapToMsg(*baseline_monitor->getOcTreePtr(),rhs));}
    EXPECT_EQ(lhs.data,rhs.data);
  }
}

TEST_F(ObservedCloud, EmptyInvalidAndRepeatedFramesDoNotInventNewCaptureEvidence) {
  add("head"); auto pub=publisher("head"); auto msg=cloud({{1.05f,.05f,.05f}});
  pub->publish(msg);spin();ASSERT_EQ(received.size(),1u);
  pub->publish(msg);spin();ASSERT_EQ(received.size(),2u);
  EXPECT_EQ(received.front().header.stamp,received.back().header.stamp); // Consumer must test SOURCE age.
  const auto before=received.back().octomap.data;
  auto invalid=msg;invalid.data.pop_back(); pub->publish(invalid);spin();EXPECT_EQ(received.size(),2u);
  auto empty=cloud({});pub->publish(empty);spin();ASSERT_EQ(received.size(),3u);
  EXPECT_TRUE(received.back().ray_free_keys.empty());EXPECT_EQ(received.back().octomap.data,before);
  auto nonfinite=cloud({{std::numeric_limits<float>::infinity(),0,0}});
  pub->publish(nonfinite);spin();ASSERT_EQ(received.size(),4u);
  EXPECT_TRUE(received.back().ray_free_keys.empty());EXPECT_EQ(received.back().octomap.data,before);
}

TEST_F(ObservedCloud, MissingTransformDoesNotPublishIntegratedEvidence) {
  add("head");auto pub=publisher("head");auto msg=cloud({{1.05f,0,0}});msg.header.frame_id="missing_camera";
  pub->publish(msg);spin(200ms);EXPECT_TRUE(received.empty());EXPECT_EQ(monitor->getOcTreePtr()->size(),0u);
}

TEST_F(ObservedCloud, MissingCameraOriginDoesNotUseCloudFrameOrigin) {
  add("head",false,nullptr,"missing_camera");auto pub=publisher("head");
  pub->publish(cloud({{1.05f,0,0}}));spin(200ms);
  EXPECT_TRUE(received.empty());EXPECT_EQ(monitor->getOcTreePtr()->size(),0u);
}

TEST_F(ObservedCloud, RejectedRobotTransformsDoNotPublishIntegratedEvidence) {
  auto updater=add("head");
  updater->setTransformCacheCallback([](const auto&,const auto&,auto&){return false;});
  auto pub=publisher("head");pub->publish(cloud({{1.05f,0,0}}));spin();
  EXPECT_TRUE(received.empty());EXPECT_EQ(monitor->getOcTreePtr()->size(),0u);
}

TEST_F(ObservedCloud, SnapshotContainsActualSensorTransform) {
  add("head",false,nullptr,"camera");tf2_ros::StaticTransformBroadcaster broadcaster(node);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id="base";transform.child_frame_id="camera";transform.header.stamp=node->now();
  transform.transform.translation.y=.5;transform.transform.rotation.w=1.;
  broadcaster.sendTransform(transform);spin(100ms);
  auto pub=publisher("head");auto msg=cloud({{1.05f,.05f,.05f}});msg.header.frame_id="camera";
  pub->publish(msg);spin();ASSERT_EQ(received.size(),1u);
  EXPECT_EQ(received.back().sensor_to_map,transform.transform);
  EXPECT_EQ(received.back().header,msg.header);
  auto map=tree(received.back());ASSERT_NE(map->search(1.05,.55,.05),nullptr);
  EXPECT_TRUE(map->isNodeOccupied(map->search(1.05,.55,.05)));
  EXPECT_EQ(map->search(1.05,.05,.05),nullptr);
}

TEST_F(ObservedCloud, BaseFrameCloudRaysStartAtCameraAndUseCameraRange) {
  auto updater=add("head",false,nullptr,"camera");
  updater->stop();node->set_parameter(rclcpp::Parameter("head.max_range",1.0));
  ASSERT_TRUE(updater->setParams("head"));updater->start();
  tf2_ros::StaticTransformBroadcaster broadcaster(node);
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id="base";tf.child_frame_id="camera";tf.header.stamp=node->now();
  tf.transform.translation.y=.5;tf.transform.rotation.w=1.;broadcaster.sendTransform(tf);spin(100ms);
  auto pub=publisher("head");
  // Already transformed into base coordinates, as the real RGB-D producer does.
  // This endpoint is inside 1 m of the camera and outside 1 m of the base origin.
  pub->publish(cloud({{.05f,1.35f,.05f}}));spin();ASSERT_EQ(received.size(),1u);
  auto map=tree(received.back());ASSERT_NE(map->search(.05,1.35,.05),nullptr);
  EXPECT_TRUE(map->isNodeOccupied(map->search(.05,1.35,.05)));
  EXPECT_EQ(map->search(.05,.05,.05),nullptr); // No fictitious ray through the robot origin.
  EXPECT_EQ(received.back().sensor_to_map,tf.transform);
  pub->publish(cloud({{.05f,2.25f,.05f}}));spin();ASSERT_EQ(received.size(),2u);
  auto clipped=tree(received.back());EXPECT_EQ(clipped->search(.05,2.25,.05),nullptr);
  ASSERT_NE(clipped->search(.05,1.45,.05),nullptr);
  EXPECT_FALSE(clipped->isNodeOccupied(clipped->search(.05,1.45,.05)));
}

TEST_F(ObservedCloud, ConcurrentCameraCallbacksKeepRevisionAndTreeTogether) {
  add("head");add("torso");auto a=publisher("head"),b=publisher("torso");
  executor.remove_node(node);
  rclcpp::executors::MultiThreadedExecutor parallel(rclcpp::ExecutorOptions(),2);
  parallel.add_node(node);std::thread worker([&]{parallel.spin();});
  a->publish(cloud({{1.05f,.05f,.05f}}));b->publish(cloud({{.05f,1.05f,.05f}}));
  std::this_thread::sleep_for(200ms);parallel.cancel();worker.join();
  parallel.remove_node(node);executor.add_node(node);
  ASSERT_EQ(received.size(),2u);
  std::sort(received.begin(),received.end(),[](const auto& x,const auto& y){return x.map_revision<y.map_revision;});
  EXPECT_EQ(received[0].map_epoch,received[1].map_epoch);
  EXPECT_LT(received[0].map_revision,received[1].map_revision);
  auto first=tree(received[0]),second=tree(received[1]);
  const bool head_first=received[0].source_id=="head";
  EXPECT_EQ(first->search(head_first?.05:1.05,head_first?1.05:.05,.05),nullptr);
  ASSERT_NE(second->search(1.05,.05,.05),nullptr);ASSERT_NE(second->search(.05,1.05,.05),nullptr);
  EXPECT_TRUE(second->isNodeOccupied(second->search(1.05,.05,.05)));
  EXPECT_TRUE(second->isNodeOccupied(second->search(.05,1.05,.05)));
}

TEST_F(ObservedCloud, SameCloudProcessingBenchmark) {
  if (!std::getenv("ASTRIBOT_OCTOMAP_BENCHMARK")) GTEST_SKIP() << "Explicit performance run only";
  monitor.reset();
  monitor=std::make_unique<Monitor>(node,std::shared_ptr<tf2_ros::Buffer>{},"base",.05);
  auto other_node=std::make_shared<rclcpp::Node>("upstream_benchmark");
  other_node->declare_parameter("sensors",std::vector<std::string>{});
  baseline_monitor=std::make_unique<Monitor>(other_node,std::shared_ptr<tf2_ros::Buffer>{},"base",.05);
  add("head");add("baseline",true,baseline_monitor.get());
  bool head_done=false,baseline_done=false;
  auto head_sub=node->create_subscription<sensor_msgs::msg::PointCloud2>("/filtered_head",rclcpp::SensorDataQoS(),
    [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr){head_done=true;});
  auto baseline_sub=node->create_subscription<sensor_msgs::msg::PointCloud2>("/filtered_baseline",rclcpp::SensorDataQoS(),
    [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr){baseline_done=true;});
  auto current=publisher("head"),original=publisher("baseline");
  std::vector<std::array<float,3>> points;
  for(int row=0;row<360;++row) for(int col=0;col<640;++col)
    points.push_back({2.025f,(col-320)*.004f,(row-180)*.004f});
  std::vector<double> before,after;
  for(int trial=-2;trial<30;++trial) for(int order=0;order<2;++order) {
    const bool observed=(order+(trial&1))%2;
    bool& done=observed?head_done:baseline_done;done=false;
    auto msg=cloud(points);msg.height=360;msg.width=640;msg.row_step=msg.point_step*msg.width;
    const auto started=std::chrono::steady_clock::now();
    (observed?current:original)->publish(msg);
    while(!done && std::chrono::steady_clock::now()-started<2s) {
      executor.spin_some();std::this_thread::sleep_for(10us);
    }
    ASSERT_TRUE(done);
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    if(trial>=0)(observed?after:before).push_back(ms);
  }
  for(const auto& values:{std::pair<const char*,std::vector<double>>{"baseline_ms",before},{"observed_ms",after}}) {
    std::ostringstream out;for(double value:values.second)out<<value<<",";
    RecordProperty(values.first,out.str());
  }
  RecordProperty("input_points",points.size());RecordProperty("resolution_m","0.05");
  RecordProperty("observed_snapshot_bytes",received.back().octomap.data.size());
  RecordProperty("ray_free_key_count",received.back().ray_free_keys.size()/3);
}
