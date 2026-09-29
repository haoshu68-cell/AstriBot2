#define ASTRIBOT_MAPPING_SESSION_NO_MAIN
#include "../src/mapping_session_node.cpp"
#include <gtest/gtest.h>
#include <unistd.h>
using namespace astribot_s1_autonomy;

static void makeSession(const fs::path & dir) {
  fs::create_directories(dir / "kf");
  std::ofstream yaml(dir / (dir.filename().string() + ".yaml"));
  yaml << "image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n"; yaml.close();
  std::ofstream pgm(dir / "map.pgm", std::ios::binary); pgm << "P5\n4 4\n255\n" << std::string(16, char(254));
  std::ofstream poses(dir / "alidarState.txt");
  for (int i = 0; i < 26; ++i) poses << (i == 7 ? 1 : 0) << (i == 25 ? '\n' : ' ');
  std::ofstream cloud(dir / "kf/0.pcd"); cloud << std::string(120, 'x');
}
TEST(SessionArchive, RejectsIncompleteAndCommitsCompatibleManifest) {
  auto dir = fs::temp_directory_path() / ("session_archive_" + std::to_string(getpid()));
  makeSession(dir);
  auto result = inspectSession(dir); EXPECT_EQ(result["scans"], 1);
  commitSession(dir, result, 1); EXPECT_NO_THROW(commitSession(dir, result, 1));
  std::ofstream(dir / "map.pgm", std::ios::app) << 'x';
  EXPECT_THROW(inspectSession(dir), std::runtime_error);
  fs::remove_all(dir);
}

class SessionTest : public ::testing::Test {
protected:
  virtual bool requireZones()const{return false;}
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::shared_ptr<MappingSession> session;
  rclcpp::Node::SharedPtr slam, driver;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
  rclcpp::Publisher<astribot_slam_msgs::msg::KeyframePoseArray>::SharedPtr final;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr finalize, retry;
  fs::path dir;
  nlohmann::json status;
  void SetUp() override {
    rclcpp::init(0, nullptr);
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    dir = fs::temp_directory_path() / ("session_integration_" + std::to_string(getpid()));
    makeSession(dir);
    slam = std::make_shared<rclcpp::Node>("fake_voxel");
    slam->declare_parameter("General.save_path", dir.parent_path().string());
    slam->declare_parameter("General.mapname", dir.filename().string());
    slam->declare_parameter("General.is_save_map", 1);
    slam->declare_parameter("finish", false);
    rclcpp::NodeOptions options; options.parameter_overrides({
      rclcpp::Parameter("slam_node", "/fake_voxel"), rclcpp::Parameter("odom_topic", "/fake_odom"),
      rclcpp::Parameter("require_navigation_zones",requireZones()),rclcpp::Parameter("save_timeout_sec", 3.0), rclcpp::Parameter("settle_sec", 0.15)});
    session = std::make_shared<MappingSession>(options);
    driver = std::make_shared<rclcpp::Node>("mapping_test_driver");
    odom = driver->create_publisher<nav_msgs::msg::Odometry>("/fake_odom", 10);
    final = driver->create_publisher<astribot_slam_msgs::msg::KeyframePoseArray>("/voxel_slam/keyframe_pose_array", 10);
    status_sub = driver->create_subscription<std_msgs::msg::String>("/mapping_session/status", 10,
      [this](std_msgs::msg::String::ConstSharedPtr m) {status = nlohmann::json::parse(m->data);});
    finalize = driver->create_client<std_srvs::srv::Trigger>("/mapping_session/finalize");
    retry = driver->create_client<std_srvs::srv::Trigger>("/mapping_session/retry");
    executor->add_node(session); executor->add_node(slam); executor->add_node(driver); spin(300);
  }
  void spin(int ms, double speed = -1., bool stale = false) {
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) {
      if (speed >= 0) {
        nav_msgs::msg::Odometry msg; msg.header.stamp = stale ? rclcpp::Time(0) : driver->now();
        msg.twist.twist.linear.x = speed; odom->publish(msg);
      }
      executor->spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  bool call(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client) {
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    for (int i = 0; i < 100 && future.wait_for(std::chrono::seconds(0)) != std::future_status::ready; ++i) spin(10);
    if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) throw std::runtime_error("Service timeout");
    return future.get()->success;
  }
  void TearDown() override {
    executor->remove_node(session); executor->remove_node(slam); executor->remove_node(driver);
    session.reset(); slam.reset(); driver.reset(); executor.reset(); rclcpp::shutdown(); fs::remove_all(dir);
  }
};
TEST_F(SessionTest, WaitsForStopAndFinalEventThenCommitsOnce) {
  EXPECT_FALSE(call(retry));
  ASSERT_TRUE(call(finalize)); spin(250, .2);
  EXPECT_FALSE(slam->get_parameter("finish").as_bool());
  spin(250, 0., true); EXPECT_FALSE(slam->get_parameter("finish").as_bool());
  auto duplicate = driver->create_publisher<nav_msgs::msg::Odometry>("/fake_odom", 10);
  spin(300, 0.); EXPECT_FALSE(slam->get_parameter("finish").as_bool());
  duplicate.reset();
  spin(500, 0.); EXPECT_TRUE(slam->get_parameter("finish").as_bool());
  EXPECT_FALSE(fs::exists(dir / "manifest.json"));
  astribot_slam_msgs::msg::KeyframePoseArray msg; msg.is_final = true;
  msg.save_dir = dir.string(); msg.map_name = "wrong_session"; final->publish(msg); spin(150);
  EXPECT_FALSE(fs::exists(dir / "manifest.json"));
  msg.map_name = dir.filename().string(); msg.updates.resize(1); final->publish(msg); spin(350);
  EXPECT_EQ(status["state"], "SAVED"); EXPECT_TRUE(fs::exists(dir / "manifest.json"));
  EXPECT_TRUE(call(finalize)); EXPECT_EQ(status["state"], "SAVED");
}
TEST_F(SessionTest, DisabledSavingFailsWithoutFinishAndCanRetry) {
  slam->set_parameter(rclcpp::Parameter("General.is_save_map", 0));
  ASSERT_TRUE(call(finalize)); spin(500, 0.);
  EXPECT_EQ(status["state"], "FAILED"); EXPECT_FALSE(slam->get_parameter("finish").as_bool());
  EXPECT_FALSE(call(finalize));
  slam->set_parameter(rclcpp::Parameter("General.is_save_map", 1));
  EXPECT_TRUE(call(retry)); spin(500, 0.); EXPECT_TRUE(slam->get_parameter("finish").as_bool());
}
TEST_F(SessionTest, MissingOdometryTimesOutWithoutFinish) {
  ASSERT_TRUE(call(finalize)); spin(3200);
  EXPECT_EQ(status["state"], "FAILED"); EXPECT_FALSE(slam->get_parameter("finish").as_bool());
}
TEST_F(SessionTest, FinalSignalAloneIsNotSavedAndRetryOnlyValidatesFiles) {
  auto cancel = driver->create_client<std_srvs::srv::Trigger>("/mapping_session/finalize_canceled");
  ASSERT_TRUE(call(cancel)); spin(500, 0.);
  ASSERT_TRUE(slam->get_parameter("finish").as_bool());
  fs::remove(dir / "map.pgm");
  astribot_slam_msgs::msg::KeyframePoseArray msg; msg.is_final = true;
  msg.save_dir = dir.string(); msg.map_name = dir.filename().string(); msg.updates.resize(1);
  final->publish(msg); spin(2800);
  EXPECT_EQ(status["state"], "FAILED"); EXPECT_FALSE(fs::exists(dir / "manifest.json"));
  makeSession(dir); EXPECT_TRUE(call(retry)); spin(400);
  EXPECT_EQ(status["state"], "SAVED");
  std::ifstream file(dir / "manifest.json"); nlohmann::json metadata; file >> metadata;
  EXPECT_EQ(metadata["exploration_outcome"], "CANCELED_PARTIAL");
}

class ZonesSessionTest:public SessionTest{protected:bool requireZones()const override{return true;}};
TEST_F(ZonesSessionTest, SaveFreezesCurrentMapBoundZonesIntoManifest) {
 ASSERT_TRUE(call(finalize));spin(650,0.);EXPECT_FALSE(slam->get_parameter("finish").as_bool());
 auto pub=driver->create_publisher<std_msgs::msg::String>("/navigation_zones/constraints",rclcpp::QoS(1).transient_local());
 uint64_t sequence=0;const auto context="mapping:"+status.at("session_id").get<std::string>();
 nlohmann::json regions=nlohmann::json::array({{{"id","wall"},{"name","wall"},{"enabled",true},{"type","wall"},{"points",{{0,0},{1,0}}},{"width_m",.1}}});
 auto timer=driver->create_wall_timer(std::chrono::milliseconds(30),[&]{std_msgs::msg::String m;m.data=nlohmann::json({{"schema_version",1},{"frame","map"},{"boot_id","zones"},{"context_id",context},{"revision",7},{"sequence",++sequence},{"valid",true},{"stamp",driver->now().seconds()},{"regions",regions}}).dump();pub->publish(m);});
 spin(450,0.);EXPECT_TRUE(slam->get_parameter("finish").as_bool());
 astribot_slam_msgs::msg::KeyframePoseArray m;m.is_final=true;m.save_dir=dir.string();m.map_name=dir.filename().string();m.updates.resize(1);final->publish(m);spin(400);
 ASSERT_EQ(status.at("state"),"SAVED");std::ifstream file(dir/"manifest.json");nlohmann::json manifest;file>>manifest;EXPECT_EQ(manifest.at("navigation_zones").at("context_id"),context);EXPECT_EQ(manifest.at("navigation_zones").at("revision"),7);EXPECT_EQ(manifest.at("navigation_zones").at("regions").size(),1u);
}
