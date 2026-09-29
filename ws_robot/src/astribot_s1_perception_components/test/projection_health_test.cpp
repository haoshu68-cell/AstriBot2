#include <gtest/gtest.h>
#include <thread>
#include <signal.h>
#include <astribot_perception_msgs/msg/projection_health.hpp>
#include "astribot_s1_perception_components/projection_process.hpp"
#define main projection_health_node_main
#include "../src/rgbd_pointcloud_node.cpp"
#undef main
using namespace std::chrono_literals;
class ProcessingHealthContract:public ::testing::Test {
protected:
  using H=astribot_perception_msgs::msg::ProjectionHealth;
  static void SetUpTestSuite(){rclcpp::init(0,nullptr);}
  static void TearDownTestSuite(){rclcpp::shutdown();}
  void start(const std::string& mode="",bool fallback=false,double worker_timeout=.08) {
    rclcpp::NodeOptions opts;opts.parameter_overrides({rclcpp::Parameter("depth_topic","/ph/depth"),
      rclcpp::Parameter("camera_info_topic","/ph/info"),rclcpp::Parameter("output_topic","/ph/cloud"),
      rclcpp::Parameter("processing_health_topic","/ph/health"),rclcpp::Parameter("camera_id","wrist"),
      rclcpp::Parameter("decimation",1),rclcpp::Parameter("allow_cpu_fallback",fallback),
      rclcpp::Parameter("projection_backend",mode=="cuda"?"cuda":"cpu")});
    std::unique_ptr<astribot::vision::DepthProjector> projector;
    if(!mode.empty()&&mode!="cuda") {
      astribot::vision::ProjectionProcessOptions o;o.executable=PROJECTION_FIXTURE_PATH;o.arguments={mode};
      o.request_timeout_sec=worker_timeout;o.restart_backoff_sec=0.;
      auto process=std::make_unique<astribot::vision::ProjectionProcess>(o);process_=process.get();projector=std::move(process);
    }
    node=std::make_shared<astribot_s1_autonomy::RgbdPointcloudNode>(opts,std::move(projector));
    probe=std::make_shared<rclcpp::Node>("processing_health_probe");
    dp=probe->create_publisher<sensor_msgs::msg::Image>("/ph/depth",rclcpp::SensorDataQoS());
    ip=probe->create_publisher<sensor_msgs::msg::CameraInfo>("/ph/info",rclcpp::SensorDataQoS());
    hs=probe->create_subscription<H>("/ph/health",rclcpp::QoS(10).transient_local(),[this](H::ConstSharedPtr h){health=h;});
    cs=probe->create_subscription<sensor_msgs::msg::PointCloud2>("/ph/cloud",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::PointCloud2::ConstSharedPtr){++clouds;});
    exec.add_node(node);exec.add_node(probe);pump(40);
    image.header.frame_id="camera";image.width=2;image.height=1;image.step=4;image.encoding="16UC1";image.data={232,3,208,7};
    info.header=image.header;info.width=2;info.height=1;info.k={100,0,0,0,100,0,0,0,1};
  }
  void pump(int ms){auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);do{exec.spin_some();std::this_thread::sleep_for(1ms);}while(std::chrono::steady_clock::now()<until);}
  void send(){image.header.stamp=info.header.stamp=probe->now();dp->publish(image);ip->publish(info);pump(40);}
  void TearDown()override{if(node)exec.remove_node(node);if(probe)exec.remove_node(probe);node.reset();}
  rclcpp::executors::SingleThreadedExecutor exec;std::shared_ptr<astribot_s1_autonomy::RgbdPointcloudNode> node;
  rclcpp::Node::SharedPtr probe;astribot::vision::ProjectionProcess* process_{};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr dp;rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ip;
  rclcpp::Subscription<H>::SharedPtr hs;rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cs;
  H::ConstSharedPtr health;sensor_msgs::msg::Image image;sensor_msgs::msg::CameraInfo info;int clouds=0;
};
TEST_F(ProcessingHealthContract, ReportsCpuOutputAndExpiresOnWallTimeWithFrozenRosClock) {
  start();ASSERT_TRUE(health);EXPECT_FALSE(health->valid);send();ASSERT_EQ(clouds,1);ASSERT_TRUE(health->valid);
  EXPECT_EQ(health->backend,"cpu");EXPECT_EQ(health->camera_id,"wrist");EXPECT_EQ(health->capture_stamp,image.header.stamp);
  auto epoch=health->processing_epoch;auto* clock=node->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,rclcpp::Time(image.header.stamp).nanoseconds()+100000000),RCL_RET_OK);
  pump(270);ASSERT_FALSE(health->valid);EXPECT_NE(health->processing_epoch,epoch);
}
TEST_F(ProcessingHealthContract, HangingWorkerDoesNotBlockHealthAndProducesNoCloud) {
  start("hang");send();pump(190);ASSERT_TRUE(health);EXPECT_FALSE(health->valid);EXPECT_EQ(clouds,0);
  EXPECT_GE(health->errors,1u);EXPECT_EQ(health->state,"BACKOFF");EXPECT_EQ(health->reason_code,"WORKER_REQUEST_TIMEOUT");
  const auto seq=health->sequence;pump(50);EXPECT_GT(health->sequence,seq);
}
TEST_F(ProcessingHealthContract, MalformedInputRevokesPreviousOutputUntilNewPair) {
  start();send();ASSERT_TRUE(health);ASSERT_TRUE(health->valid);auto epoch=health->processing_epoch;
  image.data.resize(1);send();ASSERT_FALSE(health->valid);EXPECT_NE(health->processing_epoch,epoch);
  image.data={232,3,208,7};send();ASSERT_TRUE(health->valid);EXPECT_EQ(clouds,2);
}
TEST_F(ProcessingHealthContract, NodeShutdownCancelsBlockedWorker) {
  start("hang");send();const auto before=std::chrono::steady_clock::now();
  exec.remove_node(node);node.reset();
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-before).count(),.5);
}
TEST_F(ProcessingHealthContract, ConfirmedWorkerCrashRequiresNewEpochAndFreshPair) {
  start("good");send();ASSERT_TRUE(health->valid);const auto epoch=health->processing_epoch;
  const auto worker=health->worker_epoch;ASSERT_EQ(kill(health->worker_pid,SIGKILL),0);
  send();pump(30);ASSERT_FALSE(health->valid);EXPECT_NE(health->processing_epoch,epoch);
  const int before=clouds;dp->publish(image);ip->publish(info);pump(30);EXPECT_EQ(clouds,before);
  send();ASSERT_TRUE(health->valid);EXPECT_GT(health->worker_epoch,worker);EXPECT_EQ(clouds,before+1);
}
TEST_F(ProcessingHealthContract, ExplicitFallbackWaitsForFreshPairAndKeepsOriginalBudget) {
  start("hang",true);send();pump(170);ASSERT_TRUE(health);ASSERT_FALSE(health->valid);EXPECT_EQ(clouds,0);
  EXPECT_EQ(health->worker_pid,-1);send();ASSERT_TRUE(health->valid);
  EXPECT_EQ(health->backend,"cpu");EXPECT_EQ(health->state,"READY_CPU_FALLBACK");
  EXPECT_EQ(health->capture_stamp,image.header.stamp);pump(260);EXPECT_FALSE(health->valid);
}
TEST_F(ProcessingHealthContract, FallbackAfterHealthExpiryDropsQueuedFrameAndAdvancesEpoch) {
  start("good",true,.45);send();ASSERT_TRUE(health->valid);ASSERT_EQ(clouds,1);
  ASSERT_EQ(kill(health->worker_pid,SIGSTOP),0);send();pump(240);
  ASSERT_FALSE(health->valid);const auto fault_epoch=health->processing_epoch;
  send();pump(240);EXPECT_EQ(clouds,1);EXPECT_FALSE(health->valid);
  EXPECT_EQ(health->state,"CPU_FALLBACK_WAIT");EXPECT_NE(health->processing_epoch,fault_epoch);
  send();ASSERT_TRUE(health->valid);EXPECT_EQ(clouds,2);
}
#ifdef ASTRIBOT_CUDA_PROJECTION
TEST_F(ProcessingHealthContract, RealCudaWorkerStopRecoversWithoutReusingOldCloud) {
  start("cuda");for(int i=0;i<25&&(!health||!health->valid);++i)send();
  ASSERT_TRUE(health);ASSERT_TRUE(health->valid);ASSERT_EQ(health->backend,"cuda");
  const auto worker=health->worker_epoch;const auto epoch=health->processing_epoch;
  ASSERT_GT(health->worker_pid,0);ASSERT_EQ(kill(health->worker_pid,SIGSTOP),0);
  send();pump(320);ASSERT_FALSE(health->valid);EXPECT_NE(health->processing_epoch,epoch);
  EXPECT_EQ(health->worker_pid,-1);
  for(int i=0;i<25&&!health->valid;++i)send();
  ASSERT_TRUE(health->valid);EXPECT_GT(health->worker_epoch,worker);EXPECT_EQ(health->backend,"cuda");
}
#endif
