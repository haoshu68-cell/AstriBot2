#include <gtest/gtest.h>
#include <atomic>
#include <cstring>
#include <thread>
#include <opencv2/imgcodecs.hpp>
#define main yolo_program_main
#include "../src/yolo_detector_node.cpp"
#undef main
#define main gate_program_main
#include "../src/detection_gate_node.cpp"
#undef main
#define main pose_program_main
#include "../src/rgbd_object_pose_node.cpp"
#undef main
using namespace std::chrono_literals;
class YoloPipeline : public ::testing::Test {
protected:
  static void SetUpTestSuite(){rclcpp::init(0,nullptr);}
  static void TearDownTestSuite(){rclcpp::shutdown();}
  void SetUp() override {
    const char * directory=std::getenv("VISION_TEST_MODELS");
    if(!directory) GTEST_SKIP()<<"set VISION_TEST_MODELS for real ONNX integration; no simulated inference substitute";
    const std::string path=directory;
    rclcpp::NodeOptions opts;opts.parameter_overrides({rclcpp::Parameter("model_path",path+"/yolov5n-v6.0-fixed640-detections.onnx"),
      rclcpp::Parameter("labels_path",path+"/labels.txt"),rclcpp::Parameter("model_revision","fixture_verified_sha256"),
      rclcpp::Parameter("output_topic","/perception/detections")});
    yolo=std::make_shared<YoloDetectorNode>(opts);gate=std::make_shared<DetectionGate>();
    rclcpp::NodeOptions poseopts;poseopts.parameter_overrides({rclcpp::Parameter("detection_topic","/perception/valid_detections")});
    pose=std::make_shared<RgbdObjectPoseNode>(poseopts);probe=std::make_shared<rclcpp::Node>("yolo_pipeline_probe");
    color_pub=probe->create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw",rclcpp::SensorDataQoS());
    depth_pub=probe->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw",rclcpp::SensorDataQoS());
    info_pub=probe->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/color/camera_info",rclcpp::SensorDataQoS());
    health_pub=probe->create_publisher<astribot_perception_msgs::msg::CameraHealth>("/perception/camera_health/head_rgbd",rclcpp::QoS(1).transient_local());
    det_sub=probe->create_subscription<astribot_perception_msgs::msg::Detection2D>("/perception/valid_detections",10,
      [this](astribot_perception_msgs::msg::Detection2D::ConstSharedPtr d){
        EXPECT_EQ(d->source_epoch,"test_epoch");EXPECT_EQ(d->calibration_revision,1u);EXPECT_FALSE(d->class_name.empty());++detections;
      });
    pose_sub=probe->create_subscription<astribot_perception_msgs::msg::ObjectPoseObservation>("/perception/object_pose",10,
      [this](astribot_perception_msgs::msg::ObjectPoseObservation::ConstSharedPtr p){
        EXPECT_TRUE(p->position_valid);EXPECT_FALSE(p->orientation_valid);EXPECT_NEAR(p->pose.pose.position.z,1.,1e-6);++observations;
      });
    exec=std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(),3);
    exec->add_node(yolo);exec->add_node(gate);exec->add_node(pose);exec->add_node(probe);
    thread=std::thread([this](){exec->spin();});
    auto frame=cv::imread(path+"/bus.jpg");ASSERT_FALSE(frame.empty());cv::resize(frame,frame,cv::Size(480,640));
    color.width=frame.cols;color.height=frame.rows;color.encoding="bgr8";color.step=frame.cols*3;
    color.data.assign(frame.data,frame.data+frame.total()*frame.elemSize());color.header.frame_id="camera";
    depth=color;depth.encoding="32FC1";depth.step=depth.width*4;depth.data.resize(size_t(depth.step)*depth.height);
    const float z=1.f;for(size_t i=0;i<depth.data.size();i+=4)std::memcpy(depth.data.data()+i,&z,4);
    info.header=color.header;info.width=color.width;info.height=color.height;info.k={500.,0.,240.,0.,500.,320.,0.,0.,1.};
    const auto deadline=std::chrono::steady_clock::now()+2s;
    while(color_pub->get_subscription_count()==0 && std::chrono::steady_clock::now()<deadline)std::this_thread::sleep_for(5ms);
    ASSERT_GT(color_pub->get_subscription_count(),0u);
  }
  void TearDown() override {if(exec)exec->cancel();if(thread.joinable())thread.join();}
  void publish(bool with_health=true,bool stale=false) {
    const auto now=probe->now();color.header.stamp=depth.header.stamp=info.header.stamp=now-rclcpp::Duration::from_seconds(stale?1.:0.);
    if(with_health) {
      astribot_perception_msgs::msg::CameraHealth h;h.header=color.header;h.header.stamp=now;h.capture_stamp=now;
      h.camera_id="head_rgbd";h.source_epoch="test_epoch";h.frame_id="camera";h.calibration_revision=1;h.valid=true;
      h.valid_until=now+rclcpp::Duration::from_seconds(.25);health_pub->publish(h);
    }
    depth_pub->publish(depth);info_pub->publish(info);std::this_thread::sleep_for(5ms);color_pub->publish(color);
  }
  std::shared_ptr<YoloDetectorNode> yolo;std::shared_ptr<DetectionGate> gate;std::shared_ptr<RgbdObjectPoseNode> pose;
  rclcpp::Node::SharedPtr probe;std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> exec;std::thread thread;
  sensor_msgs::msg::Image color,depth;sensor_msgs::msg::CameraInfo info;std::atomic<int> detections{0},observations{0};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub,depth_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
  rclcpp::Publisher<astribot_perception_msgs::msg::CameraHealth>::SharedPtr health_pub;
  rclcpp::Subscription<astribot_perception_msgs::msg::Detection2D>::SharedPtr det_sub;
  rclcpp::Subscription<astribot_perception_msgs::msg::ObjectPoseObservation>::SharedPtr pose_sub;
};
TEST_F(YoloPipeline, RealModelThroughDetectionGateAndSyntheticDepth) {
  for(int i=0;i<5;++i){publish();std::this_thread::sleep_for(100ms);}
  std::this_thread::sleep_for(200ms);EXPECT_GT(detections.load(),0);EXPECT_GT(observations.load(),0);
}
TEST_F(YoloPipeline, RejectsExpiredImage) {publish(true,true);std::this_thread::sleep_for(300ms);EXPECT_EQ(detections.load(),0);}
TEST_F(YoloPipeline, RejectsMissingHealth) {publish(false);std::this_thread::sleep_for(300ms);EXPECT_EQ(detections.load(),0);}
TEST_F(YoloPipeline, RejectsMalformedColor) {color.data.resize(1);publish();std::this_thread::sleep_for(300ms);EXPECT_EQ(detections.load(),0);}
