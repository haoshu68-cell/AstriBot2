#include <gtest/gtest.h>
#include <thread>
#define main gate_main
#include "../src/detection_gate_node.cpp"
#undef main
TEST(DetectionGateClock, ExpirySurvivesPausedClockAndLostHealth) {
  rclcpp::init(0,nullptr);
  auto node=std::make_shared<DetectionGate>();auto probe=std::make_shared<rclcpp::Node>("detection_clock_probe");
  using D=astribot_perception_msgs::msg::Detection2D;using H=astribot_perception_msgs::msg::CameraHealth;
  auto hp=probe->create_publisher<H>("/perception/camera_health/head_rgbd",rclcpp::QoS(1).transient_local());
  auto dp=probe->create_publisher<D>("/perception/detections",10);int count=0;
  auto sub=probe->create_subscription<D>("/perception/valid_detections",10,[&](D::ConstSharedPtr){++count;});
  rclcpp::executors::SingleThreadedExecutor ex;ex.add_node(node);ex.add_node(probe);
  const auto pump=[&](int ms){auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);do{ex.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));}while(std::chrono::steady_clock::now()<end);};
  pump(25);D d;d.header.stamp=probe->now();d.header.frame_id="camera";d.source_camera_id="head_rgbd";d.source_epoch="s1";
  d.detection_id=d.object_id="hyp1";d.model_name="fixture";d.model_revision="v1";d.calibration_revision=1;d.confidence=.9;d.width=d.height=10;
  H h;h.header=d.header;h.camera_id=d.source_camera_id;h.source_epoch=d.source_epoch;h.frame_id=d.header.frame_id;h.valid=true;h.calibration_revision=1;
  h.capture_stamp=d.header.stamp;h.valid_until=probe->now()+rclcpp::Duration::from_seconds(.25);
  hp->publish(h);pump(15);dp->publish(d);pump(15);EXPECT_EQ(count,1);
  auto * clock=node->get_clock()->get_clock_handle();
  EXPECT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  EXPECT_EQ(rcl_set_ros_time_override(clock,rclcpp::Time(d.header.stamp).nanoseconds()+60000000),RCL_RET_OK);
  pump(550);dp->publish(d);pump(30);EXPECT_EQ(count,1);
  ex.remove_node(node);ex.remove_node(probe);node.reset();probe.reset();rclcpp::shutdown();
}
