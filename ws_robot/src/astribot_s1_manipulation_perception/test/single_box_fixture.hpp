#pragma once
#include <gtest/gtest.h>
#include <astribot_s1_manipulation_perception/single_box_request_source.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cstring>

namespace pp = astribot::perception_planning;
class SingleBoxRequest : public ::testing::Test {
protected:
  pp::Request task;
  pp::SingleBoxFixture fixture;
  pp::RgbdFrame frame;
  geometry_msgs::msg::TransformStamped tf,station_tf;
  std::shared_ptr<sensor_msgs::msg::Image> rgb;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> info;
  std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud;
  void SetUp() override {
    rgb=std::make_shared<sensor_msgs::msg::Image>();
    rgb->header.frame_id="optical";rgb->header.stamp.sec=10;
    rgb->width=80;rgb->height=60;rgb->step=240;rgb->encoding="rgb8";
    rgb->data.resize(rgb->step*rgb->height);
    for(unsigned y=5;y<55;++y)for(unsigned x=5;x<55;++x) {
      const auto i=y*rgb->step+x*3;rgb->data[i]=230;rgb->data[i+1]=115;rgb->data[i+2]=26;
    }
    info=std::make_shared<sensor_msgs::msg::CameraInfo>();
    info->header=rgb->header;info->width=80;info->height=60;
    info->k={100,0,40,0,100,30,0,0,1};info->r={1,0,0,0,1,0,0,0,1};
    info->p={100,0,40,0,0,100,30,0,0,0,1,0};
    cloud=std::make_shared<sensor_msgs::msg::PointCloud2>();cloud->header=rgb->header;cloud->height=1;
    sensor_msgs::PointCloud2Modifier mod(*cloud);mod.setPointCloud2FieldsByString(1,"xyz");mod.resize(4800);
    for(unsigned y=0;y<60;++y)for(unsigned x=0;x<80;++x) {
      float p[3]={float((int(x)-40)*.01),float((int(y)-30)*.01),1.f};
      std::memcpy(cloud->data.data()+(y*80+x)*16,p,12);
    }
    frame.color=rgb;frame.info=info;frame.points=cloud;
    const pp::SensorReceipt receipt{10100000000LL,std::chrono::steady_clock::now()};
    frame.color_received=frame.points_received=frame.info_received=receipt;
    frame.camera_received=frame.camera_capture_received=receipt;
    frame.projection_received=frame.projection_capture_received=receipt;
    auto& h=frame.camera;h.header=rgb->header;h.frame_id="optical";h.camera_id="head_rgbd";
    h.source_epoch="camera_boot";h.valid=true;h.capture_stamp=h.header.stamp;
    h.valid_until.sec=11;h.calibration_revision=4;
    auto& p=frame.projection;p.header=rgb->header;p.camera_id="head_rgbd";
    p.processing_epoch="projector_boot:1";p.valid=true;p.capture_stamp=p.header.stamp;
    p.epoch_first_capture_stamp.sec=9;p.valid_until.sec=10;p.valid_until.nanosec=250000000;
    auto& c=task.context;c.object_instance="transport_box_01";c.identity_revision="inventory_3";
    c.camera_id="head_rgbd";c.optical_frame="optical";c.source_epoch=h.source_epoch;c.processing_epoch=p.processing_epoch;
    c.model_id="box";c.pose_model_revision="cad_hash:visibility_hash";c.grasp_model_revision="grasp_hash";
    c.calibration_revision=4;c.scene_revision=8;c.envelope_epoch=10;c.clock_epoch=2;c.scene_signature="full_scene_hash";
    c.camera_info_revision=pp::camera_info_revision(*info);
    task.task_id="pick1";task.context_id="context1";
    fixture.station_id="exclusive_orange_station";fixture.object_instance=c.object_instance;
    fixture.identity_revision=c.identity_revision;fixture.confirmed_instances=1;fixture.issued_at.sec=9;fixture.valid_until.sec=16;
    fixture.station_frame="pick_station";fixture.region_revision=c.station_region_revision="station_config_2";
    fixture.region_min_m.x=-.4;fixture.region_min_m.y=-.3;fixture.region_min_m.z=.8;
    fixture.region_max_m.x=.5;fixture.region_max_m.y=.4;fixture.region_max_m.z=1.2;
    tf.header=rgb->header;tf.header.frame_id="astribot_torso_base";tf.child_frame_id="optical";tf.transform.rotation.w=1;
    station_tf=tf;station_tf.header.frame_id="pick_station";
  }
  pp::Request build(int64_t now=10100000000LL) { return pp::single_box_request(task,fixture,frame,tf,station_tf,now); }
  void reject(const char* reason) {
    try { (void)build(); FAIL()<<"accepted "<<reason; }
    catch(const std::runtime_error& e) { EXPECT_NE(std::string(e.what()).find(reason),std::string::npos)<<e.what(); }
  }
};
