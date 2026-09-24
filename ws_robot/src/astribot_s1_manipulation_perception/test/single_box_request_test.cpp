#include "single_box_fixture.hpp"
TEST_F(SingleBoxRequest, PreservesRealPointsCaptureInfoIdentityAndDeadline) {
  auto r=build();EXPECT_EQ(r.object_cloud.width,2500u);EXPECT_EQ(r.object_cloud.header,rgb->header);
  EXPECT_EQ(r.capture_camera_info,*info);EXPECT_EQ(r.context.identity_revision,"inventory_3");
  EXPECT_EQ(r.context.processing_epoch,"projector_boot:1");EXPECT_EQ(r.valid_until.sec,15);
  EXPECT_EQ(r.base_from_camera,tf);EXPECT_EQ(r.segmentation_source,"single_instance_fixture");
  float p[3];std::memcpy(p,r.object_cloud.data.data(),12);
  EXPECT_FLOAT_EQ(p[0],-.35f);EXPECT_FLOAT_EQ(p[1],-.25f);EXPECT_FLOAT_EQ(p[2],1.f);
  auto goals=pp::inference_goals(r,r.context,10100000000LL);
  EXPECT_EQ(goals.pose.header,rgb->header);EXPECT_EQ(goals.grasps.processing_epoch,r.context.processing_epoch);
}
TEST_F(SingleBoxRequest, MultipleColorRegionsCannotEstablishIdentity) {
  for(unsigned y=10;y<20;++y)for(unsigned x=65;x<75;++x) {
    auto i=y*rgb->step+x*3;rgb->data[i]=230;rgb->data[i+1]=115;rgb->data[i+2]=26;
  }
  reject("SINGLE_COLOR_COMPONENT_REQUIRED");
}
TEST_F(SingleBoxRequest, InsufficientActualPointsAreNeverPadded) {
  for(unsigned y=5;y<55;++y)for(unsigned x=5;x<15;++x) {
    float invalid[3]={NAN,NAN,NAN};std::memcpy(cloud->data.data()+(y*80+x)*16,invalid,12);
  }
  reject("GRASPNET_POINTS_INSUFFICIENT");
}
TEST_F(SingleBoxRequest, MissingDepthFractionIsNotInflated) {
  for(unsigned y=5;y<55;++y)for(unsigned x=5;x<20;++x) {
    float invalid[3]={NAN,NAN,NAN};std::memcpy(cloud->data.data()+(y*80+x)*16,invalid,12);
  }
  reject("TARGET_DEPTH_VALID_FRACTION");
}
TEST_F(SingleBoxRequest, WrongLayoutOrRegistrationRejected) {
  cloud->point_step=12;reject("PROJECTOR_LAYOUT");cloud->point_step=16;
  float wrong=1.f;std::memcpy(cloud->data.data()+(5*80+5)*16,&wrong,4);
  reject("PIXEL_REGISTRATION");
}
TEST_F(SingleBoxRequest, DifferentStampDistortionAndCameraInfoRejected) {
  rgb->header.stamp.nanosec=1;reject("CAPTURE_STAMP");rgb->header.stamp.nanosec=0;
  info->d={.1,0,0,0,0};reject("RECTIFIED_PINHOLE");info->d.clear();
  info->k[0]=101;reject("CAMERA_INFO_REVISION");
}
TEST_F(SingleBoxRequest, SourceAndProcessingEpochAndExpiryRejected) {
  frame.camera.source_epoch="other";reject("CAMERA_CONTEXT");frame.camera.source_epoch="camera_boot";
  frame.projection.processing_epoch="other";reject("PROJECTION_CONTEXT");frame.projection.processing_epoch="projector_boot:1";
  frame.projection.epoch_first_capture_stamp.sec=11;reject("PROJECTION_CONTEXT");frame.projection.epoch_first_capture_stamp.sec=9;
  frame.color_received.steady-=std::chrono::milliseconds(251);reject("RGBD_RECEIVE_STALE");
  frame.color_received.steady+=std::chrono::milliseconds(251);
  frame.projection.valid=false;reject("PROJECTION_HEALTH");
}
TEST_F(SingleBoxRequest, FixtureAndCaptureTransformMustBeBound) {
  fixture.confirmed_instances=2;reject("FIXTURE_IDENTITY");fixture.confirmed_instances=1;
  fixture.identity_revision="old";reject("FIXTURE_IDENTITY");fixture.identity_revision="inventory_3";
  fixture.issued_at.sec=11;reject("FIXTURE_WINDOW");fixture.issued_at.sec=9;
  tf.header.stamp.sec=11;reject("CAPTURE_TF");
}
TEST_F(SingleBoxRequest, CameraInfoHashIgnoresOnlyStamp) {
  auto changed=*info;changed.header.stamp.sec=12;
  EXPECT_EQ(pp::camera_info_revision(changed),task.context.camera_info_revision);
  changed.d={0,0,0,0,0};EXPECT_NE(pp::camera_info_revision(changed),task.context.camera_info_revision);
  auto r=build();auto current=r.context;current.camera_info_revision="changed";
  EXPECT_THROW(pp::check_context(r,current,10100000000LL),std::runtime_error);
}
TEST_F(SingleBoxRequest, RegionExcludesBackgroundWithoutEntityPose) {
  for(unsigned y=10;y<20;++y)for(unsigned x=65;x<75;++x) {
    auto i=y*rgb->step+x*3;rgb->data[i]=230;rgb->data[i+1]=115;rgb->data[i+2]=26;
  }
  fixture.region_max_m.x=.2;
  EXPECT_EQ(build().object_cloud.width,2500u);
  fixture.region_revision="wrong";reject("STATION_REGION");
}
TEST_F(SingleBoxRequest, RegionTransformAndBoundsAreMandatory) {
  station_tf.header.stamp.sec=9;reject("STATION_CAPTURE_TF");station_tf.header.stamp.sec=10;
  fixture.region_min_m.x=2;reject("STATION_REGION");
}
TEST_F(SingleBoxRequest, LateCaptureCannotRenewBudgetWhenRosTimePauses) {
  frame.color_received.ros_ns=10200000000LL;
  frame.color_received.steady=std::chrono::steady_clock::now()-std::chrono::milliseconds(100);
  EXPECT_THROW(build(10200000000LL),std::runtime_error); // 200ms late + 100ms frozen = 300ms
}
TEST_F(SingleBoxRequest, LaterPlanningKeepsOriginalSteadyDeadlines) {
  auto r=build();
  EXPECT_EQ(r.result_deadline_steady,
    frame.color_received.steady+std::chrono::nanoseconds(15000000000LL-frame.color_received.ros_ns));
  r.admission_deadline_steady=std::chrono::steady_clock::now()-std::chrono::milliseconds(1);
  EXPECT_THROW(pp::inference_goals(r,r.context,10100000000LL),std::runtime_error);
  r.result_deadline_steady=r.admission_deadline_steady;
  EXPECT_THROW(pp::check_context(r,r.context,10100000000LL),std::runtime_error);
}
