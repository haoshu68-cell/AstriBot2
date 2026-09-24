#include <gtest/gtest.h>
#include <astribot_perception_msgs/msg/projection_health.hpp>
#define main perception_server_program_main
#include "../src/manipulation_perception_server.cpp"
#undef main
#include <cstring>

class ActionContract : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void setup(const std::string &behavior = "ok",bool require_projection=false) {
    dir = std::make_unique<ai::JobDirectory>();
    auto model = dir->path() + "/model";
    {
      std::ofstream f(model);
      f << behavior;
    }
    auto registry = dir->path() + "/registry.json";
    {
      std::ofstream f(registry);
      f << nlohmann::json{{"test", {{"path", model}}}};
    }
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(
        {rclcpp::Parameter("camera_id", "test_camera"),
         rclcpp::Parameter("optical_frame", "camera"),
         rclcpp::Parameter("camera_health_topic", "/test_health"),
         rclcpp::Parameter("projection_health_topic", require_projection?"/test_projection_health":""),
         rclcpp::Parameter("calibration_revision", 1),
         rclcpp::Parameter("planning_scene_revision", 2),
         rclcpp::Parameter("envelope_epoch", 3),
         rclcpp::Parameter("grasp_worker", FIXTURE_WORKER_PATH),
         rclcpp::Parameter("grasp_model", model),
         rclcpp::Parameter("pose_worker", FIXTURE_WORKER_PATH),
         rclcpp::Parameter("model_registry", registry)});
    server = std::make_shared<ManipulationPerceptionServer>(opts);
    probe = std::make_shared<rclcpp::Node>("perception_action_test");
    health_pub =
        probe->create_publisher<pm::msg::CameraHealth>("/test_health", 10);
    projection_pub=probe->create_publisher<pm::msg::ProjectionHealth>("/test_projection_health",rclcpp::QoS(10).transient_local());
    gc = rclcpp_action::create_client<Grasp>(probe,
                                             "/perception/compute_grasps");
    pc = rclcpp_action::create_client<Pose>(probe,
                                            "/perception/estimate_object_pose");
    exec.add_node(server);
    exec.add_node(probe);
    ASSERT_TRUE(gc->wait_for_action_server(std::chrono::seconds(2)));
    ASSERT_TRUE(pc->wait_for_action_server(std::chrono::seconds(2)));
    spin_for(.15);
  }
  void TearDown() override {
    if (server)
      exec.remove_node(server);
    if (probe)
      exec.remove_node(probe);
    server.reset();
    probe.reset();
    dir.reset();
  }
  void tick() {
    if(projection_enabled) {
      pm::msg::ProjectionHealth h;h.header.frame_id="camera";h.camera_id="test_camera";
      h.header.stamp=h.capture_stamp=probe->now();h.valid_until=probe->now()+rclcpp::Duration::from_seconds(.25);
      h.epoch_first_capture_stamp=rclcpp::Time(1);h.processing_epoch=projection_epoch;h.valid=projection_valid;
      h.sequence=++projection_sequence;projection_pub->publish(h);
    }
    if (health_enabled) {
      pm::msg::CameraHealth h;
      h.header.frame_id = h.frame_id = "camera";
      h.header.stamp = h.capture_stamp = probe->now();
      h.camera_id = "test_camera";
      h.source_epoch = epoch;
      h.calibration_revision = 1;
      h.valid = true;
      h.valid_until = probe->now() + rclcpp::Duration::from_seconds(.4);
      health_pub->publish(h);
    }
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  void spin_for(double seconds) {
    auto until = Steady::now() + std::chrono::duration<double>(seconds);
    while (Steady::now() < until)
      tick();
  }
  template <class Future> bool wait(Future &f, double seconds = 3) {
    auto until = Steady::now() + std::chrono::duration<double>(seconds);
    while (f.wait_for(std::chrono::seconds(0)) != std::future_status::ready &&
           Steady::now() < until)
      tick();
    return f.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }
  template <class Goal> void fill(Goal &g) {
    g.header.frame_id = "camera";
    g.header.stamp = probe->now();
    g.task_id = "fixture_task";
    g.camera_id = "test_camera";
    g.object_id = "fixture_object";
    g.source_epoch = "epoch1";
    g.calibration_revision = 1;
    g.planning_scene_revision = 2;
    g.envelope_epoch = 3;
    g.timeout_sec = 3;
    g.valid_until = probe->now() + rclcpp::Duration::from_seconds(4);
    auto &c = g.object_cloud;
    c.header = g.header;
    c.width = 2048;
    c.height = 1;
    c.point_step = 12;
    c.row_step = c.width * 12;
    c.data.resize(c.row_step);
    for (unsigned a = 0; a < 3; ++a) {
      sensor_msgs::msg::PointField f;
      f.name = std::string(1, "xyz"[a]);
      f.offset = 4 * a;
      f.datatype = f.FLOAT32;
      f.count = 1;
      c.fields.push_back(f);
    }
    float p[3] = {.1f, .2f, 1.f};
    for (unsigned i = 0; i < c.width; ++i)
      std::memcpy(c.data.data() + 12 * i, p, 12);
  }
  Grasp::Goal grasp() {
    Grasp::Goal g;
    fill(g);
    g.arm_id = "right";
    g.max_candidates = 8;
    return g;
  }
  Pose::Goal pose() {
    Pose::Goal g;
    fill(g);
    g.model_id = "test";
    return g;
  }
  std::unique_ptr<ai::JobDirectory> dir;
  std::shared_ptr<ManipulationPerceptionServer> server;
  rclcpp::Node::SharedPtr probe;
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::Publisher<pm::msg::CameraHealth>::SharedPtr health_pub;
  rclcpp::Publisher<pm::msg::ProjectionHealth>::SharedPtr projection_pub;
  rclcpp_action::Client<Grasp>::SharedPtr gc;
  rclcpp_action::Client<Pose>::SharedPtr pc;
  std::string epoch = "epoch1";
  bool health_enabled = true;
  bool projection_enabled=false,projection_valid=true;uint64_t projection_sequence=0;
  std::string projection_epoch="projection1";
};
TEST_F(ActionContract, RawCameraHealthCannotSubstituteForProjectionCapability) {
  setup("ok",true);auto g=grasp();g.processing_epoch=projection_epoch;
  auto missing=gc->async_send_goal(g);ASSERT_TRUE(wait(missing));EXPECT_FALSE(missing.get());
  projection_enabled=true;projection_valid=false;spin_for(.05);g=grasp();g.processing_epoch=projection_epoch;
  auto invalid=gc->async_send_goal(g);ASSERT_TRUE(wait(invalid));EXPECT_FALSE(invalid.get());
  projection_valid=true;spin_for(.05);g=grasp();g.processing_epoch="old";
  auto old=gc->async_send_goal(g);ASSERT_TRUE(wait(old));EXPECT_FALSE(old.get());
  g=grasp();g.processing_epoch=projection_epoch;spin_for(.03);
  auto valid=gc->async_send_goal(g);ASSERT_TRUE(wait(valid));auto handle=valid.get();ASSERT_TRUE(handle);
  auto result=gc->async_get_result(handle);ASSERT_TRUE(wait(result));EXPECT_TRUE(result.get().result->success);
}
TEST_F(ActionContract, ProjectionRestartRevokesInFlightGrasp) {
  setup("slow",true);projection_enabled=true;spin_for(.05);auto g=grasp();g.processing_epoch=projection_epoch;spin_for(.03);
  auto sent=gc->async_send_goal(g);ASSERT_TRUE(wait(sent));auto handle=sent.get();ASSERT_TRUE(handle);
  projection_epoch="projection2";
  auto result=gc->async_get_result(handle);ASSERT_TRUE(wait(result));
  EXPECT_FALSE(result.get().result->success);EXPECT_TRUE(result.get().result->candidates.empty());
}
class ProjectionClockOrder:public ActionContract,public ::testing::WithParamInterface<bool> {};
TEST_P(ProjectionClockOrder, ClockRollbackDoesNotStrandProjection) {
  setup("ok",true);health_enabled=false;
  auto set_time=[&](int64_t t){for(auto n:{std::static_pointer_cast<rclcpp::Node>(server),probe}) {
    auto* clock=n->get_clock()->get_clock_handle();ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock,t),RCL_RET_OK);
  }};
  auto raw=[&]{pm::msg::CameraHealth h;h.camera_id="test_camera";h.frame_id=h.header.frame_id="camera";
    h.header.stamp=h.capture_stamp=probe->now();h.valid_until=probe->now()+rclcpp::Duration::from_seconds(.25);
    h.valid=true;h.source_epoch="epoch1";h.calibration_revision=1;health_pub->publish(h);spin_for(.02);};
  set_time(100000000000LL);raw();projection_enabled=true;spin_for(.02);projection_enabled=false;
  set_time(50000000000LL);
  if(GetParam()) {
    auto g=grasp();g.processing_epoch=projection_epoch;
    auto sent=gc->async_send_goal(g);ASSERT_TRUE(wait(sent));EXPECT_FALSE(sent.get());
  }
  raw();projection_epoch="projection2";projection_enabled=true;spin_for(.02);
  auto g=grasp();g.processing_epoch=projection_epoch;
  auto sent=gc->async_send_goal(g);ASSERT_TRUE(wait(sent));auto h=sent.get();ASSERT_TRUE(h);
  auto done=gc->async_get_result(h);ASSERT_TRUE(wait(done));EXPECT_TRUE(done.get().result->success);
}
INSTANTIATE_TEST_SUITE_P(RawOrGoalFirst,ProjectionClockOrder,::testing::Values(false,true));
class FutureHealthOrder:public ActionContract,public ::testing::WithParamInterface<bool> {};
TEST_P(FutureHealthOrder, SameContextFutureHeartbeatPreservesOnlyPreviousLease) {
  setup("ok",true);
  for(auto n:{std::static_pointer_cast<rclcpp::Node>(server),probe}) {
    auto* clock=n->get_clock()->get_clock_handle();ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
  }
  projection_enabled=true;spin_for(.03);health_enabled=projection_enabled=false;
  pm::msg::ProjectionHealth projected;projected.camera_id="test_camera";projected.header.frame_id="camera";
  projected.header.stamp=probe->now()+rclcpp::Duration::from_seconds(.001);projected.capture_stamp=probe->now();
  projected.valid_until=probe->now()+rclcpp::Duration::from_seconds(.25);projected.epoch_first_capture_stamp=rclcpp::Time(1);
  projected.processing_epoch=projection_epoch;projected.valid=true;projected.sequence=++projection_sequence;
  pm::msg::CameraHealth raw;raw.camera_id="test_camera";raw.frame_id=raw.header.frame_id="camera";
  raw.header.stamp=projected.header.stamp;raw.capture_stamp=probe->now();raw.valid_until=projected.valid_until;
  raw.source_epoch=epoch;raw.calibration_revision=1;raw.valid=true;
  auto publish=[&]{if(GetParam())projection_pub->publish(projected);else health_pub->publish(raw);};
  publish();spin_for(.02);
  auto g=grasp();g.processing_epoch=projection_epoch;
  auto sent=gc->async_send_goal(g);ASSERT_TRUE(wait(sent));auto handle=sent.get();ASSERT_TRUE(handle);
  auto result=gc->async_get_result(handle);ASSERT_TRUE(wait(result));EXPECT_TRUE(result.get().result->success);
  // Rejected future messages cannot refresh capture or wall-clock expiry.
  auto end=Steady::now()+std::chrono::milliseconds(270);
  while(Steady::now()<end){publish();tick();}
  sent=gc->async_send_goal(g);ASSERT_TRUE(wait(sent));EXPECT_FALSE(sent.get());
}
TEST_P(FutureHealthOrder, ExplicitFutureFaultStillRevokesPreviousCapability) {
  setup("ok",true);
  for(auto n:{std::static_pointer_cast<rclcpp::Node>(server),probe}) {
    auto* clock=n->get_clock()->get_clock_handle();ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
  }
  projection_enabled=true;spin_for(.03);health_enabled=projection_enabled=false;
  if(GetParam()) {
    pm::msg::ProjectionHealth h;h.camera_id="test_camera";h.header.frame_id="camera";h.processing_epoch=projection_epoch;
    h.header.stamp=probe->now()+rclcpp::Duration::from_seconds(1.);h.capture_stamp=probe->now();
    h.valid_until=probe->now()+rclcpp::Duration::from_seconds(.25);h.epoch_first_capture_stamp=rclcpp::Time(1);
    h.sequence=++projection_sequence;h.valid=false;projection_pub->publish(h);
  }else {
    pm::msg::CameraHealth h;h.camera_id="test_camera";h.frame_id=h.header.frame_id="camera";h.source_epoch=epoch;
    h.header.stamp=probe->now()+rclcpp::Duration::from_seconds(1.);h.capture_stamp=probe->now();
    h.valid_until=probe->now()+rclcpp::Duration::from_seconds(.25);h.calibration_revision=1;h.valid=false;health_pub->publish(h);
  }
  spin_for(.02);auto g=grasp();g.processing_epoch=projection_epoch;
  auto sent=gc->async_send_goal(g);ASSERT_TRUE(wait(sent));EXPECT_FALSE(sent.get());
}
INSTANTIATE_TEST_SUITE_P(RawOrProjection,FutureHealthOrder,::testing::Values(false,true));
TEST_F(ActionContract, DefaultCameraFrameMatchesReferenceAndVersionsRemainUninitialized) {
  server = std::make_shared<ManipulationPerceptionServer>();
  EXPECT_EQ(server->get_parameter("camera_id").as_string(), "head_rgbd");
  EXPECT_EQ(server->get_parameter("optical_frame").as_string(),
            "head_rgbd_camera_optical_frame");
  for (const auto *name : {"calibration_revision", "planning_scene_revision",
                           "envelope_epoch"})
    EXPECT_EQ(server->get_parameter(name).as_int(), 0);
  exec.add_node(server);
}
TEST_F(ActionContract, GraspProposalPreservesSnapshotAndCannotClaimCollision) {
  setup();
  auto g = grasp();
  auto sent = gc->async_send_goal(g);
  ASSERT_TRUE(wait(sent));
  auto h = sent.get();
  ASSERT_TRUE(h);
  auto done = gc->async_get_result(h);
  ASSERT_TRUE(wait(done));
  auto r = done.get();
  ASSERT_TRUE(r.result->success);
  ASSERT_EQ(r.result->candidates.size(), 1u);
  auto &c = r.result->candidates[0];
  EXPECT_EQ(c.header, g.header);
  EXPECT_EQ(c.source_epoch, g.source_epoch);
  EXPECT_FALSE(c.collision_checked);
  EXPECT_FALSE(c.collision_free);
  EXPECT_EQ(c.model_revision.size(), 64u);
}
TEST_F(ActionContract, PoseReturnsFullTransformAndUnknownCovariance) {
  setup();
  auto sent = pc->async_send_goal(pose());
  ASSERT_TRUE(wait(sent));
  auto h = sent.get();
  ASSERT_TRUE(h);
  auto done = pc->async_get_result(h);
  ASSERT_TRUE(wait(done));
  auto r = done.get();
  ASSERT_TRUE(r.result->success);
  EXPECT_TRUE(r.result->observation.orientation_valid);
  EXPECT_DOUBLE_EQ(r.result->observation.pose.pose.position.z, 1);
  EXPECT_DOUBLE_EQ(r.result->observation.pose.covariance[0], 1e6);
  EXPECT_EQ(r.result->observation.planning_scene_revision, 2u);
  EXPECT_EQ(r.result->observation.envelope_epoch, 3u);
  EXPECT_FALSE(r.result->visibility_model_used);
}
TEST_F(ActionContract, RejectsStaleOrWrongContext) {
  setup();
  auto g = grasp();
  g.header.stamp.sec -= 1;
  g.object_cloud.header = g.header;
  auto a = gc->async_send_goal(g);
  ASSERT_TRUE(wait(a));
  EXPECT_FALSE(a.get());
  g = grasp();
  g.calibration_revision = 9;
  auto b = gc->async_send_goal(g);
  ASSERT_TRUE(wait(b));
  EXPECT_FALSE(b.get());
  g = grasp();
  g.object_cloud.header.frame_id = "world";
  auto c = gc->async_send_goal(g);
  ASSERT_TRUE(wait(c));
  EXPECT_FALSE(c.get());
}
TEST_F(ActionContract, UnknownModelRejected) {
  setup();
  auto g = pose();
  g.model_id = "/tmp/arbitrary";
  auto sent = pc->async_send_goal(g);
  ASSERT_TRUE(wait(sent));
  EXPECT_FALSE(sent.get());
}
TEST_F(ActionContract, MalformedAndNanCannotReturnSuccess) {
  setup("nan");
  auto sent = gc->async_send_goal(grasp());
  ASSERT_TRUE(wait(sent));
  auto h = sent.get();
  ASSERT_TRUE(h);
  auto done = gc->async_get_result(h);
  ASSERT_TRUE(wait(done));
  EXPECT_FALSE(done.get().result->success);
  EXPECT_EQ(done.get().result->reason_code, "NONFINITE_GRASP_OUTPUT");
}
TEST_F(ActionContract, ExplicitRegistrationRejectionPreserved) {
  setup("reject");
  auto sent = pc->async_send_goal(pose());
  ASSERT_TRUE(wait(sent));
  auto h = sent.get();
  ASSERT_TRUE(h);
  auto done = pc->async_get_result(h);
  ASSERT_TRUE(wait(done));
  EXPECT_FALSE(done.get().result->success);
  EXPECT_EQ(done.get().result->reason_code, "UNOBSERVABLE_GEOMETRY");
}
TEST_F(ActionContract, CancelKillsWorkerAndBusyIsRejected) {
  setup("slow");
  auto first = gc->async_send_goal(grasp());
  ASSERT_TRUE(wait(first));
  auto h = first.get();
  ASSERT_TRUE(h);
  auto second = pc->async_send_goal(pose());
  ASSERT_TRUE(wait(second));
  EXPECT_FALSE(second.get());
  auto cancel = gc->async_cancel_goal(h);
  ASSERT_TRUE(wait(cancel));
  auto done = gc->async_get_result(h);
  ASSERT_TRUE(wait(done));
  EXPECT_EQ(done.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_TRUE(done.get().result->candidates.empty());
}
TEST_F(ActionContract, WorkerTimeoutAborts) {
  setup("slow");
  auto g = grasp();
  g.timeout_sec = .08;
  auto sent = gc->async_send_goal(g);
  ASSERT_TRUE(wait(sent));
  auto h = sent.get();
  ASSERT_TRUE(h);
  auto done = gc->async_get_result(h);
  ASSERT_TRUE(wait(done));
  EXPECT_FALSE(done.get().result->success);
  EXPECT_EQ(done.get().result->reason_code, "INFERENCE_TIMEOUT");
}
TEST_F(ActionContract, EpochChangeDiscardsLateResult) {
  setup("slow");
  auto sent = gc->async_send_goal(grasp());
  ASSERT_TRUE(wait(sent));
  auto h = sent.get();
  ASSERT_TRUE(h);
  epoch = "epoch2";
  auto done = gc->async_get_result(h);
  ASSERT_TRUE(wait(done));
  EXPECT_FALSE(done.get().result->success);
  EXPECT_TRUE(done.get().result->candidates.empty());
}
TEST_F(ActionContract, CameraDropoutDiscardsLateResult) {
  setup("slow");
  auto sent = gc->async_send_goal(grasp());
  ASSERT_TRUE(wait(sent));
  auto h = sent.get();
  ASSERT_TRUE(h);
  health_enabled = false;
  auto done = gc->async_get_result(h);
  ASSERT_TRUE(wait(done));
  EXPECT_FALSE(done.get().result->success);
  EXPECT_EQ(done.get().result->reason_code, "CAMERA_HEALTH_INVALID");
}
TEST_F(ActionContract, ExpiredResultNeverRetimestampsSnapshot) {
  setup("slow");
  auto g = grasp();
  g.valid_until = probe->now() + rclcpp::Duration::from_seconds(.2);
  auto sent = gc->async_send_goal(g);
  ASSERT_TRUE(wait(sent));
  auto h = sent.get();
  ASSERT_TRUE(h);
  auto done = gc->async_get_result(h);
  ASSERT_TRUE(wait(done));
  EXPECT_FALSE(done.get().result->success);
  EXPECT_EQ(done.get().result->reason_code, "RESULT_EXPIRED");
}
TEST_F(ActionContract, OfficialRawScoreAboveOneIsNotProbability) {
  setup("raw_score");
  auto sent = gc->async_send_goal(grasp());
  ASSERT_TRUE(wait(sent));
  auto h = sent.get();
  ASSERT_TRUE(h);
  auto done = gc->async_get_result(h);
  ASSERT_TRUE(wait(done));
  auto result = done.get().result;
  ASSERT_TRUE(result->success);
  ASSERT_EQ(result->candidates.size(), 1u);
  EXPECT_NEAR(result->candidates[0].raw_model_score, 1.4, 1e-6);
  EXPECT_NEAR(result->candidates[0].score, 1.4 / 2.4, 1e-6);
  EXPECT_EQ(result->candidates[0].score_semantics,
            "graspnet_raw_over_one_plus_raw_not_probability");
}
TEST_F(ActionContract, ModelReplacementRejectsInsteadOfClaimingOldRevision) {
  setup();
  {
    std::ofstream model(dir->path() + "/model");
    model << "replaced";
  }
  auto sent = gc->async_send_goal(grasp());
  ASSERT_TRUE(wait(sent));
  auto h = sent.get();
  ASSERT_TRUE(h);
  auto done = gc->async_get_result(h);
  ASSERT_TRUE(wait(done));
  EXPECT_FALSE(done.get().result->success);
  EXPECT_EQ(done.get().result->reason_code, "MODEL_REVISION_CHANGED");
}
TEST_F(ActionContract, FiltersNonpositiveScoreAndZeroWidthWithoutLosingBatch) {
  setup("mixed_negative");
  auto sent = gc->async_send_goal(grasp());
  ASSERT_TRUE(wait(sent));
  auto h = sent.get();
  ASSERT_TRUE(h);
  auto done = gc->async_get_result(h);
  ASSERT_TRUE(wait(done));
  auto result = done.get().result;
  ASSERT_TRUE(result->success);
  EXPECT_EQ(result->candidates.size(), 1u);
  EXPECT_EQ(result->raw_candidate_count, 3u);
  EXPECT_EQ(result->filtered_candidate_count, 2u);
}
TEST_F(ActionContract, AllFilteredBatchReturnsNoUsableGrasps) {
  setup("negative_only");
  auto sent = gc->async_send_goal(grasp());
  ASSERT_TRUE(wait(sent));
  auto h = sent.get();
  ASSERT_TRUE(h);
  auto done = gc->async_get_result(h);
  ASSERT_TRUE(wait(done));
  auto result = done.get().result;
  EXPECT_FALSE(result->success);
  EXPECT_EQ(result->reason_code, "NO_USABLE_GRASPS");
  EXPECT_EQ(result->raw_candidate_count, 1u);
  EXPECT_EQ(result->filtered_candidate_count, 1u);
}
