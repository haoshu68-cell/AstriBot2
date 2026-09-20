#define ASTRIBOT_RECORDER_NO_MAIN
#include "../src/recorder.cpp"
#include <gtest/gtest.h>
#include <rosbag2_cpp/reader.hpp>
TEST(RecorderIntegration, ParametersBagAndLifecycle) {
  // This test only creates fake data/parameter nodes in the test domain. No motion topics.
  rclcpp::init(0,nullptr);
  auto root=std::filesystem::temp_directory_path()/("operator_test_"+std::to_string(getpid()));
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("output_root",root.string()),
    rclcpp::Parameter("min_free_bytes",int64_t(1)),
    rclcpp::Parameter("topic_stale_sec",0.1),
    rclcpp::Parameter("topics",std::vector<std::string>{"/operator_test/data","/operator_backend/events"}),
    rclcpp::Parameter("parameters",std::vector<std::string>{"/operator_test_source:speed","/operator_test_source:missing","/operator_test_absent:x","/operator_timeout:x","/operator_silent_change:speed"})});
  auto recorder=std::make_shared<Recorder>(options);
  auto source=std::make_shared<rclcpp::Node>("operator_test_source");source->declare_parameter("speed",0.3);
  auto driver=std::make_shared<rclcpp::Node>("operator_test_driver");
  auto silent=std::make_shared<rclcpp::Node>("operator_timeout",rclcpp::NodeOptions().start_parameter_services(false));
  double unreported_speed=0.2;
  auto manual=std::make_shared<rclcpp::Node>("operator_silent_change",rclcpp::NodeOptions().start_parameter_services(false));
  auto manual_service=manual->create_service<Get>("~/get_parameters",[&](std::shared_ptr<Get::Request>,std::shared_ptr<Get::Response> r){rcl_interfaces::msg::ParameterValue v;v.type=3;v.double_value=unreported_speed;r->values={v};});
  auto unresponsive=silent->create_service<Get>("~/get_parameters",[](std::shared_ptr<rmw_request_id_t>,std::shared_ptr<Get::Request>){ });
  auto publisher=source->create_publisher<std_msgs::msg::String>("/operator_test/data",10);
  auto operation_publisher=source->create_publisher<std_msgs::msg::String>("/operator_backend/events",10);
  auto start=driver->create_client<Trigger>("/diagnostics_recorder/start");
  auto stop=driver->create_client<Trigger>("/diagnostics_recorder/stop");
  auto snapshot=driver->create_client<Trigger>("/diagnostics_recorder/snapshot");
  auto mark=driver->create_client<Trigger>("/diagnostics_recorder/mark");
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(source);executor.add_node(recorder);executor.add_node(driver);executor.add_node(silent);executor.add_node(manual);
  auto spin=[&](int ms){auto end=Clock::now()+std::chrono::milliseconds(ms);while(Clock::now()<end){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}};
  auto call=[&](auto client){auto f=client->async_send_request(std::make_shared<Trigger::Request>());auto end=Clock::now()+std::chrono::seconds(5);while(f.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready && Clock::now()<end)spin(10);if(f.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready)throw std::runtime_error("test service timeout");return f.get();};
  spin(500);
  auto initial=call(start);EXPECT_TRUE(initial->success);auto path=std::filesystem::path(initial->message);
  spin(500);EXPECT_TRUE(call(start)->success);
  std_msgs::msg::String message;message.data="fake sensor evidence";
  for(int i=0;i<20;++i){publisher->publish(message);spin(30);}
  std_msgs::msg::String operation;operation.data=R"({"operation":"navigate","command_id":"fixture","state":"ACCEPTED"})";operation_publisher->publish(operation);spin(100);
  source->set_parameter(rclcpp::Parameter("speed",0.15));spin(200);
  unreported_speed=0.8;EXPECT_TRUE(call(mark)->success);
  EXPECT_TRUE(call(snapshot)->success);spin(200);EXPECT_TRUE(call(mark)->success);
  auto before=station::parameters_at(station::read_events(path/"events.jsonl"),UINT64_MAX);
  auto old_id=before["/operator_test_source"]["node_instance_id"].get<std::string>();
  EXPECT_FALSE(old_id.empty());
  spin(3300);
  auto in_flight=station::read_events(path/"events.jsonl");
  bool timeout=false,stale=false,missed_event=false,business=false;
  for(const auto & e:in_flight) {
    if(e.value("kind","")=="operation_event"&&e.at("payload").value("command_id","")=="fixture")business=true;
    if(e.value("kind","")=="parameter_gap" && e.value("reason","")=="timeout")timeout=true;
    if(e.value("kind","")=="parameter_gap" && e.value("reason","")=="readback_differs_without_event")missed_event=true;
    if(e.value("kind","")=="data_quality" && e["topics"]["/operator_test/data"]["state"]=="stale")stale=true;
  }
  EXPECT_TRUE(timeout);EXPECT_TRUE(stale);EXPECT_TRUE(missed_event);EXPECT_TRUE(business);
  auto duplicate=std::make_shared<rclcpp::Node>("operator_test_source");executor.add_node(duplicate);spin(1100);
  EXPECT_TRUE(call(snapshot)->success);spin(100);
  auto ambiguous=station::parameters_at(station::read_events(path/"events.jsonl"),UINT64_MAX);
  EXPECT_EQ(ambiguous["/operator_test_source"]["quality"],"unknown");
  executor.remove_node(duplicate);duplicate.reset();spin(1100);
  publisher.reset();executor.remove_node(source);source.reset();spin(1100);
  source=std::make_shared<rclcpp::Node>("operator_test_source");source->declare_parameter("speed",0.55);executor.add_node(source);spin(1100);
  EXPECT_TRUE(call(snapshot)->success);spin(300);EXPECT_TRUE(call(stop)->success);
  auto events=station::read_events(path/"events.jsonl");auto state=station::parameters_at(events,UINT64_MAX);
  EXPECT_EQ(state["/operator_test_source"]["values"]["speed"]["value"],0.55);
  EXPECT_NE(state["/operator_test_source"]["node_instance_id"],old_id);
  EXPECT_EQ(state["/operator_test_source"]["values"]["missing"]["available"],false);
  EXPECT_EQ(state["/operator_test_absent"]["quality"],"unknown");
  bool changed=false;for(const auto & e:events)if(e.value("kind","")=="parameter_event")changed=true;EXPECT_TRUE(changed);
  rosbag2_cpp::Reader reader;reader.open((path/"bag").string());int count=0,business_count=0;while(reader.has_next()){auto m=reader.read_next();if(m->topic_name=="/operator_test/data")++count;else if(m->topic_name=="/operator_backend/events")++business_count;else ADD_FAILURE()<<"Unexpected topic: "<<m->topic_name;}EXPECT_GT(count,0);EXPECT_GT(business_count,0);
  std::ifstream manifest(path/"manifest.json");auto metadata=Json::parse(manifest);EXPECT_EQ(metadata["state"],"closed");
  EXPECT_EQ(metadata["topic_quality"]["/operator_test/data"]["state"],"missing_publisher");
  EXPECT_GT(metadata["topic_quality"]["/operator_test/data"]["bytes"].get<uint64_t>(),0u);
  EXPECT_FALSE(call(mark)->success);
  auto again=call(start);EXPECT_TRUE(again->success);EXPECT_NE(again->message,path.string());spin(100);EXPECT_TRUE(call(stop)->success);
  executor.remove_node(recorder);executor.remove_node(source);executor.remove_node(driver);executor.remove_node(silent);executor.remove_node(manual);recorder.reset();source.reset();rclcpp::shutdown();
  std::filesystem::remove_all(root);
}

TEST(ParameterEncoding, NonFiniteAndIntegerTypesArePreserved) {
 rcl_interfaces::msg::ParameterValue p;p.type=3;p.double_value=std::numeric_limits<double>::quiet_NaN();
 EXPECT_EQ(value(p)["value"]["non_finite"],"nan");
 p.type=2;p.integer_value=INT64_MAX;EXPECT_EQ(value(p)["value"].get<int64_t>(),INT64_MAX);
 p.type=8;p.double_array_value={0.1,std::numeric_limits<double>::infinity()};
 EXPECT_EQ(value(p)["value"][1]["non_finite"],"+inf");
}
