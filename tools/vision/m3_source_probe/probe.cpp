// One owned validation job. No planner, controller, scene-apply or payload client.
#include <astribot_s1_manipulation_perception/single_box_request_source.hpp>
#include <astribot_s1_transport_native/scene_binding.hpp>
#include <astribot_navigation_msgs/msg/robot_geometry_state.hpp>
#include <astribot_navigation_msgs/msg/navigation_envelope_v2.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/serialization.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <urdf/model.h>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include "pending_action.hpp"
#include "stationary_gate.hpp"
#include <algorithm>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>

namespace pp=astribot::perception_planning;
namespace fs=std::filesystem;
using Json=nlohmann::json;
using Steady=std::chrono::steady_clock;
using namespace std::chrono_literals;
using Geometry=astribot_navigation_msgs::msg::RobotGeometryState;
using Envelope=astribot_navigation_msgs::msg::NavigationEnvelopeV2;
using Camera=astribot_perception_msgs::msg::CameraHealth;
using SceneQuery=moveit_msgs::srv::GetPlanningScene;
static volatile std::sig_atomic_t interrupted=0;
void require(bool okay,const std::string& why) {if(!okay)throw std::runtime_error(why);}
int64_t ns(const builtin_interfaces::msg::Time& t){return int64_t(t.sec)*1000000000LL+t.nanosec;}
int64_t steady_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(Steady::now().time_since_epoch()).count();}
Json read_json(const fs::path& path){std::ifstream file(path);require(bool(file),"READ_FAILED:"+path.string());Json value;file>>value;return value;}
void write_json(const fs::path& path,const Json& value){std::ofstream file(path);require(bool(file),"WRITE_FAILED:"+path.string());file<<value.dump(2)<<'\n';file.flush();require(bool(file),"WRITE_FAILED:"+path.string());}
std::string digest(const unsigned char* bytes,size_t size) {
  unsigned char out[EVP_MAX_MD_SIZE];unsigned length=0;
  require(EVP_Digest(bytes,size,out,&length,EVP_sha256(),nullptr)==1,"SHA256_FAILED");
  std::ostringstream result;result<<std::hex<<std::setfill('0');for(unsigned i=0;i<length;++i)result<<std::setw(2)<<unsigned(out[i]);return result.str();
}
std::string file_digest(const fs::path& path) {
  std::ifstream in(path,std::ios::binary);require(bool(in),"READ_FAILED:"+path.string());
  std::unique_ptr<EVP_MD_CTX,decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(),EVP_MD_CTX_free);
  require(ctx&&EVP_DigestInit_ex(ctx.get(),EVP_sha256(),nullptr)==1,"SHA256_FAILED");
  char data[65536];while(in.read(data,sizeof(data))||in.gcount())require(EVP_DigestUpdate(ctx.get(),data,size_t(in.gcount()))==1,"SHA256_FAILED");
  require(in.eof(),"READ_FAILED:"+path.string());unsigned char out[EVP_MAX_MD_SIZE];unsigned length=0;
  require(EVP_DigestFinal_ex(ctx.get(),out,&length)==1,"SHA256_FAILED");
  std::ostringstream result;result<<std::hex<<std::setfill('0');for(unsigned i=0;i<length;++i)result<<std::setw(2)<<unsigned(out[i]);return result.str();
}
template<class T>std::string save_message(const fs::path& path,const T& message) {
  rclcpp::SerializedMessage encoded;rclcpp::Serialization<T> serializer;serializer.serialize_message(&message,&encoded);
  const auto& data=encoded.get_rcl_serialized_message();std::ofstream file(path,std::ios::binary);
  require(bool(file),"WRITE_FAILED:"+path.string());file.write(reinterpret_cast<const char*>(data.buffer),data.buffer_length);file.flush();
  require(bool(file),"WRITE_FAILED:"+path.string());return digest(data.buffer,data.buffer_length);
}
struct Receipt {int64_t ros{};Steady::time_point steady;};
struct LiveState {
  LiveState(const std::string& owner,const std::string& hold,const std::string& coordinator)
    :stationary(owner,hold,coordinator){}
  std::mutex mutex;
  Geometry::ConstSharedPtr geometry;Envelope::ConstSharedPtr envelope;Camera::ConstSharedPtr camera;
  Receipt geometry_at,envelope_at;
  m3_probe::StationaryGate stationary;
};
bool fresh(int64_t sample,int64_t until,const Receipt& receipt,int64_t now,int64_t max_age) {
  return sample>0&&sample<=receipt.ros&&receipt.ros<=now&&now-sample<=max_age&&now<until&&
    Steady::now()<receipt.steady+std::chrono::nanoseconds(std::min(until,sample+max_age)-receipt.ros);
}
Json save_stationary(const fs::path& output,const std::string& phase,const m3_probe::StationaryGate& gate) {
  const auto& m=gate.metrics();
  Json result={{"armed",gate.armed()},{"armed_ros_ns",gate.armed_ros()},{"first_fault",gate.fault()},
    {"stop",{{"passed",m.passed},{"reason",m.reason},{"samples",m.samples},{"span_ns",m.span},
      {"drift_m",m.drift},{"rotation_rad",m.rotation},{"speed_mps",m.speed},
      {"angular_speed_radps",m.angular_speed},{"command_max",m.command},{"fitted_speed_mps",m.fitted_speed}}},
    {"navigation_events",Json::array()},{"acks",Json::object()}};
  auto save=[&](const std::string& name,const auto& value,m3_probe::Time receipt) {
    const auto file="stationary_"+phase+"_"+name+".cdr";
    return Json{{"file",file},{"sha256",save_message(output/file,value)},
      {"received_ros_ns",receipt.ros},{"received_steady_ns",receipt.steady}};
  };
  if(gate.hold())result["hold"]=save("hold",gate.hold()->value,gate.hold()->at);
  if(gate.odom())result["odom"]=save("odom",gate.odom()->value,gate.odom()->at);
  if(gate.command())result["command"]=save("command",gate.command()->value,gate.command()->at);
  if(gate.geometry())result["geometry"]=save("geometry",gate.geometry()->value,gate.geometry()->at);
  if(gate.envelope()) {
    result["envelope"]=save("envelope",gate.envelope()->value,gate.envelope()->at);
    result["envelope"]["effective_valid_until_ros_ns"]=gate.envelope_deadline();
  }
  for(const auto& item:gate.acks())result["acks"][item.first]=save("ack_"+item.first,item.second.value,item.second.at);
  if(gate.reference())result["reference_cdr_sha256"]=save_message(output/("stationary_"+phase+"_reference.cdr"),*gate.reference());
  for(const auto& item:gate.navigation()) {
    const auto& v=item.second;
    result["navigation_events"].push_back({{"task_id",v.task_id},{"source",v.source},{"sequence",v.sequence},
      {"stamp_ns",ns(v.stamp)},{"state",v.state},{"reason",v.reason},{"action_status",v.action_status}});
  }
  return result;
}

int main(int argc,char** argv) {
  const auto args=rclcpp::remove_ros_arguments(argc,argv);
  if(args.size()==2&&args[1]=="--help") {
    std::cout<<"m3_source_probe OWNER_CONFIG.json --ros-args -p use_sim_time:=true\n"
      <<"One source + two real inference Actions; no planning or execution.\n";return 0;
  }
  if(args.size()!=2){std::cerr<<"Expected OWNER_CONFIG.json; see --help\n";return 2;}
  Json config,report;fs::path output;
  try {
    config=read_json(args[1]);output=config.at("output_directory").get<std::string>();
    require(fs::create_directory(output),"OUTPUT_MUST_BE_NEW_DIRECTORY");
    require(config.at("confirmed_instances").get<unsigned>()==1,"OWNER_SINGLE_INSTANCE_ASSERTION_REQUIRED");
    require(!config.at("session_id").get<std::string>().empty()&&
      !config.at("owner_evidence").get<std::string>().empty(),"OWNER_IDENTITY_REQUIRED");
    require(config.at("registration_sequence").get<uint64_t>()>0,"OWNER_REGISTRATION_SEQUENCE_REQUIRED");
    for(const auto* key:{"hold_owner_id","hold_id","coordinator_session_id"})
      require(!config.at(key).get<std::string>().empty(),"OWNER_HOLD_IDENTITY_REQUIRED");
    require(fs::is_directory(config.at("input_recording").get<std::string>()),"OWNER_INPUT_RECORDING_DIRECTORY_REQUIRED");
    report={{"schema","astribot.m3.source_models_probe/1"},{"owner_config",config},
      {"owner_config_sha256",file_digest(args[1])},{"owner_evidence_sha256",file_digest(config.at("owner_evidence").get<std::string>())},
      {"scope","validation-owned static snapshot; not M1 production integration"},
      {"source_live_request","NOT_RUN"},{"pose",{{"state","NOT_RUN"}}},{"grasp",{{"state","NOT_RUN"}}},
      {"raw_input_evidence","EXTERNAL_RECORDING_PENDING_VERIFICATION"},
      {"navigation_idle_authority","M2 cold-start and exclusive command-chain evidence required externally; not verified by this probe; status silence is not proof"},
      {"both_within_original_deadline",nullptr},{"source_and_models_validated",false},
      {"planner","NOT_RUN"},{"execution","NOT_RUN"}};
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
  rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT,[](int){interrupted=1;});std::signal(SIGTERM,[](int){interrupted=1;});
  auto node=std::make_shared<rclcpp::Node>("m3_source_model_probe",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
  std::exception_ptr spin_error;std::mutex spin_mutex;
  std::thread spin([&]{try{executor.spin();}catch(...){std::lock_guard<std::mutex> lock(spin_mutex);spin_error=std::current_exception();}});
  pp::detail::Pending<pp::Pose> pose;pp::detail::Pending<pp::Grasps> grasps;
  // Callbacks and source outlive inference cleanup and are destroyed only after
  // the executor has stopped. Source destruction requires that owner boundary.
  auto live=std::make_shared<LiveState>(config.at("hold_owner_id").get<std::string>(),
    config.at("hold_id").get<std::string>(),config.at("coordinator_session_id").get<std::string>());
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  std::unique_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<tf2_ros::TransformListener> listener;
  std::unique_ptr<pp::SingleBoxRequestSource> source;
  std::optional<pp::Request> captured;
  int exit_code=1;
  // All times below are client polling observations, never claimed DDS arrivals.
  auto observe=[&](auto& pending,const char* name) {
    auto& entry=report[name];entry["sent"]=pending.sent;
    pending.poll();entry["cancel_sent"]=pending.cancel_sent;
    if((pending.handle||pending.rejected)&&!entry.contains("goal_response_observed_steady_ns")) {
      entry["goal_response_observed_steady_ns"]=steady_ns();entry["goal_response_observed_ros_ns"]=node->now().nanoseconds();
      entry["accepted"]=bool(pending.handle);
      if(pending.handle){std::ostringstream id;id<<std::hex<<std::setfill('0');for(auto byte:pending.handle->get_goal_id())id<<std::setw(2)<<unsigned(byte);entry["goal_uuid"]=id.str();}
    }
    const bool received=pending.received();
    if(received&&!entry.contains("result_observed_steady_ns")) {
      entry["result_observed_steady_ns"]=steady_ns();entry["result_observed_ros_ns"]=node->now().nanoseconds();
    }
    const auto code=received?pending.result.get().code:rclcpp_action::ResultCode::UNKNOWN;
    if(received)entry["status"]=int(code);
    const bool terminal=!pending.sent||pending.rejected||(received&&(code==rclcpp_action::ResultCode::SUCCEEDED||
      code==rclcpp_action::ResultCode::ABORTED||code==rclcpp_action::ResultCode::CANCELED));entry["terminal_confirmed"]=terminal;
    entry["state"]=!pending.sent?"NOT_RUN":pending.rejected?"REJECTED":!pending.handle?"SENT_ACCEPTANCE_PENDING":
      !received?"PENDING_TERMINAL":terminal?"RESULT_TERMINAL":"RESULT_UNKNOWN";
    if(received&&terminal&&!entry.contains("terminal_observed_steady_ns")) {
      entry["terminal_observed_steady_ns"]=entry["result_observed_steady_ns"];
      entry["terminal_observed_ros_ns"]=entry["result_observed_ros_ns"];
    }
    return terminal;
  };
  try {
    require(node->get_parameter("use_sim_time").as_bool(),"SIM_TIME_REQUIRED");
    const auto start=Steady::now();
    auto tick=[&] {
      {std::lock_guard<std::mutex> lock(spin_mutex);if(spin_error)std::rethrow_exception(spin_error);}
      {
        std::lock_guard<std::mutex> lock(live->mutex);
        require(live->stationary.fault().empty(),live->stationary.fault());
        if(live->stationary.armed()) {
          const auto now=node->now().nanoseconds();
          const auto why=live->stationary.check(*live->envelope,*live->geometry,{now,steady_ns()});
          require(why.empty(),why);
          require(fresh(ns(live->geometry->header.stamp),ns(live->geometry->valid_until),live->geometry_at,now,300000000LL),"OWNER_GEOMETRY_CHANGED_OR_STALE");
          require(fresh(ns(live->envelope->header.stamp),ns(live->envelope->valid_until),live->envelope_at,now,300000000LL),"OWNER_HOLD_CHANGED_OR_STALE");
        }
      }
      require(!interrupted,"CANCELED");require(rclcpp::ok(),"ROS_CONTEXT_STOPPED");
      require(Steady::now()-start<60s,"PROBE_TOTAL_WALL_TIMEOUT");std::this_thread::sleep_for(2ms);
    };
    subscriptions.push_back(node->create_subscription<Geometry>("/navigation/geometry_state",10,[live,node](Geometry::ConstSharedPtr value){
      std::lock_guard<std::mutex> lock(live->mutex);
      if(!live->geometry||value->header.stamp!=live->geometry->header.stamp)
        live->geometry_at={node->now().nanoseconds(),Steady::now()};
      live->stationary.receive(*value,{node->now().nanoseconds(),steady_ns()});
      live->geometry=value;
    }));
    subscriptions.push_back(node->create_subscription<Envelope>("/navigation/envelope_v2",10,[live,node](Envelope::ConstSharedPtr value){
      std::lock_guard<std::mutex> lock(live->mutex);
      if(!live->envelope||value->header.stamp!=live->envelope->header.stamp)
        live->envelope_at={node->now().nanoseconds(),Steady::now()};
      live->stationary.receive(*value,{node->now().nanoseconds(),steady_ns()});
      live->envelope=value;
    }));
    subscriptions.push_back(node->create_subscription<Camera>("/perception/camera_health/head_rgbd",10,[live](Camera::ConstSharedPtr value){std::lock_guard<std::mutex> lock(live->mutex);live->camera=value;}));
    subscriptions.push_back(node->create_subscription<m3_probe::Hold>("/navigation/arm_hold",10,[live,node](m3_probe::Hold::ConstSharedPtr value){
      std::lock_guard<std::mutex> lock(live->mutex);live->stationary.receive(*value,{node->now().nanoseconds(),steady_ns()});}));
    subscriptions.push_back(node->create_subscription<m3_probe::Ack>("/navigation/envelope_applied",20,[live,node](m3_probe::Ack::ConstSharedPtr value){
      std::lock_guard<std::mutex> lock(live->mutex);live->stationary.receive(*value,{node->now().nanoseconds(),steady_ns()});}));
    subscriptions.push_back(node->create_subscription<m3_probe::Odom>("/odom",rclcpp::SensorDataQoS(),[live,node](m3_probe::Odom::ConstSharedPtr value){
      std::lock_guard<std::mutex> lock(live->mutex);live->stationary.receive(*value,{node->now().nanoseconds(),steady_ns()});}));
    subscriptions.push_back(node->create_subscription<m3_probe::Command>("/cmd_vel",rclcpp::SensorDataQoS(),[live,node](m3_probe::Command::ConstSharedPtr value){
      std::lock_guard<std::mutex> lock(live->mutex);live->stationary.receive(*value,{node->now().nanoseconds(),steady_ns()});}));
    subscriptions.push_back(node->create_subscription<m3_probe::Navigation>("/navigation/execution_status",rclcpp::QoS(10).reliable().transient_local(),[live,node](m3_probe::Navigation::ConstSharedPtr value){
      std::lock_guard<std::mutex> lock(live->mutex);live->stationary.receive(*value,{node->now().nanoseconds(),steady_ns()});}));
    tf=std::make_unique<tf2_ros::Buffer>(node->get_clock());listener=std::make_unique<tf2_ros::TransformListener>(*tf,node,false);
    source=std::make_unique<pp::SingleBoxRequestSource>(node,*tf);
    auto robot_params=std::make_shared<rclcpp::AsyncParametersClient>(node,"/robot_state_publisher");
    auto scenes=node->create_client<SceneQuery>("/get_planning_scene");
    while(!robot_params->service_is_ready()||!scenes->service_is_ready())tick();
    auto description=robot_params->get_parameters({"robot_description"});
    while(description.wait_for(0s)!=std::future_status::ready)tick();
    const auto actual_urdf=description.get().at(0).as_string();urdf::Model robot;
    require(robot.initString(actual_urdf),"ACTUAL_ROBOT_DESCRIPTION_INVALID");std::set<std::string> links;
    for(const auto& link:robot.links_)links.insert(link.first);
    report["robot_description_sha256"]=digest(reinterpret_cast<const unsigned char*>(actual_urdf.data()),actual_urdf.size());
    report["known_links"]=links;
    pp::Request task;auto& context=task.context;
    context.model_id=config.at("model_id").get<std::string>();context.grasp_model_revision=file_digest(config.at("grasp_model").get<std::string>());
    auto registry=read_json(config.at("model_registry").get<std::string>()).at(context.model_id);
    context.pose_model_revision=file_digest(registry.at("path").get<std::string>())+":"+file_digest(registry.at("visibility_model").get<std::string>());
    report["configured_worker_sha256"]["pose"]=file_digest(config.at("pose_worker").get<std::string>());
    report["configured_worker_sha256"]["grasp"]=file_digest(config.at("grasp_worker").get<std::string>());
    while(true) {
      bool ready=false;
      {
        std::lock_guard<std::mutex> lock(live->mutex);
        const auto now=node->now().nanoseconds();
        if(live->geometry&&live->envelope&&live->camera&&
          fresh(ns(live->geometry->header.stamp),ns(live->geometry->valid_until),live->geometry_at,now,300000000LL)&&
          fresh(ns(live->envelope->header.stamp),ns(live->envelope->valid_until),live->envelope_at,now,300000000LL)) {
          const auto why=live->stationary.arm(*live->envelope,*live->geometry,{now,steady_ns()});
          report["stationary_wait_reason"]=why;ready=why.empty();
        }
      }
      if(ready)break;
      tick();
    }
    report["stationary_wait_reason"]="";
    const auto entry_gate=[&]{std::lock_guard<std::mutex> lock(live->mutex);return live->stationary;}();
    report["stationary_entry"]=save_stationary(output,"entry",entry_gate);
    auto scene_request=std::make_shared<SceneQuery::Request>();scene_request->components.components=
      moveit_msgs::msg::PlanningSceneComponents::SCENE_SETTINGS|moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE|
      moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS|moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES|
      moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY|moveit_msgs::msg::PlanningSceneComponents::OCTOMAP|
      moveit_msgs::msg::PlanningSceneComponents::TRANSFORMS|moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX|
      moveit_msgs::msg::PlanningSceneComponents::LINK_PADDING_AND_SCALING|moveit_msgs::msg::PlanningSceneComponents::OBJECT_COLORS;
    const auto initial_scene_sent=Steady::now();
    auto initial_scene=scenes->async_send_request(scene_request);while(initial_scene.wait_for(0s)!=std::future_status::ready)tick();
    const auto full_scene=initial_scene.get()->scene;
    auto binding=astribot::transport::bind_scene(full_scene,links);
    require(full_scene.robot_state.attached_collision_objects.empty(),"EMPTY_STATION_SCENE_REQUIRED");
    const auto instance=config.at("object_instance").get<std::string>();const auto station=config.at("station_id").get<std::string>();
    require(std::count_if(full_scene.world.collision_objects.begin(),full_scene.world.collision_objects.end(),[&](const auto& o){return o.id==instance;})==1,"CURRENT_WORLD_INSTANCE_REQUIRED");
    const auto& objects=binding.first.world.collision_objects;
    auto table=std::find_if(objects.begin(),objects.end(),[&](const auto& o){return o.id==station;});
    require(table!=objects.end()&&table->header.frame_id=="astribot_torso_base"&&table->primitives.size()==1&&
      table->primitive_poses.size()==1&&table->primitives[0].type==shape_msgs::msg::SolidPrimitive::BOX&&table->primitives[0].dimensions.size()==3,"CURRENT_SINGLE_BOX_STATION_REQUIRED");
    context.object_instance=instance;
    task.task_id=config.at("session_id").get<std::string>();task.context_id=task.task_id+":snapshot:"+std::to_string(config.at("registration_sequence").get<uint64_t>());
    context.scene_revision=config.at("registration_sequence").get<uint64_t>();
    const auto scene_hash=save_message(output/"registered_scene.cdr",full_scene);
    const auto canonical_hash=save_message(output/"canonical_scene.cdr",binding.first);
    context.scene_signature=canonical_hash+":"+digest(reinterpret_cast<const unsigned char*>(binding.second.data()),binding.second.size());
    context.identity_revision=task.context_id+":"+context.scene_signature;
    pp::SingleBoxFixture fixture;fixture.station_id=station;fixture.object_instance=instance;fixture.identity_revision=context.identity_revision;fixture.confirmed_instances=1;
    fixture.station_frame="astribot_torso_base";fixture.region_min_m.x=fixture.region_min_m.y=fixture.region_min_m.z=std::numeric_limits<double>::infinity();
    fixture.region_max_m.x=fixture.region_max_m.y=fixture.region_max_m.z=-std::numeric_limits<double>::infinity();
    tf2::Transform table_pose,shape_pose;tf2::fromMsg(table->pose,table_pose);tf2::fromMsg(table->primitive_poses[0],shape_pose);
    const auto transform=table_pose*shape_pose;const auto& dims=table->primitives[0].dimensions;
    for(int x:{-1,1})for(int y:{-1,1})for(double height:{0.,.3}) {
      const auto corner=transform*tf2::Vector3(x*dims[0]/2,y*dims[1]/2,dims[2]/2+height);
      fixture.region_min_m.x=std::min(fixture.region_min_m.x,corner.x());fixture.region_max_m.x=std::max(fixture.region_max_m.x,corner.x());
      fixture.region_min_m.y=std::min(fixture.region_min_m.y,corner.y());fixture.region_max_m.y=std::max(fixture.region_max_m.y,corner.y());
      fixture.region_min_m.z=std::min(fixture.region_min_m.z,corner.z());fixture.region_max_m.z=std::max(fixture.region_max_m.z,corner.z());
    }
    fixture.region_revision=context.station_region_revision="probe_table_region_v1:"+save_message(output/"registered_table.cdr",*table)+":up0.3:xy0";
    std::string geometry_source,geometry_model,attachments,coordinator,hold_id;
    {
      std::lock_guard<std::mutex> lock(live->mutex);context.clock_epoch=live->geometry->clock_epoch;context.envelope_epoch=live->envelope->epoch;
      context.calibration_revision=live->camera->calibration_revision;geometry_source=live->geometry->source_id;geometry_model=live->geometry->model_revision;
      attachments=live->geometry->attachment_revision;coordinator=live->envelope->coordinator_session_id;hold_id=live->envelope->hold_id;
    }
    require(coordinator==config.at("coordinator_session_id").get<std::string>()&&hold_id==config.at("hold_id").get<std::string>()&&!hold_id.empty(),"ACTUAL_OWNER_HOLD_REQUIRED");
    const auto registered_ros=node->now().nanoseconds();const auto registered_steady=Steady::now();
    fixture.issued_at=rclcpp::Time(registered_ros);fixture.valid_until=rclcpp::Time(registered_ros+30000000000LL);
    auto scene_checked=initial_scene_sent;auto next_query=scene_checked;Steady::time_point query_sent;
    std::optional<rclcpp::Client<SceneQuery>::FutureAndRequestId> query;
    auto current=[&] {
      tick();const auto now=node->now().nanoseconds();require(now>=registered_ros&&Steady::now()-registered_steady<30s,"OWNER_SNAPSHOT_LEASE_EXPIRED_OR_CLOCK_ROLLBACK");
      if(query&&query->wait_for(0s)==std::future_status::ready) {
        require(astribot::transport::bind_scene(query->get()->scene,links)==binding,"OWNER_SCENE_CHANGED");
        query.reset();scene_checked=query_sent;next_query=Steady::now()+100ms;
      }
      require(Steady::now()-scene_checked<1s,"OWNER_SCENE_READBACK_STALE");
      if(!query&&Steady::now()>=next_query){query_sent=Steady::now();query.emplace(scenes->async_send_request(scene_request));}
      {
        std::lock_guard<std::mutex> lock(live->mutex);
        const auto now=node->now().nanoseconds();
        const auto& geometry=live->geometry;const auto& envelope=live->envelope;const auto& camera=live->camera;
        require(geometry->complete&&geometry->attachment_state_confirmed&&geometry->attachment_ids.empty()&&
          geometry->source_id==geometry_source&&geometry->model_revision==geometry_model&&geometry->attachment_revision==attachments&&
          geometry->clock_epoch==context.clock_epoch&&fresh(ns(geometry->header.stamp),ns(geometry->valid_until),live->geometry_at,now,300000000LL),"OWNER_GEOMETRY_CHANGED_OR_STALE");
        require(envelope->epoch==context.envelope_epoch&&envelope->clock_epoch==context.clock_epoch&&
          envelope->coordinator_session_id==coordinator&&envelope->hold_id==hold_id&&envelope->navigation_allowed&&
          envelope->mode==Envelope::FIXED_POSTURE&&envelope->limits.transport_ready&&
          fresh(ns(envelope->header.stamp),ns(envelope->valid_until),live->envelope_at,now,300000000LL),"OWNER_HOLD_CHANGED_OR_STALE");
        const auto stopped=live->stationary.check(*envelope,*geometry,{now,steady_ns()});require(stopped.empty(),stopped);
        require(camera->camera_id=="head_rgbd"&&camera->calibration_revision==context.calibration_revision,"OWNER_CALIBRATION_CHANGED");
      }
      return source->bind_context(context);
    };
    context=current();task.scene=full_scene;
    Json registration={{"session_id",task.task_id},{"context_id",task.context_id},{"registration_sequence",context.scene_revision},
      {"source","independent live full-scene registration owned by this validation session"},{"scene_cdr_sha256",scene_hash},
      {"scene_signature",context.scene_signature},{"identity_revision",context.identity_revision},{"clock_epoch",context.clock_epoch},
      {"calibration_revision",context.calibration_revision},{"planning_scene_revision",context.scene_revision},{"envelope_epoch",context.envelope_epoch},
      {"coordinator_session_id",coordinator},{"hold_id",hold_id},{"issued_ros_ns",registered_ros},{"valid_until_ros_ns",ns(fixture.valid_until)},
      {"hold_owner_id",config.at("hold_owner_id")},{"stationary_gate","typed_hold_six_ack_measured_stop"},
      {"pose_model_revision",context.pose_model_revision},{"grasp_model_revision",context.grasp_model_revision},
      {"region_revision",fixture.region_revision},{"region_min",{fixture.region_min_m.x,fixture.region_min_m.y,fixture.region_min_m.z}},
      {"region_max",{fixture.region_max_m.x,fixture.region_max_m.y,fixture.region_max_m.z}},
      {"projection_health_topic","/perception/projection_health/single_box/head_rgbd"},{"camera_id",context.camera_id},{"optical_frame",context.optical_frame}};
    write_json(output/"registration.json",registration);report["registration"]=registration;
    std::cout<<"REGISTERED "<<(output/"registration.json")<<std::endl;
    pose.client=rclcpp_action::create_client<pp::Pose>(node,"/perception/estimate_object_pose");
    grasps.client=rclcpp_action::create_client<pp::Grasps>(node,"/perception/compute_grasps");
    for(const auto& server:{std::string("/object_pose_server"),std::string("/grasp_proposal_server")}) {
      auto params=std::make_shared<rclcpp::AsyncParametersClient>(node,server);
      while(!params->service_is_ready()){(void)current();}
      const std::vector<std::string> keys={"camera_id","optical_frame","calibration_revision","planning_scene_revision","envelope_epoch","projection_health_topic","max_input_age_sec","max_result_age_sec",
        "camera_health_topic","pose_worker","grasp_worker","grasp_model","model_registry","grasp_device"};
      auto values=params->get_parameters(keys);auto descriptions=params->describe_parameters(keys);
      while(values.wait_for(0s)!=std::future_status::ready||descriptions.wait_for(0s)!=std::future_status::ready)(void)current();
      const auto v=values.get();const auto d=descriptions.get();
      require(v.size()==keys.size()&&d.size()==keys.size(),"SERVER_PARAMETER_READBACK_INCOMPLETE");
      for(const auto& descriptor:d)require(descriptor.read_only,"SERVER_PARAMETER_NOT_READONLY");
      require(v[0].as_string()==context.camera_id&&v[1].as_string()==context.optical_frame&&
        uint64_t(v[2].as_int())==context.calibration_revision&&uint64_t(v[3].as_int())==context.scene_revision&&uint64_t(v[4].as_int())==context.envelope_epoch&&
        v[5].as_string()=="/perception/projection_health/single_box/head_rgbd"&&v[6].as_double()<=.5&&v[7].as_double()==5.&&
        v[8].as_string()=="/perception/camera_health/head_rgbd","SERVER_PARAMETER_CONTEXT_MISMATCH");
      if(server=="/object_pose_server")require(v[9].as_string()==config.at("pose_worker").get<std::string>()&&
        v[10].as_string().empty()&&v[12].as_string()==config.at("model_registry").get<std::string>(),"POSE_WORKER_DEPLOYMENT_MISMATCH");
      else require(v[9].as_string().empty()&&v[10].as_string()==config.at("grasp_worker").get<std::string>()&&
        v[11].as_string()==config.at("grasp_model").get<std::string>()&&v[13].as_string()=="cuda","GRASP_WORKER_DEPLOYMENT_MISMATCH");
      for(size_t index=0;index<keys.size();++index)report["server_parameter_readback"][server][keys[index]]=v[index].value_to_string();
      auto sim_time=params->get_parameters({"use_sim_time"});
      while(sim_time.wait_for(0s)!=std::future_status::ready)(void)current();
      require(sim_time.get().at(0).as_bool(),"SERVER_SIM_TIME_REQUIRED");
    }
    while(!pose.client->action_server_is_ready()||!grasps.client->action_server_is_ready())(void)current();
    captured.emplace(source->capture(task,fixture,current));const auto& request=*captured;report["source_live_request"]="PASS";
    report["capture"]={{"stamp_ns",ns(request.object_cloud.header.stamp)},{"frame",request.object_cloud.header.frame_id},{"points",request.object_cloud.width},
      {"camera_info_revision",request.context.camera_info_revision},{"source_epoch",request.context.source_epoch},{"processing_epoch",request.context.processing_epoch},
      {"valid_until_ns",ns(request.valid_until)},{"admission_steady_ns",std::chrono::duration_cast<std::chrono::nanoseconds>(request.admission_deadline_steady.time_since_epoch()).count()},
      {"result_steady_ns",std::chrono::duration_cast<std::chrono::nanoseconds>(request.result_deadline_steady.time_since_epoch()).count()}};
    const auto goals=pp::inference_goals(request,current(),node->now().nanoseconds());
    report["pose"]["send_attempted_ros_ns"]=node->now().nanoseconds();report["pose"]["send_attempted_steady_ns"]=steady_ns();pose.send(goals.pose);
    report["grasp"]["send_attempted_ros_ns"]=node->now().nanoseconds();report["grasp"]["send_attempted_steady_ns"]=steady_ns();grasps.send(goals.grasps);
    while(true) {
      pp::check_context(request,current(),node->now().nanoseconds());
      const bool pose_done=observe(pose,"pose"),grasp_done=observe(grasps,"grasp");
      require(!pose.rejected&&!grasps.rejected,"INFERENCE_GOAL_REJECTED");
      require(!pose.received()||pose.terminal(),"POSE_TERMINAL_UNKNOWN");require(!grasps.received()||grasps.terminal(),"GRASP_TERMINAL_UNKNOWN");
      if(pose_done)require(pose.result.get().code==rclcpp_action::ResultCode::SUCCEEDED&&pose.result.get().result->success,"POSE_FAILED:"+pose.result.get().result->reason_code);
      if(grasp_done)require(grasps.result.get().code==rclcpp_action::ResultCode::SUCCEEDED&&grasps.result.get().result->success,"GRASP_FAILED:"+grasps.result.get().result->reason_code);
      if(pose_done&&grasp_done)break;
    }
    const auto p=pose.result.get().result;const auto g=grasps.result.get().result;const auto& o=p->observation;const auto& c=request.context;
    require(o.position_valid&&o.orientation_valid&&o.header==request.object_cloud.header&&o.object_id==c.object_instance&&o.source_camera_id==c.camera_id&&
      o.source_epoch==c.source_epoch&&o.source_model==c.model_id&&o.model_revision==c.pose_model_revision&&o.calibration_revision==c.calibration_revision&&
      o.planning_scene_revision==c.scene_revision&&o.envelope_epoch==c.envelope_epoch&&o.valid_until==request.valid_until,"POSE_RESULT_BINDING_MISMATCH");
    require(!g->candidates.empty(),"NO_GRASP_CANDIDATES");
    for(const auto& candidate:g->candidates)require(candidate.header==request.object_cloud.header&&candidate.grasp_pose.header==candidate.header&&
      candidate.object_id==c.object_instance&&candidate.camera_id==c.camera_id&&candidate.source_epoch==c.source_epoch&&candidate.arm_id=="left"&&
      candidate.model_name=="graspnet_baseline_torchscript"&&candidate.model_revision==c.grasp_model_revision&&candidate.calibration_revision==c.calibration_revision&&
      candidate.planning_scene_revision==c.scene_revision&&candidate.envelope_epoch==c.envelope_epoch&&candidate.valid_until==request.valid_until&&candidate.geometry_valid,"GRASP_RESULT_BINDING_MISMATCH");
    pp::check_context(request,current(),node->now().nanoseconds());
    report["source_and_models_validated"]=true;report["reason"]="SOURCE_AND_BOTH_MODELS_VALIDATED_ONLY";
    exit_code=0;
  }catch(const std::exception& e){report["reason"]=e.what();std::cerr<<e.what()<<'\n';}
  // One endpoint exception must not skip cancellation/evidence for its peer.
  auto cleanup=[&](auto& pending,const char* name) {
    bool terminal=false;
    try{terminal=observe(pending,name);}catch(const std::exception& e){report[name]["observe_error"]=e.what();report[name]["terminal_confirmed"]=false;report[name]["state"]="OBSERVATION_ERROR";exit_code=1;}
    if(!terminal&&pending.handle&&!pending.cancel_sent&&!report[name].contains("cancel_error")) {
      try{pending.client->async_cancel_goal(pending.handle);pending.cancel_sent=true;}
      catch(const std::exception& e){report[name]["cancel_error"]=e.what();exit_code=1;}
    }
    report[name]["cancel_sent"]=pending.cancel_sent;
    return terminal;
  };
  const auto cleanup_until=Steady::now()+1s;bool pose_terminal=false,grasp_terminal=false;
  do{pose_terminal=cleanup(pose,"pose");grasp_terminal=cleanup(grasps,"grasp");if(pose_terminal&&grasp_terminal)break;std::this_thread::sleep_for(2ms);}while(Steady::now()<cleanup_until);
  report["terminal_confirmed"]=pose_terminal&&grasp_terminal;if(!pose_terminal||!grasp_terminal)exit_code=1;
  auto save_result=[&](auto& pending,const char* name) {
    try{if(pending.received()){const auto value=pending.result.get();report[name]["success"]=value.result->success;
      report[name]["reason"]=value.result->reason_code;report[name]["inference_time_sec"]=value.result->inference_time_sec;
      report[name]["result_cdr_sha256"]=save_message(output/(std::string(name)+"_result.cdr"),*value.result);}}
    catch(const std::exception& e){report[name]["result_evidence_error"]=e.what();exit_code=1;}
  };
  save_result(pose,"pose");save_result(grasps,"grasp");
  if(captured&&report["pose"].contains("terminal_observed_steady_ns")&&report["grasp"].contains("terminal_observed_steady_ns")) {
    const auto deadline=std::chrono::duration_cast<std::chrono::nanoseconds>(captured->result_deadline_steady.time_since_epoch()).count();
    bool in_time=true;for(const auto* name:{"pose","grasp"})in_time=in_time&&report[name]["terminal_observed_steady_ns"].get<int64_t>()<deadline&&
      report[name]["terminal_observed_ros_ns"].get<int64_t>()>=ns(captured->object_cloud.header.stamp)&&report[name]["terminal_observed_ros_ns"].get<int64_t>()<ns(captured->valid_until);
    report["both_within_original_deadline"]=in_time;
  }
  executor.cancel();spin.join();
  if(exit_code==0) {
    const auto now=node->now().nanoseconds();
    const auto why=live->stationary.check(*live->envelope,*live->geometry,{now,steady_ns()});
    if(!why.empty()){report["reason"]=why;exit_code=1;}
    else if(!fresh(ns(live->geometry->header.stamp),ns(live->geometry->valid_until),live->geometry_at,now,300000000LL)) {
      report["reason"]="OWNER_GEOMETRY_CHANGED_OR_STALE";exit_code=1;
    } else if(!fresh(ns(live->envelope->header.stamp),ns(live->envelope->valid_until),live->envelope_at,now,300000000LL)) {
      report["reason"]="OWNER_HOLD_CHANGED_OR_STALE";exit_code=1;
    }
  }
  source.reset();listener.reset();tf.reset();subscriptions.clear();rclcpp::shutdown();
  if(exit_code!=0)report["source_and_models_validated"]=false;
  report["exit_code"]=exit_code;report["finished_steady_ns"]=steady_ns();
  try{
    report["stationary_final"]=save_stationary(output,"final",live->stationary);
    if(captured){
      report["request_cloud_sha256"]=save_message(output/"request_cloud.cdr",captured->object_cloud);
      report["request_camera_info_sha256"]=save_message(output/"request_camera_info.cdr",captured->capture_camera_info);
      report["request_tf_sha256"]=save_message(output/"request_tf.cdr",captured->base_from_camera);
    }
    write_json(output/"report.json",report);
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
  return exit_code;
}
