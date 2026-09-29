// Read-only measured geometry producer. This node owns no motion interfaces.
#include "astribot_s1_robot_geometry/robot_model.hpp"
#include "astribot_s1_robot_geometry/joint_snapshot.hpp"
#include "astribot_s1_robot_geometry/geometry_state_core.hpp"
#include "astribot_s1_robot_geometry/attachment_geometry.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <astribot_navigation_msgs/msg/robot_geometry_state.hpp>
#include <astribot_navigation_msgs/msg/envelope_slice.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <future>
#include <random>
#include <yaml-cpp/yaml.h>

namespace astribot_s1_robot_geometry {
namespace {
using json=nlohmann::json;
using GeometryState=astribot_navigation_msgs::msg::RobotGeometryState;
using GetParameters=rcl_interfaces::srv::GetParameters;
using GetScene=moveit_msgs::srv::GetPlanningScene;
using Scene=moveit_msgs::msg::PlanningScene;
using Clock=std::chrono::steady_clock;
int64_t ns(const builtin_interfaces::msg::Time &stamp) {return int64_t(stamp.sec)*1000000000+stamp.nanosec;}
builtin_interfaces::msg::Time stamp(int64_t value) {
  builtin_interfaces::msg::Time out;out.sec=static_cast<int32_t>(value/1000000000);out.nanosec=static_cast<uint32_t>(value%1000000000);return out;
}
double wall() {return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
geometry_msgs::msg::Polygon rosPolygon(const Polygon2 &points) {
  geometry_msgs::msg::Polygon out;
  if(points.empty())return out;  // An explicitly configured height layer may contain no body.
  for(const auto &p:serializedPolygon(points)) {geometry_msgs::msg::Point32 v;v.x=p[0];v.y=p[1];out.points.push_back(v);}return out;
}
Transform poseMatrix(const geometry_msgs::msg::Pose &pose) {
  const auto &p=pose.position;const auto &q=pose.orientation;Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
  geometryRequire(rotation.coeffs().allFinite() && rotation.norm()>0,"invalid shape pose quaternion");rotation.normalize();
  Transform out=Transform::Identity();out.block<3,3>(0,0)=rotation.toRotationMatrix();out.block<3,1>(0,3)=Eigen::Vector3d(p.x,p.y,p.z);
  geometryRequire(out.allFinite(),"invalid shape pose");return out;
}
json timeJson(const builtin_interfaces::msg::Time &time) {return {{"sec",time.sec},{"nanosec",time.nanosec}};}
json headerJson(const std_msgs::msg::Header &header) {return {{"stamp",timeJson(header.stamp)},{"frame_id",header.frame_id}};}
json poseJson(const geometry_msgs::msg::Pose &pose) {
  const auto &p=pose.position;const auto &q=pose.orientation;
  return {{"position",{{"x",p.x},{"y",p.y},{"z",p.z}}},{"orientation",{{"x",q.x},{"y",q.y},{"z",q.z},{"w",q.w}}}};
}
template<class Poses> json posesJson(const Poses &poses) {json out=json::array();for(const auto &p:poses)out.push_back(poseJson(p));return out;}
json attachmentJson(const moveit_msgs::msg::AttachedCollisionObject &item) {
  const auto &obj=item.object;json primitives=json::array();
  for(const auto &primitive:obj.primitives) {
    json points=json::array();for(const auto &p:primitive.polygon.points)points.push_back({{"x",double(p.x)},{"y",double(p.y)},{"z",double(p.z)}});
    primitives.push_back({{"type",primitive.type},{"dimensions",primitive.dimensions},{"polygon",{{"points",points}}}});
  }
  json trajectory=json::array();
  for(const auto &p:item.detach_posture.points)trajectory.push_back({{"positions",p.positions},{"velocities",p.velocities},{"accelerations",p.accelerations},{"effort",p.effort},
    {"time_from_start",{{"sec",p.time_from_start.sec},{"nanosec",p.time_from_start.nanosec}}}});
  return {{"link_name",item.link_name},{"touch_links",item.touch_links},{"weight",item.weight},
    {"detach_posture",{{"header",headerJson(item.detach_posture.header)},{"joint_names",item.detach_posture.joint_names},{"points",trajectory}}},
    {"object",{{"header",headerJson(obj.header)},{"pose",poseJson(obj.pose)},{"id",obj.id},{"type",{{"key",obj.type.key},{"db",obj.type.db}}},
    {"primitives",primitives},{"primitive_poses",posesJson(obj.primitive_poses)},{"meshes",json::array()},{"mesh_poses",posesJson(obj.mesh_poses)},
    {"planes",json::array()},{"plane_poses",posesJson(obj.plane_poses)},{"subframe_names",obj.subframe_names},{"subframe_poses",posesJson(obj.subframe_poses)},
    // rosidl_runtime_py converts the ROS byte field to a one-character string.
    {"operation",std::string(1,static_cast<char>(obj.operation))}}}};
}
json parameterJson(const rcl_interfaces::msg::ParameterValue &v) {
  using P=rcl_interfaces::msg::ParameterType;
  switch(v.type) {
    case P::PARAMETER_BOOL:return v.bool_value;
    case P::PARAMETER_INTEGER:return v.integer_value;
    case P::PARAMETER_DOUBLE:return v.double_value;
    case P::PARAMETER_STRING:return v.string_value;
    default:return nullptr;
  }
}
template<class Service> struct Request {
  typename rclcpp::Client<Service>::SharedPtr client;
  std::optional<typename rclcpp::Client<Service>::FutureAndRequestId> pending;
  double sent=-INFINITY;
  bool done() {return pending && pending->wait_for(std::chrono::seconds(0))==std::future_status::ready;}
  void expire(double timeout,double now) {
    if(pending && !done() && now-sent>timeout) {client->remove_pending_request(pending->request_id);pending.reset();}
  }
  void send(std::shared_ptr<typename Service::Request> request,double now) {pending=client->async_send_request(request);sent=now;}
};
}
class GeometryStateNode : public rclcpp::Node {
  std::string frame_,source_,urdf_hash_,attachment_revision_,filter_revision_,height_profile_revision_;
  std::string attachment_mode_;
  std::unique_ptr<astribot::payload::Consumer> attachment_consumer_;
  uint64_t attachment_generation_=0;
  int64_t attachment_valid_until_=-1;
  rclcpp::Subscription<astribot::payload::State>::SharedPtr attachment_subscription_;
  double error_,padding_,filter_at_=-INFINITY,next_sample_=-INFINITY;
  std::vector<double> layer_edges_;
  double ground_in_base_;
  int64_t maximum_age_,maximum_skew_,attachment_at_=-1,coverage_at_=-1;
  uint64_t sequence_=0,input_generation_=0,model_generation_=0;
  std::shared_ptr<const RobotModel> model_;
  std::unique_ptr<JointSnapshot> samples_;
  std::optional<std::vector<Shape>> attachments_;
  std::vector<std::string> attachment_ids_;
  std::optional<double> coverage_max_;
  Request<GetParameters> parameters_,coverage_;
  Request<GetScene> scene_;
  std::future<GeometryState> work_;
  GeometryWorkContext work_context_{};
  rclcpp::Publisher<GeometryState>::SharedPtr publisher_;
  rclcpp::Publisher<Scene>::SharedPtr attached_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr filter_subscription_;
  rclcpp::TimerBase::SharedPtr timer_,completion_timer_;
  GeometryState message(int64_t now) {
    GeometryState out;out.header.frame_id=frame_;out.published_at=stamp(now);out.source_id=source_;out.sequence=++sequence_;
    out.height_profile_revision=height_profile_revision_;out.ground_in_base_m=ground_in_base_;return out;
  }
  GeometryWorkContext context() const {return {input_generation_,model_generation_,attachment_revision_,samples_?samples_->epoch():0};}
  void refreshLedger(int64_t now) {
    if(!attachment_consumer_)return;
    const auto steady_now=std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
    const auto *state=attachment_consumer_->current(now,steady_now);
    if(attachment_generation_!=attachment_consumer_->generation()) {
      attachment_generation_=attachment_consumer_->generation();++input_generation_;
    }
    if(!state) {
      attachments_.reset();attachment_at_=attachment_valid_until_=-1;
      return;
    }
    auto geometry=confirmedAttachments(*attachment_consumer_,now,steady_now);
    attachments_=std::move(geometry.shapes);attachment_ids_=std::move(geometry.ids);
    attachment_revision_=geometry.revision;attachment_at_=geometry.observed_at;attachment_valid_until_=geometry.valid_until;
    Scene snapshot;snapshot.name=attachment_revision_;snapshot.robot_state.joint_state.header.stamp=stamp(attachment_at_);
    snapshot.robot_state.attached_collision_objects=state->observation.objects;attached_publisher_->publish(snapshot);
  }
  void refresh(int64_t now) {
    const double current=wall();parameters_.expire(2.,current);scene_.expire(.5,current);coverage_.expire(.5,current);
    if(coverage_.done()) {
      const auto response=coverage_.pending->get();coverage_.pending.reset();coverage_max_.reset();json values=json::object();
      const auto &names=projectionParameters();for(std::size_t i=0;i<std::min(names.size(),response->values.size());++i)values[names[i]]=parameterJson(response->values[i]);
      coverage_max_=projectionCeiling(values,frame_);coverage_at_=now;
    }
    if(!coverage_.pending && current-coverage_.sent>.5 && coverage_.client->service_is_ready()) {
      auto request=std::make_shared<GetParameters::Request>();request->names=projectionParameters();coverage_.send(request,current);
    }
    if(parameters_.done()) {
      const auto response=parameters_.pending->get();parameters_.pending.reset();geometryRequire(!response->values.empty(),"MODEL_UNAVAILABLE");
      const auto urdf=response->values.front().string_value;const auto hash=sha256(urdf);
      if(hash!=urdf_hash_) {
        model_.reset();samples_.reset();++model_generation_;
        auto model=std::make_shared<RobotModel>(urdf,frame_,ament_index_cpp::get_package_share_directory);
        auto samples=std::make_unique<JointSnapshot>(model->required(),maximum_age_,maximum_skew_);
        model_=std::move(model);samples_=std::move(samples);urdf_hash_=hash;
      }
    }
    if(!parameters_.pending && current-parameters_.sent>2. && parameters_.client->service_is_ready()) {
      auto request=std::make_shared<GetParameters::Request>();request->names={"robot_description"};parameters_.send(request,current);
    }
    refreshLedger(now);
    if(attachment_mode_=="planning_scene_legacy" && scene_.done()) {
      const auto response=scene_.pending->get();scene_.pending.reset();auto items=response->scene.robot_state.attached_collision_objects;
      attachments_.reset();attachment_at_=-1;std::vector<Shape> bodies;json serialized=json::array();std::vector<std::string> ids;
      for(auto &item:items) {
        auto &obj=item.object;obj.header.stamp=builtin_interfaces::msg::Time();
        geometryRequire((obj.header.frame_id.empty() || obj.header.frame_id==item.link_name) && obj.meshes.empty() && obj.planes.empty() && obj.primitives.size()==obj.primitive_poses.size(),"unsupported attachment geometry/frame");
        serialized.push_back(attachmentJson(item));ids.push_back(obj.id);
        for(std::size_t i=0;i<obj.primitives.size();++i) {
          const auto &primitive=obj.primitives[i];std::string kind;std::vector<double> dimensions(primitive.dimensions.begin(),primitive.dimensions.end());
          if(primitive.type==primitive.BOX)kind="box";
          else if(primitive.type==primitive.SPHERE)kind="sphere";
          else if(primitive.type==primitive.CYLINDER) {geometryRequire(dimensions.size()==2,"unsupported attachment primitive dimensions");kind="cylinder";std::swap(dimensions[0],dimensions[1]);}
          else throw std::invalid_argument("unsupported attachment primitive");
          bodies.emplace_back(item.link_name,kind,dimensions,poseMatrix(obj.pose)*poseMatrix(obj.primitive_poses[i]));
        }
      }
      attachments_=std::move(bodies);attachment_revision_=sha256(pythonJson(serialized));attachment_at_=now;attachment_valid_until_=now+500000000;attachment_ids_=std::move(ids);
      Scene snapshot;snapshot.name=attachment_revision_;snapshot.robot_state.joint_state.header.stamp=stamp(now);
      snapshot.robot_state.attached_collision_objects=std::move(items);attached_publisher_->publish(snapshot);
    }
    if(attachment_mode_=="planning_scene_legacy" && !scene_.pending && current-scene_.sent>.2 && scene_.client->service_is_ready()) {
      auto request=std::make_shared<GetScene::Request>();request->components.components=4;scene_.send(request,current);
    }
  }
  static GeometryState calculate(std::shared_ptr<const RobotModel> model,JointValues q,JointValues errors,std::vector<Shape> attachments,double padding,std::vector<double> layer_edges,GeometryState msg) {
    const auto geometry=model->geometry(q,errors,attachments,padding,{.31,.31},layer_edges);
    msg.physical_footprint=rosPolygon(geometry.physical);msg.reserved_footprint=rosPolygon(geometry.reserved);msg.height_m=geometry.height;
    for(const auto &slice:geometry.slices) {
      astribot_navigation_msgs::msg::EnvelopeSlice output;output.z_min_m=slice.z_min;output.z_max_m=slice.z_max;output.footprint=rosPolygon(slice.footprint);msg.height_slices.push_back(output);
    }
    return msg;
  }
  void tick(bool allow_sample=true) {
    const int64_t now=get_clock()->now().nanoseconds();
    try {
      if(samples_)samples_->observeClock(now);
      refresh(now);
      if(work_.valid()) {
        if(work_.wait_for(std::chrono::seconds(0))!=std::future_status::ready)return;
        completion_timer_->cancel();
        auto msg=work_.get();
        if(attachment_consumer_) {
          const auto steady_now=std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
          msg.valid_until=stamp(attachmentCompletionDeadline(*attachment_consumer_,ns(msg.valid_until),now,steady_now));
        }
        validateGeometryCompletion(work_context_,context(),attachments_.has_value() && bool(model_) && bool(samples_),ns(msg.header.stamp),ns(msg.valid_until),now,msg.height_m,coverage_max_,coverage_at_,filter_revision_,wall()-filter_at_);
        msg.published_at=stamp(now);msg.complete=true;msg.attachment_state_confirmed=true;msg.reason="GEOMETRY_CURRENT";publisher_->publish(msg);
      }
      // Completion may release a finished result sooner, but only the original
      // 20 ms ROS timer can select inputs and start another 100 ms sample job.
      if(!allow_sample || wall()<next_sample_)return;
      next_sample_=wall()+.1;geometryRequire(bool(model_) && bool(samples_),"MODEL_UNAVAILABLE");
      auto [q,times,valid_until]=samples_->snapshot(now);
      geometryRequire(attachments_ && attachment_at_<=now && now<attachment_valid_until_,attachment_consumer_?attachment_consumer_->reason().c_str():"ATTACHMENT_STATE_UNAVAILABLE");
      JointValues errors;for(const auto &name:model_->required())errors[name]=error_;
      auto msg=message(now);msg.header.stamp=stamp(*std::min_element(times.begin(),times.end()));msg.valid_until=stamp(std::min(valid_until,attachment_valid_until_));
      msg.clock_epoch=samples_->epoch();msg.model_revision=model_->revision();msg.attachment_revision=attachment_revision_;msg.attachment_ids=attachment_ids_;
      msg.joints.name=model_->required();msg.joints.header=msg.header;
      for(const auto &name:msg.joints.name) {msg.joints.position.push_back(q.at(name));msg.joint_position_error_bounds.push_back(errors.at(name));}
      for(auto time:times)msg.joint_source_stamps.push_back(stamp(time));
      work_context_=context();work_=std::async(std::launch::async,calculate,model_,std::move(q),std::move(errors),*attachments_,padding_,layer_edges_,std::move(msg));
      completion_timer_->reset();
    }catch(const std::exception &error) {auto msg=message(now);msg.reason=error.what();publisher_->publish(msg);}
  }
public:
  GeometryStateNode():Node("robot_geometry_state") {
    frame_=declare_parameter<std::string>("base_frame","astribot_torso_base");error_=declare_parameter("hold_error_rad",.003);padding_=declare_parameter("model_padding_m",.01);
    const auto profile=declare_parameter<std::string>("height_profile_path",
      ament_index_cpp::get_package_share_directory("astribot_s1_mapping")+"/config/height_slices.yaml");
    // Nominal flat-warehouse reference; hardware must supply its own value.
    ground_in_base_=declare_parameter<double>("ground_in_base_m",get_parameter("use_sim_time").as_bool()?-.095:NAN);
    geometryRequire(std::isfinite(ground_in_base_),"GROUND_REFERENCE_REQUIRED: supply ground_in_base_m");
    std::ifstream profile_stream(profile,std::ios::binary);
    geometryRequire(bool(profile_stream),"HEIGHT_PROFILE_UNAVAILABLE");
    const std::string profile_bytes((std::istreambuf_iterator<char>(profile_stream)),{});
    geometryRequire(!profile_stream.bad(),"HEIGHT_PROFILE_READ_FAILED");
    height_profile_revision_=sha256(profile_bytes);
    layer_edges_=YAML::Load(profile_bytes)["/**"]["ros__parameters"]["height_edges"].as<std::vector<double>>();
    geometryRequire(layer_edges_.size()>=2,"HEIGHT_PROFILE_REQUIRES_EDGES");
    for(size_t i=0;i<layer_edges_.size();++i)
      geometryRequire(std::isfinite(layer_edges_[i]) && (!i || layer_edges_[i]>layer_edges_[i-1]),"HEIGHT_PROFILE_INVALID");
    for(auto &edge:layer_edges_)edge+=ground_in_base_;
    maximum_age_=std::llround(declare_parameter("max_joint_age_s",.3)*1e9);maximum_skew_=std::llround(declare_parameter("max_joint_skew_s",.1)*1e9);
    geometryRequire(error_>0 && error_<=.025 && padding_>=0 && padding_<=.1,"invalid geometry margins");
    std::random_device entropy;std::string seed;for(int i=0;i<8;++i)seed+=std::to_string(entropy());source_=sha256(seed+std::to_string(wall()));
    parameters_.client=create_client<GetParameters>("/robot_state_publisher/get_parameters");scene_.client=create_client<GetScene>("/get_planning_scene");coverage_.client=create_client<GetParameters>("/pointcloud_slice_scan_node/get_parameters");
    publisher_=create_publisher<GeometryState>("/navigation/geometry_state",10);attached_publisher_=create_publisher<Scene>("/navigation/attached_geometry",10);
    rcl_interfaces::msg::ParameterDescriptor identity;identity.read_only=true;
    attachment_mode_=declare_parameter<std::string>("attachment_source_mode","ledger",identity);
    geometryRequire(attachment_mode_=="ledger" || attachment_mode_=="planning_scene_legacy","INVALID_ATTACHMENT_SOURCE_MODE");
    if(attachment_mode_=="ledger") {
      astribot::payload::Config config;
      config.environment=declare_parameter<std::string>("payload_environment","simulation",identity);
      config.session=declare_parameter<std::string>("payload_session_id","",identity);
      config.source=declare_parameter<std::string>("payload_source_id","",identity);
      const auto links=declare_parameter<std::vector<std::string>>("allowed_attachment_links",
        {"astribot_arm_left_tcp_link","astribot_arm_right_tcp_link"},identity);
      config.allowed_links={links.begin(),links.end()};attachment_consumer_=std::make_unique<astribot::payload::Consumer>(config);
      attachment_subscription_=create_subscription<astribot::payload::State>("/payload/attachment_state",rclcpp::QoS(4).reliable(),
        [this](astribot::payload::State::ConstSharedPtr state) {
          const auto now=get_clock()->now().nanoseconds();
          const auto steady_now=std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
          attachment_consumer_->receive(*state,now,steady_now);
          try {refreshLedger(now);}catch(const std::exception &) {attachments_.reset();++input_generation_;}
          // Publish revocation even when simulated ROS time is paused.
          if(!attachments_) {auto invalid=message(now);invalid.reason=attachment_consumer_->reason();publisher_->publish(invalid);}
        });
    } else RCLCPP_WARN(get_logger(),"Legacy scene-only attachment mode: no physical inventory proof; compatibility validation only");
    filter_subscription_=create_subscription<std_msgs::msg::String>("/navigation/attachment_filter_applied",10,[this](std_msgs::msg::String::ConstSharedPtr msg){filter_revision_=msg->data;filter_at_=wall();});
    joints_subscription_=create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      if(!samples_)return;
      try {samples_->receive(msg->name,msg->position,ns(msg->header.stamp),get_clock()->now().nanoseconds());}
      catch(const std::exception &) {samples_->clear();++input_generation_;}
    });
    // Poll only while one immutable job is in flight. Both timers use the
    // default mutually exclusive callback group; all input state and completion
    // validation therefore stay on the same executor as the original tick.
    completion_timer_=create_wall_timer(std::chrono::milliseconds(1),[this] {
      if(!work_.valid()) {completion_timer_->cancel();return;}
      if(work_.wait_for(std::chrono::seconds(0))!=std::future_status::ready)return;
      // Cancel before refreshing inputs too: a failed refresh must not produce
      // incomplete messages at 1 kHz or retry a result outside the main timer.
      completion_timer_->cancel();tick(false);
    });
    completion_timer_->cancel();
    timer_=rclcpp::create_timer(this,get_clock(),rclcpp::Duration::from_seconds(.02),[this]{tick();});
  }
  ~GeometryStateNode() override {if(work_.valid())work_.wait();}
};
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<astribot_s1_robot_geometry::GeometryStateNode>());}
  catch(const std::exception &e) {std::cerr<<"geometry_state: "<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
