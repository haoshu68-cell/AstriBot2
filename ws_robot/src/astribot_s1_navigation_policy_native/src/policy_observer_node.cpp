#include "astribot_s1_navigation_policy_native/policy_observer_node.hpp"
#include "astribot_s1_navigation_policy_native/policy_profile.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <astribot_navigation_msgs/msg/navigation_execution_status.hpp>
#include <rclcpp/create_timer.hpp>
#include <tf2/time.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <locale>
#include <set>
#include <sstream>

namespace astribot::navigation::policy {
namespace {
using json=nlohmann::json;
std::int64_t ns(const builtin_interfaces::msg::Time& t){return static_cast<std::int64_t>(t.sec)*1000000000LL+t.nanosec;}
builtin_interfaces::msg::Time wire_time(std::int64_t value){
  if(value<0 || value/1000000000LL>INT32_MAX)throw std::out_of_range("ROS Time seconds out of range");
  return rclcpp::Time(value);
}
std::uint64_t wire_unsigned(const Integer& value){
  if(value<0||value>UINT64_MAX)throw std::out_of_range("ROS unsigned integer out of range");
  return value.convert_to<std::uint64_t>();
}
std::int64_t steady_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
double steady_seconds(){return steady_ns()*1e-9;}
double thread_seconds(){timespec t{};clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t);return t.tv_sec+t.tv_nsec*1e-9;}
template<class Q> std::array<double,4> quaternion(const Q& q){return {q.x,q.y,q.z,q.w};}
std::array<double,7> transform_values(const geometry_msgs::msg::TransformStamped& tf){const auto& t=tf.transform.translation;const auto& q=tf.transform.rotation;return {t.x,t.y,t.z,q.x,q.y,q.z,q.w};}
json xyz(const Vec3& p){return json{{"x",p.x},{"y",p.y},{"z",p.z}};}
json numeric(double value){return std::isfinite(value)?json(value):json(nullptr);}
json risk_json(const Risk& r){return json{{"blocked",r.blocked},{"immediate",r.immediate},{"clearance_m",numeric(r.clearance_m)},{"conflict_time_s",numeric(r.conflict_time_s)},{"obstacle_ids",r.obstacle_ids},{"moving",r.moving},{"uncertain",r.uncertain},{"immediate_obstacle_ids",r.immediate_obstacle_ids}};}
std::string general(double v){std::ostringstream out;out.imbue(std::locale::classic());out<<std::setprecision(6)<<std::defaultfloat<<v;return out.str();}
bool truthy(const json& value){if(value.is_null())return false;if(value.is_boolean())return value.get<bool>();if(value.is_number())return value.get<double>()!=0.;return !value.empty()&&(!value.is_string()||!value.get_ref<const std::string&>().empty());}
FusionProfile fusion_profile(const json& p) {
  FusionProfile f;
#define VALUE(name) f.name=p.at(#name).get<double>()
  VALUE(sensor_timeout_s);VALUE(track_memory_s);VALUE(association_distance_m);VALUE(velocity_confirmation_s);
  VALUE(velocity_fit_window_s);VALUE(velocity_fit_max_residual_m);VALUE(min_tracked_speed_m_s);VALUE(max_obstacle_speed_m_s);
  VALUE(stationary_velocity_variance_m2_s2);VALUE(prediction_horizon_s);VALUE(prediction_step_s);
  VALUE(half_length_m);VALUE(half_width_m);VALUE(clearance_margin_m);VALUE(payload_extra_margin_m);
#undef VALUE
  return f;
}
PointCloudPacket cloud_packet(const sensor_msgs::msg::PointCloud2& m) {
  PointCloudPacket p;p.sec=m.header.stamp.sec;p.nanosec=m.header.stamp.nanosec;p.width=m.width;p.height=m.height;p.point_step=m.point_step;p.row_step=m.row_step;p.frame_id=m.header.frame_id;p.is_bigendian=m.is_bigendian;p.is_dense=m.is_dense;p.data=m.data;
  for(const auto& f:m.fields)p.fields.push_back({f.name,f.offset,f.datatype,f.count});
  return p;
}
}  // namespace

PolicyObserverNode::PolicyObserverNode(const rclcpp::NodeOptions& options,bool observation_only)
 :Node("navigation_policy_observer",options),observation_only_(observation_only) {
  const auto default_profile=ament_index_cpp::get_package_share_directory("astribot_s1_navigation_policy")+"/config/simulation.json";
  const auto filename=declare_parameter<std::string>("profile",default_profile);
  const auto scan_topic=declare_parameter<std::string>("scan_topic","/scan_from_cloud");
  const auto vision_topic=declare_parameter<std::string>("vision_topic","/navigation_policy/vision_observations");
  const auto plan_topic=declare_parameter<std::string>("plan_topic",observation_only_?"/plan":"/path_tracking/active_path");
  const auto mode=declare_parameter<std::string>("navigation_geometry_mode","legacy");
  if(mode!="legacy" && mode!="fixed_v2")throw std::invalid_argument("invalid navigation_geometry_mode");
  const bool sim=get_parameter("use_sim_time").as_bool();clock_id_=sim?"sim":"ros";
  const auto baseline=load_policy_profile(filename,sim);
  profile_=std::make_unique<PolicyEnvelopeProfile>(baseline,mode=="fixed_v2");
  fusion_=std::make_unique<ConservativeFusion>(fusion_profile(baseline),profile_->tracking_frame());
  base_frame_=declare_parameter<std::string>("robot_base_frame","astribot_torso_base");
  declare_parameter<double>("localization_jump_m",.20);declare_parameter<double>("localization_jump_rad",.15);
  const auto specifications=parse_integer_json(declare_parameter<std::string>("observation_sources","[]"));
  std::set<std::string> required{"scan"};
  for(const auto& spec:specifications)if(truthy(spec.value("required",json(false))))required.insert(spec.at("sensor_id").get<std::string>());
  health_registry_=std::make_unique<SensorHealthRegistry>(profile_->value("sensor_timeout_s"),std::move(required));
  tf_=std::make_unique<tf2_ros::Buffer>(get_clock());
  listener_=std::make_shared<tf2_ros::TransformListener>(*tf_,this,false);
  // TF callbacks use the default group; processing uses a separate group in
  // the three-thread executor. Buffer's ROS-time overloads require this flag
  // even for a zero timeout; without it capture-time scans stay pending.
  tf_->setUsingDedicatedThread(true);
  processing_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions processing;processing.callback_group=processing_group_;
  observation_pub_=create_publisher<std_msgs::msg::String>("/navigation_policy/observation",10);
  health_pub_=create_publisher<astribot_navigation_msgs::msg::SensorHealthArray>("/navigation/sensor_health",10);
  if(mode=="legacy")subscriptions_.push_back(create_subscription<Envelope>("/navigation/robot_envelope",1,[this](Envelope::ConstSharedPtr m){envelope(m);}));
  else {
    ack_pub_=create_publisher<astribot_navigation_msgs::msg::EnvelopeApplyStatus>("/navigation/envelope_applied",10);
    subscriptions_.push_back(create_subscription<EnvelopeV2>("/navigation/envelope_v2",10,[this](EnvelopeV2::ConstSharedPtr m){envelope(m);}));
  }
  subscriptions_.push_back(create_subscription<Scan>(scan_topic,rclcpp::SensorDataQoS(),[this](Scan::ConstSharedPtr m){scan(m);}));
  subscriptions_.push_back(create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),[this](const nav_msgs::msg::Odometry& m){odom(m);}));
  subscriptions_.push_back(create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).transient_local(),[this](const nav_msgs::msg::OccupancyGrid& m){mapping(m);},processing));
  const auto plan_qos=observation_only_?rclcpp::QoS(10):rclcpp::QoS(1).transient_local();
  subscriptions_.push_back(create_subscription<nav_msgs::msg::Path>(plan_topic,plan_qos,[this](nav_msgs::msg::Path::ConstSharedPtr m){std::lock_guard<std::mutex> lock(plan_mutex_);pending_plan_=std::move(m);}));
  auto make=[this](const std::string& name,const AdapterOptions& opts){return make_observation_adapter(name,AdapterProfile{profile_->value("sensor_timeout_s"),profile_->tracking_frame()},[this](const auto& target,const auto& source,std::int64_t when){return adapter_transform(target,source,when);},[this](){return stamp();},steady_ns,opts);};
  vision_source_=std::make_unique<Source>();vision_source_->adapter=make("vision_json",{});
  subscriptions_.push_back(create_subscription<std_msgs::msg::String>(vision_topic,10,[this](const std_msgs::msg::String& m){accept_observations(*vision_source_,[&](){return vision_source_->adapter->normalize(VisionPacket{m.data});});},processing));
  for(const auto& spec:specifications) {
    auto source=std::make_unique<Source>();source->adapter=make(spec.at("adapter"),{});
    source->adapter->set_raw_options(spec);source->configuration=spec;
    source->requires_calibration=spec.contains("camera_info_topic")&&truthy(spec.at("camera_info_topic"));
    auto* ptr=source.get();
    if(source->requires_calibration)subscriptions_.push_back(create_subscription<sensor_msgs::msg::CameraInfo>(spec.at("camera_info_topic").get<std::string>(),rclcpp::SensorDataQoS(),[this,ptr](const sensor_msgs::msg::CameraInfo& m){camera_info(*ptr,m);},processing));
    const auto topic=spec.at("topic").get<std::string>();
    if(source->adapter->message_kind()==AdapterMessageKind::VISION_STRING)subscriptions_.push_back(create_subscription<std_msgs::msg::String>(topic,rclcpp::SensorDataQoS(),[this,ptr](const std_msgs::msg::String& m){accept_observations(*ptr,[&](){return ptr->adapter->normalize(VisionPacket{m.data});});},processing));
    else subscriptions_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(topic,rclcpp::SensorDataQoS(),[this,ptr](const sensor_msgs::msg::PointCloud2& m){accept_observations(*ptr,[&](){auto packet=cloud_packet(m);return ptr->adapter->normalize(packet);});},processing));
    sources_.push_back(std::move(source));
  }
  using Status=astribot_navigation_msgs::msg::NavigationExecutionStatus;
  subscriptions_.push_back(create_subscription<Status>("/navigation/execution_status",rclcpp::QoS(10).transient_local(),[this](const Status& m){execution_.task_wire(m.task_id,m.state,m.sequence);},processing));
  timer_=rclcpp::create_timer(this,get_clock(),std::chrono::milliseconds(100),[this](){tick();},processing_group_);
}

Stamp PolicyObserverNode::stamp() {
  const auto current=get_clock()->now().nanoseconds();
  last_time_=current;return Stamp(current,clock_id_,epoch_.load());
}
AdapterTransform PolicyObserverNode::adapter_transform(const std::string& target,const std::string& source,std::int64_t capture_ns) {
  (void)capture_ns;
  const auto at=tf2::TimePointZero;
  json_strings::require_tf_string(target);json_strings::require_tf_string(source);
  const auto tf=tf_->lookupTransform(target,source,at);
  const auto& t=tf.transform.translation;const auto& q=tf.transform.rotation;return {{t.x,t.y,t.z},{q.x,q.y,q.z,q.w}};
}
void PolicyObserverNode::envelope(const Envelope::ConstSharedPtr& message){std::lock_guard<std::mutex> lock(envelope_mutex_);pending_envelope_=message;}
void PolicyObserverNode::envelope(const EnvelopeV2::ConstSharedPtr& message) {
  {std::lock_guard<std::mutex> lock(envelope_mutex_);pending_envelope_=message;}
  std::lock_guard<std::mutex> lock(profile_mutex_);
  if(profile_->confirms_applied(*message,Stamp(get_clock()->now().nanoseconds(),clock_id_,epoch_.load())))ack(*message);
}
void PolicyObserverNode::ack(const EnvelopeV2& m) {
  if(!ack_pub_)return;
  astribot_navigation_msgs::msg::EnvelopeApplyStatus a;a.header.stamp=get_clock()->now();a.header.frame_id=profile_->base_frame();a.coordinator_session_id=m.coordinator_session_id;a.consumer_id="policy";a.envelope_epoch=m.epoch;a.installed_geometry_hash=m.installed_geometry_hash;a.applied=true;a.reason="POLYGON_APPLIED";ack_pub_->publish(a);
}
void PolicyObserverNode::process_envelope() {
  decltype(pending_envelope_) pending;
  {std::lock_guard<std::mutex> lock(envelope_mutex_);pending.swap(pending_envelope_);}
  if(std::holds_alternative<std::monostate>(pending))return;
  std::lock_guard<std::mutex> lock(profile_mutex_);
  try {
    const auto now=stamp();bool accepted=false;std::uint64_t envelope_epoch=0;
    if(auto legacy=std::get_if<Envelope::ConstSharedPtr>(&pending)){accepted=profile_->accept(**legacy,now);envelope_epoch=(*legacy)->epoch;}
    else {const auto& current=*std::get<EnvelopeV2::ConstSharedPtr>(pending);accepted=profile_->accept(current,now);envelope_epoch=current.epoch;}
    if(accepted){const auto v=execution_version(execution_);execution_.set_version(v.goal_id,v.path_revision,v.map_epoch,envelope_epoch,v.localization_epoch,v.clock_epoch);if(auto current=std::get_if<EnvelopeV2::ConstSharedPtr>(&pending))ack(**current);}
  } catch(const std::invalid_argument& error){RCLCPP_WARN(get_logger(),"%s",error.what());}
  fusion_->set_envelope_bounds(profile_->value("half_length_m"),profile_->value("half_width_m"));
}
void PolicyObserverNode::odom(const nav_msgs::msg::Odometry& m) {
  const auto& p=m.pose.pose.position;const auto& v=m.twist.twist;const auto a=observer_yaw(quaternion(m.pose.pose.orientation));
  const std::array<double,6> values{p.x,p.y,a,v.linear.x,v.linear.y,v.angular.z};
  if(m.header.frame_id!=profile_->tracking_frame()||!std::all_of(values.begin(),values.end(),[](double x){return std::isfinite(x);}))return;
  std::lock_guard<std::mutex> lock(odom_mutex_);
  if(odom_at_ && m.header.stamp.sec+m.header.stamp.nanosec*1e-9<*odom_at_)return;
  robot_=std::make_shared<RobotState>(values[0],values[1],values[2],values[3],values[4],values[5]);odom_at_=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;
}
void PolicyObserverNode::mapping(const nav_msgs::msg::OccupancyGrid& m) {
  const auto& p=m.info.origin.position;const auto& q=m.info.origin.orientation;
  if(map_.accept(m.header.frame_id,m.info.width,m.info.height,m.info.resolution,{p.x,p.y,p.z,q.x,q.y,q.z,q.w},m.data))execution_.map(map_.context_key());
}
void PolicyObserverNode::process_plan() {
  nav_msgs::msg::Path::ConstSharedPtr message;
  {std::lock_guard<std::mutex> lock(plan_mutex_);message=pending_plan_;}
  if(!message)return;
  try {
    const auto tf=transform_values(tf_->lookupTransform(profile_->tracking_frame(),message->header.frame_id,tf2::TimePointZero));
    Polygon path;path.reserve(message->poses.size());for(const auto& pose:message->poses){const auto& p=pose.pose.position;const auto point=observer_point({p.x,p.y,p.z},tf);path.push_back({point[0],point[1]});}
    path_=std::move(path);++path_revision_;execution_.path();fusion_->set_version(execution_version(execution_));
    std::lock_guard<std::mutex> lock(plan_mutex_);if(pending_plan_==message)pending_plan_.reset();
  } catch(const std::exception&){path_.clear();++errors_;}
}
void PolicyObserverNode::scan(const Scan::ConstSharedPtr& m) {
  if(!navigation::scan_usable(std::vector<double>(m->ranges.begin(),m->ranges.end()),m->range_min,m->range_max,m->angle_min,m->angle_increment,profile_->baseline.at("scan_min_valid_fraction").get<double>()))return;
  std::lock_guard<std::mutex> lock(scan_mutex_);pending_scans_.push_back(m);while(pending_scans_.size()>5)pending_scans_.pop_front();
}
void PolicyObserverNode::process_scans(const Stamp& now) {
  if(!map_.has_map())return;
  std::deque<Scan::ConstSharedPtr> batch,pending;Scan::ConstSharedPtr ready;
  {std::lock_guard<std::mutex> lock(scan_mutex_);batch.swap(pending_scans_);}
  for(const auto& m:batch) {
    if(scan_at_ && ns(m->header.stamp)*1e-9<=*scan_at_)continue;
    if(ready && ns(m->header.stamp)<=ns(ready->header.stamp))continue;
    if(tf_->canTransform(profile_->tracking_frame(),m->header.frame_id,tf2::TimePointZero)&&tf_->canTransform("map",m->header.frame_id,tf2::TimePointZero))ready=m;
    else pending.push_back(m);
  }
  {std::lock_guard<std::mutex> lock(scan_mutex_);pending.insert(pending.end(),pending_scans_.begin(),pending_scans_.end());while(pending.size()>5)pending.pop_front();pending_scans_.swap(pending);}
  if(ready)process_scan(*ready);
}
void PolicyObserverNode::process_scan(const Scan& m) {
  try {
    const auto now=stamp();const Stamp capture(ns(m.header.stamp),clock_id_,epoch_.load());
    const auto at=tf2::TimePointZero;
    const auto tracking=transform_values(tf_->lookupTransform(profile_->tracking_frame(),m.header.frame_id,at));
    const auto to_map=transform_values(tf_->lookupTransform("map",m.header.frame_id,at));
    const std::vector<double> ranges(m.ranges.begin(),m.ranges.end());
    const double size=profile_->value("scan_occupancy_resolution_m"),height=profile_->value("height_m"),padding=profile_->value("scan_obstacle_padding_m");
    const auto cells=astribot_s1_robot_geometry::scanOccupiedCells(ranges,m.range_min,m.range_max,m.angle_min,m.angle_increment,tracking,to_map,map_.projection_info(),map_.mask(),size);
    std::vector<Observation> observations;observations.reserve(cells.size());
    for(const auto& cell:cells) {
      const auto identity="cell:"+general(size)+":"+std::to_string(cell[0])+":"+std::to_string(cell[1]);const auto measured=std::to_string(capture.ns)+":"+identity;
      MetricBox box(Vec3((static_cast<double>(cell[0])+.5)*size,(static_cast<double>(cell[1])+.5)*size,height/2.),Vec3(size+2*padding,size+2*padding,height),Covariance3({.0004,0.,0.,0.,.0004,0.,0.,0.,.01}));
      observations.emplace_back("scan",measured,identity,capture,Stamp(steady_ns(),"steady",0),Stamp(capture.ns+static_cast<std::int64_t>(profile_->value("sensor_timeout_s")*1e9),clock_id_,epoch_.load()),profile_->tracking_frame(),0,box,1.,std::vector<std::pair<std::string,double>>{},std::vector<std::string>{"scan:"+measured},false,true);
    }
    fusion_->ingest(observations,now);
    const auto inverse=transform_values(tf_->lookupTransform(m.header.frame_id,profile_->tracking_frame(),at));
    fusion_->clear_observed_free(now,{},[&](const std::vector<MetricBox>& boxes){std::vector<std::array<double,4>> rows;rows.reserve(boxes.size());for(const auto& b:boxes)rows.push_back({b.center_m.x,b.center_m.y,b.size_m.x,b.size_m.y});return astribot_s1_robot_geometry::scanBoxesFree(rows,inverse,ranges,m.range_min,m.range_max,m.angle_min,m.angle_increment,size);});
    const auto body=tf_->lookupTransform(base_frame_,m.header.frame_id,at);
    const auto coverage=policy::scan_coverage(ranges,m.range_min,m.range_max,m.angle_min,m.angle_increment,observer_yaw(quaternion(body.transform.rotation)));
    health_registry_->record("scan",capture,now,base_frame_,coverage,true,0);scan_at_=capture.ns*1e-9;++scan_count_;
  } catch(const std::exception& error){++errors_;last_error_=error.what();RCLCPP_DEBUG(get_logger(),"scan rejected: %s",error.what());}
}
void PolicyObserverNode::camera_info(Source& source,const sensor_msgs::msg::CameraInfo& m) {
  const auto& raw_sensor=source.configuration.at("sensor_id");
  if(raw_sensor.is_array()||raw_sensor.is_object())throw std::invalid_argument("unhashable camera identifier");
  const auto sensor=raw_sensor.is_string()?raw_sensor.get<std::string>():std::string{};
  const auto old=calibrations_.records().find(sensor);const Integer epoch=old==calibrations_.records().end()?Integer(0):old->second.calibration_epoch;
  try {
    label(sensor,"camera_id");label(m.header.frame_id,"optical_frame");label(m.distortion_model,"distortion_model");
    require(m.width>0,"image_width_px");require(m.height>0,"image_height_px");require(epoch>=0,"calibration_epoch");
    // Humble exposes CameraInfo.K as numpy.float64 in the original callback.
    // Its strict built-in numeric contract rejects every wire coefficient.
    // Preserve this existing admission behavior here; accepting it is a
    // separate bug fix, not an implicit expansion of this migration's authority.
    throw ContractError(ErrorCode::INVALID_INPUT,"camera.coefficient");
  } catch(const std::invalid_argument& error){RCLCPP_WARN(get_logger(),"%s",error.what());}
}
void PolicyObserverNode::accept_observations(Source& source,const std::function<std::vector<Observation>()>& normalize) {
  try {
    const auto observations=normalize();const auto now=stamp();const auto& data=source.adapter->last_packet().value();
    const Stamp capture(adapter_int64(data.at("stamp_ns")),now.clock,now.epoch);
    const auto& raw_sensor=data.at("sensor_id");const auto epoch=adapter_integer(data.at("calibration_epoch"));
    // Empty observation packets defer sensor validation until the original
    // health-record transaction. An unhashable sensor fails during lookup;
    // other non-string sensors reach record(), whose clock adoption is visible.
    if(raw_sensor.is_array()||raw_sensor.is_object())throw std::invalid_argument("unhashable sensor identifier");
    const auto sensor=raw_sensor.is_string()?raw_sensor.get<std::string>():std::string{};
    if(source.requires_calibration)calibrations_.calibration(sensor,epoch);
    const auto previous=health_registry_->records().find(sensor);
    if(previous!=health_registry_->records().end()) {
      const auto& old=previous->second;
      if(epoch<old.calibration_epoch || (old.stamp.clock==now.clock && old.stamp.epoch==now.epoch && capture.ns<=old.stamp.ns))throw std::invalid_argument("duplicate, out-of-order or obsolete calibration");
    }
    std::vector<BearingCone> coverage;const auto raw_coverage=source.configuration.value("coverage_body_yaw_half_angle",AdapterJson::array());
    if(raw_coverage.is_array())for(const auto& cone:raw_coverage) {
      if(!cone.is_array()||cone.size()<2)throw std::invalid_argument("invalid coverage indexing");
      const auto& yaw_value=cone.at(0);const auto& half=cone.at(1);
      if(!yaw_value.is_number_float()&&!json_is_integer(yaw_value)&&!yaw_value.is_boolean())throw std::invalid_argument("invalid coverage yaw type");
      const auto yaw=adapter_float(yaw_value);
      const auto direction=Vec3(std::cos(yaw),std::sin(yaw),0.);
      require(half.is_number_float()||json_is_integer(half),"bearing.half_angle");coverage.emplace_back(direction,adapter_float(half));
    } else if(!((raw_coverage.is_object()||raw_coverage.is_string())&&raw_coverage.empty()))throw std::invalid_argument("invalid coverage iterable");
    const bool depth=!observations.empty()&&std::all_of(observations.begin(),observations.end(),[](const Observation& o){return std::holds_alternative<MetricBox>(o.geometry);});
    fusion_->ingest(observations,now);health_registry_->record(sensor,capture,now,base_frame_,coverage,depth,epoch);
    // Freshness is checked before iterating even an empty/malformed ID value.
    fusion_->resolve_unassociated(sensor,{},capture,now);
    const auto ids=data.value("resolved_measurement_ids",AdapterJson::array());
    // Resolve sequentially so malformed later items cannot undo earlier valid
    // clearance evidence. Non-string hashable IDs cannot match string IDs.
    const auto resolve=[&](const AdapterJson& id){if(id.is_array()||id.is_object())throw std::invalid_argument("unhashable resolution identifier");if(id.is_string())fusion_->resolve_unassociated(sensor,{id.get<std::string>()},capture,now);};
    if(ids.is_array())for(const auto& id:ids)resolve(id);
    else if(ids.is_object())for(auto it=ids.begin();it!=ids.end();++it)resolve(it.key());
    else if(ids.is_string()){const auto text=ids.get<std::string>();for(std::size_t i=0;i<text.size();){const auto c=static_cast<unsigned char>(text[i]);const auto count=c<128?1:c<224?2:c<240?3:4;resolve(text.substr(i,count));i+=count;}}
    else throw std::invalid_argument("resolution identifiers must be iterable");
    ++vision_count_;
  } catch(const std::exception& error){++errors_;RCLCPP_WARN(get_logger(),"observation rejected: %s",error.what());}
}
RiskProfile PolicyObserverNode::risk_profile() const {
  const auto v=[this](const char* name){return profile_->value(name);};
  return RiskProfile{SweepProfile{v("half_length_m"),v("half_width_m"),v("clearance_margin_m"),v("payload_extra_margin_m"),profile_->polygon()},v("max_speed_m_s"),v("reaction_time_s"),v("brake_deceleration_m_s2"),v("angular_brake_deceleration_rad_s2"),profile_->baseline.value("linear_stop_delay_s",0.),v("prediction_horizon_s")};
}
void PolicyObserverNode::tick() {
  const auto start=steady_seconds(),cpu_start=thread_seconds();process_envelope();auto now=stamp();
  try {
    const auto tf=tf_->lookupTransform(profile_->tracking_frame(),"map",tf2::TimePointZero);const auto& p=tf.transform.translation;
    if(execution_.localization({p.x,p.y,observer_yaw(quaternion(tf.transform.rotation))},get_parameter("localization_jump_m").as_double(),get_parameter("localization_jump_rad").as_double())&&!path_.empty()){path_.clear();last_error_="LOCALIZATION_CHANGED_REQUIRES_PATH_REFRESH";}
  } catch(const std::exception&){}
  const auto v=execution_version(execution_);execution_.set_version(v.goal_id,v.path_revision,v.map_epoch,v.envelope_epoch,v.localization_epoch,now.epoch);
  process_plan();fusion_->set_version(execution_version(execution_));const auto scan_start=steady_seconds();process_scans(now);const auto scan_elapsed=steady_seconds()-scan_start;
  std::shared_ptr<const RobotState> robot;std::optional<double> odom_at;
  {std::lock_guard<std::mutex> lock(odom_mutex_);robot=robot_;odom_at=odom_at_;}
  const auto scan_at=scan_at_;const auto sample=stamp();
  std::optional<std::array<double,3>> region;if(robot)region=std::array<double,3>{robot->x,robot->y,std::max(profile_->value("max_speed_m_s"),euclidean_norm(robot->vx,robot->vy))*profile_->value("prediction_horizon_s")};
  auto health=health_registry_->health(sample);fusion_->set_sensors(health);
  astribot_navigation_msgs::msg::SensorHealthArray message;message.stamp=get_clock()->now();
  for(const auto& source:health){astribot_navigation_msgs::msg::SensorHealth h;h.sensor_id=source.sensor_id;h.state=to_string(source.health);h.frame_id=source.frame_id;h.capture_stamp=wire_time(source.stamp.ns);h.valid_until=wire_time(source.valid_until.ns);h.calibration_epoch=wire_unsigned(source.calibration_epoch);h.depth_available=source.depth_available;h.reason=source.reason;for(const auto& cone:source.coverage){h.coverage_yaw.push_back(std::atan2(cone.direction.y,cone.direction.x));h.coverage_half_angle.push_back(cone.half_angle_rad);}message.sensors.push_back(std::move(h));}
  // Python constructs every message and applies its numeric/time setters
  // before converting strings for publish. Check UTF-8 at that same boundary.
  for(auto& source:message.sensors) {
    source.sensor_id=json_strings::health_wire_string(source.sensor_id);
    source.state=json_strings::health_wire_string(source.state);
    source.frame_id=json_strings::health_wire_string(source.frame_id);
    source.reason=json_strings::health_wire_string(source.reason);
  }
  health_pub_->publish(message);const auto snapshot_start=steady_seconds();auto world=fusion_->snapshot(sample,region);const auto risk_start=steady_seconds();
  std::optional<Risk> risk;if(robot)risk=evaluate_risk(world,*robot,path_,risk_profile());const auto risk_elapsed=steady_seconds()-risk_start;
  last_risk_=risk;last_world_=std::make_shared<WorldSnapshot>(std::move(world));last_robot_=robot;
  const auto seconds=get_clock()->now().nanoseconds()*1e-9;
  const bool required=health_registry_->required_valid(sample);const bool ready=profile_->ready(sample);
  const bool fresh=scan_at&&odom_at&&map_.has_map()&&required&&(observation_only_||ready);
  last_inputs_valid_=fresh;last_evaluation_epoch_=sample.epoch;json blocking=json::array();std::set<std::string> reported;
  if(risk){auto ids=risk->obstacle_ids;ids.insert(ids.end(),risk->immediate_obstacle_ids.begin(),risk->immediate_obstacle_ids.end());for(const auto& id:ids){if(!reported.insert(id).second)continue;const auto& track=*fusion_->tracks().at(id);const auto& o=track.observation;const auto& b=std::get<MetricBox>(o.geometry);blocking.push_back(json{{"id",id},{"center_m",xyz(b.center_m)},{"size_m",xyz(b.size_m)},{"velocity_m_s",xyz(track.velocity)},{"age_s",seconds-o.capture_stamp.ns*1e-9}});}}
  const auto& envelope=profile_->envelope();std::size_t predicted=0;for(const auto& track:last_world_->tracks)predicted+=has_predictions(track);
  json result{{"stamp_ns",sample.ns},{"epoch",sample.epoch},{"observation_only",observation_only_},{"inputs_valid",fresh},{"scan_count",scan_count_},{"vision_count",vision_count_},{"errors",errors_},{"last_error",last_error_},{"scan_age_s",scan_at?json(seconds-*scan_at):json(nullptr)},{"odom_age_s",odom_at?json(seconds-*odom_at):json(nullptr)},{"required_sensors_valid",required},{"envelope_ready",ready},{"envelope_reason",envelope?envelope->reason:"NO_ENVELOPE"},{"envelope_age_s",envelope?json((sample.ns-ns(envelope->stamp))*1e-9):json(nullptr)},{"tracks",last_world_->tracks.size()},{"unassociated",last_world_->unassociated.size()},{"path_revision",path_revision_},{"predicted_tracks",predicted},{"processing_wall_s",steady_seconds()-start},{"blocking_tracks",blocking},{"geometry_backend","cpp"},{"processing_cpu_s",thread_seconds()-cpu_start},{"processing_stages_s",{{"scan",scan_elapsed},{"snapshot",risk_start-snapshot_start},{"risk",risk_elapsed}}},{"risk",risk?risk_json(*risk):json(nullptr)}};
  std_msgs::msg::String output;output.data=dump_integer_json(result);observation_pub_->publish(output);
}
}  // namespace astribot::navigation::policy
