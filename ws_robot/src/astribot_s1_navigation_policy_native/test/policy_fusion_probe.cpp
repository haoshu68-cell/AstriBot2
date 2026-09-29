// Test-only JSON transport. The production core has no JSON dependency.
#include "astribot_s1_navigation_policy_native/policy_fusion.hpp"
#include "astribot_s1_navigation_policy_native/policy_integer_json.hpp"
#include <iostream>
#include <iterator>
#include <type_traits>
static_assert(!std::is_copy_assignable_v<astribot::navigation::policy::Observation>);
static_assert(!std::is_copy_assignable_v<astribot::navigation::policy::MetricBox>);
static_assert(!std::is_copy_assignable_v<astribot::navigation::policy::WorldSnapshot>);
using namespace astribot::navigation::policy;
using J=IntegerJson;
namespace {
std::int64_t integer(const J& j,const std::string& field) {
  require(j.is_number_integer()&&!j.is_boolean(),field);
  if(j.is_number_unsigned())require(j.get<std::uint64_t>()<=static_cast<std::uint64_t>(INT64_MAX),field);
  return j.get<std::int64_t>();
}
Integer wide_integer(const J& j,const std::string& field) {
  require(json_is_integer(j),field);return json_integer(j);
}
PixelScalar pixel(const J& j,const std::string& field) {
  require(json_is_integer(j)||j.is_number_float(),field);
  return json_is_integer(j)?PixelScalar(json_integer(j)):PixelScalar(j.get<double>());
}
J encode(const PixelScalar& value) { return value.is_integer()?integer_json(value.integer()):J(value.floating()); }
double number(const J& j,const std::string& field){require(j.is_number(),field);return j.get<double>();}
Stamp stamp(const J& j){return Stamp(integer(j.at("ns"),"stamp.ns"),j.at("clock").get<std::string>(),integer(j.at("epoch"),"stamp.epoch"));}
Vec3 vec(const J& j){require(j.is_array()&&j.size()==3,"vector.shape");return Vec3(number(j[0],"x"),number(j[1],"y"),number(j[2],"z"));}
Covariance3 cov(const J& j){require(j.is_array()&&j.size()==9,"covariance.shape");std::array<double,9> v{};for(int i=0;i<9;++i)v[i]=number(j[i],"covariance.value");return Covariance3(v);}
MetricBox box(const J& j){std::optional<Vec3> v;std::optional<Covariance3> c;
  if(j.contains("velocity_m_s")&&!j["velocity_m_s"].is_null())v.emplace(vec(j["velocity_m_s"]));
  if(j.contains("velocity_covariance_m2_s2")&&!j["velocity_covariance_m2_s2"].is_null())c.emplace(cov(j["velocity_covariance_m2_s2"]));
  // Evaluate validation in Python's argument order rather than C++'s unspecified order.
  const auto center=vec(j.at("center_m"));const auto size=vec(j.at("size_m"));const auto covariance=cov(j.at("position_covariance_m2"));
  return MetricBox(center,size,covariance,v,c);
}
Geometry geometry(const J& j){const auto kind=j.at("kind").get<std::string>();if(kind=="MetricBox")return box(j);
  if(kind=="BearingCone")return BearingCone(vec(j.at("direction")),j.at("half_angle_rad").get<double>());
  if(kind=="ImageBox") {
    const auto camera=j.at("camera_id").get<std::string>();
    const auto width=wide_integer(j.at("image_width_px"),"image.width");
    const auto height=wide_integer(j.at("image_height_px"),"image.height");
    const auto xmin=pixel(j.at("xmin_px"),"xmin_px");
    const auto ymin=pixel(j.at("ymin_px"),"ymin_px");
    const auto xmax=pixel(j.at("xmax_px"),"xmax_px");
    const auto ymax=pixel(j.at("ymax_px"),"ymax_px");
    return ImageBox(camera,width,height,xmin,ymin,xmax,ymax);
  }
  throw ContractError(ErrorCode::INVALID_INPUT,"geometry.kind");
}
Observation observation(const J& j){
  const auto capture=stamp(j.at("capture_stamp"));const auto receive=stamp(j.at("received_at"));const auto until=stamp(j.at("valid_until"));
  const auto g=geometry(j.at("geometry"));std::optional<std::string> source;if(!j.at("source_track_id").is_null())source=j.at("source_track_id").get<std::string>();
  std::vector<std::pair<std::string,double>> probabilities;for(const auto& p:j.at("class_probabilities")){require(p.size()==2,"class_probability.shape");probabilities.emplace_back(p[0].get<std::string>(),number(p[1],"class_probability"));}
  require(j.at("velocity_observable").is_boolean(),"velocity_observable");require(j.at("spatial_occupancy").is_boolean(),"spatial_occupancy");
  return Observation(j.at("sensor_id"),j.at("measurement_id"),source,capture,receive,until,j.at("frame_id"),wide_integer(j.at("calibration_epoch"),"calibration_epoch"),g,
    number(j.at("geometry_quality"),"geometry_quality"),probabilities,j.at("provenance").get<std::vector<std::string>>(),j.at("velocity_observable"),j.at("spatial_occupancy"));
}
Version::Counter counter(const J& j) {
  if(j.is_number_unsigned())return Version::Counter(j.get<std::uint64_t>());
  if(j.is_number_integer())return Version::Counter(j.get<std::int64_t>());
  return Version::Counter(false); // Defer type rejection to the original field order.
}
Version version(const J& j){return Version(j.at("goal_id"),counter(j.at("path_revision")),counter(j.at("map_epoch")),counter(j.at("envelope_epoch")),counter(j.value("localization_epoch",J(0))),counter(j.value("clock_epoch",J(0))));}
MotionLimits limits(const J& j){return MotionLimits(j.at("linear_speed_m_s"),j.at("angular_speed_rad_s"),j.at("linear_accel_m_s2"),j.at("angular_accel_rad_s2"));}
Planning planning(const J& j){const auto s=j.get<std::string>();if(s=="NONE")return Planning::NONE;if(s=="LOCAL")return Planning::LOCAL;if(s=="GLOBAL")return Planning::GLOBAL;throw std::invalid_argument("planning");}
Decision decision(const J& j){
  const std::map<std::string,Motion> motions{{"CONTINUE",Motion::CONTINUE},{"SLOW",Motion::SLOW},{"HOLD",Motion::HOLD},{"STOP",Motion::STOP},{"FOLLOW_COMMITTED_PATH",Motion::FOLLOW_COMMITTED_PATH},{"RETREAT",Motion::RETREAT}};
  const std::map<std::string,Trigger> triggers{{"NONE",Trigger::NONE},{"NEW_GOAL",Trigger::NEW_GOAL},{"PATH_RISK",Trigger::PATH_RISK}};
  std::optional<std::string> request;if(j.contains("request_id")&&!j["request_id"].is_null())request=j["request_id"].get<std::string>();
  std::optional<Version::Counter> committed;if(j.contains("committed_path_revision")&&!j["committed_path_revision"].is_null())committed.emplace(counter(j["committed_path_revision"]));
  return Decision(j.at("decision_id"),j.at("episode_id"),version(j.at("version")),stamp(j.at("issued_at")),stamp(j.at("valid_until")),motions.at(j.at("motion")),planning(j.at("planning")),triggers.at(j.at("trigger")),limits(j.at("limits")),j.at("reason"),request,committed);
}
ExecutionContext context(const J& j){std::set<Planning> ps;for(const auto& p:j.at("allowed_planning"))ps.insert(planning(p));return ExecutionContext(version(j.at("version")),stamp(j.at("now")),j.at("required_inputs_valid"),j.at("motion_enabled"),ps,j.at("retreat_enabled"),limits(j.at("baseline_limits")));}
Prediction prediction(const J& j){return Prediction(integer(j.at("offset_ns"),"prediction.offset_ns"),box(j.at("geometry")));}
PredictionModel model(const J& j){std::vector<std::pair<std::int64_t,double>> steps;
  for(const auto& s:j.at("steps")){require(s.size()==2,"prediction_model.step_size");steps.emplace_back(integer(s[0],"prediction_model.time_order"),number(s[1],"prediction_model.time"));}
  return PredictionModel(vec(j.at("velocity")),j.at("variance_m2_s2"),steps);
}
TrackedObstacle track(const J& j){std::vector<Prediction> points;for(const auto& p:j.at("predictions"))points.push_back(prediction(p));std::optional<PredictionModel> m;
  if(j.contains("prediction_model")&&!j["prediction_model"].is_null())m.emplace(model(j["prediction_model"]));
  return TrackedObstacle(j.at("fused_track_id"),j.at("frame_id"),stamp(j.at("stamp")),box(j.at("geometry")),points,j.at("provenance").get<std::vector<std::string>>(),m);
}
WorldSnapshot world(const J& j){std::vector<TrackedObstacle> tracks;for(const auto& t:j.at("tracks"))tracks.push_back(track(t));std::vector<Observation> unknown;for(const auto& o:j.at("unassociated"))unknown.push_back(observation(o));
  return WorldSnapshot(version(j.at("version")),stamp(j.at("stamp")),j.at("frame_id"),tracks,unknown,{},integer(j.at("observation_seq"),"observation_seq"));
}
void check_contract(const std::string& kind,const J& v){
  if(kind=="Decision")(void)decision(v);else if(kind=="ExecutionContext")(void)context(v);else if(kind=="Version")(void)version(v);else if(kind=="MotionLimits")(void)limits(v);
  else if(kind=="Prediction")(void)prediction(v);else if(kind=="PredictionModel")(void)model(v);else if(kind=="TrackedObstacle")(void)track(v);else if(kind=="WorldSnapshot")(void)world(v);
  else if(kind=="BearingCone")(void)BearingCone(vec(v.at("direction")),v.at("half_angle_rad"));else throw std::invalid_argument("unknown checked contract");
}
J encode(const Stamp& v){return J{{"ns",v.ns},{"clock",v.clock},{"epoch",v.epoch}};}
J encode(const Vec3& v){return J{{"x",v.x},{"y",v.y},{"z",v.z}};}
J encode(const Covariance3& v){return J{{"values",v.values}};}
J encode(const MetricBox& b){return J{{"center_m",encode(b.center_m)},{"size_m",encode(b.size_m)},{"position_covariance_m2",encode(b.position_covariance_m2)},
  {"velocity_m_s",b.velocity_m_s?encode(*b.velocity_m_s):J(nullptr)},{"velocity_covariance_m2_s2",b.velocity_covariance_m2_s2?encode(*b.velocity_covariance_m2_s2):J(nullptr)}};}
J encode(const BearingCone& b){return J{{"direction",encode(b.direction)},{"half_angle_rad",b.half_angle_rad}};}
J encode(const ImageBox& b){return J{{"camera_id",b.camera_id},{"image_width_px",b.image_width_px},{"image_height_px",b.image_height_px},{"xmin_px",encode(b.xmin_px)},{"ymin_px",encode(b.ymin_px)},{"xmax_px",encode(b.xmax_px)},{"ymax_px",encode(b.ymax_px)}};}
J encode(const Observation& o){return J{{"sensor_id",o.sensor_id},{"measurement_id",o.measurement_id},{"source_track_id",o.source_track_id?J(*o.source_track_id):J(nullptr)},
  {"capture_stamp",encode(o.capture_stamp)},{"received_at",encode(o.received_at)},{"valid_until",encode(o.valid_until)},{"frame_id",o.frame_id},{"calibration_epoch",o.calibration_epoch},
  {"geometry",std::visit([](const auto& g){return encode(g);},o.geometry)},{"geometry_quality",o.geometry_quality},{"class_probabilities",o.class_probabilities},
  {"provenance",o.provenance},{"velocity_observable",o.velocity_observable},{"spatial_occupancy",o.spatial_occupancy}};}
J encode(const Version& v){return J{{"goal_id",v.goal_id},{"path_revision",v.path_revision},{"map_epoch",v.map_epoch},{"envelope_epoch",v.envelope_epoch},{"localization_epoch",v.localization_epoch},{"clock_epoch",v.clock_epoch}};}
J encode(const SensorHealth& s){J cones=J::array();for(const auto& c:s.coverage)cones.push_back(encode(c));return J{{"sensor_id",s.sensor_id},{"health",to_string(s.health)},
  {"stamp",encode(s.stamp)},{"valid_until",encode(s.valid_until)},{"frame_id",s.frame_id},{"coverage",cones},{"depth_available",s.depth_available},{"calibration_epoch",s.calibration_epoch},{"reason",s.reason}};}
J encode(const PredictionModel& m){return J{{"velocity",encode(m.velocity)},{"variance_m2_s2",m.variance_m2_s2},{"steps",m.steps}};}
J encode(const TrackedObstacle& t){J predictions=J::array();for(const auto& p:t.predictions)predictions.push_back(J{{"offset_ns",p.offset_ns},{"geometry",encode(p.geometry)}});
  return J{{"fused_track_id",t.fused_track_id},{"frame_id",t.frame_id},{"stamp",encode(t.stamp)},{"geometry",encode(t.geometry)},
    {"predictions",predictions},{"provenance",t.provenance},{"prediction_model",t.prediction_model?encode(*t.prediction_model):J(nullptr)}};}
J encode(const WorldSnapshot& w){J tracks=J::array(),unknown=J::array(),sensors=J::array();for(const auto& t:w.tracks)tracks.push_back(encode(t));for(const auto& o:w.unassociated)unknown.push_back(encode(o));for(const auto& s:w.sensors)sensors.push_back(encode(s));
  return J{{"version",encode(w.version)},{"stamp",encode(w.stamp)},{"frame_id",w.frame_id},{"tracks",tracks},{"unassociated",unknown},{"sensors",sensors},{"observation_seq",w.observation_seq}};}
FusionProfile profile(const J& j){FusionProfile p;
#define PARAM(name) if(j.contains(#name))p.name=j.at(#name).get<double>()
  PARAM(sensor_timeout_s);PARAM(track_memory_s);PARAM(association_distance_m);PARAM(velocity_confirmation_s);PARAM(velocity_fit_window_s);PARAM(velocity_fit_max_residual_m);
  PARAM(min_tracked_speed_m_s);PARAM(max_obstacle_speed_m_s);PARAM(stationary_velocity_variance_m2_s2);PARAM(prediction_horizon_s);PARAM(prediction_step_s);
  PARAM(half_length_m);PARAM(half_width_m);PARAM(clearance_margin_m);PARAM(payload_extra_margin_m);
#undef PARAM
  return p;
}
}
int main(){try{
  const J input=parse_integer_json(std::string(std::istreambuf_iterator<char>(std::cin),{}));
  ConservativeFusion fusion(profile(input.value("profile",J::object())),input.value("frame_id",std::string("odom")));
  J output=J::array();for(const auto& op:input.at("operations")){try{
    J result=nullptr;const std::string action=op.at("action");
    if(action=="update"||action=="ingest"){std::vector<Observation> observations;for(const auto& item:op.at("observations"))observations.push_back(observation(item));const auto now=stamp(op.at("now"));
      if(action=="update")result=encode(fusion.update(observations,now));else fusion.ingest(observations,now);
    }else if(action=="snapshot"){std::optional<std::array<double,3>> region;if(op.contains("region"))region=op["region"].get<std::array<double,3>>();if(op.contains("region_velocity"))region=std::array<double,3>{0.,0.,std::max(fusion.profile.max_obstacle_speed_m_s,euclidean_norm(op["region_velocity"][0],op["region_velocity"][1]))*fusion.profile.prediction_horizon_s};result=encode(fusion.snapshot(stamp(op.at("now")),region));
    }else if(action=="clear"){
      if(op.contains("free_rule")) {const double x=op["free_rule"][0],covariance=op["free_rule"][1];
        fusion.clear_observed_free(stamp(op.at("now")),{},[&](const auto& boxes){std::vector<bool> flags;for(const auto& b:boxes)flags.push_back(b.center_m.x>=x&&b.position_covariance_m2.values[0]<covariance);return flags;});
      } else {const auto flags=op.at("flags").get<std::vector<bool>>();fusion.clear_observed_free(stamp(op.at("now")),{},[&](const auto&){return flags;});}
    }else if(action=="resolve")fusion.resolve_unassociated(op.at("sensor_id"),op.at("measurement_ids").get<std::vector<std::string>>(),stamp(op.at("capture")),stamp(op.at("now")));
    else if(action=="set_envelope_bounds")fusion.set_envelope_bounds(op.at("bounds")[0],op.at("bounds")[1]);
    else if(action=="norm"){const auto& v=op.at("value");result=v.size()==2?euclidean_norm(v[0],v[1]):euclidean_norm(v[0],v[1],v[2]);}
    else if(action=="check_contract")check_contract(op.at("type"),op.at("value"));
    else if(action=="check_executable")check_executable(decision(op.at("decision")),context(op.at("context")));
    else if(action=="check_fresh")observation(op.at("observation")).check_fresh(stamp(op.at("now")),integer(op.at("max_age_ns"),"max_age_ns"),op.value("future_tolerance_ns",0LL));
    else if(action=="set_version")fusion.set_version(version(op.at("version")));
    else if(action=="validate"){const auto type=op.at("type").get<std::string>();const auto& v=op.at("value");
      if(type=="Observation")result=encode(observation(v));else if(type=="Covariance3")result=encode(cov(v));else if(type=="Vec3")result=encode(vec(v));else if(type=="Stamp")result=encode(stamp(v));else if(type=="MetricBox")result=encode(box(v));
      else throw std::invalid_argument("unknown contract");
    }else throw std::invalid_argument("unknown action");
    output.push_back(J{{"ok",true},{"result",result}});
  }catch(const ContractError& e){output.push_back(J{{"ok",false},{"code",to_string(e.code)},{"field",e.field}});}}
  std::cout<<dump_integer_json(output)<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
