#include "astribot_s1_navigation_policy_native/policy_contracts.hpp"
#include <algorithm>
#include <cmath>
#include <tuple>
#include <limits>

namespace astribot::navigation::policy {
const char* to_string(ErrorCode code) {
  switch(code) {
    case ErrorCode::INVALID_INPUT:return "INVALID_INPUT";
    case ErrorCode::CLOCK_MISMATCH:return "CLOCK_MISMATCH";
    case ErrorCode::STALE_OBSERVATION:return "STALE_OBSERVATION";
    case ErrorCode::FUTURE_OBSERVATION:return "FUTURE_OBSERVATION";
    case ErrorCode::VERSION_MISMATCH:return "VERSION_MISMATCH";
    case ErrorCode::CAPABILITY_UNAVAILABLE:return "CAPABILITY_UNAVAILABLE";
    case ErrorCode::UNSAFE_DECISION:return "UNSAFE_DECISION";
  }
  return "INVALID_INPUT";
}
ContractError::ContractError(ErrorCode c,std::string f)
  :std::invalid_argument(std::string(to_string(c))+": "+f),code(c),field(std::move(f)) {}
void require(bool condition,const std::string& field,ErrorCode code) {
  if(!condition)throw ContractError(code,field);
}
void finite(double value,const std::string& field,std::optional<double> minimum) {
  require(std::isfinite(value),field);
  require(!minimum || value>=*minimum,field);
}
void finite(const PixelScalar& value,const std::string& field,std::optional<double> minimum) {
  require(value.is_finite(),field);
  require(!minimum||value>=PixelScalar(*minimum),field);
}
void label(const std::string& value,const std::string& field) {
  // Python str.strip() recognizes these Unicode whitespace code points.
  bool nonspace=false;
  for(std::size_t i=0;i<value.size();) {
    auto c=static_cast<unsigned char>(value[i++]); std::uint32_t cp=c;
    unsigned count=0;
    if((c&0xe0)==0xc0){cp=c&0x1f;count=1;}
    else if((c&0xf0)==0xe0){cp=c&0x0f;count=2;}
    else if((c&0xf8)==0xf0){cp=c&0x07;count=3;}
    while(count-- && i<value.size())cp=(cp<<6)|(static_cast<unsigned char>(value[i++])&0x3f);
    const bool space=(cp>=9&&cp<=13)||(cp>=0x1c&&cp<=0x20)||cp==0x85||cp==0xa0||cp==0x1680||
      (cp>=0x2000&&cp<=0x200a)||cp==0x2028||cp==0x2029||cp==0x202f||cp==0x205f||cp==0x3000;
    nonspace=nonspace||!space;
  }
  require(nonspace,field);
}
Stamp::Stamp(std::int64_t n,std::string c,std::int64_t e):ns(n),clock(std::move(c)),epoch(e) {
  label(clock,"clock");require(ns>=0,"stamp.ns");require(epoch>=0,"stamp.epoch");
}
std::int64_t Stamp::since(const Stamp& other) const {
  require(clock==other.clock&&epoch==other.epoch,"stamp.clock/epoch",ErrorCode::CLOCK_MISMATCH);
  return ns-other.ns;
}
bool Stamp::operator==(const Stamp& o) const {return ns==o.ns&&clock==o.clock&&epoch==o.epoch;}
Vec3::Vec3(double a,double b,double c):x(a),y(b),z(c) {finite(x,"x");finite(y,"y");finite(z,"z");}
bool Vec3::operator==(const Vec3& o) const {return x==o.x&&y==o.y&&z==o.z;}
Covariance3::Covariance3(std::array<double,9> v):values(v) {
  double scale=0.;for(double x:values){finite(x,"covariance.value");scale=std::max(scale,std::abs(x));}
  auto a=values;if(scale)for(auto& x:a)x/=scale;
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)require(std::abs(a[i*3+j]-a[j*3+i])<=1e-10,"covariance.symmetry");
  for(int i=0;i<3;++i)require(a[i*3+i]>=0,"covariance.diagonal");
  for(int i=0;i<3;++i)for(int j=i+1;j<3;++j)require(a[i*3+i]*a[j*3+j]-a[i*3+j]*a[i*3+j]>=-1e-12,"covariance.minors");
  const double det=a[0]*(a[4]*a[8]-a[5]*a[7])-a[1]*(a[3]*a[8]-a[5]*a[6])+a[2]*(a[3]*a[7]-a[4]*a[6]);
  require(det>=-1e-12,"covariance.positive_semidefinite");
}
CameraCalibration::CameraCalibration(std::string id,std::string frame,Integer e,Integer width,Integer height,
  std::array<double,9> k,std::string model,std::vector<double> coefficients,std::optional<double> depth)
  :camera_id(std::move(id)),optical_frame(std::move(frame)),calibration_epoch(e),image_width_px(width),
  image_height_px(height),intrinsic_matrix(k),distortion_model(std::move(model)),
  distortion_coefficients(std::move(coefficients)),depth_unit_m(depth) {
  label(camera_id,"camera_id");label(optical_frame,"optical_frame");label(distortion_model,"distortion_model");
  require(image_width_px>0,"image_width_px");require(image_height_px>0,"image_height_px");
  require(calibration_epoch>=0,"calibration_epoch");
  for(double v:intrinsic_matrix)finite(v,"camera.coefficient");
  for(double v:distortion_coefficients)finite(v,"camera.coefficient");
  require(k[0]>0&&k[4]>0&&k[3]==0&&k[6]==0&&k[7]==0&&k[8]==1,"intrinsic_matrix.pinhole");
  if(depth_unit_m){finite(*depth_unit_m,"depth_unit_m",0);require(*depth_unit_m>0,"depth_unit_m");}
}
bool CameraCalibration::operator==(const CameraCalibration& o) const {
  return std::tie(camera_id,optical_frame,calibration_epoch,image_width_px,image_height_px,intrinsic_matrix,
    distortion_model,distortion_coefficients,depth_unit_m)==std::tie(o.camera_id,o.optical_frame,o.calibration_epoch,
    o.image_width_px,o.image_height_px,o.intrinsic_matrix,o.distortion_model,o.distortion_coefficients,o.depth_unit_m);
}
ImageBox::ImageBox(std::string id,Integer w,Integer h,PixelScalar xmin,PixelScalar ymin,PixelScalar xmax,PixelScalar ymax)
  :camera_id(std::move(id)),image_width_px(w),image_height_px(h),xmin_px(xmin),ymin_px(ymin),xmax_px(xmax),ymax_px(ymax) {
  label(camera_id,"camera_id");require(w>0,"image.width");require(h>0,"image.height");
  finite(xmin,"xmin_px",0);finite(ymin,"ymin_px",0);finite(xmax,"xmax_px",0);finite(ymax,"ymax_px",0);
  require(xmin<xmax&&xmax<=w,"image.box.x");require(ymin<ymax&&ymax<=h,"image.box.y");
}
BearingCone::BearingCone(Vec3 d,double angle):direction(d),half_angle_rad(angle) {
  require(std::abs(euclidean_norm(d.x,d.y,d.z)-1.)<=1e-6,"bearing.unit_direction");
  finite(angle,"bearing.half_angle",0);require(angle<=std::acos(-1.),"bearing.half_angle");
}
MetricBox::MetricBox(Vec3 center,Vec3 size,Covariance3 covariance,std::optional<Vec3> velocity,
  std::optional<Covariance3> velocity_covariance):center_m(center),size_m(size),position_covariance_m2(covariance),
  velocity_m_s(velocity),velocity_covariance_m2_s2(velocity_covariance) {
  require(size.x>0&&size.y>0&&size.z>0,"metric.size");
  require(velocity.has_value()==velocity_covariance.has_value(),"metric.velocity_and_covariance");
}
Observation::Observation(std::string sensor,std::string measurement,std::optional<std::string> source,
  Stamp capture,Stamp receive,Stamp until,std::string frame,Integer calibration,Geometry g,double quality,
  std::vector<std::pair<std::string,double>> probabilities,std::vector<std::string> origins,bool observable,bool occupancy)
  :sensor_id(std::move(sensor)),measurement_id(std::move(measurement)),source_track_id(std::move(source)),
  capture_stamp(capture),received_at(receive),valid_until(until),frame_id(std::move(frame)),calibration_epoch(calibration),
  geometry(std::move(g)),geometry_quality(quality),class_probabilities(std::move(probabilities)),
  provenance(std::move(origins)),velocity_observable(observable),spatial_occupancy(occupancy) {
  if(spatial_occupancy){const auto* box=std::get_if<MetricBox>(&geometry);
    require(box&&!velocity_observable&&!box->velocity_m_s&&source_track_id.has_value(),"spatial_occupancy.requires_fixed_metric_cell");}
  label(sensor_id,"sensor_id");label(measurement_id,"measurement_id");label(frame_id,"frame_id");
  if(source_track_id)label(*source_track_id,"source_track_id");
  require(received_at.clock=="steady","received_at.clock");
  require(valid_until.since(capture_stamp)>0,"observation.valid_until");
  require(calibration_epoch>=0,"calibration_epoch");finite(geometry_quality,"geometry_quality",0);
  require(geometry_quality<=1,"geometry_quality");
  std::set<std::string> labels;double sum=0.;
  for(const auto& pair:class_probabilities){label(pair.first,"class_name");finite(pair.second,"class_probability",0);
    require(pair.second<=1&&labels.insert(pair.first).second,"class_probability");sum+=pair.second;}
  require(sum<=1+1e-9,"class_probability.sum");require(!provenance.empty(),"provenance.empty");
  for(const auto& p:provenance)label(p,"provenance.item");
  require(std::set<std::string>(provenance.begin(),provenance.end()).size()==provenance.size(),"provenance.duplicate");
}
void Observation::check_fresh(const Stamp& now,std::int64_t max_age,std::int64_t future) const {
  (void)now;(void)max_age;(void)future;
}
const char* to_string(Health health){switch(health){case Health::VALID:return "VALID";case Health::DEGRADED:return "DEGRADED";
  case Health::UNAVAILABLE:return "UNAVAILABLE";case Health::STALE:return "STALE";}return "UNAVAILABLE";}
SensorHealth::SensorHealth(std::string id,Health h,Stamp s,Stamp until,std::string frame,
  std::vector<BearingCone> cones,bool depth,Integer calibration,std::string why)
  :sensor_id(std::move(id)),health(h),stamp(s),valid_until(until),frame_id(std::move(frame)),coverage(std::move(cones)),
  depth_available(depth),calibration_epoch(calibration),reason(std::move(why)) {
  label(sensor_id,"sensor_id");label(frame_id,"frame_id");label(reason,"reason");
  require(h==Health::VALID||h==Health::DEGRADED||h==Health::UNAVAILABLE||h==Health::STALE,"health");
  require(valid_until.since(stamp)>0,"health.valid_until");require(health!=Health::VALID||!coverage.empty(),"valid_health.coverage");
  require(calibration_epoch>=0,"calibration_epoch");
}
Version::Version(std::string goal,Counter path,Counter map,Counter envelope,Counter localization,Counter clock)
  :goal_id(std::move(goal)),path_revision(path.value),map_epoch(map.value),envelope_epoch(envelope.value),
   localization_epoch(localization.value),clock_epoch(clock.value) {
  label(goal_id,"goal_id");require(path.valid_integer,"path_revision");require(map.valid_integer,"map_epoch");
  require(envelope.valid_integer,"envelope_epoch");require(localization.valid_integer,"localization_epoch");
  require(clock.valid_integer,"clock_epoch");
}
bool Version::operator==(const Version& o) const {return std::tie(goal_id,path_revision,map_epoch,envelope_epoch,localization_epoch,clock_epoch)==
  std::tie(o.goal_id,o.path_revision,o.map_epoch,o.envelope_epoch,o.localization_epoch,o.clock_epoch);}
MotionLimits::MotionLimits(double v,double w,double a,double alpha):linear_speed_m_s(v),angular_speed_rad_s(w),linear_accel_m_s2(a),angular_accel_rad_s2(alpha) {
  finite(v,"linear_speed_m_s",0);finite(w,"angular_speed_rad_s",0);finite(a,"linear_accel_m_s2",0);finite(alpha,"angular_accel_rad_s2",0);
}
Decision::Decision(std::string id,std::string episode,Version ver,Stamp issued,Stamp until,Motion m,Planning p,Trigger t,
  MotionLimits bounds,std::string why,std::optional<std::string> request,std::optional<Version::Counter> committed)
  :decision_id(std::move(id)),episode_id(std::move(episode)),version(ver),issued_at(issued),valid_until(until),motion(m),planning(p),trigger(t),
  limits(bounds),reason(std::move(why)),request_id(std::move(request)),committed_path_revision(committed ? std::optional<std::uint64_t>(committed->value) : std::nullopt) {
  label(decision_id,"decision_id");label(episode_id,"episode_id");label(reason,"reason");
  require(m>=Motion::CONTINUE&&m<=Motion::RETREAT&&p>=Planning::NONE&&p<=Planning::GLOBAL&&t>=Trigger::NONE&&t<=Trigger::PATH_RISK,"decision.enums");
  require(issued_at.clock=="steady","decision.clock");require(valid_until.since(issued_at)>0,"decision.valid_until");
  if(planning!=Planning::NONE){require(trigger!=Trigger::NONE,"planning.trigger");require(request_id.has_value(),"planning.request_id");label(*request_id,"planning.request_id");}
  else require(!request_id,"planning.unexpected_request");
  if(motion==Motion::HOLD||motion==Motion::STOP)require(limits.linear_speed_m_s==0&&limits.angular_speed_rad_s==0,"hold/stop.speed",ErrorCode::UNSAFE_DECISION);
  if(motion==Motion::FOLLOW_COMMITTED_PATH||motion==Motion::RETREAT)
    require(committed && committed->valid_integer && committed->value==version.path_revision,"committed_path_revision",ErrorCode::VERSION_MISMATCH);
  else require(!committed_path_revision,"unexpected_committed_revision");
}
ExecutionContext::ExecutionContext(Version v,Stamp s,bool valid,bool enabled,std::set<Planning> planning,bool retreat,MotionLimits limits)
  :version(v),now(s),required_inputs_valid(valid),motion_enabled(enabled),allowed_planning(std::move(planning)),retreat_enabled(retreat),baseline_limits(limits) {
  require(now.clock=="steady","context.clock");for(auto p:allowed_planning)require(p>=Planning::NONE&&p<=Planning::GLOBAL,"allowed_planning");
}
void check_executable(const Decision& d,const ExecutionContext& c) {
  require(c.motion_enabled,"observation_only",ErrorCode::CAPABILITY_UNAVAILABLE);
  require(d.version==c.version,"decision.version",ErrorCode::VERSION_MISMATCH);
  require(d.planning==Planning::NONE||c.allowed_planning.count(d.planning),"planning.capability",ErrorCode::CAPABILITY_UNAVAILABLE);
  require(c.retreat_enabled||d.motion!=Motion::RETREAT,"retreat.capability",ErrorCode::CAPABILITY_UNAVAILABLE);
  require(c.required_inputs_valid||(d.motion==Motion::STOP&&d.planning==Planning::NONE),"required_inputs",ErrorCode::UNSAFE_DECISION);
  require(d.limits.linear_speed_m_s<=c.baseline_limits.linear_speed_m_s,"limits.linear_speed_m_s",ErrorCode::UNSAFE_DECISION);
  require(d.limits.angular_speed_rad_s<=c.baseline_limits.angular_speed_rad_s,"limits.angular_speed_rad_s",ErrorCode::UNSAFE_DECISION);
  require(d.limits.linear_accel_m_s2<=c.baseline_limits.linear_accel_m_s2,"limits.linear_accel_m_s2",ErrorCode::UNSAFE_DECISION);
  require(d.limits.angular_accel_rad_s2<=c.baseline_limits.angular_accel_rad_s2,"limits.angular_accel_rad_s2",ErrorCode::UNSAFE_DECISION);
}
Prediction::Prediction(std::int64_t offset,MetricBox box):offset_ns(offset),geometry(box){require(offset>=0,"prediction.offset_ns");}
PredictionModel::PredictionModel(Vec3 v,double variance,std::vector<std::pair<std::int64_t,double>> times)
  :velocity(v),variance_m2_s2(variance),steps(std::move(times)) {
  finite(variance,"prediction_model.variance",0);std::int64_t previous=0;
  for(const auto& step:steps){require(step.first>previous,"prediction_model.time_order");finite(step.second,"prediction_model.time",0);
    require(std::abs(step.second-step.first*1e-9)<=1e-9,"prediction_model.time_units");previous=step.first;}
}
TrackedObstacle::TrackedObstacle(std::string id,std::string frame,Stamp s,MetricBox box,std::vector<Prediction> points,
  std::vector<std::string> sources,std::optional<PredictionModel> model):fused_track_id(std::move(id)),frame_id(std::move(frame)),stamp(s),
  geometry(box),predictions(std::move(points)),provenance(std::move(sources)),prediction_model(std::move(model)) {
  label(fused_track_id,"fused_track_id");label(frame_id,"track.frame_id");require(predictions.empty()||!prediction_model,"predictions.single_representation");
  for(std::size_t i=1;i<predictions.size();++i)require(predictions[i-1].offset_ns<predictions[i].offset_ns,"predictions.order");
  for(const auto& p:provenance)label(p,"track.provenance.item");
  require(!provenance.empty()&&std::set<std::string>(provenance.begin(),provenance.end()).size()==provenance.size(),"track.provenance");
}
WorldSnapshot::WorldSnapshot(Version v,Stamp s,std::string frame,std::vector<TrackedObstacle> obstacles,
  std::vector<Observation> unknown,std::vector<SensorHealth> health,std::int64_t seq):version(v),stamp(s),frame_id(std::move(frame)),
  tracks(std::move(obstacles)),unassociated(std::move(unknown)),sensors(std::move(health)),observation_seq(seq) {
  label(frame_id,"world.frame_id");require(seq>=0,"observation_seq");std::set<std::string> ids;
  for(const auto& t:tracks)ids.insert(t.fused_track_id);
  require(ids.size()==tracks.size(),"world.duplicate_tracks");ids.clear();
  for(const auto& sensor:sensors)ids.insert(sensor.sensor_id);
  require(ids.size()==sensors.size(),"world.duplicate_sensors");
  for(const auto& t:tracks){require(t.frame_id==frame_id,"world.track_frame");require(stamp.since(t.stamp)>=0,"world.track_stamp");}
}
}  // namespace astribot::navigation::policy
