#include "astribot_s1_navigation_policy_native/policy_fusion.hpp"
#include "astribot_s1_robot_geometry/fusion_snapshot.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>

namespace astribot::navigation::policy {
namespace {
using Key=std::pair<std::string,std::string>;
using TrackMap=std::map<std::string,std::shared_ptr<const FusionTrack>>;
const Vec3 zero(0.,0.,0.);
const MetricBox& metric(const Observation& o){return std::get<MetricBox>(o.geometry);}
std::int64_t nanoseconds(double seconds) {
  const double ns=seconds*1e9;
  require(std::isfinite(ns)&&ns<std::ldexp(1.,63),"fusion.time_range");
  return static_cast<std::int64_t>(ns);
}
// Python round uses ties-to-even, independently of the caller's FP rounding mode.
double round_even(double x) {
  const double lo=std::floor(x),fraction=x-lo;
  return fraction<.5?lo:(fraction>.5?lo+1.:(std::fmod(lo,2.)==0.?lo:lo+1.));
}
Observation with_geometry(const Observation& o,MetricBox box,bool observable,std::vector<std::string> provenance) {
  return Observation(o.sensor_id,o.measurement_id,o.source_track_id,o.capture_stamp,o.received_at,o.valid_until,
    o.frame_id,o.calibration_epoch,std::move(box),o.geometry_quality,o.class_probabilities,std::move(provenance),observable,o.spatial_occupancy);
}
// The shared kernel intentionally rejects non-finite derived values. Preserve
// the default Python contract's exception fields and its conservative +infinity
// region radius at finite extreme inputs, without changing the batched hot path.
void snapshot_default(const std::vector<astribot_s1_robot_geometry::SnapshotTrackInput>& inputs,
  std::int64_t now,const astribot_s1_robot_geometry::SnapshotParams& p,
  std::vector<astribot_s1_robot_geometry::SnapshotTrackOutput>& outputs) {
  try {astribot_s1_robot_geometry::snapshotTracks(inputs,now,p,outputs);}
  catch(const std::invalid_argument& error) {
    const std::string reason=error.what();
    if(reason=="snapshot center overflow") {
      for(const auto& in:inputs) {
        const double dt=std::min(std::max(0.,static_cast<double>(now-in.capture_ns)*1e-9),p.track_memory_s);
        if(!in.spatial_occupancy)for(int k=0;k<3;++k)
          finite(in.center[k]+in.track_velocity[k]*dt,k==0?"x":(k==1?"y":"z"));
      }
    }
    if(reason=="finite snapshot covariance required")throw ContractError(ErrorCode::INVALID_INPUT,"covariance.value");
    if(reason=="snapshot region overflow") {
      if(inputs.size()==1) {
        auto no_region=p;no_region.has_region=false;
        snapshot_default(inputs,now,no_region,outputs);
        // All radius terms are nonnegative finite inputs. Overflow can only be
        // +infinity, for which Python's finite-or-infinite distance <= radius.
        outputs[0].relevant=true;
      } else {
        outputs.clear();
        for(const auto& in:inputs) {
          std::vector<astribot_s1_robot_geometry::SnapshotTrackOutput> one;
          snapshot_default({in},now,p,one);outputs.push_back(one.front());
        }
      }
      return;
    }
    throw;
  }
}
struct AssociationIndex {
  using Evidence=std::tuple<std::int64_t,std::string,std::int64_t,std::string>;
  using Cell=std::pair<double,double>;
  const TrackMap& tracks;
  const double memory,cell_size;
  std::unique_ptr<Stamp> stamp;
  std::map<Cell,std::set<std::string>> cells;
  std::map<std::string,Cell> track_cells;
  std::map<Evidence,std::set<std::string>> evidence;
  std::map<Key,std::set<std::string>> identities;
  AssociationIndex(const TrackMap& t,const FusionProfile& p):tracks(t),memory(p.track_memory_s),cell_size(p.association_distance_m+.2) {
    for(const auto& item:tracks)add_evidence(item.first,item.second->observation);
  }
  static Evidence evidence_key(const Observation& o,const std::string& p) {
    return {o.capture_stamp.ns,o.capture_stamp.clock,o.capture_stamp.epoch,p};
  }
  void add_evidence(const std::string& id,const Observation& o) {
    if(o.source_track_id)identities[{o.sensor_id,*o.source_track_id}].insert(id);
    for(const auto& p:o.provenance)evidence[evidence_key(o,p)].insert(id);
  }
  bool correlated(const Observation& o) const {
    if(!std::holds_alternative<MetricBox>(o.geometry))return false;
    std::set<std::string> ids;
    for(const auto& p:o.provenance){const auto found=evidence.find(evidence_key(o,p));if(found!=evidence.end())ids.insert(found->second.begin(),found->second.end());}
    const auto& c=metric(o).center_m;
    for(const auto& id:ids){const auto& source=tracks.at(id)->observation;const auto& a=metric(source).center_m;
      if(source.spatial_occupancy==o.spatial_occupancy&&euclidean_norm(a.x-c.x,a.y-c.y)<.05)return true;}
    return false;
  }
  void add_cell(const std::string& id,const FusionTrack& t) {
    if(!stamp)return;
    const double dt=stamp->since(t.observation.capture_stamp)*1e-9;if(dt<0)return;
    const auto& c=metric(t.observation).center_m;const double time=std::min(dt,memory);
    const Cell cell{std::floor((c.x+t.velocity.x*time)/cell_size),std::floor((c.y+t.velocity.y*time)/cell_size)};
    require(std::isfinite(cell.first)&&std::isfinite(cell.second),"fusion.association_cell");
    cells[cell].insert(id);track_cells[id]=cell;
  }
  std::vector<std::string> nearby(const Observation& o) {
    if(!stamp||*stamp!=o.capture_stamp){stamp=std::make_unique<Stamp>(o.capture_stamp);cells.clear();track_cells.clear();for(const auto& t:tracks)add_cell(t.first,*t.second);}
    const auto& c=metric(o).center_m;const double x=std::floor(c.x/cell_size),y=std::floor(c.y/cell_size);
    std::vector<std::string> result;
    for(int dx:{-1,0,1})for(int dy:{-1,0,1}){const auto it=cells.find({x+dx,y+dy});if(it!=cells.end())result.insert(result.end(),it->second.begin(),it->second.end());}
    return result;
  }
  template<class Map,class K> static void erase_bucket(Map& m,const K& key,const std::string& id) {
    const auto it=m.find(key);if(it==m.end())return;it->second.erase(id);if(it->second.empty())m.erase(it);
  }
  void remove(const std::string& id) {
    const auto t=tracks.find(id);if(t!=tracks.end()){
      const auto& o=t->second->observation;
      if(o.source_track_id)erase_bucket(identities,Key{o.sensor_id,*o.source_track_id},id);
      for(const auto& p:o.provenance)erase_bucket(evidence,evidence_key(o,p),id);
    }
    const auto cell=track_cells.find(id);if(cell!=track_cells.end()){erase_bucket(cells,cell->second,id);track_cells.erase(cell);}
  }
  void add(const std::string& id,const FusionTrack& t){add_evidence(id,t.observation);add_cell(id,t);}
};
}
void FusionProfile::validate() const {
  for(const auto& p:std::vector<std::pair<const char*,double>>{{"sensor_timeout_s",sensor_timeout_s},{"track_memory_s",track_memory_s},
    {"association_distance_m",association_distance_m},{"velocity_confirmation_s",velocity_confirmation_s},
    {"velocity_fit_window_s",velocity_fit_window_s},{"velocity_fit_max_residual_m",velocity_fit_max_residual_m},
    {"min_tracked_speed_m_s",min_tracked_speed_m_s},{"max_obstacle_speed_m_s",max_obstacle_speed_m_s},
    {"stationary_velocity_variance_m2_s2",stationary_velocity_variance_m2_s2},{"prediction_horizon_s",prediction_horizon_s},
    {"prediction_step_s",prediction_step_s},{"half_length_m",half_length_m},{"half_width_m",half_width_m},{"clearance_margin_m",clearance_margin_m}}) {
      finite(p.second,p.first,0);require(p.second>0,p.first);
  }
  finite(payload_extra_margin_m,"payload_extra_margin_m",0);
  require(prediction_step_s<=prediction_horizon_s,"prediction_step");
  require(velocity_confirmation_s<=velocity_fit_window_s,"velocity_fit_window");
  nanoseconds(sensor_timeout_s);nanoseconds(2*track_memory_s);nanoseconds(prediction_horizon_s);
}
FusionTrack::FusionTrack(std::string id,Observation obs,Vec3 vel,std::vector<FusionSample> history)
  :identifier(std::move(id)),observation(std::move(obs)),velocity(vel),samples(std::move(history)) {}
Covariance3 expanded_covariance(const Covariance3& c,double extra) {
  auto values=c.values;for(int i:{0,4,8})values[i]+=extra;return Covariance3(values);
}
MetricBox translate(const MetricBox& box,const Vec3& velocity,double dt,double extra) {
  const auto& c=box.center_m;
  const Vec3 center=velocity==zero?c:Vec3(c.x+velocity.x*dt,c.y+velocity.y*dt,c.z+velocity.z*dt);
  return MetricBox(center,box.size_m,expanded_covariance(box.position_covariance_m2,extra),box.velocity_m_s,box.velocity_covariance_m2_s2);
}
Vec3 fitted_velocity(const std::vector<FusionSample>& samples,const FusionProfile& p) {
  if(samples.size()<5||samples.back()[0]-samples.front()[0]<p.velocity_confirmation_s-1e-8)return zero;
  const double count=samples.size();double mt=0,mx=0,my=0;
  for(const auto& s:samples){mt+=s[0];mx+=s[1];my+=s[2];}mt/=count;mx/=count;my/=count;
  double variance=0,vx=0,vy=0;
  for(const auto& s:samples){const double t=s[0]-mt;variance+=t*t;vx+=t*(s[1]-mx);vy+=t*(s[2]-my);}
  vx/=variance;vy/=variance;double residual=0;
  for(const auto& s:samples){const double dx=s[1]-mx-vx*(s[0]-mt),dy=s[2]-my-vy*(s[0]-mt);residual+=dx*dx+dy*dy;}
  residual=std::sqrt(residual/count);const double speed=euclidean_norm(vx,vy);
  if(residual>p.velocity_fit_max_residual_m||speed<p.min_tracked_speed_m_s)return zero;
  const double scale=std::min(1.,p.max_obstacle_speed_m_s/std::max(speed,1e-9));return Vec3(vx*scale,vy*scale,0.);
}
struct ConservativeFusion::State {
  TrackMap tracks;
  std::vector<std::string> track_order;
  std::map<Key,std::shared_ptr<const Observation>> unassociated;
  std::vector<Key> unassociated_order;
  std::map<Key,std::int64_t> seen;
  std::map<std::string,std::int64_t> last_sensor_stamp;
  std::uint64_t next_id=1;
  std::optional<std::pair<std::string,std::int64_t>> epoch;
  std::int64_t sequence=0;
  double half_length_m=0.,half_width_m=0.;
  std::vector<SensorHealth> sensors;
  std::unique_ptr<Version> version=std::make_unique<Version>("idle",0,0,0);
  std::vector<std::pair<std::int64_t,double>> prediction_steps;
};
ConservativeFusion::ConservativeFusion(FusionProfile p,std::string frame):profile(p),frame_id(std::move(frame)),state_(std::make_unique<State>()) {
  profile.validate();label(frame_id,"world.frame_id");
  state_->half_length_m=profile.half_length_m;state_->half_width_m=profile.half_width_m;
  const double count=round_even(profile.prediction_horizon_s/profile.prediction_step_s);
  require(std::isfinite(count)&&count<static_cast<double>(std::numeric_limits<std::size_t>::max()),"fusion.prediction_count");
  for(std::size_t i=1;i<=static_cast<std::size_t>(count);++i){const double t=i*profile.prediction_step_s;
    state_->prediction_steps.emplace_back(static_cast<std::int64_t>(round_even(t*1e9)),t);}
}
ConservativeFusion::~ConservativeFusion()=default;
void ConservativeFusion::set_envelope_bounds(double length,double width) {
  finite(length,"half_length_m",profile.half_length_m);finite(width,"half_width_m",profile.half_width_m);
  state_->half_length_m=length;state_->half_width_m=width;
}
void ConservativeFusion::set_version(const Version& version){state_->version=std::make_unique<Version>(version);}
void ConservativeFusion::set_sensors(std::vector<SensorHealth> sensors){state_->sensors=std::move(sensors);}
const Version& ConservativeFusion::version() const{return *state_->version;}
const std::vector<SensorHealth>& ConservativeFusion::sensors() const{return state_->sensors;}
const TrackMap& ConservativeFusion::tracks() const{return state_->tracks;}
std::int64_t ConservativeFusion::sequence() const{return state_->sequence;}
void ConservativeFusion::ingest(const std::vector<Observation>& observations,const Stamp& now) {
  auto& s=*state_;const auto timeout=nanoseconds(profile.sensor_timeout_s);
  for(const auto& obs:observations){obs.check_fresh(now,timeout,0);if(std::holds_alternative<MetricBox>(obs.geometry))require(obs.frame_id==frame_id,"fusion.requires_capture_time_transform");}
  const auto epoch=std::make_pair(now.clock,now.epoch);
  if(s.epoch&&*s.epoch!=epoch){s.tracks.clear();s.track_order.clear();s.unassociated.clear();s.unassociated_order.clear();s.seen.clear();s.last_sensor_stamp.clear();}
  s.epoch=epoch;std::set<std::string> assigned;AssociationIndex index(s.tracks,profile);
  std::vector<const Observation*> ordered;for(const auto& o:observations)ordered.push_back(&o);
  std::stable_sort(ordered.begin(),ordered.end(),[](const auto* a,const auto* b){return a->capture_stamp.ns<b->capture_stamp.ns;});
  for(const auto* input:ordered){auto obs=std::make_shared<const Observation>(*input);
    obs->check_fresh(now,timeout,0);const auto last=s.last_sensor_stamp.find(obs->sensor_id);
    if(last!=s.last_sensor_stamp.end()&&obs->capture_stamp.ns<last->second)continue;
    s.last_sensor_stamp[obs->sensor_id]=obs->capture_stamp.ns;
    const Key key{obs->sensor_id,obs->measurement_id};if(s.seen.count(key))continue;
    if(index.correlated(*obs)){s.seen[key]=now.ns;continue;}s.seen[key]=now.ns;
    if(!std::holds_alternative<MetricBox>(obs->geometry)||obs->geometry_quality<=0){
      if(!s.unassociated.count(key))s.unassociated_order.push_back(key);
      s.unassociated[key]=obs;continue;
    }
    require(obs->frame_id==frame_id,"fusion.requires_capture_time_transform");
    std::vector<std::tuple<bool,double,std::string>> candidates;
    for(bool exact:{true,false}){
      std::vector<std::string> identifiers;
      if(exact){if(obs->source_track_id){const auto ids=index.identities.find({obs->sensor_id,*obs->source_track_id});if(ids!=index.identities.end())identifiers.assign(ids->second.begin(),ids->second.end());}}
      else identifiers=index.nearby(*obs);
      for(const auto& id:identifiers){const auto& track=*s.tracks.at(id);const auto& source=track.observation;
        if(assigned.count(id)&&source.sensor_id==obs->sensor_id)continue;
        if(source.spatial_occupancy!=obs->spatial_occupancy)continue;
        const bool same=source.sensor_id==obs->sensor_id;
        if(!same&&source.capture_stamp!=obs->capture_stamp)continue;
        if(same&&source.source_track_id&&obs->source_track_id&&source.source_track_id!=obs->source_track_id)continue;
        const double dt=obs->capture_stamp.since(source.capture_stamp)*1e-9;if(dt<0)continue;
        const auto& a=metric(source).center_m;const auto& b=metric(*obs).center_m;const double t=std::min(dt,profile.track_memory_s);
        const double distance=euclidean_norm(a.x+track.velocity.x*t-b.x,a.y+track.velocity.y*t-b.y);
        if(distance<=profile.association_distance_m+std::min(dt,1.)*.2){const bool identity=same&&obs->source_track_id&&source.source_track_id==obs->source_track_id;
          candidates.emplace_back(!identity,distance,id);}
      }
      if(!candidates.empty())break;
    }
    const std::string id=candidates.empty()?"obstacle-"+std::to_string(s.next_id++):std::get<2>(*std::min_element(candidates.begin(),candidates.end()));
    auto velocity=std::make_unique<Vec3>(metric(*obs).velocity_m_s.value_or(zero));std::vector<FusionSample> samples;
    const auto old_it=s.tracks.find(id);
    if(old_it!=s.tracks.end()){
      const auto& old=*old_it->second;const double dt=obs->capture_stamp.since(old.observation.capture_stamp)*1e-9;
      const auto& a=metric(old.observation);const auto& b=metric(*obs);
      const bool consistent=std::abs(b.size_m.x-a.size_m.x)<=profile.velocity_fit_max_residual_m&&std::abs(b.size_m.y-a.size_m.y)<=profile.velocity_fit_max_residual_m;
      if(dt<=profile.sensor_timeout_s&&obs->velocity_observable&&old.observation.velocity_observable&&consistent)samples=old.samples;
      if(dt==0){
        const std::array<double,3> ac{a.center_m.x,a.center_m.y,a.center_m.z},as{a.size_m.x,a.size_m.y,a.size_m.z},bc{b.center_m.x,b.center_m.y,b.center_m.z},bs{b.size_m.x,b.size_m.y,b.size_m.z};
        std::array<double,3> center{},size{};for(int i=0;i<3;++i){const double lo=std::min(ac[i]-as[i]/2,bc[i]-bs[i]/2),hi=std::max(ac[i]+as[i]/2,bc[i]+bs[i]/2);center[i]=(lo+hi)/2;size[i]=hi-lo;}
        std::array<double,9> covariance{};for(int i:{0,4,8})covariance[i]=std::max(a.position_covariance_m2.values[i],b.position_covariance_m2.values[i]);
        MetricBox box(Vec3(center[0],center[1],center[2]),Vec3(size[0],size[1],size[2]),Covariance3(covariance),b.velocity_m_s?b.velocity_m_s:a.velocity_m_s,
          b.velocity_covariance_m2_s2?b.velocity_covariance_m2_s2:a.velocity_covariance_m2_s2);
        std::set<std::string> origins(obs->provenance.begin(),obs->provenance.end());origins.insert(old.observation.provenance.begin(),old.observation.provenance.end());
        obs=std::make_shared<const Observation>(with_geometry(*obs,box,obs->velocity_observable||old.observation.velocity_observable,{origins.begin(),origins.end()}));
        velocity=std::make_unique<Vec3>(box.velocity_m_s.value_or(old.velocity));
      } else if(!b.velocity_m_s)velocity=std::make_unique<Vec3>(zero);
    }
    const double at=obs->capture_stamp.ns*1e-9;const auto& center=metric(*obs).center_m;
    samples.erase(std::remove_if(samples.begin(),samples.end(),[&](const auto& sample){return !(at-profile.velocity_fit_window_s<=sample[0]&&sample[0]<at);}),samples.end());
    samples.push_back({at,center.x,center.y});
    if(!metric(*obs).velocity_m_s)velocity=std::make_unique<Vec3>(obs->velocity_observable?fitted_velocity(samples,profile):zero);
    index.remove(id);if(!s.tracks.count(id))s.track_order.push_back(id);
    s.tracks[id]=std::make_shared<const FusionTrack>(id,*obs,*velocity,std::move(samples));index.add(id,*s.tracks.at(id));assigned.insert(id);
  }
  const auto retention=nanoseconds(2*profile.track_memory_s);
  for(auto it=s.seen.begin();it!=s.seen.end();)if(now.ns-it->second>retention)it=s.seen.erase(it);else ++it;
  require(s.sequence<std::numeric_limits<std::int64_t>::max(),"fusion.sequence_range");++s.sequence;
}
WorldSnapshot ConservativeFusion::update(const std::vector<Observation>& observations,const Stamp& now){ingest(observations,now);return snapshot(now);}
void ConservativeFusion::resolve_unassociated(const std::string& sensor,const std::vector<std::string>& ids,const Stamp& capture,const Stamp& now) {
  (void)now;
  for(const auto& id:ids){const Key key{sensor,id};const auto it=state_->unassociated.find(key);if(it!=state_->unassociated.end()&&capture.since(it->second->capture_stamp)>0)state_->unassociated.erase(it);}
  auto& order=state_->unassociated_order;order.erase(std::remove_if(order.begin(),order.end(),[&](const auto& key){return !state_->unassociated.count(key);}),order.end());
}
void ConservativeFusion::clear_observed_free(const Stamp& now,FreeAt free_at,FreeMany free_many) {
  std::vector<std::string> pending;std::vector<MetricBox> boxes;
  for(const auto& id:state_->track_order){const auto& track=*state_->tracks.at(id);const auto& obs=track.observation;const double age=now.since(obs.capture_stamp)*1e-9;
    if(age>profile.sensor_timeout_s){pending.push_back(id);boxes.push_back(obs.spatial_occupancy?metric(obs):translate(metric(obs),track.velocity,std::min(age,profile.track_memory_s)));}}
  if(pending.empty())return;
  std::vector<bool> flags;if(free_many)flags=free_many(boxes);else {require(static_cast<bool>(free_at),"clearance.callback");for(const auto& box:boxes)flags.push_back(free_at(box));}
  require(flags.size()==pending.size(),"clearance.batch_size");
  for(std::size_t i=0;i<pending.size();++i)if(flags[i])state_->tracks.erase(pending[i]);
  auto& order=state_->track_order;order.erase(std::remove_if(order.begin(),order.end(),[&](const auto& id){return !state_->tracks.count(id);}),order.end());
}
WorldSnapshot ConservativeFusion::snapshot(const Stamp& now,std::optional<std::array<double,3>> region) {
  auto& s=*state_;
  if(s.epoch&&*s.epoch!=std::make_pair(now.clock,now.epoch))return update({},now);
  astribot_s1_robot_geometry::SnapshotParams p;
  p.norm2=static_cast<double(*)(double,double)>(&euclidean_norm);
  p.track_memory_s=profile.track_memory_s;p.velocity_confirmation_s=profile.velocity_confirmation_s;
  p.min_tracked_speed_m_s=profile.min_tracked_speed_m_s;p.velocity_fit_max_residual_m=profile.velocity_fit_max_residual_m;
  p.stationary_velocity_variance_m2_s2=profile.stationary_velocity_variance_m2_s2;p.prediction_horizon_s=profile.prediction_horizon_s;
  p.half_length_m=s.half_length_m;p.half_width_m=s.half_width_m;p.clearance_margin_m=profile.clearance_margin_m;p.payload_extra_margin_m=profile.payload_extra_margin_m;
  if(region){p.has_region=!(std::isinf((*region)[2])&&(*region)[2]>0.);p.region_x=(*region)[0];p.region_y=(*region)[1];p.region_travel=(*region)[2];}
  std::vector<astribot_s1_robot_geometry::SnapshotTrackInput> inputs;
  for(const auto& id:s.track_order){const auto& t=*s.tracks.at(id);const auto& o=t.observation;const auto& b=metric(o);
    (void)now.since(o.capture_stamp);astribot_s1_robot_geometry::SnapshotTrackInput in;
    in.capture_ns=o.capture_stamp.ns;in.center[0]=b.center_m.x;in.center[1]=b.center_m.y;in.center[2]=b.center_m.z;
    in.size[0]=b.size_m.x;in.size[1]=b.size_m.y;in.size[2]=b.size_m.z;
    std::copy(b.position_covariance_m2.values.begin(),b.position_covariance_m2.values.end(),in.pos_cov);
    in.track_velocity[0]=t.velocity.x;in.track_velocity[1]=t.velocity.y;in.track_velocity[2]=t.velocity.z;
    in.spatial_occupancy=o.spatial_occupancy;in.velocity_observable=o.velocity_observable;in.has_velocity_m_s=b.velocity_m_s.has_value();in.has_velocity_covariance=b.velocity_covariance_m2_s2.has_value();
    if(b.velocity_covariance_m2_s2)std::copy(b.velocity_covariance_m2_s2->values.begin(),b.velocity_covariance_m2_s2->values.end(),in.velocity_covariance);
    for(const auto& sample:t.samples)in.samples.push_back({sample[0],sample[1],sample[2]});
    inputs.push_back(std::move(in));
  }
  std::vector<astribot_s1_robot_geometry::SnapshotTrackOutput> outputs;
  snapshot_default(inputs,now.ns,p,outputs);
  std::vector<TrackedObstacle> tracks;
  for(std::size_t i=0;i<outputs.size();++i){const auto& out=outputs[i];const auto& t=*s.tracks.at(s.track_order[i]);const auto& b=metric(t.observation);
    std::array<double,9> covariance{};std::copy(out.covariance,out.covariance+9,covariance.begin());
    MetricBox box(Vec3(out.center[0],out.center[1],out.center[2]),Vec3(out.size[0],out.size[1],out.size[2]),Covariance3(covariance),b.velocity_m_s,b.velocity_covariance_m2_s2);
    std::optional<PredictionModel> model;if(out.relevant)model.emplace(Vec3(out.velocity[0],out.velocity[1],out.velocity[2]),out.variance,s.prediction_steps);
    tracks.emplace_back(t.identifier,frame_id,now,std::move(box),std::vector<Prediction>{},t.observation.provenance,std::move(model));
  }
  std::vector<Observation> unknown;for(const auto& key:s.unassociated_order)unknown.push_back(*s.unassociated.at(key));
  return WorldSnapshot(*s.version,now,frame_id,std::move(tracks),std::move(unknown),s.sensors,s.sequence);
}
}  // namespace astribot::navigation::policy
