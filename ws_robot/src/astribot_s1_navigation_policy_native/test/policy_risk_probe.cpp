#include "astribot_s1_navigation_policy_native/policy_risk.hpp"
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <chrono>
#include <ctime>
#include <sys/resource.h>
using nlohmann::json;
namespace p=astribot::navigation::policy;
p::Vec3 vec(const json& v){return {v.at(0),v.at(1),v.at(2)};}
p::MetricBox box(const json& v){
  auto d=v.value("variance",std::vector<double>{0.,0.,0.});
  return {vec(v.at("center")),vec(v.at("size")),p::Covariance3({d.at(0),0.,0.,0.,d.at(1),0.,0.,0.,d.at(2)})};
}
p::WorldSnapshot world(const json& v){
  p::Stamp stamp(1000000000,"ros",0); std::vector<p::TrackedObstacle> tracks;
  for(const auto& t:v.at("tracks")){
    std::vector<p::Prediction> predictions;
    for(const auto& sample:t.value("predictions",json::array()))predictions.emplace_back(sample.at("ns"),box(sample.at("box")));
    std::optional<p::PredictionModel> model;
    if(t.contains("model")){const auto& m=t.at("model");model.emplace(vec(m.at("velocity")),m.at("variance"),m.at("steps").get<std::vector<std::pair<std::int64_t,double>>>());}
    tracks.emplace_back(t.at("id"),"odom",stamp,box(t.at("box")),std::move(predictions),std::vector<std::string>{"source"},std::move(model));
  }
  std::vector<p::Observation> unassociated;
  if(v.value("uncertain",false))unassociated.emplace_back("camera","measurement",std::nullopt,
    stamp,p::Stamp(0,"steady",0),p::Stamp(2000000000,"ros",0),"optical",0,
    p::ImageBox("camera",100,100,0.,0.,1.,1.),1.,std::vector<std::pair<std::string,double>>{},std::vector<std::string>{"camera"});
  return {p::Version("goal",1,1,1),stamp,"odom",std::move(tracks),std::move(unassociated),{},1};
}
json scalar(double x){if(std::isnan(x))return "NaN";if(std::isinf(x))return x>0?"Infinity":"-Infinity";return x;}
json result_json(const p::Risk& risk){return {{"blocked",risk.blocked},{"immediate",risk.immediate},{"clearance_m",scalar(risk.clearance_m)},
    {"conflict_time_s",scalar(risk.conflict_time_s)},{"obstacle_ids",risk.obstacle_ids},{"moving",risk.moving},
    {"uncertain",risk.uncertain},{"immediate_obstacle_ids",risk.immediate_obstacle_ids}};}
double cpu_ns(){timespec ts{};clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&ts);return ts.tv_sec*1e9+ts.tv_nsec;}
long rss_kib(){std::ifstream status("/proc/self/status");std::string key;
  while(status>>key){if(key=="VmRSS:"){long value;status>>value;return value;}std::string rest;std::getline(status,rest);}return -1;}
int main(){std::string line;while(std::getline(std::cin,line))try{
  const auto data=json::parse(line);const auto w=world(data);
  if(data.at("op")=="meta"){
    json out=json::array();for(const auto& t:w.tracks){const auto final=p::final_prediction(t);
      out.push_back({{"has_predictions",p::has_predictions(t)},{"prediction_count",p::prediction_count(t)},
        {"center",{final.center_m.x,final.center_m.y,final.center_m.z}},
        {"covariance",final.position_covariance_m2.values}});}
    std::cout<<json({{"meta",out}}).dump()<<'\n';continue;
  }
  if(data.at("op")=="rows"){
    const auto rows=p::prediction_rows(w,data.value("include_current",false),data.value("swept",false));
    json out=json::array();for(const auto& r:rows)out.push_back({{"owner",r.owner},{"offset_ns",r.offset_ns},{"lower",r.bounds.lower},{"upper",r.bounds.upper}});
    std::cout<<json({{"rows",out}}).dump()<<'\n';continue;
  }
  const auto& j=data.at("profile");
  p::SweepProfile sweep{j.at("half_length_m"),j.at("half_width_m"),j.at("clearance_margin_m"),j.at("payload_extra_margin_m"),std::nullopt};
  if(j.contains("footprint_xy"))sweep.footprint_xy=j.at("footprint_xy").get<p::Polygon>();
  p::RiskProfile profile{sweep,j.at("max_speed_m_s"),j.at("reaction_time_s"),j.at("brake_deceleration_m_s2"),
    j.at("angular_brake_deceleration_rad_s2"),j.value("linear_stop_delay_s",0.),j.at("prediction_horizon_s")};
  const auto& r=data.at("robot");p::RobotState robot(r.at(0),r.at(1),r.at(2),r.at(3),r.at(4),r.at(5));
  std::optional<double> cap;if(data.contains("speed_limit"))cap=data.at("speed_limit");
  const auto path=data.at("path").get<p::Polygon>();
  const auto risk=p::evaluate_risk(w,robot,path,profile,cap);
  json result={{"risk",result_json(risk)}};
  if(data.contains("benchmark")){
    const int warmup=data.at("benchmark").value("warmup",20),samples=data.at("benchmark").value("samples",150);
    json wall=json::array(),cpu=json::array();
    for(int i=-warmup;i<samples;++i){
      const auto start=std::chrono::steady_clock::now();const double before=cpu_ns();
      const auto repeated=p::evaluate_risk(w,robot,path,profile,cap);
      const double used=cpu_ns()-before;const auto end=std::chrono::steady_clock::now();
      if(result_json(repeated)!=result.at("risk"))throw std::runtime_error("unstable risk result");
      if(i>=0){wall.push_back(std::chrono::duration<double,std::nano>(end-start).count());cpu.push_back(used);}
    }
    rusage usage{};getrusage(RUSAGE_SELF,&usage);
    result["benchmark"]={{"wall_ns",wall},{"cpu_ns",cpu},{"peak_rss_kib",usage.ru_maxrss},{"ending_rss_kib",rss_kib()}};
  }
  std::cout<<result.dump()<<'\n';
}catch(const std::exception& e){std::cout<<json({{"error",e.what()}}).dump()<<'\n';}}
