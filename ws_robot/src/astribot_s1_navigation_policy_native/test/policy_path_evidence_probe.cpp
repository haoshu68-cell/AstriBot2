#include "astribot_s1_navigation_policy_native/policy_path_evidence.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
using nlohmann::json;
namespace p=astribot::navigation::policy;
double number(const json& v){if(v.is_string()){const auto s=v.get<std::string>();if(s=="NaN")return NAN;if(s=="Infinity")return INFINITY;if(s=="-Infinity")return -INFINITY;}return v.get<double>();}
json numeric(double v){return std::isfinite(v)?json(v):json(std::isnan(v)?"NaN":v>0?"Infinity":"-Infinity");}
void header(std_msgs::msg::Header& h,const json& j){h.frame_id=j.at(0);h.stamp.sec=j.at(1);h.stamp.nanosec=j.at(2);}
nav_msgs::msg::Path path(const json& j){nav_msgs::msg::Path m;header(m.header,j.at("header"));for(const auto& v:j.at("poses")){geometry_msgs::msg::PoseStamped s;header(s.header,v.at("header"));auto a=v.at("geometry");s.pose.position.x=number(a[0]);s.pose.position.y=number(a[1]);s.pose.position.z=number(a[2]);s.pose.orientation.x=number(a[3]);s.pose.orientation.y=number(a[4]);s.pose.orientation.z=number(a[5]);s.pose.orientation.w=number(a[6]);m.poses.push_back(s);}return m;}
int main(){std::string line;while(std::getline(std::cin,line))try{const auto j=json::parse(line);json out;
  if(j.at("op")=="identity")out={{"same",p::path_identity(path(j.at("left")))==p::path_identity(path(j.at("right")))}};
  else {std::optional<p::PathIdentity> key;std::optional<p::PathEvidence> evidence;
    if(!j.at("path").is_null())key=p::path_identity(path(j.at("path")));
    if(!j.at("evidence").is_null()){const auto& e=j.at("evidence");evidence=p::PathEvidence{p::path_identity(path(e.at("path"))),number(e.at("stamp_s")),number(e.at("received_wall_s")),e.at("epoch"),e.at("known"),e.at("blocked"),number(e.at("distance_m"))};}
    const auto& c=j.at("profile");const auto result=p::assess_path(evidence,key,number(j.at("now_s")),number(j.at("wall_s")),j.at("epoch"),j.at("legacy_blocked"),{number(c.at("path_risk_timeout_s")),number(c.at("max_speed_m_s")),number(c.at("clearance_margin_m")),number(c.at("payload_extra_margin_m"))});
    out={{"blocked",result.blocked},{"conflict_time_s",numeric(result.conflict_time_s)},{"status",result.status},{"distance_m",numeric(result.distance_m)}};
  }std::cout<<out.dump()<<'\n';
}catch(const std::exception& e){std::cout<<json({{"error",e.what()}}).dump()<<'\n';}}
