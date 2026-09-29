#include "astribot_s1_navigation_policy_native/navigation_math.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
#include <memory>
#include <string>
using nlohmann::json;
namespace n=astribot::navigation;
double number(const json& v){if(v.is_string()){const auto s=v.get<std::string>();if(s=="NaN")return NAN;if(s=="Infinity")return INFINITY;if(s=="-Infinity")return -INFINITY;}return v.get<double>();}
json numeric(double v){return std::isfinite(v)?json(v):json(std::isnan(v)?"NaN":v>0?"Infinity":"-Infinity");}
int main(){std::string line;while(std::getline(std::cin,line))try{
  auto input=json::parse(line);auto p=input.at("profile");
  n::YieldPolicy policy(p.at("max_speed_m_s"),p.at("narrow_speed_m_s"),p.at("reaction_time_s"),p.at("brake_deceleration_m_s2"),p.at("linear_stop_delay_s"),p.at("wait_budget_s"),p.at("clear_hold_s"));
  json result=json::array();for(const auto& op:input.at("operations")){
    if(op.contains("profile"))for(auto it=op.at("profile").begin();it!=op.at("profile").end();++it)p[it.key()]=it.value();
    policy.update_profile(p.at("max_speed_m_s"),p.at("narrow_speed_m_s"),p.at("reaction_time_s"),p.at("brake_deceleration_m_s2"),p.at("linear_stop_delay_s"),p.at("wait_budget_s"),p.at("clear_hold_s"));
    json value;try{
      const auto& r=op.at("risk");const bool present=!r.is_null();
      const bool immediate=present&&r.at("immediate").get<bool>(),blocked=present&&r.at("blocked").get<bool>(),uncertain=present&&r.at("uncertain").get<bool>();
      const auto conflict=present?number(r.at("conflict_time_s")):0.;
      const auto selection=policy.select(immediate,blocked,uncertain,conflict,present&&op.at("valid").get<bool>(),number(op.at("now")));
      value={{"motion",selection.motion},{"speed",selection.speed},{"reason",selection.reason},{"planning",0},{"episode",selection.episode}};
    }catch(const std::exception& e){value={{"error",e.what()}};}
    auto state=policy.state();value["state"]={{"blocked_at",state[0]?numeric(state[1]):json(nullptr)},{"clear_at",state[2]?numeric(state[3]):json(nullptr)},{"last_time",state[4]?numeric(state[5]):json(nullptr)},{"held",state[6]!=0.},{"episode",static_cast<int>(state[7])},{"in_episode",state[8]!=0.}};result.push_back(value);
  }std::cout<<result.dump()<<'\n';
}catch(const std::exception& e){std::cout<<json({{"error",e.what()}}).dump()<<'\n';}}
