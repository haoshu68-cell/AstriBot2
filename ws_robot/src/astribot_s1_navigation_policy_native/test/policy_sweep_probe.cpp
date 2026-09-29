#include "astribot_s1_navigation_policy_native/policy_sweep.hpp"
#include <iostream>
#include <nlohmann/json.hpp>
#include <limits>
using nlohmann::json;
namespace p = astribot::navigation::policy;
double number(const json &v) {
  if (v.is_string()) {
    if (v == "NaN") return NAN;
    if (v == "Infinity") return INFINITY;
    if (v == "-Infinity") return -INFINITY;
  }
  return v.get<double>();
}
std::vector<double> numbers(const json &v) {
  std::vector<double> out; for (const auto &x:v) out.push_back(number(x)); return out;
}
json output(const std::vector<double> &v) {
  json out=json::array();for(double x:v) {
    if(std::isnan(x))out.push_back("NaN");else if(std::isinf(x))out.push_back(x>0?"Infinity":"-Infinity");else out.push_back(x);
  }return out;
}
int main() {
  std::string line;while(std::getline(std::cin,line)) {
    try {
      const auto v=json::parse(line);const auto &j=v.at("profile");
      p::SweepProfile profile{j.at("half_length_m"),j.at("half_width_m"),
        j.at("clearance_margin_m"),j.at("payload_extra_margin_m"),std::nullopt};
      if(j.contains("footprint_xy")&&!j.at("footprint_xy").is_null())profile.footprint_xy=j.at("footprint_xy").get<p::Polygon>();
      std::vector<p::Bounds> boxes;
      for(std::size_t i=0;i<v.at("lower").size();++i)boxes.push_back({
        {number(v["lower"][i][0]),number(v["lower"][i][1])},
        {number(v["upper"][i][0]),number(v["upper"][i][1])}});
      std::vector<double> result;
      if(v.at("op")=="clearance") {
        result=p::clearance_many(numbers(v.at("x")),numbers(v.at("y")),numbers(v.at("yaw")),boxes,profile,v.value("sampling_margin",0.));
      } else {
        const auto a=numbers(v.at("command")), b=numbers(v.at("origin"));
        result=p::motion_clearance({a.at(0),a.at(1),a.at(2)},numbers(v.at("begin")),numbers(v.at("end")),boxes,profile,{b.at(0),b.at(1),b.at(2)});
      }
      std::cout<<json({{"values",output(result)}}).dump()<<'\n';
    }catch(const std::exception &e){std::cout<<json({{"error",e.what()}}).dump()<<'\n';}
  }
}
