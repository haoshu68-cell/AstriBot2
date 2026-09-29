#include "astribot_s1_navigation_policy_native/policy_observer_core.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
using nlohmann::json;
using namespace astribot::navigation;
using namespace astribot::navigation::policy;
double number(const json& v) {if(v.is_number())return v.get<double>();const auto s=v.get<std::string>();return s=="NaN"?NAN:s=="Infinity"?INFINITY:-INFINITY;}
template<std::size_t N> std::array<double,N> array(const json& v){std::array<double,N> r{};for(std::size_t i=0;i<N;++i)r[i]=number(v.at(i));return r;}
json run(const json& packet) {
  astribot::navigation::ExecutionContext context; ObserverMap map; json output=json::array();
  for(const auto& op:packet.at("operations")) {
    json row;
    try {
      const auto action=op.at("action").get<std::string>();
      if(action=="task") {
        const auto& sequence=op.at("sequence");
        if(sequence.is_number_unsigned())context.task_wire(op.at("id"),op.at("state"),sequence.get<std::uint64_t>());
        else context.task(op.at("id"),op.at("state"),sequence.get<std::int64_t>());
      }
      else if(action=="path")context.path();
      else if(action=="localization") {auto v=array<3>(op.at("pose"));row["changed"]=context.localization({v.begin(),v.end()},number(op.at("position_limit")),number(op.at("angle_limit")));}
      else if(action=="version") {const auto& v=op.at("version");context.set_version(v[0],v[1],v[2],v[3],v[4],v[5]);}
      else if(action=="map") {
        const bool accepted=map.accept(op.at("frame"),op.at("width"),op.at("height"),number(op.at("resolution")),array<7>(op.at("origin")),op.at("cells").get<std::vector<std::int8_t>>());
        row["accepted"]=accepted;if(accepted)row["changed"]=context.map(map.context_key());
      } else if(action=="lookup") {
        row["values"]=json::array();for(const auto& point:op.at("points"))row["values"].push_back(map.static_at(number(point[0]),number(point[1])));
        if(map.has_map())row["mask"]=map.mask();
      } else if(action=="point")row["point"]=observer_point(array<3>(op.at("point")),array<7>(op.at("transform")));
      else throw std::invalid_argument("unknown action");
    } catch(const std::exception& error) {row["error"]=error.what();}
    const auto v=context.version();row["version"]={std::get<0>(v),std::get<1>(v),std::get<2>(v),std::get<3>(v),std::get<4>(v),std::get<5>(v)};output.push_back(row);
  } return output;
}
int main(){std::string line;while(std::getline(std::cin,line)){try{std::cout<<run(json::parse(line)).dump()<<'\n';}catch(const std::exception& e){std::cout<<json{{"error",e.what()}}.dump()<<'\n';}}}
