#include <nlohmann/json.hpp>
#include <array>
#include <vector>
#include <iostream>
int main(){for(const auto& raw:{"{\"coverage_body_yaw_half_angle\":[[0]]}","{\"coverage_body_yaw_half_angle\":null}","{\"coverage_body_yaw_half_angle\":{\"front\":[0,0.5]}}"}) {
  const auto spec=nlohmann::json::parse(raw);std::vector<std::array<double,2>> coverage;
  try {for(const auto& c:spec.value("coverage_body_yaw_half_angle",nlohmann::json::array()))coverage.push_back({c.at(0),c.at(1)});
    std::cout<<raw<<" startup success, coverage="<<coverage.size()<<"\n";
  }catch(const std::exception& e){std::cout<<raw<<" startup exception "<<e.what()<<"\n";}
}}
