#include "astribot_s1_robot_geometry/geometry_kernels.hpp"
#include <iostream>
#include <nlohmann/json.hpp>
using nlohmann::json;
using namespace astribot_s1_robot_geometry;
int main() {
  std::string line;
  while (std::getline(std::cin,line)) {
    try {
      const auto a=json::parse(line); const auto op=a.at("op").get<std::string>(); json out;
      if(op=="hull") out=convexHull(a.at("points").get<Polygon2>());
      else if(op=="inflate") out=inflatePolygon(a.at("points").get<Polygon2>(),a.at("radius"));
      else if(op=="distance") out=boxDistances(a.at("points").get<Polygon2>(),a.at("rows").get<std::vector<std::array<double,7>>>());
      else if(op=="hash") out=geometryHash(a.at("points").get<Polygon2>(),a.at("frame"),a.at("clearance"));
      else if(op=="validate") out=validatePolygon(a.at("points").get<Polygon2>());
      else if(op=="serialize") out=serializedPolygon(a.at("points").get<Polygon2>());
      else if(op=="scan_free") out=scanBoxesFree(a.at("boxes").get<std::vector<std::array<double,4>>>(),a.at("tf").get<std::array<double,7>>(),a.at("ranges").get<std::vector<double>>(),a.at("lo"),a.at("hi"),a.at("start"),a.at("step"),a.at("resolution"));
      else if(op=="scan_cells") out=scanOccupiedCells(a.at("ranges").get<std::vector<double>>(),a.at("lo"),a.at("hi"),a.at("start"),a.at("step"),a.at("tf").get<std::array<double,7>>(),a.at("map_tf").get<std::array<double,7>>(),a.at("info").get<std::array<double,5>>(),a.at("mask").get<std::vector<std::vector<bool>>>(),a.at("resolution"));
      else throw std::invalid_argument("unknown operation");
      std::cout << json{{"value",out}}.dump() << '\n';
    } catch(const std::exception &e) {std::cout << json{{"error",e.what()}}.dump() << '\n';}
  }
}
