#pragma once
#include <nlohmann/json.hpp>
#include <array>
#include <vector>
#include <string>
#include <cmath>
namespace astribot_navigation_zones {
using Json=nlohmann::json;
using Point=std::array<double,2>;
struct Region {
 std::string id,name,type;bool enabled{true};std::vector<Point> points;
 double width{0},margin{0};
};
std::vector<Region> parseRegions(const Json & value);
Json encodeRegions(const std::vector<Region> & regions);
bool blocked(const std::vector<Region> &,double x,double y,double radius=0.);
// Conservatively bounds each interval by a circle enlarged by centre travel.
bool sweptCircleBlocked(const std::vector<Region> &,double x,double y,double yaw,
 double vx,double vy,double wz,double radius,double horizon);
std::array<double,4> bounds(const Region & region,double extra=0.);
}
