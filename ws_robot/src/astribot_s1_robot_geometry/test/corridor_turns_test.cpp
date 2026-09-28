#include <iostream>
#include "astribot_s1_robot_geometry/corridor_turns.hpp"

int main() {
  using Points = std::vector<std::array<double, 2>>;
  using astribot_s1_robot_geometry::corridorTurnsOutside;
  const Points rear{{-1.,-.32},{.32,-.32},{.32,.32},{-1.,.32}};
  const Points front{{-.32,-.32},{1.,-.32},{1.,.32},{-.32,.32}};
  int failures = 0, checks = 0;
  auto check = [&](bool value, const char * name) {
    ++checks;
    if (!value) {std::cerr << name << '\n';++failures;}
  };
  auto fits = [&](const Points & body, const Points & route, double length = 3.) {
    return corridorTurnsOutside(body,route,length,.85,.1,.1);
  };
  check(fits(rear,{{-2.,0.},{5.,0.}}),"aligned travel needs no turn stop");
  check(!fits(rear,{{-2.,0.},{3.1,0.},{3.1,1.}}),"long rear still in passage");
  check(!fits(rear,{{-2.,0.},{3.5,0.},{3.5,1.}}),"clear final heading cannot erase incoming tail");
  check(!fits(rear,{{-2.,0.},{4.12,0.},{4.12,-1.}}),"intermediate sweep exceeds both endpoint postures");
  check(fits(rear,{{-2.,0.},{4.2,0.},{4.2,-1.}}),"whole sweep clear at exit");
  check(!fits(front,{{-.5,-1.},{-.5,0.},{5.,0.}}),"front arm enters before alignment ends");
  check(fits(front,{{-1.2,-1.},{-1.2,0.},{5.,0.}}),"alignment outside front sweep");
  check(!fits(rear,{{-2.,0.},{3.5,0.},{3.5,0.},{3.5,1.}}),"duplicates preserve turn context");
  check(fits(rear,{{-2.,0.},{1.,.001},{5.,0.}}),"small travel correction unchanged");
  check(fits(rear,{{3.5,3.},{3.5,2.},{-2.,2.},{-2.,0.},{5.,0.}}),"remote corner is outside selected corridor");
  check(!fits(rear,{{-2.,0.},{.3,0.},{.3,1.}},.2),"short door still reserves long tail");
  const Points left{{-.32,-.2},{.32,-.2},{.32,.7},{-.32,.7}};
  check(!fits(left,{{-2.,0.},{3.7,0.},{3.7,1.}}),"left arm swings toward exit with positive turn");
  check(fits(left,{{-2.,0.},{3.7,0.},{3.7,-1.}}),"asymmetric opposite turn retains clearance");
  check(fits(rear,{{1.,0.}}),"stationary path keeps existing goal-heading responsibility");
  check(fits(rear,{}),"empty path keeps existing route-fit responsibility");
  check(fits(front,{{4.5,-1.3},{3.95,-1.3},{3.95,-1.2}}),"diagonal swept box overlap is not contact");
  check(fits(front,{{2.5,-.9},{3.5,-.9},{3.5,-1.9}}),"partial turn stays clear even inside full circle bound");
  const double touch = 3.1 + std::hypot(1.,.32);
  check(!fits(rear,{{-2.,0.},{touch,0.},{touch,-1.}}),"intermediate tangent contact rejected");
  check(fits(rear,{{-2.,0.},{touch+1e-5,0.},{touch+1e-5,-1.}}),"positive sweep clearance admitted");
  std::cout << checks - failures << '/' << checks << " checks passed\n";
  return failures ? 1 : 0;
}
