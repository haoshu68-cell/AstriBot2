#include "astribot_s1_robot_geometry/narrow_translation.hpp"
#include <cassert>
#include <iostream>
using namespace astribot_s1_robot_geometry;
int main() {
  const std::vector<PassagePoint> body{{-.32,-.32},{.32,-.32},{.32,.32},{-.32,.32}};
  const PassagePoint entry{-.6,0.},exit{.8,0.};
  assert(passageContains({-.6,0.},entry,exit,1.25));
  assert(!passageContains({-.600001,0.},entry,exit,1.25));
  assert(passageContains({0.,.625},entry,exit,1.25));
  assert(!passageContains({0.,.625001},entry,exit,1.25));
  assert(!passageBodyOutside(body,{-1.019999,0.,0.},entry,exit,.1));
  assert(!passageBodyOutside(body,{-1.02,0.,0.},entry,exit,.1));
  assert(passageBodyOutside(body,{-1.020001,0.,0.},entry,exit,.1));
  std::vector<PassagePose> backward{{0.,0.,0.},{-.2,0.,0.},{-1.1,0.,0.},{-1.1,0.,M_PI},{-1.8,0.,M_PI}};
  assert(narrowTranslationEnd(body,backward,entry,exit,1.25,.1)==2);
  auto premature=backward;premature[2][0]=premature[3][0]=-.9;
  assert(narrowTranslationEnd(body,premature,entry,exit,1.25,.1)==-1);
  auto sideways=backward;for(std::size_t i=0;i<3;++i)sideways[i][2]=M_PI/2;
  assert(narrowTranslationEnd(body,sideways,entry,exit,1.25,.1)==2);
  auto outside=backward;outside[0][0]=-1.1;
  assert(narrowTranslationEnd(body,outside,entry,exit,1.25,.1)==-1);
  auto forward=backward;for(auto &p:forward)p[2]=M_PI;
  assert(narrowTranslationEnd(body,forward,entry,exit,1.25,.1)==-1);
  auto p=translationLookahead({{0.,0.},{-.5,0.},{-1.,0.}},{-.2,.02},.15);
  assert(std::abs(p[0]+.35)<1e-12 && p[1]==0.);
  p=translationLookahead({{0.,0.},{0.,.5},{0.,1.}},{.02,.2},.15);
  assert(p[0]==0. && std::abs(p[1]-.35)<1e-12);
  p=translationLookahead({{0.,0.},{.1,0.},{.1,.5}},{0.,0.},.15);
  assert(std::abs(p[0]-.1)<1e-12 && std::abs(p[1]-.05)<1e-12);
  std::cout<<"narrow bounds, full-body exit, backward/sideways prefix and polyline lookahead passed\n";
}
