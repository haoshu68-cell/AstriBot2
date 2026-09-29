#pragma once
#include <array>
#include <cmath>
#include <vector>
#include <algorithm>

namespace astribot_s1_robot_geometry {
using PassagePose = std::array<double,3>;
using PassagePoint = std::array<double,2>;

inline PassagePoint passageCoordinates(const PassagePoint &p,const PassagePoint &entry,
                                      const PassagePoint &exit) {
  const double h=std::atan2(exit[1]-entry[1],exit[0]-entry[0]);
  const double x=p[0]-entry[0],y=p[1]-entry[1];
  return {std::cos(h)*x+std::sin(h)*y,-std::sin(h)*x+std::cos(h)*y};
}
inline bool passageContains(const PassagePoint &p,const PassagePoint &entry,
                            const PassagePoint &exit,double width) {
  const auto q=passageCoordinates(p,entry,exit);
  return q[0]>=0. && q[0]<=std::hypot(exit[0]-entry[0],exit[1]-entry[1]) && std::abs(q[1])<=width/2.;
}
inline bool passageBodyOutside(const std::vector<PassagePoint> &polygon,
    PassagePose pose,const PassagePoint &entry,const PassagePoint &exit,double margin) {
  double lo=INFINITY,hi=-INFINITY;
  const double c=std::cos(pose[2]),s=std::sin(pose[2]);
  for(const auto &p:polygon) {
    const auto q=passageCoordinates({pose[0]+c*p[0]-s*p[1],pose[1]+s*p[0]+c*p[1]},entry,exit);
    lo=std::min(lo,q[0]);hi=std::max(hi,q[0]);
  }
  return hi+margin<0. || lo-margin>std::hypot(exit[0]-entry[0],exit[1]-entry[1]);
}
// A planned fixed-heading prefix is identified by its pose orientations, not
// by a new motion owner. The duplicate exit pose carries the later rotation.
inline int narrowTranslationEnd(const std::vector<PassagePoint> &polygon,
    const std::vector<PassagePose> &path,const PassagePoint &entry,
    const PassagePoint &exit,double width,double margin) {
  if(path.size()<3 || !passageContains({path.front()[0],path.front()[1]},entry,exit,width))return -1;
  const auto &a=path.front();
  auto first=std::find_if(path.begin()+1,path.end(),[&](const auto &p){return std::hypot(p[0]-a[0],p[1]-a[1])>1e-6;});
  if(first==path.end() || std::abs(std::remainder(std::atan2((*first)[1]-a[1],(*first)[0]-a[0])-a[2],2*M_PI))<=.05)return -1;
  for(std::size_t i=1;i<path.size();++i) {
    if(std::abs(std::remainder(path[i][2]-a[2],2*M_PI))>.05) {
      return passageBodyOutside(polygon,path[i-1],entry,exit,margin)?int(i-1):-1;
    }
  }
  return passageBodyOutside(polygon,path.back(),entry,exit,margin)?int(path.size()-1):-1;
}

// Follow the admitted polyline without rotating toward its tangent.
inline PassagePoint translationLookahead(const std::vector<PassagePoint> &path,
    const PassagePoint &current,double lookahead) {
  double best=INFINITY;std::size_t segment=1;PassagePoint target=path.front();
  for(std::size_t i=1;i<path.size();++i) {
    const auto &a=path[i-1],&b=path[i];const double dx=b[0]-a[0],dy=b[1]-a[1],l2=dx*dx+dy*dy;
    if(l2==0.)continue;
    const double f=std::clamp(((current[0]-a[0])*dx+(current[1]-a[1])*dy)/l2,0.,1.);
    PassagePoint p{a[0]+f*dx,a[1]+f*dy};const double d=std::hypot(p[0]-current[0],p[1]-current[1]);
    if(d<best){best=d;segment=i;target=p;}
  }
  for(std::size_t i=segment;i<path.size();++i) {
    const double dx=path[i][0]-target[0],dy=path[i][1]-target[1],d=std::hypot(dx,dy);
    if(d>lookahead)return {target[0]+lookahead*dx/d,target[1]+lookahead*dy/d};
    target=path[i];lookahead-=d;
  }
  return target;
}
}
