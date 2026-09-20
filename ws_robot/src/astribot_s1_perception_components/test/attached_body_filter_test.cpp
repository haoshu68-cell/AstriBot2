#include <iostream>
#include "astribot_s1_autonomy/attached_body_filter.hpp"
using namespace astribot_s1_autonomy;
void check(bool b,const char * what){if(!b)throw std::runtime_error(what);}
int main(){
  AttachedBody b;b.primitive.type=b.primitive.BOX;b.primitive.dimensions={.06,.06,.12};
  AttachedBody::validate(b.primitive);
  Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();pose.translation()<<.2,.4,1.2;
  pose.linear()=Eigen::AngleAxisd(.6,Eigen::Vector3d::UnitY()).toRotationMatrix();b.from_base=pose.inverse();
  check(b.contains(.2,.4,1.2),"attached center must be filtered");
  const Eigen::Vector3d inside=pose*Eigen::Vector3d(.02,.02,.05);
  check(b.contains(inside.x(),inside.y(),inside.z()),"rotated primitive support");
  check(!b.contains(.2,0.,1.2),"space between arms must remain visible");
  check(!b.contains(.2,.4,.5),"obstacle below payload must remain visible");
  const Eigen::Vector3d outside=pose*Eigen::Vector3d(.04,0.,0.);
  check(!b.contains(outside.x(),outside.y(),outside.z()),"no whole hull clearing");
  b.primitive.type=b.primitive.CYLINDER;b.primitive.dimensions={.2,.04};b.from_base=Eigen::Isometry3d::Identity();
  AttachedBody::validate(b.primitive);check(b.contains(.03,0.,.09),"cylinder interior");
  check(!b.contains(.04,.04,0.),"cylinder bounding-box corner is not self");
  b.primitive.type=b.primitive.SPHERE;b.primitive.dimensions={.04};
  AttachedBody::validate(b.primitive);check(!b.contains(.04,.04,0.),"sphere bounding-box corner is not self");
  bool rejected=false;try{b.primitive.dimensions={-1.};AttachedBody::validate(b.primitive);}catch(const std::invalid_argument &){rejected=true;}
  check(rejected,"invalid dimensions must reject");
  std::cout<<"PASS attached box/cylinder/sphere, transformed geometry, under-payload and inter-arm obstacles, invalid dimension\n";
}
