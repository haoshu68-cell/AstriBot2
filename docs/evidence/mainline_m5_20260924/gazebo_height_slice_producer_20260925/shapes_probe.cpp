#include "astribot_s1_gazebo_bringup/height_slice_shapes.hpp"
#include <sdf/Box.hh>
#include <sdf/Mesh.hh>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace astribot::simulation;
std::array<double,6> bounds(const std::vector<SliceTriangle>& triangles) {
  std::array<double,6> b{INFINITY,INFINITY,INFINITY,-INFINITY,-INFINITY,-INFINITY};
  for(const auto&t:triangles)for(const auto&p:t){
    const double v[]{p.x,p.y,p.z};
    for(int i=0;i<3;++i){b[i]=std::min(b[i],v[i]);b[i+3]=std::max(b[i+3],v[i]);}
  }
  return b;
}
void check(const std::array<double,6>& got,const std::array<double,6>& expected) {
  for(int i=0;i<6;++i) if(std::abs(got[i]-expected[i])>5e-6)
    throw std::runtime_error("bounds mismatch at "+std::to_string(i)+": "+std::to_string(got[i])+" != "+std::to_string(expected[i]));
}
int main(int argc,char**argv) {
  assert(argc==2);
  const std::string repo=argv[1], root=repo+"/ws_robot/src/aws-robomaker-small-warehouse-world";
  const auto resources=root+"/models:"+root+"/worlds";
  setenv("GZ_SIM_RESOURCE_PATH",resources.c_str(),1);
  setenv("IGN_GAZEBO_RESOURCE_PATH",resources.c_str(),1);
  sdf::Geometry box;box.SetType(sdf::GeometryType::BOX);sdf::Box shape;shape.SetSize({2,4,6});box.SetBoxShape(shape);
  auto bt=collision_triangles(box,{10,20,30,0,0,M_PI/2});assert(bt.size()==12);
  check(bounds(bt),{8,19,27,12,21,33});
  // Each oriented BOX triangle contributes to a closed volume of 2*4*6.
  double volume=0;
  for(const auto&t:bt){const auto&a=t[0];const auto&b=t[1];const auto&c=t[2];volume+=(a.x*(b.y*c.z-b.z*c.y)+a.y*(b.z*c.x-b.x*c.z)+a.z*(b.x*c.y-b.y*c.x))/6;}
  assert(std::abs(volume-48)<1e-9);
  std::cout<<"BOX: 12 triangles, rotated bounds and signed volume PASS\n";
  const char* names[]{"WallB","GroundB","ShelfD"};
  const unsigned counts[]{64,12,48};
  // Independently computed from each actual DAE POSITION array, asset meter=.01,
  // Z_UP and its visual_scene node matrix (ShelfD matrix is non-identity).
  const std::array<double,6> expected[]{
    {-6.99023315,-10.45333130,0,6.99023682,10.45333252,9.01986084},
    {-6.99023499,-10.45333252,-.00000124,6.99023499,10.45333252,.12431792},
    {-1.95912277,-.43999569,.02929092,1.95919159,.43972584,2.64266922}};
  for(int n=0;n<3;++n){
    const std::string model="aws_robomaker_warehouse_"+std::string(names[n])+"_01";
    const std::string relative="models/"+model+"/meshes/"+model+"_collision.DAE";
    sdf::Mesh mesh;mesh.SetUri("file://"+relative);mesh.SetFilePath(root+"/models/"+model+"/model.sdf");
    sdf::Geometry g;g.SetType(sdf::GeometryType::MESH);g.SetMeshShape(mesh);
    auto ts=collision_triangles(g,{});assert(ts.size()==counts[n]);check(bounds(ts),expected[n]);
    // Actual GroundB uses an override model.sdf in a different package.
    mesh.SetFilePath(repo+"/ws_robot/src/astribot_s1_gazebo_bringup/models/aws_robomaker_warehouse_GroundB_01/model.sdf");
    g.SetMeshShape(mesh);check(bounds(collision_triangles(g,{})),expected[n]);
    mesh.SetFilePath("");mesh.SetUri("model://"+model+"/meshes/"+model+"_collision.DAE");g.SetMeshShape(mesh);
    check(bounds(collision_triangles(g,{})),expected[n]);
    mesh.SetUri(root+"/"+relative);mesh.SetScale({2,3,4});g.SetMeshShape(mesh);
    const auto moved=collision_triangles(g,{1,2,3,0,0,M_PI/2});
    const auto& e=expected[n];
    check(bounds(moved),{1-3*e[4],2+2*e[0],3+4*e[2],1-3*e[1],2+2*e[3],3+4*e[5]});
    std::cout<<names[n]<<": "<<ts.size()<<" real triangles, DAE unit/node matrix, URI variants, scale+pose PASS\n";
  }
  int rejected=0;
  try {sdf::Geometry g;collision_triangles(g,{});}catch(const std::runtime_error&){++rejected;}
  for(bool submesh:{false,true}){sdf::Mesh m;m.SetUri("file://models/does_not_exist.DAE");if(submesh)m.SetSubmesh("part");sdf::Geometry g;g.SetType(sdf::GeometryType::MESH);g.SetMeshShape(m);try{collision_triangles(g,{});}catch(const std::runtime_error&){++rejected;}}
  assert(rejected==3);std::cout<<"Missing mesh / submesh / unsupported geometry: explicit failure PASS\n";
}
