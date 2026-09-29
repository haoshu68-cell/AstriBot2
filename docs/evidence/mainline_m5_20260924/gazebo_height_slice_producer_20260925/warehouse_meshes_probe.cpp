#include "astribot_s1_gazebo_bringup/height_slice_shapes.hpp"
#include <sdf/Mesh.hh>
#include <tinyxml2.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
using namespace astribot::simulation;
int main(int argc,char**argv) {
  assert(argc==2);const std::string repo=argv[1];
  const auto root=repo+"/ws_robot/src/aws-robomaker-small-warehouse-world";
  setenv("IGN_GAZEBO_RESOURCE_PATH",(root+"/models:"+root+"/worlds").c_str(),1);
  setenv("GZ_SIM_RESOURCE_PATH",(root+"/models:"+root+"/worlds").c_str(),1);
  const auto config=YAML::LoadFile(repo+"/ws_robot/src/astribot_s1_mapping/config/height_slices.yaml")["/**"]["ros__parameters"];
  const auto edges=config["height_edges"].as<std::vector<double>>();
  const auto resolution=config["resolution"].as<double>();
  const char*names[]{"Bucket","ClutteringA","ClutteringC","ClutteringD","GroundB","Lamp","PalletJackB","RoofB","ShelfD","ShelfE","ShelfF","TrashCanC","WallB"};
  auto start=std::chrono::steady_clock::now();std::size_t total=0;
  for(const auto*name:names){
    auto ms=std::chrono::steady_clock::now();
    const std::string model="aws_robomaker_warehouse_"+std::string(name)+"_01";
    const auto file=root+"/models/"+model+"/model.sdf";
    tinyxml2::XMLDocument doc;if(doc.LoadFile(file.c_str())!=tinyxml2::XML_SUCCESS)throw std::runtime_error("model load failed: "+file);
    std::vector<std::vector<SliceTriangle>> meshes;
    double lo[3]{INFINITY,INFINITY,INFINITY},hi[3]{-INFINITY,-INFINITY,-INFINITY};
    std::size_t count=0;
    for(auto*link=doc.FirstChildElement("sdf")->FirstChildElement("model")->FirstChildElement("link");link;link=link->NextSiblingElement("link")){
      ignition::math::Pose3d link_pose;if(auto*p=link->FirstChildElement("pose"))std::istringstream(p->GetText())>>link_pose;
      for(auto*c=link->FirstChildElement("collision");c;c=c->NextSiblingElement("collision")){
        ignition::math::Pose3d collision_pose;if(auto*p=c->FirstChildElement("pose"))std::istringstream(p->GetText())>>collision_pose;
        auto*m=c->FirstChildElement("geometry")->FirstChildElement("mesh");if(!m)throw std::runtime_error("non-mesh actual geometry: "+model);
        sdf::Mesh sm;sm.SetFilePath(file);sm.SetUri(m->FirstChildElement("uri")->GetText());
        if(auto*s=m->FirstChildElement("scale")){ignition::math::Vector3d scale;std::istringstream(s->GetText())>>scale;sm.SetScale(scale);}
        sdf::Geometry g;g.SetType(sdf::GeometryType::MESH);g.SetMeshShape(sm);
        auto ts=collision_triangles(g,link_pose*collision_pose);count+=ts.size();
        for(const auto&t:ts)for(const auto&p:t){const double v[]{p.x,p.y,p.z};for(int i=0;i<3;++i){lo[i]=std::min(lo[i],v[i]);hi[i]=std::max(hi[i],v[i]);}}
        meshes.push_back(std::move(ts));
      }
    }
    assert(count>0);total+=count;
    const double ox=std::floor(lo[0]/resolution)*resolution-resolution,oy=std::floor(lo[1]/resolution)*resolution-resolution;
    const std::size_t w=std::ceil((hi[0]-ox)/resolution)+2,h=std::ceil((hi[1]-oy)/resolution)+2;
    assert(w*h<20000000);assert(hi[0]-lo[0]<100&&hi[1]-lo[1]<100&&hi[2]-lo[2]<100);
    std::cout<<name<<" triangles="<<count<<" bounds_m="<<lo[0]<<","<<lo[1]<<","<<lo[2]<<":"<<hi[0]<<","<<hi[1]<<","<<hi[2]<<" occupied=";
    for(std::size_t i=0;i+1<edges.size();++i){
      SliceRaster r{w,h,resolution,ox,oy,std::vector<int8_t>(w*h,0)};
      for(const auto&t:meshes)rasterize_mesh(t,edges[i],edges[i+1],r);
      const auto occupied=std::count(r.data.begin(),r.data.end(),100);std::cout<<occupied<<",";
      if(std::string(name)=="WallB"){
        assert(occupied>0&&occupied<static_cast<long>(w*h/2));
        const auto cx=static_cast<std::size_t>((0-ox)/resolution),cy=static_cast<std::size_t>((0-oy)/resolution);assert(r.data[cy*w+cx]==0);
      }
    }
    std::cout<<" elapsed_ms="<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-ms).count()<<"\n";
  }
  std::cout<<"PASS models=13 total_triangles="<<total<<" elapsed_ms="<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()<<"; local model coordinates only, profile ground=0 as offline fixture, not world coverage\n";
}
