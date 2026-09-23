#include "astribot_s1_transport_mtc/canonical_octomap.hpp"
#include <octomap/OcTree.h>
#include <functional>
#include <cstring>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

using astribot::transport::canonical_octomap;
void check(bool okay,const char* reason) { if(!okay) throw std::runtime_error(reason); }
std::string serialized(const octomap::OcTree& tree,bool binary=false) {
  std::ostringstream s(std::ios::binary);
  if(binary) tree.writeBinaryData(s); else tree.writeData(s);
  return s.str();
}
void rejects(const std::function<void()>& call) {
  bool rejected=false;try{call();}catch(const std::invalid_argument&){rejected=true;}
  check(rejected,"invalid input was accepted");
}
int main(int argc,char**) {
 try {
  octomap::OcTree a(.05),b(.05),shifted(.05);
  a.setNodeValue(octomap::point3d(.025f,.025f,.025f),3.511f);
  b.setNodeValue(octomap::point3d(.025f,.025f,.025f),3.105f);
  auto raw_a=serialized(a),raw_b=serialized(b);
  auto signature=[&](const std::string& data,bool binary=false) {
    // Reproduce the previous raw-data hash equality rule for a red run.
    return argc>1 ? data:canonical_octomap(data,binary,.05,"OcTree");
  };
  check(raw_a!=raw_b,"probability fixture must differ");
  check(signature(raw_a)==signature(raw_b),"log-odds-only change invalidated geometry");
  std::cout<<"PASS probability-only change\n";
  b.setNodeValue(octomap::point3d(.025f,.025f,.025f),-1.f);
  check(signature(raw_a)!=signature(serialized(b)),"occupancy crossing disappeared");
  std::cout<<"PASS occupancy threshold crossing\n";
  shifted.setNodeValue(octomap::point3d(.125f,.025f,.025f),3.511f);
  check(signature(raw_a)!=signature(serialized(shifted)),"spatial key change disappeared");
  std::cout<<"PASS spatial key change\n";
  octomap::OcTree expanded(.05);
  for(float x:{.025f,.075f})for(float y:{.025f,.075f})for(float z:{.025f,.075f})
    expanded.setNodeValue(octomap::point3d(x,y,z),3.f,true);
  expanded.updateInnerOccupancy();expanded.prune();auto compact=serialized(expanded);
  expanded.expand();
  check(signature(compact)!=signature(serialized(expanded)),"leaf depth change disappeared");
  std::cout<<"PASS leaf depth and allocated topology change\n";
  octomap::OcTree free_only(.05),free_extra(.05);
  free_only.setNodeValue(octomap::point3d(.025f,.025f,.025f),-1.f);
  free_extra.setNodeValue(octomap::point3d(.025f,.025f,.025f),-1.f);
  free_extra.setNodeValue(octomap::point3d(.125f,.025f,.025f),-1.f);
  check(signature(serialized(free_only))!=signature(serialized(free_extra)),"free/unknown allocation disappeared");
  std::cout<<"PASS free versus unknown space\n";
  check(!signature(serialized(a,true),true).empty(),"binary input unavailable");
  check(signature(serialized(a,true),true)!=signature(serialized(b,true),true),"binary occupancy crossing disappeared");
  std::cout<<"PASS actual library binary input\n";
  rejects([&]{canonical_octomap(raw_a.substr(0,raw_a.size()-1),false,.05,"OcTree");});
  rejects([&]{canonical_octomap(raw_a+"x",false,.05,"OcTree");});
  rejects([&]{canonical_octomap("x",true,.05,"OcTree");});
  rejects([&]{canonical_octomap(serialized(a,true)+"x",true,.05,"OcTree");});
  rejects([&]{canonical_octomap(raw_a,false,.05,"ColorOcTree");});
  rejects([&]{canonical_octomap(raw_a,false,0,"OcTree");});
  rejects([&]{canonical_octomap("",false,.05,"OcTree");});
  std::string corrupt=raw_a;float nan=std::numeric_limits<float>::quiet_NaN();
  std::memcpy(corrupt.data(),&nan,sizeof(nan));
  rejects([&]{canonical_octomap(corrupt,false,.05,"OcTree");});
  std::cout<<"PASS truncated/trailing/type/resolution/empty/nonfinite refusal\n";
  std::cout<<"PASS all canonical octomap boundaries\n";
  return 0;
 }catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
