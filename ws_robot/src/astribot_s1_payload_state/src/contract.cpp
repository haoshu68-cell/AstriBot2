#include "astribot_s1_payload_state/ledger.hpp"
#include <openssl/sha.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
namespace astribot::payload {
namespace {
void require(bool ok,const char *reason) {if(!ok)throw std::invalid_argument(reason);}
bool name(const std::string &s) {return !s.empty() && s.size()<=256 && s.find('\0')==std::string::npos;}
nlohmann::json pose(const geometry_msgs::msg::Pose &p) {
  double q[]={p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w};
  const double norm=std::hypot(std::hypot(q[0],q[1]),std::hypot(q[2],q[3]));
  require(std::isfinite(norm) && std::abs(norm-1.)<=1e-6,"INVALID_QUATERNION");
  for(double v:{p.position.x,p.position.y,p.position.z})require(std::isfinite(v),"INVALID_POSITION");
  // Quaternion sign describes the same rotation; deterministic canonical form.
  double sign=1.;for(int i=3;i>=0;--i)if(q[i]!=0.) {sign=q[i]<0.?-1.:1.;break;}
  return {{"position",{p.position.x,p.position.y,p.position.z}},
          {"quaternion",{sign*q[0]/norm,sign*q[1]/norm,sign*q[2]/norm,sign*q[3]/norm}}};
}
}
int64_t ns(const builtin_interfaces::msg::Time &v) {
  require(v.sec>=0 && v.nanosec<1000000000,"INVALID_TIME");return int64_t(v.sec)*1000000000+v.nanosec;
}
builtin_interfaces::msg::Time stamp(int64_t v) {
  require(v>=0 && v<=int64_t(INT32_MAX)*1000000000+999999999,"INVALID_TIME");
  builtin_interfaces::msg::Time s;s.sec=static_cast<int32_t>(v/1000000000);s.nanosec=static_cast<uint32_t>(v%1000000000);return s;
}
std::string digest(const std::string &text) {
  unsigned char result[SHA256_DIGEST_LENGTH];SHA256(reinterpret_cast<const unsigned char *>(text.data()),text.size(),result);
  std::ostringstream out;for(auto c:result)out<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(c);return out.str();
}
nlohmann::json canonical(const Objects &objects,const std::set<std::string> &allowed_links) {
  require(objects.size()<=32,"TOO_MANY_PAYLOADS");
  std::map<std::string,nlohmann::json> sorted;
  for(const auto &a:objects) {
    const auto &o=a.object;
    require(name(o.id) && !sorted.count(o.id),"DUPLICATE_OR_INVALID_OBJECT_ID");
    require(name(a.link_name) && allowed_links.count(a.link_name),"UNKNOWN_ATTACHMENT_LINK");
    require(o.header.frame_id.empty() || o.header.frame_id==a.link_name,"ATTACHMENT_FRAME_MISMATCH");
    require(o.operation==o.ADD,"FULL_STATE_REQUIRES_ADD");
    require(o.meshes.empty() && o.mesh_poses.empty() && o.planes.empty() && o.plane_poses.empty(),"UNSUPPORTED_PAYLOAD_GEOMETRY");
    require(!o.primitives.empty() && o.primitives.size()<=64 && o.primitives.size()==o.primitive_poses.size(),"INCOMPLETE_PAYLOAD_GEOMETRY");
    require(std::isfinite(a.weight) && a.weight>=0.,"INVALID_PAYLOAD_WEIGHT");
    require(a.touch_links.size()<=64,"TOO_MANY_TOUCH_LINKS");auto touch=a.touch_links;std::sort(touch.begin(),touch.end());
    require(std::adjacent_find(touch.begin(),touch.end())==touch.end(),"DUPLICATE_TOUCH_LINK");
    for(const auto &link:touch)require(name(link),"INVALID_TOUCH_LINK");
    nlohmann::json shapes=nlohmann::json::array();
    for(std::size_t i=0;i<o.primitives.size();++i) {
      const auto &p=o.primitives[i];std::size_t count=0;
      if(p.type==p.BOX)count=3;else if(p.type==p.SPHERE)count=1;else if(p.type==p.CYLINDER)count=2;
      require(count && p.dimensions.size()==count && p.polygon.points.empty(),"UNSUPPORTED_PAYLOAD_PRIMITIVE");
      for(double d:p.dimensions)require(std::isfinite(d) && d>0.,"INVALID_PAYLOAD_DIMENSIONS");
      shapes.push_back({{"type",p.type},{"dimensions",p.dimensions},{"pose",pose(o.primitive_poses[i])}});
    }
    // Ignore observation stamps and non-collision metadata, not geometry or collision permissions.
    sorted[o.id]={{"id",o.id},{"link",a.link_name},{"pose",pose(o.pose)},
                  {"shapes",shapes},{"touch_links",touch},{"weight",a.weight}};
  }
  nlohmann::json result=nlohmann::json::array();for(const auto &[id,value]:sorted) { (void)id;result.push_back(value); }return result;
}
bool scene_matches(const Objects &physical,const Objects &readback,const std::set<std::string> &allowed_links) {
  auto expected=canonical(physical,allowed_links),actual=canonical(readback,allowed_links);
  if(expected.size()!=actual.size())return false;
  const auto same_pose=[](const nlohmann::json &a,const nlohmann::json &b) {
    for(std::size_t n=0;n<3;++n)
      if(std::abs(a.at("position").at(n).get<double>()-b.at("position").at(n).get<double>())>1e-12)return false;
    double direct=0.,opposite=0.;
    for(std::size_t n=0;n<4;++n) {
      const double x=a.at("quaternion").at(n),y=b.at("quaternion").at(n);
      direct=std::max(direct,std::abs(x-y));opposite=std::max(opposite,std::abs(x+y));
    }
    return std::min(direct,opposite)<=1e-12;
  };
  for(std::size_t n=0;n<expected.size();++n) {
    auto &a=expected[n];auto &b=actual[n];
    // canonical already rejects negative/nonfinite mass. Nonzero conflicting
    // readback remains a mismatch; only Humble's absent value is accommodated.
    if(b.at("weight")==0.)b["weight"]=a.at("weight");
    if(!same_pose(a.at("pose"),b.at("pose")))return false;
    b["pose"]=a.at("pose");
    if(a.at("shapes").size()!=b.at("shapes").size())return false;
    for(std::size_t k=0;k<a.at("shapes").size();++k) {
      if(!same_pose(a["shapes"][k].at("pose"),b["shapes"][k].at("pose")))return false;
      b["shapes"][k]["pose"]=a["shapes"][k].at("pose");
    }
  }
  return expected==actual;
}
} // namespace astribot::payload
