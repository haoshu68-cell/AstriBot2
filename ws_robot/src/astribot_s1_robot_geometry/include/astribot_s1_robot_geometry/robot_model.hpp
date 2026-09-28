#pragma once
#include "astribot_s1_robot_geometry/geometry_kernels.hpp"
#include <Eigen/Geometry>
#include <tinyxml2.h>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <sstream>

namespace astribot_s1_robot_geometry {
using Transform=Eigen::Matrix4d;
using JointValues=std::map<std::string,double>;
inline std::vector<double> numbers(const std::string &text) {
  std::istringstream stream(text);std::vector<double> result;double value;
  while(stream>>value)result.push_back(value);
  geometryRequire(stream.eof(),"invalid numeric model attribute");finiteValues(result);return result;
}
inline std::string attribute(const tinyxml2::XMLElement *element,const char *name,const char *fallback=nullptr) {
  const char *value=element?element->Attribute(name):nullptr;
  if(!value)value=fallback;
  geometryRequire(value!=nullptr,"missing model attribute");return value;
}
inline Eigen::Vector3d vector3(const std::string &value) {
  const auto n=numbers(value);geometryRequire(n.size()==3,"three model coordinates required");return {n[0],n[1],n[2]};
}
inline Transform modelOrigin(const tinyxml2::XMLElement *element) {
  Transform out=Transform::Identity();if(!element)return out;
  out.block<3,1>(0,3)=vector3(attribute(element,"xyz","0 0 0"));
  const auto rpy=vector3(attribute(element,"rpy","0 0 0"));
  out.block<3,3>(0,0)=(Eigen::AngleAxisd(rpy.z(),Eigen::Vector3d::UnitZ())*
    Eigen::AngleAxisd(rpy.y(),Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(rpy.x(),Eigen::Vector3d::UnitX())).toRotationMatrix();
  return out;
}
struct MeshBounds {Eigen::Vector3d low,high;std::string hash;bool float32=false;};
inline MeshBounds meshBounds(const std::string &filename) {
  std::ifstream stream(filename,std::ios::binary);geometryRequire(bool(stream),"mesh file unavailable");
  const std::string data((std::istreambuf_iterator<char>(stream)),{});std::vector<Eigen::Vector3d> vertices;
  std::string suffix=filename.substr(filename.find_last_of('.')+1);
  std::transform(suffix.begin(),suffix.end(),suffix.begin(),[](unsigned char c){return std::tolower(c);});
  uint32_t count=0;if(data.size()>=84)for(int i=0;i<4;++i)count|=uint32_t(static_cast<unsigned char>(data[80+i]))<<(8*i);
  if(suffix=="obj") {
    std::istringstream lines(data);std::string line;
    while(std::getline(lines,line)) {
      line=line.substr(0,line.find('#'));std::istringstream words(line);std::string kind;words>>kind;
      if(kind!="v")continue;
      std::string rest;std::getline(words,rest);const auto n=numbers(rest);
      geometryRequire(n.size()>=3,"invalid mesh vertex");
      geometryRequire(n.size()!=4 || n[3]==1.,"unsupported homogeneous mesh vertex");
      vertices.emplace_back(n[0],n[1],n[2]);
    }
  } else if(count && data.size()==84ULL+50ULL*count) {
    for(uint32_t i=0;i<count;++i)for(int j=0;j<3;++j) {
      Eigen::Vector3d vertex;
      for(int k=0;k<3;++k) {
        const auto offset=84ULL+50ULL*i+12+12*j+4*k;uint32_t bits=0;
        for(int b=0;b<4;++b)bits|=uint32_t(static_cast<unsigned char>(data[offset+b]))<<(8*b);
        float value;std::memcpy(&value,&bits,sizeof(value));vertex[k]=value;
      }
      vertices.push_back(vertex);
    }
  } else {
    std::istringstream lines(data);std::string line;
    while(std::getline(lines,line)) {
      std::istringstream words(line);std::string kind;words>>kind;if(kind!="vertex")continue;
      std::string rest;std::getline(words,rest);vertices.push_back(vector3(rest));
    }
  }
  geometryRequire(!vertices.empty(),"invalid mesh");MeshBounds result{vertices.front(),vertices.front(),sha256(data),suffix!="obj" && count && data.size()==84ULL+50ULL*count};
  for(const auto &v:vertices) {geometryRequire(v.allFinite(),"invalid mesh");result.low=result.low.cwiseMin(v);result.high=result.high.cwiseMax(v);}return result;
}
struct Shape {
  std::string link,kind;std::vector<double> dimensions;Transform pose;
  Shape(std::string link_,std::string kind_,std::vector<double> dimensions_,Transform pose_)
    :link(std::move(link_)),kind(std::move(kind_)),dimensions(std::move(dimensions_)),pose(std::move(pose_)) {
    const std::map<std::string,std::size_t> sizes{{"box",3},{"sphere",1},{"cylinder",2}};
    geometryRequire(sizes.count(kind) && dimensions.size()==sizes.at(kind),"unsupported/invalid shape");
    finiteValues(dimensions);for(double value:dimensions)geometryRequire(value>0,"unsupported/invalid shape");
    geometryRequire(pose.allFinite(),"invalid shape pose");
  }
  double radius() const {
    if(kind=="sphere")return dimensions[0];
    if(kind=="cylinder")return std::hypot(dimensions[0],dimensions[1]/2);
    return std::sqrt(dimensions[0]*dimensions[0]+dimensions[1]*dimensions[1]+dimensions[2]*dimensions[2])/2;
  }
  double support(const Eigen::Vector3d &direction,const Transform &transform) const {
    const Transform total=transform*pose;const Eigen::Vector3d local=total.block<3,3>(0,0).transpose()*direction;
    double extent=0;
    if(kind=="box")for(int i=0;i<3;++i)extent+=std::abs(local[i])*dimensions[i]/2;
    else if(kind=="sphere")extent=dimensions[0]*local.norm();
    else extent=dimensions[0]*std::hypot(local.x(),local.y())+dimensions[1]/2*std::abs(local.z());
    return direction.dot(total.block<3,1>(0,3))+extent;
  }
  Polygon2 polygon(const Transform &transform,std::size_t sides=32) const {
    Polygon2 points;const Transform total=transform*pose;
    if(kind=="box") {
      for(double x:{-1.,1.})for(double y:{-1.,1.})for(double z:{-1.,1.}) {
        const Eigen::Vector4d v=total*Eigen::Vector4d(x*dimensions[0]/2,y*dimensions[1]/2,z*dimensions[2]/2,1.);
        points.push_back({v.x(),v.y()});
      }
    } else {
      std::vector<Eigen::Vector3d> directions;std::vector<double> supports;
      for(std::size_t i=0;i<sides;++i) {
        const double angle=i*2*pi/sides;directions.emplace_back(std::cos(angle),std::sin(angle),0.);
        supports.push_back(support(directions.back(),transform));
      }
      for(std::size_t i=0;i<sides;++i) {
        const auto j=(i+1)%sides;Eigen::Matrix2d a;a<<directions[i].x(),directions[i].y(),directions[j].x(),directions[j].y();
        const Eigen::Vector2d p=a.inverse()*Eigen::Vector2d(supports[i],supports[j]);points.push_back({p.x(),p.y()});
      }
    }
    return convexHull(points);
  }
};
struct GeometrySlice {double z_min,z_max;Polygon2 footprint;};
struct RobotGeometry {Polygon2 physical,reserved;double height,z_min;std::vector<GeometrySlice> slices;};
class RobotModel {
  struct Joint {
    std::string name,kind,parent,child,mimic;
    Transform origin;Eigen::Vector3d axis;
    double multiplier=1.,offset=0.,lower=-INFINITY,upper=INFINITY;
  };
  std::string base_,revision_;std::map<std::string,Joint> joints_;std::map<std::string,std::string> parents_;
  std::vector<Shape> shapes_;std::set<std::string> invariant_;std::vector<std::string> required_;
  std::string source(const std::string &name,std::set<std::string> seen={}) const {
    geometryRequire(seen.insert(name).second,"mimic cycle");const auto &j=joints_.at(name);return j.mimic.empty()?name:source(j.mimic,seen);
  }
public:
  using PackageResolver=std::function<std::string(const std::string &)>;
  RobotModel(const std::string &urdf,std::string base="astribot_torso_base",PackageResolver resolver={}) :base_(std::move(base)) {
    tinyxml2::XMLDocument document;geometryRequire(document.Parse(urdf.c_str(),urdf.size())==tinyxml2::XML_SUCCESS,"invalid URDF XML");
    const auto *root=document.FirstChildElement("robot");geometryRequire(root!=nullptr,"missing robot model");bool base_found=false;
    for(auto link=root->FirstChildElement("link");link;link=link->NextSiblingElement("link"))if(attribute(link,"name")==base_)base_found=true;
    geometryRequire(base_found,"missing base frame");
    for(auto element=root->FirstChildElement("joint");element;element=element->NextSiblingElement("joint")) {
      Joint j;j.name=attribute(element,"name");j.kind=attribute(element,"type");
      geometryRequire(j.kind=="fixed" || j.kind=="revolute" || j.kind=="continuous" || j.kind=="prismatic","unsupported joint");
      j.parent=attribute(element->FirstChildElement("parent"),"link");j.child=attribute(element->FirstChildElement("child"),"link");
      j.origin=modelOrigin(element->FirstChildElement("origin"));j.axis=vector3(attribute(element->FirstChildElement("axis"),"xyz","1 0 0"));
      geometryRequire(j.axis.norm()>=1e-9,"invalid joint axis");j.axis.normalize();
      if(const auto mimic=element->FirstChildElement("mimic")) {
        j.mimic=attribute(mimic,"joint");j.multiplier=std::stod(attribute(mimic,"multiplier","1"));j.offset=std::stod(attribute(mimic,"offset","0"));
        geometryRequire(std::isfinite(j.multiplier) && std::isfinite(j.offset),"invalid mimic values");
      }
      if(const auto limit=element->FirstChildElement("limit");limit && j.kind!="continuous") {
        j.lower=std::stod(attribute(limit,"lower","-inf"));j.upper=std::stod(attribute(limit,"upper","inf"));
        geometryRequire(!std::isnan(j.lower) && !std::isnan(j.upper) && j.lower<=j.upper,"invalid joint limits");
      }
      geometryRequire(!parents_.count(j.child) && !joints_.count(j.name),"duplicate joint parent/name");parents_[j.child]=j.name;joints_[j.name]=j;
    }
    std::vector<std::string> hashes;
    for(auto link=root->FirstChildElement("link");link;link=link->NextSiblingElement("link")) {
      const auto name=attribute(link,"name");
      for(auto collision=link->FirstChildElement("collision");collision;collision=collision->NextSiblingElement("collision")) {
        Transform pose=modelOrigin(collision->FirstChildElement("origin"));const auto geometry=collision->FirstChildElement("geometry");
        geometryRequire(geometry && geometry->FirstChildElement(),"missing collision geometry");const auto g=geometry->FirstChildElement();
        std::string kind=g->Name();std::vector<double> dimensions;
        if(kind=="box")dimensions=numbers(attribute(g,"size"));
        else if(kind=="sphere")dimensions=numbers(attribute(g,"radius"));
        else if(kind=="cylinder")dimensions={std::stod(attribute(g,"radius")),std::stod(attribute(g,"length"))};
        else if(kind=="mesh") {
          auto filename=attribute(g,"filename");
          if(filename.rfind("package://",0)==0) {
            geometryRequire(bool(resolver),"package mesh resolver unavailable");const auto slash=filename.find('/',10);
            geometryRequire(slash!=std::string::npos,"invalid package mesh URI");filename=resolver(filename.substr(10,slash-10))+filename.substr(slash);
          }
          const auto mesh=meshBounds(filename);hashes.push_back(mesh.hash);const auto scale=vector3(attribute(g,"scale","1 1 1"));
          Eigen::Vector3d center=(mesh.low+mesh.high)/2,extent=mesh.high-mesh.low;
          // The reference reads binary STL as NumPy float32; preserve its
          // center/subtraction rounding before multiplying by double scale.
          if(mesh.float32)for(int i=0;i<3;++i) {
            center[i]=static_cast<float>(static_cast<float>(mesh.low[i])+static_cast<float>(mesh.high[i]))/2.f;
            extent[i]=static_cast<float>(static_cast<float>(mesh.high[i])-static_cast<float>(mesh.low[i]));
          }
          pose.block<3,1>(0,3)+=pose.block<3,3>(0,0)*center.cwiseProduct(scale);
          const Eigen::Vector3d size=extent.cwiseProduct(scale.cwiseAbs());dimensions={size.x(),size.y(),size.z()};kind="box";
        } else throw std::invalid_argument("unsupported collision "+kind);
        chain(name);shapes_.emplace_back(name,kind,dimensions,pose);
      }
    }
    geometryRequire(!shapes_.empty(),"model has no collision geometry");
    for(const auto &[name,j]:joints_) {
      bool leaf=true;for(const auto &[other,candidate]:joints_) {static_cast<void>(other);if(candidate.parent==j.child)leaf=false;}
      bool body=false,invariant=true;
      for(const auto &shape:shapes_)if(shape.link==j.child) {
        body=true;const bool centered=shape.pose.block<3,1>(0,3).cross(j.axis).norm()<1e-10;
        const bool aligned=shape.pose.block<3,1>(0,2).cross(j.axis).norm()<1e-10;
        invariant=invariant && centered && (shape.kind=="sphere" || (shape.kind=="cylinder" && aligned));
      }
      if(leaf && body && invariant && (j.kind=="continuous" || j.kind=="revolute"))invariant_.insert(name);
    }
    std::set<std::string> required;
    for(const auto &shape:shapes_)for(const auto &name:chain(shape.link))if(joints_.at(name).kind!="fixed" && !invariant_.count(name))required.insert(source(name));
    required_.assign(required.begin(),required.end());std::string content=urdf+"\n";
    for(std::size_t i=0;i<hashes.size();++i) {if(i)content+='\n';content+=hashes[i];}revision_=sha256(content);
  }
  const std::vector<std::string> &required() const{return required_;}
  const std::string &revision() const{return revision_;}
  std::vector<std::string> chain(std::string link) const {
    std::vector<std::string> out;std::set<std::string> seen;
    while(link!=base_) {
      geometryRequire(seen.insert(link).second && parents_.count(link),"collision link not rooted at base");
      const auto name=parents_.at(link);out.push_back(name);link=joints_.at(name).parent;
    }
    std::reverse(out.begin(),out.end());return out;
  }
  double jointValue(const std::string &name,const JointValues &q,bool errors=false,std::set<std::string> seen={}) const {
    geometryRequire(seen.insert(name).second,"mimic cycle");const auto &j=joints_.at(name);if(j.kind=="fixed")return 0.;
    if(!j.mimic.empty()) {const double value=jointValue(j.mimic,q,errors,seen);return errors?std::abs(j.multiplier)*value:j.multiplier*value+j.offset;}
    geometryRequire(q.count(name) && std::isfinite(q.at(name)),"missing/nonfinite joint");return q.at(name);
  }
  Transform fk(const std::string &link,const JointValues &q,std::map<std::string,Transform> &cache) const {
    if(link==base_)return Transform::Identity();
    if(cache.count(link))return cache.at(link);
    const auto &j=joints_.at(parents_.at(link));const double value=invariant_.count(j.name)?0.:jointValue(j.name,q);
    geometryRequire(value>=j.lower-.01 && value<=j.upper+.01,"joint outside model limit");Transform motion=Transform::Identity();
    if(j.kind=="revolute" || j.kind=="continuous")motion.block<3,3>(0,0)=Eigen::AngleAxisd(value,j.axis).toRotationMatrix();
    else if(j.kind=="prismatic")motion.block<3,1>(0,3)=j.axis*value;
    const Transform out=fk(j.parent,q,cache)*j.origin*motion;cache[link]=out;return out;
  }
  double errorRadius(const Shape &shape,const JointValues &q,const JointValues &errors) const {
    double radius=shape.radius()+shape.pose.block<3,1>(0,3).norm(),bound=0.;auto names=chain(shape.link);
    for(auto it=names.rbegin();it!=names.rend();++it) {
      const auto &j=joints_.at(*it);const double error=invariant_.count(*it)?0.:jointValue(*it,errors,true);
      geometryRequire(error>=0 && error<=.1,"invalid joint hold error");
      if(j.kind=="prismatic") {bound+=error;radius+=std::abs(jointValue(*it,q))+error;}
      else if(j.kind=="revolute" || j.kind=="continuous")bound+=2*radius*std::sin(error/2);
      radius+=j.origin.block<3,1>(0,3).norm();
    }
    return bound;
  }
  RobotGeometry geometry(const JointValues &q,const JointValues &errors,const std::vector<Shape> &attachments={},double padding=.01,std::array<double,2> floor={.31,.31},const std::vector<double> &layer_edges={}) const {
    geometryRequire(std::isfinite(padding) && padding>=0 && padding<=.1,"invalid model padding");
    geometryRequire(layer_edges.size()>=2,"at least two layer edges required");
    finiteValues(layer_edges);
    for(std::size_t i=1;i<layer_edges.size();++i)geometryRequire(layer_edges[i-1]<layer_edges[i],"strictly increasing layer edges required");
    for(const auto &shape:attachments)for(const auto &name:chain(shape.link))geometryRequire(!invariant_.count(name),"attachment changes an invariant joint; rebuild collision model");
    RobotGeometry out;std::vector<GeometrySlice> bodies;std::map<std::string,Transform> cache;
    auto append=[&](const Shape &shape) {
      const auto transform=fk(shape.link,q,cache);const auto footprint=shape.polygon(transform);
      const double low=-shape.support({0.,0.,-1.},transform),high=shape.support({0.,0.,1.},transform);
      const double error=padding+errorRadius(shape,q,errors);const auto region=inflatePolygon(footprint,error);
      out.physical.insert(out.physical.end(),footprint.begin(),footprint.end());out.reserved.insert(out.reserved.end(),region.begin(),region.end());
      bodies.push_back({low-error,high+error,region});
    };
    for(const auto &shape:shapes_)append(shape);
    for(const auto &shape:attachments)append(shape);
    const Polygon2 bottom{{-floor[0],-floor[1]},{floor[0],-floor[1]},{floor[0],floor[1]},{-floor[0],floor[1]}};
    out.physical.insert(out.physical.end(),bottom.begin(),bottom.end());const auto reserve=inflatePolygon(bottom,padding);
    out.reserved.insert(out.reserved.end(),reserve.begin(),reserve.end());out.z_min=INFINITY;out.height=-INFINITY;
    for(const auto &body:bodies) {out.z_min=std::min(out.z_min,body.z_min);out.height=std::max(out.height,body.z_max);}
    // Explicit base-frame layers use the same inflated bodies. Touching either
    // boundary includes the full body projection; empty layers retain identity.
    for(std::size_t i=1;i<layer_edges.size();++i) {
      const double low=layer_edges[i-1],high=layer_edges[i];Polygon2 points;
      for(const auto &body:bodies)if(body.z_min<=high && body.z_max>=low)points.insert(points.end(),body.footprint.begin(),body.footprint.end());
      out.slices.push_back({low,high,points.empty()?Polygon2{}:convexHull(points)});
    }
    out.physical=convexHull(out.physical);out.reserved=convexHull(out.reserved);return out;
  }
};
} // namespace astribot_s1_robot_geometry
