#include "astribot_object_pose_core/registration.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

using astribot::object_pose::estimate;
using astribot::object_pose::Options;

namespace {
void check(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
struct Box { cv::Vec3d center; cv::Vec3d size; };

// Independent exact CAD surfaces, not registration-produced expectations.
cv::Mat cad(bool asymmetric=true) {
  std::vector<Box> boxes{{{0,0,0},{.18,.10,.08}}};
  if (asymmetric) {
    boxes.push_back({{.035,.015,.065},{.070,.060,.050}});
    boxes.push_back({{.115,-.020,-.005},{.060,.045,.040}});
  }
  std::vector<cv::Vec<float,6>> points;
  for (std::size_t b=0; b<boxes.size(); ++b) for(int axis=0;axis<3;++axis)
    for(int sign:{-1,1}) {
      const int u=(axis+1)%3,v=(axis+2)%3;
      const int nu=std::max(2,int(std::ceil(boxes[b].size[u]/.006)));
      const int nv=std::max(2,int(std::ceil(boxes[b].size[v]/.006)));
      for(int i=0;i<nu;++i) for(int j=0;j<nv;++j) {
        cv::Vec3d p=boxes[b].center;
        p[axis]+=sign*boxes[b].size[axis]/2;
        p[u]+=((i+.5)/nu-.5)*boxes[b].size[u];
        p[v]+=((j+.5)/nv-.5)*boxes[b].size[v];
        bool interior=false;
        for(std::size_t other=0;other<boxes.size();++other) if(other!=b) {
          bool inside=true;
          for(int k=0;k<3;++k) inside &= std::abs(p[k]-boxes[other].center[k]) < boxes[other].size[k]/2+1e-8;
          interior |= inside;
        }
        if(!interior) {
          cv::Vec<float,6> q{float(p[0]),float(p[1]),float(p[2]),0,0,0};
          q[axis+3]=float(sign);
          points.push_back(q);
        }
      }
    }
  return cv::Mat(points,true).reshape(1,int(points.size()));
}

cv::Matx33d rotation(double rx,double ry,double rz) {
  cv::Matx33d x(1,0,0,0,cos(rx),-sin(rx),0,sin(rx),cos(rx));
  cv::Matx33d y(cos(ry),0,sin(ry),0,1,0,-sin(ry),0,cos(ry));
  cv::Matx33d z(cos(rz),-sin(rz),0,sin(rz),cos(rz),0,0,0,1);
  return z*y*x;
}

cv::Mat scene(const cv::Mat& model,const cv::Matx33d& r,const cv::Vec3d& t,
              double noise=0, bool partial=false, bool camera_visible=false) {
  std::mt19937 rng(921);
  std::normal_distribution<double> perturbation(0,noise);
  std::vector<cv::Vec<float,6>> points;
  for(int i=0;i<model.rows;++i) {
    const float* q=model.ptr<float>(i);
    if(partial && q[0]<-.035) continue;
    cv::Vec3d p=r*cv::Vec3d(q[0],q[1],q[2])+t;
    cv::Vec3d n=r*cv::Vec3d(q[3],q[4],q[5]);
    if(camera_visible && n.dot(p)>-.01) continue;
    points.emplace_back(float(p[0]+perturbation(rng)),float(p[1]+perturbation(rng)),
      float(p[2]+perturbation(rng)),float(n[0]),float(n[1]),float(n[2]));
  }
  std::shuffle(points.begin(),points.end(),rng);
  return cv::Mat(points,true).reshape(1,int(points.size()));
}

double angle(const cv::Matx33d& a,const cv::Matx33d& b) {
  return acos(std::clamp((cv::trace(a.t()*b)-1)/2,-1.0,1.0));
}

Options visibleOptions() {
  Options options;
  options.visibility_boxes={{{0,0,0},{.18,.10,.08}},{{.035,.015,.065},{.070,.060,.050}},{{.115,-.020,-.005},{.060,.045,.040}}};
  return options;
}

// Independent fixture renderer: enumerate ray/face-plane intersections rather
// than using the production slab-intersection visibility routine.
cv::Mat renderedScene(const cv::Mat& model,const cv::Matx33d& r,const cv::Vec3d& t,bool asymmetric=true) {
  auto cloud=scene(model,r,t);
  std::vector<cv::Vec<float,6>> visible;
  const cv::Vec3d eye=-(r.t()*t);
  auto geometry=visibleOptions().visibility_boxes;
  if(!asymmetric) geometry.resize(1);
  for(int i=0;i<cloud.rows;++i) {
    const float* row=cloud.ptr<float>(i);
    cv::Vec3d camera_point(row[0],row[1],row[2]);
    cv::Vec3d normal(row[3],row[4],row[5]);
    if(normal.dot(camera_point)>=0) continue;
    auto object_point=r.t()*(camera_point-t);
    const auto ray=object_point-eye;
    bool blocked=false;
    for(const auto& box:geometry) for(int axis=0;axis<3;++axis) for(int sign:{-1,1}) {
      if(std::abs(ray[axis])<1e-10) continue;
      const double hit=(box.center_m[axis]+sign*box.size_m[axis]/2-eye[axis])/ray[axis];
      if(hit<0 || hit>=1-1e-5) continue;
      auto contact=eye+hit*ray;
      bool inside=true;
      for(int j=0;j<3;++j) inside &= std::abs(contact[j]-box.center_m[j])<=box.size_m[j]/2+1e-7;
      blocked |= inside;
    }
    if(!blocked) visible.emplace_back(row[0],row[1],row[2],row[3],row[4],row[5]);
  }
  return cv::Mat(visible,true).reshape(1,int(visible.size()));
}

void verify(const cv::Mat& model,const cv::Mat& cloud,const cv::Matx33d& r,
            const cv::Vec3d& t,const Options& options={}) {
  auto result=estimate(model,cloud,options);
  std::cout<<"success="<<result.success<<" reason="<<result.reason<<" coverage="
    <<result.model_coverage<<","<<result.scene_coverage<<" rmse="<<result.rmse_m
    <<" candidates="<<result.candidates.size()<<"\n";
  check(result.success,"registration rejected: "+result.reason);
  cv::Matx33d estimated;
  cv::Vec3d translation;
  for(int i=0;i<3;++i) {translation[i]=result.camera_from_object(i,3);for(int j=0;j<3;++j) estimated(i,j)=result.camera_from_object(i,j);}
  double error=angle(estimated,r);
  for(const auto& s:options.symmetry_rotations) error=std::min(error,angle(estimated,r*s));
  std::cout<<"translation_error_m="<<cv::norm(translation-t)<<" rotation_error_deg="<<error*180/CV_PI<<"\n";
  check(cv::norm(translation-t)<.020,"translation exceeds fixed 20mm gate");
  check(error<10*CV_PI/180,"rotation exceeds fixed 10 degree gate");
  check(result.scene_coverage>=options.min_scene_coverage,"accepted insufficient scene coverage");
}
}

int main(int argc,char** argv) {
  try {
    check(argc==2,"expected test name");
    const std::string name=argv[1];
    auto model=cad();
    const auto r=rotation(1.12,-.73,2.31);
    const cv::Vec3d t(.12,-.08,.85);
    if(name=="large_rotation") verify(model,scene(model,r,t),r,t);
    else if(name=="noisy_partial") verify(model,scene(model,r,t,.001,true),r,t);
    else if(name=="multi_depth") {
      verify(model,scene(model,rotation(-1.8,.8,-2.5),{-.17,.09,.5}),rotation(-1.8,.8,-2.5),{-.17,.09,.5});
      verify(model,scene(model,rotation(.7,1.1,-1.2),{.21,.12,1.6},.0007),rotation(.7,1.1,-1.2),{.21,.12,1.6});
    } else if(name=="camera_xyz") {
      // Removing normal estimation or orienting normals away from camera breaks this.
      auto visible=scene(model,r,t,.0003,false,true);
      verify(model,visible.colRange(0,3).clone(),r,t);
    } else if(name=="empty") {
      auto result=estimate(model,{});check(!result.success&&result.reason=="invalid_scene_shape","empty input must be rejected");
    } else if(name=="sparse") {
      auto result=estimate(model,model.rowRange(0,10));check(!result.success&&result.reason=="insufficient_scene_points","sparse input must be rejected");
    } else if(name=="nan") {
      auto cloud=scene(model,r,t);cloud.at<float>(12,1)=NAN;
      auto result=estimate(model,cloud);check(!result.success&&result.reason=="nonfinite_scene","NaN must be rejected before matcher");
    } else if(name=="plane") {
      cv::Mat cloud(120,6,CV_32F,cv::Scalar(0));
      for(int i=0;i<120;++i){cloud.at<float>(i,0)=(i%12)*.01f;cloud.at<float>(i,1)=(i/12)*.01f;cloud.at<float>(i,2)=1;cloud.at<float>(i,5)=-1;}
      auto result=estimate(model,cloud);check(!result.success&&result.reason=="degenerate_scene","plane must be rejected");
    } else if(name=="wrong_model") {
      auto cloud=scene(model,r,t);
      for(int i=0;i<model.rows;++i){model.at<float>(i,0)*=1.9f;model.at<float>(i,1)*=.65f;}
      auto result=estimate(model,cloud);check(!result.success&&result.reason=="no_acceptable_pose","wrong CAD must fail geometric quality gates");
    } else if(name=="undeclared_symmetry") {
      auto box=cad(false);auto result=estimate(box,scene(box,r,t));
      check(!result.success&&result.ambiguous,"undeclared box symmetry must not claim unique pose");
    } else if(name=="declared_symmetry") {
      auto box=cad(false);Options options;options.symmetry_rotations=astribot::object_pose::boxSymmetryRotations();
      auto result=estimate(box,scene(box,r,t),options);
      check(result.symmetry_equivalent,"declared symmetric object must expose equivalent orientation semantics");
      verify(box,scene(box,r,t),r,t,options);
    } else if(name=="ambiguous_partial") {
      // Hiding all asymmetric protrusions leaves competing base-box orientations.
      auto result=estimate(model,scene(cad(false),r,t));
      std::cout<<"partial_base success="<<result.success<<" reason="<<result.reason<<" coverage="<<result.model_coverage<<","<<result.scene_coverage<<" rmse="<<result.rmse_m<<'\n';
      for(const auto& candidate:result.candidates) std::cout<<"candidate "<<candidate.model_coverage<<' '<<candidate.scene_coverage<<' '<<candidate.rmse_m<<'\n';
      check(!result.success&&result.ambiguous,"unseen distinguishing geometry must not claim unique pose");
    } else if(name=="continuous_symmetry") {
      Options options;options.continuous_symmetry=true;
      auto result=estimate(model,scene(model,r,t),options);
      check(!result.success&&result.ambiguous&&result.reason=="continuous_symmetry_unobservable","continuous symmetry has no unique full orientation");
    } else if(name=="invalid_options") {
      Options options;options.inlier_distance_m=std::numeric_limits<double>::infinity();
      auto result=estimate(model,scene(model,r,t),options);
      check(!result.success&&result.reason=="invalid_options","infinite tolerance must not disable the metric gate");
    } else if(name=="invalid_normals") {
      auto cloud=scene(model,r,t);cloud.row(10).colRange(3,6).setTo(0);
      auto result=estimate(model,cloud);
      check(!result.success&&result.reason=="invalid_scene_normals","zero normals must be rejected");
    } else if(name=="invalid_symmetry") {
      Options options;options.symmetry_rotations={cv::Matx33d(-1,0,0,0,1,0,0,0,1)};
      auto result=estimate(model,scene(model,r,t),options);
      check(!result.success&&result.reason=="invalid_symmetry_group","reflection is not a rotation symmetry");
    } else if(name=="visible_two_faces") {
      const double k=std::sqrt(.5);
      const cv::Matx33d view(k,k,0,0,0,-1,-k,k,0);
      const cv::Vec3d location(0,0,.9);
      auto cloud=renderedScene(model,view,location);
      auto options=visibleOptions();
      auto result=estimate(model,cloud,options);
      std::cout<<"visible case total="<<result.model_coverage<<" visible="<<result.visible_model_coverage<<" normals="<<result.observable_normal_directions<<" reason="<<result.reason<<'\n';
      check(result.success,"fully observed two-face view must register without a total-area threshold");
      check(result.model_coverage<.35,"fixture must exercise physically limited total coverage");
      check(result.visible_model_coverage>.9&&result.observable_normal_directions>=2,"visible-surface support must be measured");
      verify(model,cloud,view,location,options);
    } else if(name=="visible_large_rotation") {
      auto cloud=renderedScene(model,r,t);
      verify(model,cloud,r,t,visibleOptions());
    } else if(name=="visible_missing_features") {
      auto result=estimate(model,renderedScene(cad(false),r,t,false),visibleOptions());
      std::cout<<"missing features success="<<result.success<<" reason="<<result.reason<<'\n';
      for(const auto& candidate:result.candidates) std::cout<<"candidate "<<candidate.model_coverage<<' '<<candidate.visible_model_coverage<<' '<<candidate.scene_coverage<<' '<<candidate.rmse_m<<'\n';
      check(!result.success,"visible-area normalization cannot accept a wrong featureless box");
    } else if(name=="invalid_visibility_geometry") {
      auto options=visibleOptions();options.visibility_boxes[0].size_m[0]=.09;
      auto result=estimate(model,scene(model,r,t),options);
      check(!result.success&&result.reason=="invalid_visibility_geometry","registered geometry must agree with the CAD cloud");
    } else if(name=="visible_single_direction") {
      const cv::Matx33d view(0,1,0,0,0,-1,-1,0,0);
      auto result=estimate(model,renderedScene(model,view,{0,0,.9}),visibleOptions());
      std::cout<<"single normal result="<<result.reason<<" directions="<<result.observable_normal_directions<<'\n';
      check(!result.success&&result.reason=="insufficient_normal_diversity","parallel stepped faces must fail the two-direction observability gate");
    } else if(name=="visible_ambiguous_view") {
      const cv::Matx33d view(0,0,1,1,0,0,0,1,0);
      auto result=estimate(model,renderedScene(model,view,{.13,-.04,.9}),visibleOptions());
      check(!result.success&&result.ambiguous,"competing partially visible orientations must be rejected");
    } else throw std::runtime_error("unknown test");
    return 0;
  } catch(const std::exception& e) {std::cerr<<"FAIL: "<<e.what()<<"\n";return 1;}
}
