#include "astribot_object_pose_core/registration.hpp"

#include <opencv2/flann.hpp>
#include <opencv2/surface_matching/ppf_match_3d.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace astribot::object_pose {
namespace {
cv::Vec3d point(const cv::Mat& cloud, int row, int offset=0) {
  const float* p=cloud.ptr<float>(row)+offset;
  return {p[0],p[1],p[2]};
}
cv::Matx33d rotation(const cv::Matx44d& t) {
  return {t(0,0),t(0,1),t(0,2),t(1,0),t(1,1),t(1,2),t(2,0),t(2,1),t(2,2)};
}
cv::Vec3d translation(const cv::Matx44d& t) { return {t(0,3),t(1,3),t(2,3)}; }
cv::Matx44d transform(const cv::Matx33d& r, const cv::Vec3d& p) {
  return {r(0,0),r(0,1),r(0,2),p[0],r(1,0),r(1,1),r(1,2),p[1],r(2,0),r(2,1),r(2,2),p[2],0,0,0,1};
}
double angle(const cv::Matx33d& r) { return std::acos(std::clamp((cv::trace(r)-1)/2,-1.0,1.0)); }

class Neighbors {
public:
  explicit Neighbors(const cv::Mat& cloud) : xyz_(cloud.colRange(0,3).clone()) {
    // A single spatial kd-tree performs exact 3D search (epsilon zero), avoids
    // randomized multi-tree traversal and preserves the inlier-distance gates.
    cv::flann::IndexParams params;
    params.setAlgorithm(cvflann::FLANN_INDEX_KDTREE_SINGLE);
    params.setInt("leaf_max_size",10);
    params.setBool("reorder",true);
    index_.build(xyz_,params);
  }
  std::pair<int,double> nearest(const cv::Vec3d& p) {
    float q[3]{float(p[0]),float(p[1]),float(p[2])};
    int i=-1; float d=0;
    cv::Mat query(1,3,CV_32F,q),indices(1,1,CV_32S,&i),distances(1,1,CV_32F,&d);
    index_.knnSearch(query,indices,distances,1,cv::flann::SearchParams(64));
    return {i,double(d)};
  }
  std::vector<int> neighborhood(const cv::Vec3d& p,int count) {
    float q[3]{float(p[0]),float(p[1]),float(p[2])};
    cv::Mat query(1,3,CV_32F,q),indices(1,count,CV_32S),distances(1,count,CV_32F);
    index_.knnSearch(query,indices,distances,count,cv::flann::SearchParams(64));
    return {indices.ptr<int>(),indices.ptr<int>()+count};
  }
private:
  cv::Mat xyz_;
  cv::flann::Index index_;
};

std::string validate(const cv::Mat& cloud, bool model, const Options& options) {
  const std::string label=model?"model":"scene";
  if(cloud.empty() || cloud.type()!=CV_32F || cloud.dims!=2 ||
     (cloud.cols!=6 && (model || cloud.cols!=3))) return "invalid_"+label+"_shape";
  if(cloud.rows<int(options.min_points)) return "insufficient_"+label+"_points";
  if(cloud.rows>int(options.max_points)) return "too_many_"+label+"_points";
  for(int i=0;i<cloud.rows;++i) for(int j=0;j<cloud.cols;++j)
    if(!std::isfinite(cloud.at<float>(i,j))) return "nonfinite_"+label;
  if(cloud.cols==6) for(int i=0;i<cloud.rows;++i)
    if(cv::norm(point(cloud,i,3))<1e-5) return "invalid_"+label+"_normals";
  cv::PCA pca(cloud.colRange(0,3),cv::Mat(),cv::PCA::DATA_AS_ROW);
  const double largest=pca.eigenvalues.at<float>(0);
  const double smallest=pca.eigenvalues.at<float>(2);
  if(largest<1e-8 || smallest/largest<1e-3) return "degenerate_"+label;
  return {};
}

cv::Mat normals(const cv::Mat& input,const Options& options) {
  cv::Mat output(input.rows,6,CV_32F);
  input.colRange(0,3).copyTo(output.colRange(0,3));
  Neighbors neighbors(input);
  for(int i=0;i<input.rows;++i) {
    cv::Vec3d n;
    if(input.cols==6) n=point(input,i,3);
    else {
      const auto ids=neighbors.neighborhood(point(input,i),std::min(options.normal_neighbors,input.rows));
      cv::Mat local(int(ids.size()),3,CV_32F);
      for(std::size_t j=0;j<ids.size();++j) input.row(ids[j]).colRange(0,3).copyTo(local.row(int(j)));
      cv::PCA pca(local,cv::Mat(),cv::PCA::DATA_AS_ROW);
      n={pca.eigenvectors.at<float>(2,0),pca.eigenvectors.at<float>(2,1),pca.eigenvectors.at<float>(2,2)};
      if(n.dot(point(input,i))>0) n=-n;
    }
    n*=1.0/cv::norm(n);
    for(int k=0;k<3;++k) output.at<float>(i,k+3)=float(n[k]);
  }
  return output;
}

bool proper(const cv::Matx44d& t) {
  for(double v:t.val) if(!std::isfinite(v)) return false;
  const auto r=rotation(t);
  return cv::norm(cv::Mat(r.t()*r-cv::Matx33d::eye()))<1e-3 && std::abs(cv::determinant(r)-1)<1e-3 &&
    std::abs(t(3,0))+std::abs(t(3,1))+std::abs(t(3,2))+std::abs(t(3,3)-1)<1e-6;
}

bool validGeometry(const cv::Mat& model,const std::vector<BoxPrimitive>& boxes) {
  if(boxes.size()>32) return false;
  for(const auto& box:boxes) for(int axis=0;axis<3;++axis)
    if(!std::isfinite(box.center_m[axis]) || !std::isfinite(box.size_m[axis]) ||
       box.size_m[axis]<=1e-5 || box.size_m[axis]>100) return false;
  for(int i=0;i<model.rows;++i) {
    const auto p=point(model,i),n=point(model,i,3);
    bool on_surface=false;
    for(const auto& box:boxes) {
      bool contained=true,interior=true;
      for(int axis=0;axis<3;++axis) {
        const double distance=std::abs(p[axis]-box.center_m[axis]);
        contained &= distance<=box.size_m[axis]/2+1e-5;
        interior &= distance<box.size_m[axis]/2-1e-5;
      }
      if(interior) return false;
      if(!contained) continue;
      for(int axis=0;axis<3;++axis) for(int sign:{-1,1})
        if(std::abs(p[axis]-box.center_m[axis]-sign*box.size_m[axis]/2)<1e-5 && sign*n[axis]>.99)
          on_surface=true;
    }
    if(!on_surface) return false;
  }
  return true;
}

bool visible(const cv::Vec3d& point,const cv::Vec3d& normal,const cv::Vec3d& eye,
             const std::vector<BoxPrimitive>& boxes) {
  const auto ray=point-eye;
  if(normal.dot(ray)>=-1e-9) return false;
  for(const auto& box:boxes) {
    double near=0,far=1;
    bool intersects=true;
    for(int axis=0;axis<3;++axis) {
      const double low=box.center_m[axis]-box.size_m[axis]/2;
      const double high=box.center_m[axis]+box.size_m[axis]/2;
      if(std::abs(ray[axis])<1e-12) {
        if(eye[axis]<low || eye[axis]>high) {intersects=false;break;}
      } else {
        const double a=(low-eye[axis])/ray[axis],b=(high-eye[axis])/ray[axis];
        near=std::max(near,std::min(a,b));far=std::min(far,std::max(a,b));
        if(near>far) {intersects=false;break;}
      }
    }
    if(intersects && near<1-1e-5 && far>1e-8) return false;
  }
  return true;
}

Candidate score(const cv::Mat& model,const cv::Mat& scene,Neighbors& model_index,
                Neighbors& scene_index,const cv::Matx44d& t,const Options& options,std::size_t votes) {
  Candidate out;out.camera_from_object=t;out.votes=votes;
  if(!proper(t)) return out;
  const auto r=rotation(t);const auto p=translation(t);
  const double threshold=options.inlier_distance_m*options.inlier_distance_m;
  double error=0;std::size_t matched_scene=0,matched_model=0,matched_visible=0;
  const bool with_geometry=!options.visibility_boxes.empty();
  const auto eye=-(r.t()*p);
  std::array<std::size_t,6> normal_support{};
  std::vector<bool> model_visible(model.rows,false);
  if(with_geometry) for(int i=0;i<model.rows;++i)
    model_visible[i]=visible(point(model,i),point(model,i,3),eye,options.visibility_boxes);
  for(int i=0;i<scene.rows;++i) {
    const auto hit=model_index.nearest(r.t()*(point(scene,i)-p));
    if(hit.second<=threshold && (!with_geometry || model_visible[hit.first]) && (r*point(model,hit.first,3)).dot(point(scene,i,3))>.35) {
      error+=hit.second;++matched_scene;
      if(with_geometry) {
        const auto n=point(model,hit.first,3);
        int axis=0;
        for(int j=1;j<3;++j) if(std::abs(n[j])>std::abs(n[axis])) axis=j;
        ++normal_support[2*axis+(n[axis]>0?1:0)];
      }
    }
  }
  for(int i=0;i<model.rows;++i) {
    const auto hit=scene_index.nearest(r*point(model,i)+p);
    const bool matched=hit.second<=threshold && (r*point(model,i,3)).dot(point(scene,hit.first,3))>.35;
    if(matched) ++matched_model;
    if(with_geometry && model_visible[i]) {
      ++out.visible_model_points;
      if(matched) ++matched_visible;
    }
  }
  out.model_coverage=double(matched_model)/model.rows;
  out.scene_coverage=double(matched_scene)/scene.rows;
  if(with_geometry) {
    out.visible_model_coverage=out.visible_model_points?double(matched_visible)/out.visible_model_points:0;
    const auto required=std::max(std::size_t(20),std::size_t(std::ceil(scene.rows*options.min_normal_support_fraction)));
    for(int axis=0;axis<3;++axis)
      if(std::max(normal_support[2*axis],normal_support[2*axis+1])>=required) ++out.observable_normal_directions;
  }
  if(matched_scene) out.rmse_m=std::sqrt(error/matched_scene);
  return out;
}

double rank(const Candidate& c,const Options& options) {
  const double coverage=options.visibility_boxes.empty()?c.model_coverage:c.visible_model_coverage;
  return c.scene_coverage+.25*coverage-.25*c.rmse_m/options.inlier_distance_m;
}

// Scene-to-model correspondences preserve the missing-surface semantics under
// partial visibility. Robust trimming and normal agreement suppress outliers.
cv::Matx44d refine(const cv::Mat& model,const cv::Mat& scene,Neighbors& model_index,
                   cv::Matx44d t,const Options& options) {
  struct Pair { cv::Vec3d a,b;double distance; };
  for(int iteration=0;iteration<45;++iteration) {
    const auto r=rotation(t);const auto p=translation(t);
    std::vector<Pair> pairs;
    const double gate=std::max(options.inlier_distance_m*1.5,.035*std::pow(.92,iteration));
    for(int i=0;i<scene.rows;++i) {
      auto hit=model_index.nearest(r.t()*(point(scene,i)-p));
      if(hit.second<gate*gate && (r*point(model,hit.first,3)).dot(point(scene,i,3))>.2)
        pairs.push_back({point(model,hit.first),point(scene,i),hit.second});
    }
    if(pairs.size()<20) break;
    std::sort(pairs.begin(),pairs.end(),[](const Pair& a,const Pair& b){return a.distance<b.distance;});
    pairs.resize(std::max(std::size_t(20),pairs.size()*9/10));
    cv::Vec3d a(0,0,0),b(0,0,0);
    for(const auto& pair:pairs) {a+=pair.a;b+=pair.b;}
    a*=1.0/pairs.size();b*=1.0/pairs.size();
    cv::Matx33d h=cv::Matx33d::zeros();
    for(const auto& pair:pairs) h+=(pair.a-a)*(pair.b-b).t();
    cv::Mat w,u,vt;cv::SVD::compute(cv::Mat(h),w,u,vt);
    cv::Mat corrected_v=vt.t();
    if(cv::determinant(corrected_v*u.t())<0) corrected_v.col(2)*=-1;
    cv::Mat solved=corrected_v*u.t();
    cv::Matx33d refined;
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) refined(i,j)=solved.at<double>(i,j);
    const auto next=transform(refined,b-refined*a);
    const double delta=cv::norm(cv::Mat(next-t));
    t=next;
    if(delta<1e-7) break;
  }
  return t;
}

bool equivalent(const cv::Matx44d& a,const cv::Matx44d& b,const Options& options) {
  if(cv::norm(translation(a)-translation(b))>options.distinct_translation_m) return false;
  const auto ra=rotation(a),rb=rotation(b);
  if(angle(ra.t()*rb)<options.distinct_rotation_rad) return true;
  for(const auto& s:options.symmetry_rotations)
    if(angle((ra*s).t()*rb)<options.distinct_rotation_rad) return true;
  return false;
}

bool geometricSymmetry(const cv::Mat& model,Neighbors& index,const cv::Matx33d& r,double tolerance) {
  std::size_t hits=0;
  for(int i=0;i<model.rows;++i) {
    auto hit=index.nearest(r*point(model,i));
    if(hit.second<tolerance*tolerance && (r*point(model,i,3)).dot(point(model,hit.first,3))>.95) ++hits;
  }
  return double(hits)/model.rows>.985;
}
}

Result estimate(const cv::Mat& model_input,const cv::Mat& scene_input,const Options& options) {
  Result result;
  auto fail=[&](const std::string& reason){result.reason=reason;return result;};
  if(options.min_points<20 || options.max_points<options.min_points || options.max_points>100000 ||
     options.refinement_points<60 || options.refinement_points>options.max_points ||
     options.max_candidates<1 || options.max_candidates>256 || options.normal_neighbors<3 ||
     !(options.relative_sampling_step>=.015 && options.relative_sampling_step<=.2) ||
     !(options.relative_distance_step>=.01 && options.relative_distance_step<=.3) ||
     !(options.relative_scene_sample_step>0 && options.relative_scene_sample_step<=1) ||
     !(options.inlier_distance_m>0) || !(options.max_rmse_m>0) ||
     !(options.min_model_coverage>0 && options.min_model_coverage<=1) ||
     !(options.min_scene_coverage>0 && options.min_scene_coverage<=1) ||
     !(options.min_visible_model_coverage>0 && options.min_visible_model_coverage<=1) ||
     !(options.min_normal_support_fraction>0 && options.min_normal_support_fraction<=.5)) return fail("invalid_options");
  for(double value:{options.relative_sampling_step, options.relative_distance_step,
      options.relative_scene_sample_step,options.inlier_distance_m,options.max_rmse_m,
      options.min_model_coverage,options.min_scene_coverage,options.min_visible_model_coverage,options.min_normal_support_fraction,options.ambiguity_coverage_margin,
      options.ambiguity_rmse_margin_m,options.distinct_translation_m,options.distinct_rotation_rad})
    if(!std::isfinite(value) || value<0) return fail("invalid_options");
  auto reason=validate(model_input,true,options);if(!reason.empty()) return fail(reason);
  reason=validate(scene_input,false,options);if(!reason.empty()) return fail(reason);
  if(options.continuous_symmetry) {result.ambiguous=true;return fail("continuous_symmetry_unobservable");}
  try {
    cv::Mat model=normals(model_input,options),scene=normals(scene_input,options);
    if(!options.visibility_boxes.empty() && !validGeometry(model,options.visibility_boxes)) return fail("invalid_visibility_geometry");
    Neighbors model_index(model),scene_index(scene);
    const int refinement_count=std::min(int(options.refinement_points),scene.rows);
    cv::Mat refinement_scene(refinement_count,6,CV_32F);
    for(int i=0;i<refinement_count;++i) scene.row(i*scene.rows/refinement_count).copyTo(refinement_scene.row(i));
    for(const auto& s:options.symmetry_rotations) {
      if(!proper(transform(s,{0,0,0})) || !geometricSymmetry(model,model_index,s,options.inlier_distance_m*.5))
        return fail("invalid_symmetry_group");
    }
    for(const auto& s:boxSymmetryRotations()) if(geometricSymmetry(model,model_index,s,options.inlier_distance_m*.5)) {
      if(!equivalent(cv::Matx44d::eye(),transform(s,{0,0,0}),options)) {
        result.ambiguous=true;return fail("undeclared_model_symmetry");
      }
    }
    result.symmetry_equivalent=!options.symmetry_rotations.empty();
    cv::ppf_match_3d::PPF3DDetector detector(options.relative_sampling_step,options.relative_distance_step,30);
    detector.setSearchParams(.012,10*CV_PI/180,true);
    detector.trainModel(model);
    std::vector<cv::ppf_match_3d::Pose3DPtr> hypotheses;
    detector.match(scene,hypotheses,options.relative_scene_sample_step,options.relative_sampling_step);
    if(hypotheses.empty()) return fail("no_global_candidates");
    const std::size_t count=std::min(options.max_candidates,hypotheses.size());
    for(std::size_t i=0;i<count;++i) {
      if(!proper(hypotheses[i]->pose)) continue;
      auto raw=score(model,scene,model_index,scene_index,hypotheses[i]->pose,options,hypotheses[i]->numVotes);
      auto improved=score(model,scene,model_index,scene_index,
        refine(model,refinement_scene,model_index,hypotheses[i]->pose,options),options,hypotheses[i]->numVotes);
      auto candidate=rank(improved,options)>rank(raw,options)?improved:raw;
      if(std::isfinite(candidate.rmse_m)) result.candidates.push_back(candidate);
    }
    // PPF voting can merge or omit alternatives when distinguishing surfaces
    // are occluded. Explicitly challenge the best hypotheses with half-turns
    // around the observed cloud's principal axes. PCA does not estimate the
    // delivered pose; these are independent ambiguity challenges, refined and
    // scored against the full CAD just like the global PPF hypotheses.
    if(!result.candidates.empty()) {
      auto best=*std::max_element(result.candidates.begin(),result.candidates.end(),
        [&](const Candidate& a,const Candidate& b){return rank(a,options)<rank(b,options);});
      cv::PCA observed_axes(scene.colRange(0,3),cv::Mat(),cv::PCA::DATA_AS_ROW);
      cv::Vec3d center(observed_axes.mean.at<float>(0),observed_axes.mean.at<float>(1),observed_axes.mean.at<float>(2));
      std::vector<cv::Matx44d> challenge_poses;
      if(!options.visibility_boxes.empty()) {
        const auto& base=*std::max_element(options.visibility_boxes.begin(),options.visibility_boxes.end(),
          [](const BoxPrimitive& a,const BoxPrimitive& b){return a.size_m[0]*a.size_m[1]*a.size_m[2]<b.size_m[0]*b.size_m[1]*b.size_m[2];});
        for(const auto& turn:boxSymmetryRotations())
          challenge_poses.push_back(best.camera_from_object*transform(turn,base.center_m-turn*base.center_m));
      }
      for(int axis=0;axis<3;++axis) {
        cv::Vec3d direction(observed_axes.eigenvectors.at<float>(axis,0),observed_axes.eigenvectors.at<float>(axis,1),observed_axes.eigenvectors.at<float>(axis,2));
        auto turn=2.0*direction*direction.t()-cv::Matx33d::eye();
        auto pose=transform(turn*rotation(best.camera_from_object),
          center+turn*(translation(best.camera_from_object)-center));
        challenge_poses.push_back(pose);
      }
      for(const auto& pose:challenge_poses) {
        if(!proper(pose) || equivalent(best.camera_from_object,pose,options)) continue;
        auto raw=score(model,scene,model_index,scene_index,pose,options,0);
        auto improved=score(model,scene,model_index,scene_index,
          refine(model,refinement_scene,model_index,pose,options),options,0);
        auto candidate=rank(improved,options)>rank(raw,options)?improved:raw;
        if(std::isfinite(candidate.rmse_m)) result.candidates.push_back(candidate);
      }
    }
    std::sort(result.candidates.begin(),result.candidates.end(),[&](const Candidate& a,const Candidate& b){return rank(a,options)>rank(b,options);});
    if(result.candidates.empty()) return fail("no_acceptable_pose");
    static_cast<Candidate&>(result)=result.candidates.front();
    const bool coverage_ok=options.visibility_boxes.empty()?result.model_coverage>=options.min_model_coverage:
      (result.visible_model_coverage>=options.min_visible_model_coverage && result.visible_model_points>=options.min_points);
    if(!coverage_ok || result.scene_coverage<options.min_scene_coverage || result.rmse_m>options.max_rmse_m)
      return fail("no_acceptable_pose");
    if(!options.visibility_boxes.empty() && result.observable_normal_directions<2) return fail("insufficient_normal_diversity");
    for(std::size_t i=1;i<result.candidates.size();++i) {
      const auto& candidate=result.candidates[i];
      if(candidate.scene_coverage+options.ambiguity_coverage_margin>=result.scene_coverage &&
         (options.visibility_boxes.empty()?candidate.model_coverage+options.ambiguity_coverage_margin>=result.model_coverage:
           candidate.visible_model_coverage>=options.min_visible_model_coverage) &&
         candidate.rmse_m<=(options.visibility_boxes.empty()?result.rmse_m+options.ambiguity_rmse_margin_m:options.max_rmse_m) &&
         !equivalent(result.camera_from_object,candidate.camera_from_object,options)) {
        result.ambiguous=true;return fail("ambiguous_pose");
      }
    }
    result.success=true;result.reason="ok";return result;
  } catch(const cv::Exception&) { return fail("registration_exception"); }
    catch(const std::exception&) { return fail("registration_exception"); }
}

std::vector<cv::Matx33d> boxSymmetryRotations() {
  return {cv::Matx33d(1,0,0,0,-1,0,0,0,-1),cv::Matx33d(-1,0,0,0,1,0,0,0,-1),cv::Matx33d(-1,0,0,0,-1,0,0,0,1)};
}
}  // namespace astribot::object_pose
