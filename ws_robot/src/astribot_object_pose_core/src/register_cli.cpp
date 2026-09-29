#include "astribot_object_pose_core/registration.hpp"
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
cv::Mat readCloud(const std::string& path, std::size_t max_points) {
  std::ifstream stream(path);
  if(!stream) throw std::runtime_error("unreadable file");
  std::vector<float> values;
  std::string line;
  int columns=0;
  std::size_t rows=0;
  while(std::getline(stream,line)) {
    const auto comment=line.find('#');
    if(comment!=std::string::npos) line.resize(comment);
    std::istringstream tokens(line);
    std::vector<float> row;
    std::string token;
    while(tokens>>token) {
      std::size_t consumed=0;
      float value=std::stof(token,&consumed);
      if(consumed!=token.size()) throw std::runtime_error("invalid number");
      row.push_back(value);
      if(row.size()>6) throw std::runtime_error("too many columns");
    }
    if(row.empty()) continue;
    if((row.size()!=3 && row.size()!=6) || (columns && int(row.size())!=columns)) throw std::runtime_error("invalid columns");
    columns=int(row.size());
    if(++rows>max_points) throw std::runtime_error("too many rows");
    values.insert(values.end(),row.begin(),row.end());
  }
  if(!rows) return {};
  return cv::Mat(int(rows),columns,CV_32F,values.data()).clone();
}
std::vector<astribot::object_pose::BoxPrimitive> readVisibility(const std::string& path) {
  cv::FileStorage input(path,cv::FileStorage::READ|cv::FileStorage::FORMAT_JSON);
  if(!input.isOpened() || std::string(input["schema"])!="astribot.box_union/1" || std::string(input["units"])!="m")
    throw std::runtime_error("invalid visibility schema");
  const auto boxes=input["boxes"];
  if(!boxes.isSeq() || boxes.empty() || boxes.size()>32) throw std::runtime_error("invalid boxes");
  std::vector<astribot::object_pose::BoxPrimitive> geometry;
  for(auto it=boxes.begin();it!=boxes.end();++it) {
    const auto center=(*it)["center_m"],size=(*it)["size_m"];
    if(!center.isSeq() || !size.isSeq() || center.size()!=3 || size.size()!=3) throw std::runtime_error("invalid box vectors");
    astribot::object_pose::BoxPrimitive box;
    for(int axis=0;axis<3;++axis) {
      if((!center[axis].isInt() && !center[axis].isReal()) || (!size[axis].isInt() && !size[axis].isReal())) throw std::runtime_error("invalid box coordinate");
      box.center_m[axis]=double(center[axis]);box.size_m[axis]=double(size[axis]);
    }
    geometry.push_back(box);
  }
  return geometry;
}
std::vector<cv::Matx33d> squarePrismZSymmetry(
    const cv::Mat& model,
    const std::vector<astribot::object_pose::BoxPrimitive>& boxes) {
  // This CLI declaration is specifically a centered, Z-aligned square prism.
  // CAD point symmetry itself remains checked by the existing estimator.
  constexpr double tolerance = 1e-6;
  if (boxes.size() != 1 || model.empty() || model.cols != 6)
    throw std::runtime_error("square_prism_z requires one registered box and CAD normals");
  const auto& box = boxes.front();
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(box.size_m[axis]) || box.size_m[axis] <= 0 ||
        !std::isfinite(box.center_m[axis]) || std::abs(box.center_m[axis]) > tolerance)
      throw std::runtime_error("square_prism_z requires a positive centered box");
  }
  if (std::abs(box.size_m[0] - box.size_m[1]) > tolerance ||
      std::abs(box.size_m[0] - box.size_m[2]) <= tolerance)
    throw std::runtime_error("square_prism_z requires x=y and z different; cubes need 24 rotations");
  for (int axis = 0; axis < 3; ++axis) {
    double low = std::numeric_limits<double>::infinity(), high = -low;
    for (int row = 0; row < model.rows; ++row) {
      const double value = model.at<float>(row, axis);
      if (!std::isfinite(value)) throw std::runtime_error("nonfinite CAD coordinate");
      low = std::min(low, value); high = std::max(high, value);
    }
    if (std::abs(low + box.size_m[axis]/2) > tolerance ||
        std::abs(high - box.size_m[axis]/2) > tolerance)
      throw std::runtime_error("square_prism_z CAD bounds disagree with registered box");
  }
  const cv::Matx33d quarter_turn(0,-1,0, 1,0,0, 0,0,1);
  const cv::Matx33d flip_x(1,0,0, 0,-1,0, 0,0,-1);
  std::vector<cv::Matx33d> rotations;
  auto around_z = cv::Matx33d::eye();
  for (int k = 0; k < 4; ++k) {
    if (k != 0) rotations.push_back(around_z);
    rotations.push_back(around_z * flip_x);
    around_z = around_z * quarter_turn;
  }
  return rotations;
}
void number(std::ostream& stream,double value) {
  if(std::isfinite(value)) stream<<value;else stream<<"null";
}
void json(std::ostream& stream,const astribot::object_pose::Result& r) {
  stream<<std::setprecision(12)<<"{\"schema\":\"astribot.object_pose/1\",\"method\":\"known_cad_ppf_icp\",\"success\":"<<(r.success?"true":"false")
    <<",\"reason\":\""<<r.reason<<"\",\"camera_from_object\":";
  if(r.success) {
    stream<<'[';
    for(int i=0;i<4;++i) {if(i)stream<<',';stream<<'[';for(int j=0;j<4;++j){if(j)stream<<',';number(stream,r.camera_from_object(i,j));}stream<<']';}
    stream<<']';
  } else stream<<"null";
  stream<<",\"coverage\":"<<r.model_coverage<<",\"model_coverage\":"<<r.model_coverage
    <<",\"total_model_coverage\":"<<r.model_coverage<<",\"visible_model_coverage\":";
  number(stream,r.visible_model_coverage);
  stream<<",\"visible_model_points\":"<<r.visible_model_points
    <<",\"observable_normal_directions\":"<<r.observable_normal_directions
    <<",\"scene_coverage\":"<<r.scene_coverage<<",\"rmse_m\":";
  number(stream,r.rmse_m);
  stream<<",\"ambiguous\":"<<(r.ambiguous?"true":"false")
    <<",\"symmetry_equivalent\":"<<(r.symmetry_equivalent?"true":"false")
    <<",\"candidate_count\":"<<r.candidates.size()<<"}\n";
}
}

int main(int argc,char** argv) {
  std::string model_path,scene_path,output_path,visibility_path;
  bool square_prism_z = false;
  astribot::object_pose::Options options;
  try {
    for(int i=1;i<argc;++i) {
      const std::string key=argv[i];
      if(key=="--help") {
        std::cout<<"object_pose_register --model MODEL.xyz --scene SCENE.xyz [--output /absolute/result.json] [--visibility-model REGISTERED.json] [--symmetry none|box|square_prism_z|continuous] [--max-candidates N] [--max-points N]\n";
        return 0;
      }
      if(i+1>=argc) throw std::runtime_error("missing option value");
      const std::string value=argv[++i];
      if(key=="--model") model_path=value;
      else if(key=="--scene") scene_path=value;
      else if(key=="--output") output_path=value;
      else if(key=="--visibility-model") visibility_path=value;
      else if(key=="--symmetry") {
        square_prism_z = value == "square_prism_z";
        if(value=="box") options.symmetry_rotations=astribot::object_pose::boxSymmetryRotations();
        else if(value=="continuous") options.continuous_symmetry=true;
        else if(value!="none" && !square_prism_z) throw std::runtime_error("unknown symmetry");
      } else if(key=="--max-candidates" || key=="--max-points") {
        std::size_t used=0;const auto number=std::stoul(value,&used);
        if(used!=value.size() || value[0]=='-') throw std::runtime_error("invalid integer");
        if(key=="--max-candidates") options.max_candidates=number;else options.max_points=number;
      } else throw std::runtime_error("unknown option");
    }
    if(model_path.empty() || scene_path.empty()) throw std::runtime_error("model and scene are required");
    if(!output_path.empty() && !std::filesystem::path(output_path).is_absolute()) throw std::runtime_error("output path must be absolute");
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 64;}
  astribot::object_pose::Result result;
  cv::Mat model,scene;
  if(!visibility_path.empty()) try {options.visibility_boxes=readVisibility(visibility_path);} catch(const std::exception&) {result.reason="invalid_visibility_model_file";}
  if(result.reason.empty()) try {model=readCloud(model_path,options.max_points);} catch(const std::exception&) {result.reason="invalid_model_file";}
  if(result.reason.empty()) try {scene=readCloud(scene_path,options.max_points);} catch(const std::exception&) {result.reason="invalid_scene_file";}
  if(result.reason.empty() && square_prism_z) {
    try {options.symmetry_rotations=squarePrismZSymmetry(model,options.visibility_boxes);}
    catch(const std::exception& error) {
      result.reason="invalid_square_prism_z_model";
      std::cerr<<error.what()<<'\n';
    }
  }
  if(result.reason.empty()) result=astribot::object_pose::estimate(model,scene,options);
  if(output_path.empty()) json(std::cout,result);
  else {
    // Atomic replacement means interrupted processes cannot publish partial JSON.
    const auto temporary=output_path+".tmp";
    std::ofstream output(temporary,std::ios::trunc);
    if(!output) {std::cerr<<"cannot open output\n";return 74;}
    json(output,result);output.close();
    if(!output) {std::cerr<<"cannot write output\n";return 74;}
    std::error_code error;std::filesystem::rename(temporary,output_path,error);
    if(error) {std::cerr<<"cannot publish output\n";return 74;}
  }
  return result.success?0:2;
}
