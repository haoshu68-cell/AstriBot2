// Static, full-height archive projection. No ray clearing or motion interfaces.
#include "astribot_s1_mapping/height_slice_grid.hpp"
#include <astribot_slam_msgs/msg/height_slice_maps.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>

namespace fs=std::filesystem;
using json=nlohmann::json;
using Bundle=astribot_slam_msgs::msg::HeightSliceMaps;
std::string sha256(const std::string &bytes) {
  unsigned char digest[EVP_MAX_MD_SIZE];unsigned int size=0;
  if(EVP_Digest(bytes.data(),bytes.size(),digest,&size,EVP_sha256(),nullptr)!=1)
    throw std::runtime_error("SHA256 failed");
  std::ostringstream out;out<<std::hex<<std::setfill('0');
  for(unsigned int i=0;i<size;++i)out<<std::setw(2)<<unsigned(digest[i]);
  return out.str();
}
std::string readFile(const fs::path &path) {
  std::ifstream input(path,std::ios::binary);
  if(!input)throw std::runtime_error("cannot read "+path.string());
  std::ostringstream out;out<<input.rdbuf();
  if(input.bad())throw std::runtime_error("read failed "+path.string());
  return out.str();
}
class HeightSliceMapNode:public rclcpp::Node {
  rclcpp::Publisher<Bundle>::SharedPtr bundle_pub_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> publishers_;
 public:
  HeightSliceMapNode():Node("height_slice_map") {
    const fs::path session=declare_parameter<std::string>("session_directory","");
    const fs::path output=declare_parameter<std::string>("output_directory","");
    const auto frame=declare_parameter<std::string>("frame_id","");
    const auto reference=declare_parameter<std::string>("ground_reference","");
    const auto ground=declare_parameter<double>("ground_z",NAN);
    const auto profile_path=declare_parameter<std::string>("height_profile_path",
      ament_index_cpp::get_package_share_directory("astribot_s1_mapping")+"/config/height_slices.yaml");
    const auto profile_bytes=readFile(profile_path);
    const auto profile_parameters=YAML::Load(profile_bytes)["/**"]["ros__parameters"];
    const auto resolution=profile_parameters["resolution"].as<double>();
    const auto edges=profile_parameters["height_edges"].as<std::vector<double>>();
    const auto names=profile_parameters["layer_names"].as<std::vector<std::string>>();
    const auto max_cells=profile_parameters["max_grid_cells"].as<int64_t>();
    if(session.empty()||output.empty()||frame.empty()||reference.empty()||max_cells<=0)
      throw std::invalid_argument("session/output/frame/ground_reference and grid budget required");
    astribot_s1_mapping::HeightSliceGrid slices(resolution,ground,edges);
    if(names.size()!=slices.occupied.size())throw std::invalid_argument("layer name/edge count mismatch");
    std::set<std::string> unique;
    for(const auto &name:names) {
      if(name.empty()||name.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789_")!=std::string::npos||!unique.insert(name).second)
        throw std::invalid_argument("invalid/duplicate layer name");
    }
    const auto manifest=json::parse(readFile(session/"manifest.json"));
    if(manifest.at("world_frame")!=frame)throw std::invalid_argument("archive/frame mismatch");
    json hashes=json::object();
    auto checkedRead=[&](const fs::path &relative) {
      const auto data=readFile(session/relative);const auto hash=sha256(data);
      if(manifest.at("sha256").at(relative.generic_string())!=hash)
        throw std::invalid_argument("archive hash mismatch: "+relative.string());
      hashes[relative.generic_string()]=hash;return data;
    };
    std::istringstream lines(checkedRead("alidarState.txt"));std::string line;
    std::vector<Eigen::Isometry3d> poses;
    while(std::getline(lines,line)) {
      std::istringstream row(line);double t,x,y,z,qx,qy,qz,qw;
      if(!(row>>t>>x>>y>>z>>qx>>qy>>qz>>qw))throw std::invalid_argument("invalid pose row");
      Eigen::Quaterniond q(qw,qx,qy,qz);Eigen::Vector3d p(x,y,z);
      if(!std::isfinite(t)||!p.allFinite()||!q.coeffs().allFinite()||std::abs(q.norm()-1.)>1e-3)
        throw std::invalid_argument("invalid archive pose");
      Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();pose.linear()=q.normalized().toRotationMatrix();pose.translation()=p;poses.push_back(pose);
    }
    std::map<size_t,fs::path> files;
    for(const auto &file:fs::directory_iterator(session/"kf"))if(file.path().extension()==".pcd") {
      const auto id=file.path().stem().string();
      if(id.empty()||id.find_first_not_of("0123456789")!=std::string::npos)throw std::invalid_argument("non-numeric keyframe id");
      if(!files.emplace(std::stoull(id),fs::path("kf")/file.path().filename()).second)throw std::invalid_argument("duplicate keyframe id");
    }
    if(files.empty()||manifest.at("keyframes")!=files.size()||manifest.at("scans")!=poses.size())
      throw std::invalid_argument("incomplete archive roster");
    for(const auto &[id,path]:files) {
      if(id>=poses.size())throw std::invalid_argument("keyframe id has no corrected pose");
      checkedRead(path);pcl::PointCloud<pcl::PointXYZ> cloud;
      if(pcl::io::loadPCDFile((session/path).string(),cloud)!=0)throw std::runtime_error("PCD load failed: "+path.string());
      for(const auto &p:cloud) {
        const auto world=(poses[id]*Eigen::Vector3d(p.x,p.y,p.z)).eval();slices.add(world.x(),world.y(),world.z());
      }
    }
    int64_t min_x=INT_MAX,min_y=INT_MAX,max_x=INT_MIN,max_y=INT_MIN;
    for(const auto &layer:slices.occupied)for(const auto &[x,y]:layer) {
      min_x=std::min(min_x,int64_t(x));max_x=std::max(max_x,int64_t(x));min_y=std::min(min_y,int64_t(y));max_y=std::max(max_y,int64_t(y));
    }
    if(min_x>max_x)throw std::invalid_argument("no finite obstacle endpoints inside height profile");
    const uint64_t width=max_x-min_x+1,height=max_y-min_y+1;
    if(width>UINT32_MAX||height>UINT32_MAX||width>uint64_t(max_cells)/height)
      throw std::invalid_argument("map exceeds configured cell budget");
    Bundle bundle;bundle.header.frame_id=frame;bundle.header.stamp=now();
    bundle.map_revision=sha256(hashes.dump());bundle.ground_z=ground;bundle.ground_reference=reference;
    bundle.height_edges=edges;bundle.layer_names=names;bundle.point_counts=slices.counts;
    bundle.below_band_points=slices.below;bundle.above_band_points=slices.above;bundle.nonfinite_points=slices.nonfinite;
    bundle.evidence_kind="static_archive_occupied_endpoints_only";
    json profile={{"height_edges",edges},{"layer_names",names},{"ground_z",ground},{"ground_reference",reference},{"frame_id",frame},{"resolution",resolution}};
    bundle.profile_revision=sha256(profile_bytes);
    if(!fs::create_directory(output))throw std::runtime_error("output directory must not exist: "+output.string());
    json report={{"session_directory",session.string()},{"map_revision",bundle.map_revision},{"profile_revision",bundle.profile_revision},{"profile",profile},
      {"evidence_kind",bundle.evidence_kind},{"input_sha256",hashes},{"width",width},{"height",height},{"point_counts",slices.counts},
      {"below_band_points",slices.below},{"above_band_points",slices.above},{"nonfinite_points",slices.nonfinite},{"occupied_cells",json::array()},
      {"unknown_is_free",false},{"live_dynamic_map",false},{"planning_enabled",false}};
    const auto qos=rclcpp::QoS(1).reliable().transient_local();
    for(size_t i=0;i<names.size();++i) {
      nav_msgs::msg::OccupancyGrid grid;grid.header=bundle.header;grid.info.map_load_time=bundle.header.stamp;
      grid.info.resolution=resolution;grid.info.width=width;grid.info.height=height;
      grid.info.origin.position.x=min_x*resolution;grid.info.origin.position.y=min_y*resolution;grid.info.origin.orientation.w=1;
      grid.data.assign(width*height,-1);
      for(const auto &[x,y]:slices.occupied[i])grid.data[(int64_t(y)-min_y)*width+int64_t(x)-min_x]=100;
      report["occupied_cells"].push_back(slices.occupied[i].size());
      std::ofstream pgm(output/(names[i]+".pgm"),std::ios::binary);pgm.exceptions(std::ios::failbit|std::ios::badbit);
      pgm<<"P5\n"<<width<<" "<<height<<"\n255\n";
      for(int64_t y=height-1;y>=0;--y)for(uint64_t x=0;x<width;++x)pgm.put(grid.data[y*width+x]==100?char(0):char(205));
      std::ofstream yaml(output/(names[i]+".yaml"));yaml.exceptions(std::ios::failbit|std::ios::badbit);
      yaml<<std::setprecision(17)<<"image: "<<names[i]<<".pgm\nmode: trinary\nresolution: "<<resolution<<"\norigin: ["<<min_x*resolution<<", "<<min_y*resolution<<", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
      publishers_.push_back(create_publisher<nav_msgs::msg::OccupancyGrid>("/height_maps/"+names[i],qos));publishers_.back()->publish(grid);
      bundle.grids.push_back(std::move(grid));
    }
    std::ofstream metadata(output/"manifest.json");metadata.exceptions(std::ios::failbit|std::ios::badbit);metadata<<report.dump(2)<<'\n';
    bundle_pub_=create_publisher<Bundle>("/height_maps/snapshot",qos);bundle_pub_->publish(bundle);
    RCLCPP_INFO(get_logger(),"Published %zu static height maps; archive=%s profile=%s; occupied/unknown only",names.size(),bundle.map_revision.c_str(),bundle.profile_revision.c_str());
  }
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<HeightSliceMapNode>());}
  catch(const std::exception &e) {RCLCPP_FATAL(rclcpp::get_logger("height_slice_map"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
