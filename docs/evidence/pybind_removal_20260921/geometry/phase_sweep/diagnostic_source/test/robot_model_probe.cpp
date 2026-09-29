#include "astribot_s1_robot_geometry/robot_model.hpp"
#include <iostream>
using namespace astribot_s1_robot_geometry;
using nlohmann::json;
int main() {
  std::string line;
  while(std::getline(std::cin,line)) {
    try {
      const auto a=json::parse(line);RobotModel model(a.at("urdf"),a.value("frame",std::string("base")),
        [&a](const std::string &name){return a.at("packages").at(name).get<std::string>();});
      std::vector<Shape> attached;
      for(const auto &s:a.value("attachments",json::array())) {
        Eigen::Matrix4d pose;
        for(int i=0;i<4;++i)for(int j=0;j<4;++j)pose(i,j)=s.at("pose")[i][j];
        attached.emplace_back(s.at("link"),s.at("kind"),s.at("dimensions").get<std::vector<double>>(),pose);
      }
      const auto g=model.geometry(a.at("q").get<std::map<std::string,double>>(),a.at("errors").get<std::map<std::string,double>>(),attached,a.value("padding",.01));
      json slices=json::array();for(const auto &s:g.slices)slices.push_back({{"z_min",s.z_min},{"z_max",s.z_max},{"footprint",s.footprint}});
      std::cout<<json{{"physical",g.physical},{"reserved",g.reserved},{"height",g.height},{"z_min",g.z_min},{"slices",slices},{"required",model.required()},{"revision",model.revision()}}.dump()<<'\n';
    }catch(const std::exception&e) {std::cout<<json{{"error",e.what()}}.dump()<<'\n';}
  }
}
