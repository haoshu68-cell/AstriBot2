#include <astribot_object_pose_core/registration.hpp>
#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>
#include <iomanip>
cv::Mat read(const char* path) {std::ifstream f(path);std::string s;std::vector<cv::Vec<float,6>> p;while(std::getline(f,s)){if(s.empty()||s[0]=='#')continue;std::istringstream in(s);cv::Vec<float,6> v;for(int i=0;i<6;++i)in>>v[i];p.push_back(v);}return cv::Mat(p,true).reshape(1,p.size());}
void candidate(std::ostream& out,const astribot::object_pose::Candidate& c) {out<<"{\"model\":"<<c.model_coverage<<",\"visible\":"<<c.visible_model_coverage<<",\"visible_count\":"<<c.visible_model_points<<",\"normals\":"<<c.observable_normal_directions<<",\"scene\":"<<c.scene_coverage<<",\"rmse\":"<<c.rmse_m<<",\"votes\":"<<c.votes<<",\"transform\":[";for(int i=0;i<16;++i){if(i)out<<',';out<<c.camera_from_object.val[i];}out<<"]}";}
int main(int argc,char**argv) {astribot::object_pose::Options o;cv::FileStorage fs(argv[3],cv::FileStorage::READ);for(const auto& b:fs["boxes"]){astribot::object_pose::BoxPrimitive box;for(int k=0;k<3;++k){box.center_m[k]=double(b["center_m"][k]);box.size_m[k]=double(b["size_m"][k]);}o.visibility_boxes.push_back(box);}if(argc>5)o.max_candidates=std::stoul(argv[5]);auto m=read(argv[1]),s=read(argv[2]);auto t=std::chrono::steady_clock::now();auto r=astribot::object_pose::estimate(m,s,o);std::ofstream out(argv[4]);out<<std::setprecision(15)<<"{\"success\":"<<(r.success?"true":"false")<<",\"reason\":\""<<r.reason<<"\",\"elapsed_s\":"<<std::chrono::duration<double>(std::chrono::steady_clock::now()-t).count()<<",\"best\":";candidate(out,r);out<<",\"candidates\":[";for(size_t i=0;i<r.candidates.size();++i){if(i)out<<',';candidate(out,r.candidates[i]);}out<<"]}\n";}
