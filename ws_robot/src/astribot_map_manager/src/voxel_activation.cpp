#include "astribot_map_manager/voxel_activation.hpp"
#include <astribot_s1_autonomy/session_archive.hpp>
#include <fstream>
#include <regex>
namespace astribot_map_manager {
SessionLoadPlan materializeSession(const Json & target,const fs::path & assets,const fs::path & runtime,const std::string & attempt){
 if(!std::regex_match(attempt,std::regex("[A-Za-z0-9_-]{1,128}")))throw std::runtime_error("MAP.INVALID_ATTEMPT");
 auto source=fs::canonical(target.at("directory").get<std::string>()),root=fs::canonical(assets);auto relative=source.lexically_relative(root);
 if(relative.empty()||relative.is_absolute()||*relative.begin()=="..")throw std::runtime_error("MAP.OUTSIDE_ASSET_ROOT");
 if(fs::is_symlink(source/"manifest.json")||fs::file_size(source/"manifest.json")>16*1024*1024||Catalog::hash(source/"manifest.json")!=target.at("version"))throw std::runtime_error("ASSET.HASH_MISMATCH");
 std::ifstream input(source/"manifest.json");auto manifest=Json::parse(input);auto session=manifest.at("session").get<std::string>();
 if(!std::regex_match(session,std::regex("[A-Za-z0-9_-]{1,128}"))||manifest.at("backend")!="voxel_slam"||manifest.at("world_frame")!="map"||manifest.value("final_pose_updates",0)<=0)throw std::runtime_error("MAP.INVALID_SESSION");
 auto destination=fs::absolute(runtime)/attempt;fs::create_directories(destination.parent_path());
 if(!fs::create_directory(destination))throw std::runtime_error("MAP.ATTEMPT_ALREADY_EXISTS");
 try {
 fs::create_directory(destination/session);uint64_t bytes=0;const auto hashes=manifest.at("sha256");
 if(!hashes.is_object()||hashes.size()>100000)throw std::runtime_error("ASSET.CAPACITY");
 for(auto it=hashes.begin();it!=hashes.end();++it){fs::path name=it.key();if(name.empty()||name.is_absolute())throw std::runtime_error("ASSET.INVALID_PATH");auto path=source;
 for(auto part:name){if(part==".."||part==".")throw std::runtime_error("ASSET.INVALID_PATH");path/=part;if(fs::is_symlink(path))throw std::runtime_error("ASSET.SYMLINK");}
 if(!fs::is_regular_file(path)||(bytes+=fs::file_size(path))>8ULL*1024*1024*1024)throw std::runtime_error("ASSET.CAPACITY");
 if(Catalog::hash(path)!=it.value())throw std::runtime_error("ASSET.HASH_MISMATCH");
 if(fs::space(destination).available<fs::file_size(path)+1024ULL*1024*1024)throw std::runtime_error("ASSET.DISK_RESERVE");
 auto output=destination/session/name;fs::create_directories(output.parent_path());fs::copy_file(path,output);
 if(Catalog::hash(output)!=it.value())throw std::runtime_error("ASSET.COPY_MISMATCH");}
 fs::copy_file(source/"manifest.json",destination/session/"manifest.json");
 if(Catalog::hash(destination/session/"manifest.json")!=target.at("version"))throw std::runtime_error("ASSET.COPY_MISMATCH");
 if(astribot_s1_autonomy::inspectSession(destination/session).at("sha256")!=hashes)throw std::runtime_error("ASSET.MANIFEST_INCOMPLETE");
 return {destination,session};
 }catch(...){fs::remove_all(destination);throw;}
}
}
