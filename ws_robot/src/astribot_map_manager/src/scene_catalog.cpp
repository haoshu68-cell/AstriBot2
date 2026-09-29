#include "astribot_map_manager/catalog.hpp"
#include <cmath>
#include <chrono>
#include <regex>
#include <set>

namespace astribot_map_manager {
namespace {
void check(bool ok,const char * code){if(!ok)throw std::runtime_error(code);}
std::string id(const Json & p,const char * key){auto s=p.at(key).get<std::string>();check(std::regex_match(s,std::regex("[A-Za-z0-9_-]{1,96}")),"SCENE.INVALID_ID");return s;}
void label(const Json & p){auto name=p.at("name").get<std::string>();check(!name.empty()&&name.size()<=128,"SCENE.INVALID_NAME");}
void type(const Json & p){static const std::set<std::string> types={"echo","freeze_dryer","plate_reader","sealer","incubator_door"};check(types.count(p.at("type").get<std::string>()),"SCENE.INVALID_TYPE");}
void pose(const Json & p){check(p.value("frame","")=="map","DEVICE.FRAME_MISMATCH");for(auto k:{"x","y","yaw"})check(p.at(k).is_number()&&std::isfinite(p.at(k).get<double>()),"DEVICE.INVALID_POSE");}
Json cleanPose(const Json & p){pose(p);return {{"frame","map"},{"x",p.at("x")},{"y",p.at("y")},{"yaw",p.at("yaw")}};}
void bindings(const Json & data,const Json & scene){
 const auto & refs=scene.at("devices");check(refs.is_array()&&refs.size()<=32,"SCENE.INVALID_DEVICES");std::set<std::string> unique;
 for(const auto & ref:refs){auto key=id(ref,"device_id");check(unique.insert(key).second&&data.at("devices").contains(key),"SCENE.INVALID_DEVICE_REF");const auto & d=data.at("devices").at(key).back();
 check(d.at("version")==ref.at("version"),"DEVICE.VERSION_MISMATCH");check(d.at("enabled").get<bool>(),"DEVICE.DISABLED");check(d.at("map_id")==scene.at("map_id")&&d.at("map_version")==scene.at("map_version"),"DEVICE.MAP_MISMATCH");}
}
}
bool Catalog::sceneReady()const {
 if(data_.at("active_scene").is_null())return true;
 try {const auto & s=data_.at("active_scene");check(!data_.at("active_map").is_null(),"SCENE.NO_MAP");check(s.at("map_id")==data_.at("active_map").at("map_id")&&s.at("map_version")==data_.at("active_map").at("version"),"SCENE.MAP_MISMATCH");
 check(data_.at("scenes").at(s.at("scene_id").get<std::string>()).back().at("version")==s.at("version"),"SCENE.VERSION_MISMATCH");bindings(data_,s);return true;}catch(...){return false;}
}
Json Catalog::sceneCommand(Json & next,const std::string & command_id,const std::string & op,const Json & p,const std::string & actor){
 check(!blocked(data_.at("transaction")),"MAP.TRANSACTION_ACTIVE");
 if(op=="scene_load"){
 auto key=id(p,"scene_id");const auto scene=data_.at("scenes").at(key).back();check(scene.at("version")==p.at("scene_version"),"SCENE.VERSION_MISMATCH");bindings(data_,scene);auto target=verifiedMap(scene.at("map_id"));check(scene.at("map_version")==target.at("version"),"MAP.VERSION_MISMATCH");
 bool same=!data_.at("active_map").is_null()&&data_.at("active_map").at("map_id")==target.at("map_id")&&data_.at("active_map").at("version")==target.at("version");
 if(same&&p.value("reuse_current_map",false)){next["active_scene"]=scene;return {{"state","COMMITTED"},{"scene",scene},{"reused_map",true}};}
 const bool transfer=p.value("manual_transfer",false);if(!data_.at("active_map").is_null())check(data_.at("active_map").at("floor")==target.at("floor")||transfer,"MAP.TRANSFER_REQUIRED");
 Json tx={{"transaction_id",command_id},{"target",target},{"previous",data_.at("active_map")},{"scene_target",scene},{"state",transfer?"WAIT_TRANSFER":"LOAD_INTENT"},{"actor",actor},{"manual_transfer",transfer},{"reason_code","SCENE.LOAD_INTENT"}};next["transaction"]=tx;return tx;
 }
 label(p);type(p);auto map_id=id(p,"map_id");const auto & map=data_.at("maps").at(map_id);check(p.at("map_version")==map.at("version"),"MAP.VERSION_MISMATCH");
 const bool device=op=="device_put";const auto collection=device?"devices":"scenes";const auto key_field=device?"device_id":"scene_id";auto key=id(p,key_field);
 auto versions=data_.at(collection).value(key,Json::array());check(versions.size()<100&& (data_.at(collection).contains(key)||data_.at(collection).size()<64),"STORE.CAPACITY");check(p.at(device?"expected_device_version":"expected_scene_version")==versions.size(),device?"DEVICE.VERSION_MISMATCH":"SCENE.VERSION_MISMATCH");
 Json record={{key_field,key},{"name",p.at("name")},{"type",p.at("type")},{"map_id",map_id},{"map_version",map.at("version")},{"version",versions.size()+1},{"actor",actor},{"updated_at",std::to_string(std::chrono::system_clock::now().time_since_epoch().count())}};
 if(device){
 record["pose"]=cleanPose(p.at("pose"));for(auto k:{"dock_pose","wait_pose"}){record[k]=nullptr;if(p.contains(k)&&!p.at(k).is_null()){record[k]=cleanPose(p.at(k));validateNavigationPose(map,record[k]);}}
 for(auto k:{"width","depth"}){auto v=p.at(k).get<double>();check(std::isfinite(v)&&v>0&&v<=100,"DEVICE.INVALID_SIZE");record[k]=v;}
 record["reviewed"]=p.value("reviewed",false);record["enabled"]=p.value("enabled",true);
 }else{record["devices"]=p.at("devices");bindings(data_,record);}
 versions.push_back(record);next[collection][key]=versions;return record;
}
}
