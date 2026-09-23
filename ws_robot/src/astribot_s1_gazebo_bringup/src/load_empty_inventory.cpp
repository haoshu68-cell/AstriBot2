// Operational loader for an explicitly owned, isolated simulation. A service
// response means queued; /payload/simulation_inventory_diagnostics proves activation.
#include <ignition/transport/Node.hh>
#include <ignition/msgs/empty.pb.h>
#include <ignition/msgs/serialized_map.pb.h>
#include <ignition/msgs/entity_plugin_v.pb.h>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/SystemPluginInfo.hh>
#include <filesystem>
#include <cstdlib>
#include <iostream>
#include <map>
#include <regex>
#include <sstream>

namespace components=ignition::gazebo::components;
static std::string environment(const char *key) {const char *value=std::getenv(key);return value?value:"";}
static std::string xml(const std::string &value) {
  std::string result;for(char c:value) {
    if(c=='&')result+="&amp;";else if(c=='<')result+="&lt;";else if(c=='>')result+="&gt;";
    else if(c=='\"')result+="&quot;";else if(c=='\'')result+="&apos;";else result+=c;
  }return result;
}
int main(int argc,char **argv) {
  try {
    const std::string usage="load_empty_inventory --world NAME --robot NAME --session ID --source ID --world-reference FILE --robot-reference FILE --plugin FILE";
    if(argc!=15 && argc!=19)throw std::invalid_argument(usage+" [--payload-registry FILE --robot-urdf FILE]");
    std::map<std::string,std::string> args;
    for(int i=1;i<argc;i+=2)if(!args.emplace(argv[i],argv[i+1]).second)throw std::invalid_argument("DUPLICATE_OPTION");
    for(const auto *key:{"--world","--robot","--session","--source"})
      if(!std::regex_match(args.at(key),std::regex("[A-Za-z0-9_.-]{1,128}")))throw std::invalid_argument("INVALID_IDENTITY");
    for(const auto *key:{"--world-reference","--robot-reference","--plugin"})
      if(args.at(key).size()>4096 || !std::filesystem::is_regular_file(args.at(key)))throw std::invalid_argument("INVALID_REFERENCE_FILE");
    if(argc==19)for(const auto *key:{"--payload-registry","--robot-urdf"})
      if(args.at(key).size()>4096 || !std::filesystem::is_regular_file(args.at(key)))throw std::invalid_argument("INVALID_PAYLOAD_REFERENCE_FILE");
    const auto domain=environment("ROS_DOMAIN_ID");
    if(!std::regex_match(domain,std::regex("[0-9]{1,3}")) || std::stoi(domain)<1 || std::stoi(domain)>101 || std::stoi(domain)==25 ||
       environment("ASTRIBOT_SIM_INSTANCE")!=args.at("--session") ||
       environment("IGN_PARTITION")!="astribot_"+args.at("--session"))throw std::invalid_argument("ISOLATED_SESSION_REQUIRED");
    ignition::transport::Node node;ignition::msgs::Empty query;ignition::msgs::SerializedStepMap state;bool success=false;
    const auto service="/world/"+args.at("--world");
    if(!node.Request(service+"/state",query,5000,state,success) || !success)throw std::runtime_error("WORLD_STATE_UNAVAILABLE");
    uint64_t world=0;unsigned matches=0;
    for(const auto &[id,entity]:state.state().entities()) {
      (void)id;
      const auto systems=entity.components().find(components::SystemPluginInfo::typeId);
      if(systems!=entity.components().end()) {
        std::istringstream bytes(systems->second.component());components::SystemPluginInfo info;info.Deserialize(bytes);
        for(const auto &plugin:info.Data().plugins())
          if(plugin.name()=="astribot::EmptyInventory")throw std::runtime_error("INVENTORY_ALREADY_LOADED");
      }
      if(!entity.components().count(components::World::typeId))continue;
      const auto name=entity.components().find(components::Name::typeId);
      if(name==entity.components().end())continue;
      std::istringstream bytes(name->second.component());components::Name component;component.Deserialize(bytes);
      if(component.Data()==args.at("--world")) {world=entity.id();++matches;}
    }
    if(matches!=1 || !world)throw std::runtime_error("WORLD_IDENTITY_AMBIGUOUS");
    ignition::msgs::EntityPlugin_V request;request.mutable_entity()->set_id(world);
    request.mutable_entity()->set_name(args.at("--world"));request.mutable_entity()->set_type(ignition::msgs::Entity::WORLD);
    auto *plugin=request.add_plugins();plugin->set_name("astribot::EmptyInventory");plugin->set_filename(std::filesystem::absolute(args.at("--plugin")).string());
    std::map<std::string,std::string> config={{"world_name",args.at("--world")},{"robot_model",args.at("--robot")},
      {"session_id",args.at("--session")},{"source_id",args.at("--source")},{"inventory_policy","static_world_empty_only_v1"},
      {"baseline_world_sdf",std::filesystem::absolute(args.at("--world-reference")).string()},
      {"baseline_robot_sdf",std::filesystem::absolute(args.at("--robot-reference")).string()}};
    if(argc==19) {
      config["inventory_policy"]="kinematic_inventory_v1";
      config["payload_registry"]=std::filesystem::absolute(args.at("--payload-registry")).string();
      config["baseline_robot_urdf"]=std::filesystem::absolute(args.at("--robot-urdf")).string();
    }
    std::string content;for(const auto &[key,value]:config)content+="<"+key+">"+xml(value)+"</"+key+">";
    plugin->set_innerxml(content);ignition::msgs::Boolean response;success=false;
    if(!node.Request(service+"/entity/system/add",request,5000,response,success) || !success || !response.data())
      throw std::runtime_error("INVENTORY_LOAD_NOT_QUEUED");
    std::cout<<"INVENTORY_LOAD_QUEUED world_entity="<<world<<"; verify diagnostics, ledger and geometry before admission\n";
  }catch(const std::exception &error) {std::cerr<<error.what()<<'\n';return 1;}
}
