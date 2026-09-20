#pragma once
#include "astribot_map_manager/catalog.hpp"
namespace astribot_map_manager {
struct SessionLoadPlan {fs::path root;std::string session;};
SessionLoadPlan materializeSession(const Json & target,const fs::path & assets,const fs::path & runtime,const std::string & attempt);
struct ActivationEvidence {
 bool nav_reset=false,nav_started=false,tracking=false,tf=false,map=false,global_costmap=false,local_costmap=false;
 bool ready()const{return nav_reset&&nav_started&&tracking&&tf&&map&&global_costmap&&local_costmap;}
};
}
