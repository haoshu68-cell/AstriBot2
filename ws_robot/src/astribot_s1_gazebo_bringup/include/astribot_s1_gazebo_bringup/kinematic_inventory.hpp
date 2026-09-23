#pragma once
#include "astribot_s1_gazebo_bringup/inventory_gate.hpp"
#include <ignition/math/Pose3.hh>
#include <optional>
#include <ignition/gazebo/EntityComponentManager.hh>

namespace astribot::simulation {
struct PayloadExecution {
  std::string epoch,parent_model,parent_link,error;
  uint64_t parent=0,accepted=0,applied=0,clock_epoch=0;
  int64_t capture=-1;
  bool attached=false,pending=false;
  ignition::math::Pose3d offset;
};
// ECM component access is implemented once in a shared library linked by both systems.
void write_execution(uint64_t model,ignition::gazebo::EntityComponentManager &,const PayloadExecution &);
std::optional<PayloadExecution> read_execution(uint64_t model,const ignition::gazebo::EntityComponentManager &);
std::string kinematic_inventory_reason(const InventorySnapshot &,const std::map<uint64_t,PayloadExecution> &,int64_t capture);
}
