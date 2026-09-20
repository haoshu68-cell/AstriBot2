// Optional, read-only Gazebo contact evidence. Never writes pose, force or commands.
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/contacts.pb.h>
#include <map>
#include <set>
#include <vector>
namespace sim=ignition::gazebo;
namespace astribot {
class ContactEvidence: public sim::System, public sim::ISystemConfigure,
    public sim::ISystemPreUpdate, public sim::ISystemPostUpdate {
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher pub_;
  std::set<sim::Entity> observed_;
  std::map<std::pair<uint64_t,uint64_t>,ignition::msgs::Contact> pending_;
  std::chrono::steady_clock::duration last_{0};
public:
  void Configure(const sim::Entity&,const std::shared_ptr<const sdf::Element>&,
      sim::EntityComponentManager&,sim::EventManager&) override {
    pub_=node_.Advertise<ignition::msgs::Contacts>("/simulation/left_arm_contacts");
  }
  void PreUpdate(const sim::UpdateInfo&,sim::EntityComponentManager& ecm) override {
    std::vector<sim::Entity> add;
    ecm.Each<sim::components::Collision>([&](const sim::Entity& e,const auto*) {
      const auto name=sim::scopedName(e,ecm,"::",false);
      if((name.find("astribot_s1::astribot_arm_left_")!=std::string::npos ||
          name.find("astribot_s1::astribot_gripper_left_")!=std::string::npos) &&
          !observed_.count(e))add.push_back(e);
      return true;
    });
    for(auto e:add) {
      if(!ecm.Component<sim::components::ContactSensorData>(e))
        ecm.CreateComponent(e,sim::components::ContactSensorData());
      observed_.insert(e);
    }
  }
  void PostUpdate(const sim::UpdateInfo& info,const sim::EntityComponentManager& ecm) override {
    if(info.paused)return;
    if(info.simTime<last_){pending_.clear();last_=info.simTime;}
    const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    for(auto e:observed_) {
      const auto* data=ecm.Component<sim::components::ContactSensorData>(e);
      if(!data)continue;
      for(const auto& raw:data->Data().contact()) {
        auto a=raw.collision1().id(),b=raw.collision2().id();
        if(a>b)std::swap(a,b);
        auto& c=pending_[{a,b}];c=raw;
        c.mutable_collision1()->set_name(sim::scopedName(raw.collision1().id(),ecm,"::",false));
        c.mutable_collision2()->set_name(sim::scopedName(raw.collision2().id(),ecm,"::",false));
        c.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);
        c.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
      }
    }
    if(info.simTime-last_<std::chrono::milliseconds(10))return;
    last_=info.simTime;ignition::msgs::Contacts message;
    message.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);
    message.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
    auto* coverage=message.mutable_header()->add_data();
    coverage->set_key("observed_collision_count");
    coverage->add_value(std::to_string(observed_.size()));
    for(const auto& item:pending_)*message.add_contact()=item.second;
    pub_.Publish(message);pending_.clear();
  }
};
}
IGNITION_ADD_PLUGIN(astribot::ContactEvidence,sim::System,
    astribot::ContactEvidence::ISystemConfigure,astribot::ContactEvidence::ISystemPreUpdate,
    astribot::ContactEvidence::ISystemPostUpdate)
