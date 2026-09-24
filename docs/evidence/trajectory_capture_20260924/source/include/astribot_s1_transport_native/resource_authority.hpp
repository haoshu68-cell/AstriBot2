#pragma once
#include "astribot_s1_transport_native/arm_hold.hpp"
#include <nlohmann/json.hpp>
#include <functional>
#include <map>
namespace astribot::transport {
enum class ResourcePhase { IDLE, RESERVED, EXECUTING, HOLDING, STOPPING, QUARANTINED };
struct Acquisition {bool accepted=false,duplicate=false;std::string reason;};
// One cooperative task authority. The runtime must own the domain-wide process
// lock and route real child results here; ROS identity strings are not DDS ACLs.
class ResourceAuthority {
public:
 using Commit=std::function<void(const nlohmann::json &)>;
 ResourceAuthority(std::string epoch,std::set<std::string> joints,Commit,
                   std::optional<nlohmann::json> restored=std::nullopt);
 Acquisition acquire(const std::string &task,const std::string &request,int64_t now,int64_t steady);
 bool renew(const std::string &lease,const std::string &epoch,uint64_t sequence,int64_t now,int64_t steady);
 std::optional<ResourceGrant> grant(int64_t now,int64_t steady);
 void submitted(int64_t now,int64_t steady);
 void holding(int64_t now,int64_t steady);
 bool record(const std::string &event,const nlohmann::json &details);
 void stop(const std::string &reason,int64_t now,int64_t steady);
 void uncertain_result(const std::string &reason,int64_t now,int64_t steady);
 bool release(bool all_children_terminal,bool measured_safe,int64_t now,int64_t steady);
 ResourcePhase phase()const{return phase_;}
 const std::string &reason()const{return reason_;}
 const std::string &lease_id()const{return lease_.lease_id;}
 const std::string &epoch()const{return epoch_;}
private:
 bool clocks(int64_t now,int64_t steady);
 bool persist(const std::string &event);
 void quarantine(const std::string &reason);
 nlohmann::json snapshot(const std::string &event)const;
 std::string epoch_,request_,reason_="IDLE";std::set<std::string> joints_;
 Commit commit_;ResourcePhase phase_=ResourcePhase::IDLE;ResourceGrant lease_;
 std::set<std::pair<std::string,std::string>> requests_;
 uint64_t number_=0,renewal_=0;bool side_effects_=false,storage_failed_=false;
 int64_t last_ros_=-1,last_steady_=-1;
};
}
