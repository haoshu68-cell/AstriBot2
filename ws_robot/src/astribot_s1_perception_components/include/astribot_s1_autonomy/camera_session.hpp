#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <string>
#include <utility>
namespace astribot::vision {
// Cooperative perception resource ownership. This is not an actuation permit.
class CameraSession {
public:
 explicit CameraSession(std::string boot):boot_(std::move(boot)){}
 void tick(int64_t ros,int64_t wall){
  if(last_ros_>0 && ros<last_ros_)clear("CLOCK_ROLLBACK");
  else if(active_ && (ros>=deadline_ || wall>=wall_deadline_))clear("LEASE_EXPIRED");
  last_ros_=ros;
 }
 bool request(uint8_t op,const std::string& owner,const std::string& execution,
              const std::string& id,const std::string& token,double lease,
              int64_t stamp,int64_t ros,int64_t wall){
  tick(ros,wall);
  auto identity=[](const auto& s){return !s.empty()&&s.size()<=128;};
  if(op>2 || !identity(owner)||!identity(execution)||!identity(id))return reject("INVALID_REQUEST");
  if(ros<=0 || stamp<=0 || ros-stamp>250000000)return reject("REQUEST_EXPIRED");
  if(stamp>ros)return reject("REQUEST_CLOCK_AHEAD");
  if(op!=2 && (!std::isfinite(lease)||lease<=0||lease>2.))return reject("INVALID_LEASE");
  const bool used=std::find(ids_.begin(),ids_.end(),id)!=ids_.end();
  if(used){
   if(op==0&&active_&&id==acquire_id_&&owner==owner_&&execution==execution_&&token.empty()&&lease==acquire_lease_){reason_="ACQUIRE_DUPLICATE";return true;}
   return reject("REQUEST_REPLAYED");
  }
  if(op==0){
   if(active_)return reject("CAMERA_BUSY");
   if(!token.empty())return reject("TOKEN_NOT_EMPTY");
   owner_=owner;execution_=execution;token_=boot_+":"+std::to_string(++generation_);
   acquire_id_=id;acquire_lease_=lease;activated_=ros;active_=true;
  }else if(!active_ || owner!=owner_ || execution!=execution_ || token!=token_){return reject("SESSION_MISMATCH");}
  else if(stamp<last_request_stamp_)return reject("REQUEST_REORDERED");
  ids_.push_back(id);if(ids_.size()>64)ids_.pop_front();last_request_stamp_=stamp;
  if(op==2){clear("RELEASED");return true;}
  const auto delta=static_cast<int64_t>(lease*1e9);
  deadline_=ros+delta;wall_deadline_=wall+delta;reason_="ACTIVE";return true;
 }
 bool active()const{return active_;}
 const std::string& token()const{return token_;}
 const std::string& owner()const{return owner_;}
 const std::string& execution()const{return execution_;}
 const std::string& reason()const{return reason_;}
 int64_t deadline()const{return deadline_;}
 int64_t activated()const{return activated_;}
private:
 bool reject(const char* reason){reason_=reason;return false;}
 void clear(const char* reason){active_=false;deadline_=wall_deadline_=0;reason_=reason;}
 std::string boot_,token_,owner_,execution_,acquire_id_,reason_="INACTIVE";
 std::deque<std::string> ids_;double acquire_lease_=0;
 uint64_t generation_=0;bool active_=false;
 int64_t deadline_=0,wall_deadline_=0,last_ros_=0,activated_=0,last_request_stamp_=0;
};
}
