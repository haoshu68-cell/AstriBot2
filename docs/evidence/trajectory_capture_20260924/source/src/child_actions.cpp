#include "astribot_s1_transport_native/child_actions.hpp"
#include <stdexcept>
namespace astribot::transport {
std::string canonical_goal_id(const std::array<uint8_t,16> &id) {
 static constexpr char hex[]="0123456789abcdef";
 std::string result;result.reserve(32);
 for(auto byte:id){result.push_back(hex[byte>>4]);result.push_back(hex[byte&15]);}
 return result;
}
ChildActions::ChildActions(const std::vector<std::string> &names) {
 if(names.empty()||names.size()>32)throw std::invalid_argument("INVALID_CHILD_COUNT");
 for(const auto &n:names)if(n.empty()||!children_.emplace(n,Child{}).second)throw std::invalid_argument("INVALID_CHILD_ID");
}
void ChildActions::submitted(const std::string &name) {
 auto &c=children_.at(name);if(c.state!=State::UNSENT)throw std::logic_error("CHILD_ALREADY_SUBMITTED");c.state=State::PENDING;
}
void ChildActions::response(const std::string &name,const std::string &uuid) {
 auto &c=children_.at(name);if(c.state!=State::PENDING)throw std::logic_error("UNEXPECTED_CHILD_RESPONSE");
 c.uuid=uuid;c.state=uuid.empty()?State::REJECTED:State::ACCEPTED;
}
void ChildActions::result(const std::string &name,const std::string &uuid,bool success,bool terminal) {
 if(!terminal)throw std::logic_error("CHILD_RESULT_UNKNOWN");
 auto &c=children_.at(name);if(c.state!=State::ACCEPTED||uuid.empty()||uuid!=c.uuid)throw std::logic_error("CHILD_RESULT_IDENTITY_MISMATCH");
 c.state=success?State::SUCCEEDED:State::FAILED;
}
bool ChildActions::all_terminal()const {for(const auto &[n,c]:children_)if(c.state==State::PENDING||c.state==State::ACCEPTED)return false;return true;}
bool ChildActions::all_successful()const {for(const auto &[n,c]:children_)if(c.state!=State::SUCCEEDED)return false;return true;}
bool ChildActions::failed()const {for(const auto &[n,c]:children_)if(c.state==State::REJECTED||c.state==State::FAILED)return true;return false;}
bool ChildActions::sent()const {for(const auto &[n,c]:children_)if(c.state!=State::UNSENT)return true;return false;}
std::string ChildActions::proof()const {if(!all_successful())throw std::logic_error("CHILDREN_NOT_SUCCESSFUL");std::string s;for(const auto &[n,c]:children_)s+=n+":"+c.uuid+";";return s;}
}
