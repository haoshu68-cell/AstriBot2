#pragma once
#include <map>
#include <array>
#include <cstdint>
#include <string>
#include <vector>
namespace astribot::transport {
std::string canonical_goal_id(const std::array<uint8_t,16> &id);
// Admission to a child is durable before submission. A pending send may have
// reached the server even when the client has not received its goal handle.
class ChildActions {
public:
 enum class State { UNSENT, PENDING, ACCEPTED, REJECTED, SUCCEEDED, FAILED };
 explicit ChildActions(const std::vector<std::string> &names);
 void submitted(const std::string &name);
 void response(const std::string &name,const std::string &uuid);
 void result(const std::string &name,const std::string &uuid,bool success,bool terminal=true);
 bool all_terminal()const;
 bool all_successful()const;
 bool failed()const;
 bool sent()const;
 std::string proof()const;
private:
 struct Child {State state=State::UNSENT;std::string uuid;};
 std::map<std::string,Child> children_;
};
}
