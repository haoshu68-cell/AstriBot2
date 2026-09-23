#pragma once
#include <nlohmann/json.hpp>
#include <optional>
namespace astribot::transport {
std::string canonical_domain(const std::string &raw);
// The lock path must be the same host/domain lock used by legacy transport.
// The state file has one durable JSON record per transition; renewal is volatile.
class ResourceJournal {
public:
 ResourceJournal(const std::string &lock_path,const std::string &state_path);
 ~ResourceJournal();
 ResourceJournal(const ResourceJournal&)=delete;
 ResourceJournal& operator=(const ResourceJournal&)=delete;
 std::optional<nlohmann::json> restored()const{return restored_;}
 void append(const nlohmann::json &);
private:
 int lock_=-1,fd_=-1;bool failed_=false;
 std::optional<nlohmann::json> restored_;
};
}
