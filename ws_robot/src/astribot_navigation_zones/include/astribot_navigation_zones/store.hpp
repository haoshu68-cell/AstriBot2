#pragma once
#include "astribot_navigation_zones/geometry.hpp"
#include <filesystem>
namespace astribot_navigation_zones {
std::filesystem::path defaultStore();
class Store {
 std::filesystem::path root_;int lock_{-1};Json data_;
 void commit(Json value);
public:
 explicit Store(std::filesystem::path root);~Store();
 Store(const Store&)=delete;Store& operator=(const Store&)=delete;
 const Json & data()const{return data_;}
 Json context(const std::string &)const;
 void ensure(const std::string & context,const std::string & directory="",const std::string & source_directory="",const Json & archived=Json());
 Json replace(const std::string & command,const std::string & context,uint64_t expected,
   const Json & regions,const std::string & actor,bool reviewed);
};
}
