#pragma once
#include <nlohmann/json.hpp>
#include <filesystem>
#include <string>
namespace astribot_map_manager {
using Json=nlohmann::json;
namespace fs=std::filesystem;
class Catalog {
 fs::path root_,imports_;int lock_{-1};Json data_;
 void commit(Json next);
 Json importMap(const Json & p);
public:
 Catalog(fs::path root,fs::path imports);~Catalog();
 Catalog(const Catalog &)=delete;Catalog & operator=(const Catalog &)=delete;
 const Json & snapshot()const{return data_;}
 Json command(const std::string & id,const std::string & op,const Json & payload,const std::string & actor);
 void transition(const std::string & tx,const std::string & state,const std::string & reason);
 Json verifiedMap(const std::string & id)const;
 static bool blocked(const Json & transaction);
 static std::string hash(const fs::path & file);
};
}
