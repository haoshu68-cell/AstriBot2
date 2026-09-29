#pragma once
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
namespace astribot_map_manager {
// An explicit parameter always wins; an empty parameter selects durable user data.
inline std::string catalogStorageRoot(const std::string & configured) {
 namespace fs=std::filesystem;
 if(!configured.empty())return fs::absolute(configured).lexically_normal().string();
 const char * xdg=std::getenv("XDG_DATA_HOME");
 if(xdg&&fs::path(xdg).is_absolute())return (fs::path(xdg)/"astribot/map_catalog").string();
 const char * home=std::getenv("HOME");
 if(!home||!fs::path(home).is_absolute())throw std::runtime_error("STORE.NO_USER_DATA_ROOT");
 return (fs::path(home)/".local/share/astribot/map_catalog").string();
}
inline std::string catalogAssetRoot(const std::string & configured) {
 return configured.empty()?(std::filesystem::path(catalogStorageRoot(""))/"assets").string():std::filesystem::absolute(configured).lexically_normal().string();
}
}
