#include "astribot_navigation_zones/store.hpp"
#include <fstream>
#include <cstdlib>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
namespace astribot_navigation_zones {
namespace fs=std::filesystem;
std::filesystem::path defaultStore(){const char * xdg=std::getenv("XDG_DATA_HOME");if(xdg&&fs::path(xdg).is_absolute())return fs::path(xdg)/"astribot/map_catalog/navigation_zones";const char *home=std::getenv("HOME");if(!home||!fs::path(home).is_absolute())throw std::runtime_error("ZONES.NO_DATA_ROOT");return fs::path(home)/".local/share/astribot/map_catalog/navigation_zones";}
Store::Store(fs::path root):root_(fs::absolute(root)){
 fs::create_directories(root_);lock_=open((root_/"zones.lock").c_str(),O_CREAT|O_RDWR|O_NOFOLLOW,0600);
 if(lock_<0||flock(lock_,LOCK_EX|LOCK_NB)){if(lock_>=0)close(lock_);lock_=-1;throw std::runtime_error("ZONES.STORE_OWNED");}
 try{auto path=root_/"zones.json";if(fs::exists(path)){if(fs::file_size(path)>16*1024*1024)throw std::runtime_error("ZONES.STORE_TOO_LARGE");std::ifstream f(path);f>>data_;if(data_.at("schema_version")!=1||!data_.at("contexts").is_object()||!data_.at("commands").is_object())throw std::runtime_error("ZONES.INVALID_STORE");for(const auto &c:data_.at("contexts"))parseRegions(c.at("regions"));}
 else data_={{"schema_version",1},{"contexts",Json::object()},{"commands",Json::object()}};
 }catch(...){close(lock_);lock_=-1;throw;}
}
Store::~Store(){if(lock_>=0){flock(lock_,LOCK_UN);close(lock_);}}
void Store::commit(Json next){
 auto text=next.dump(2);if(text.size()>16*1024*1024)throw std::runtime_error("ZONES.STORE_CAPACITY");auto tmp=root_/"zones.json.tmp";int fd=open(tmp.c_str(),O_CREAT|O_TRUNC|O_WRONLY|O_NOFOLLOW,0600);if(fd<0)throw std::runtime_error("ZONES.STORE_OPEN");
 size_t pos=0;while(pos<text.size()){auto n=write(fd,text.data()+pos,text.size()-pos);if(n<=0){close(fd);throw std::runtime_error("ZONES.STORE_WRITE");}pos+=n;}bool ok=fsync(fd)==0;close(fd);if(!ok)throw std::runtime_error("ZONES.STORE_SYNC");fs::rename(tmp,root_/"zones.json");fd=open(root_.c_str(),O_RDONLY|O_DIRECTORY);if(fd<0)throw std::runtime_error("ZONES.STORE_SYNC");ok=fsync(fd)==0;close(fd);if(!ok)throw std::runtime_error("ZONES.STORE_SYNC");data_=std::move(next);
}
Json Store::context(const std::string &key)const{return data_.at("contexts").value(key,Json{{"revision",0},{"regions",Json::array()},{"review_required",false},{"history",Json::array()},{"directory",""}});}
void Store::ensure(const std::string&key,const std::string&directory,const std::string&source,const Json&archived){
 if(key.empty()||key.size()>256)throw std::runtime_error("ZONES.INVALID_CONTEXT");auto c=context(key);
 if(data_.at("contexts").contains(key)){if(directory.empty()||c.value("directory","")==directory)return;c["directory"]=directory;}
 else{if(data_.at("contexts").size()>=128)throw std::runtime_error("ZONES.STORE_CAPACITY");c["directory"]=directory;
 if(!source.empty())for(auto it=data_.at("contexts").begin();it!=data_.at("contexts").end();++it)if(it.key().rfind("mapping:",0)==0&&it.value().value("directory","")==source&&!it.value().at("regions").empty()){
 c["revision"]=1;c["regions"]=it.value().at("regions");c["review_required"]=true;c["source_context"]=it.key();break;}
 if(!archived.is_null()){
   if(archived.at("schema_version")!=1||archived.at("frame")!="map")throw std::runtime_error("ZONES.INVALID_ARCHIVE");
   c["regions"]=encodeRegions(parseRegions(archived.at("regions")));c["revision"]=1;c["review_required"]=!c.at("regions").empty();c["source_context"]=archived.at("context_id");
 }}
 auto next=data_;next["contexts"][key]=c;commit(next);
}
Json Store::replace(const std::string&id,const std::string&key,uint64_t expected,const Json&regions,const std::string&actor,bool reviewed){
 if(id.empty()||id.size()>128||actor.empty()||actor.size()>128)throw std::runtime_error("ZONES.INVALID_ACTOR_OR_ID");const auto canonical=encodeRegions(parseRegions(regions));const auto signature=Json({key,expected,canonical,actor,reviewed}).dump();
 if(data_.at("commands").contains(id)){const auto & prior=data_.at("commands").at(id);if(prior.at("signature")!=signature)throw std::runtime_error("REQUEST.CONFLICT");return prior.at("result");}
 auto c=context(key);if(!data_.at("contexts").contains(key)||c.at("revision")!=expected)throw std::runtime_error("ZONES.VERSION_MISMATCH");
 if(!reviewed)throw std::runtime_error("ZONES.REVIEW_REQUIRED");if(c.at("history").size()>=100||data_.at("commands").size()>=4096)throw std::runtime_error("ZONES.STORE_CAPACITY");
 c["history"].push_back({{"revision",c.at("revision")},{"regions",c.at("regions")}});c["revision"]=expected+1;c["regions"]=canonical;c["review_required"]=false;c["actor"]=actor;c["command_id"]=id;
 auto result=c;result.erase("history");auto next=data_;next["contexts"][key]=c;next["commands"][id]={{"signature",signature},{"result",result}};commit(next);return result;
}
}
