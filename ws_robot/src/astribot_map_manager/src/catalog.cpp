#include "astribot_map_manager/catalog.hpp"
#include <openssl/evp.h>
#include <astribot_s1_autonomy/session_archive.hpp>
#include <yaml-cpp/yaml.h>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <fstream>
#include <array>
#include <iomanip>
#include <regex>
#include <cmath>
#include <memory>
namespace astribot_map_manager {
namespace {
void require(bool ok,const char * reason){if(!ok)throw std::runtime_error(reason);}
std::string identifier(const Json & p,const char * key){auto s=p.at(key).get<std::string>();require(std::regex_match(s,std::regex("[A-Za-z0-9_-]{1,96}")),"REQUEST.INVALID_ID");return s;}
Json read(const fs::path & p){require(fs::file_size(p)<=16*1024*1024,"ASSET.METADATA_TOO_LARGE");std::ifstream f(p);return Json::parse(f);}
void durable(const fs::path & path,const std::string & text){
 auto tmp=path.string()+".tmp";int fd=open(tmp.c_str(),O_CREAT|O_TRUNC|O_WRONLY|O_NOFOLLOW,0600);require(fd>=0,"STORE.OPEN_FAILED");
 size_t offset=0;while(offset<text.size()){auto n=write(fd,text.data()+offset,text.size()-offset);if(n<=0){close(fd);throw std::runtime_error("STORE.WRITE_FAILED");}offset+=size_t(n);}
 bool ok=fsync(fd)==0;close(fd);require(ok,"STORE.SYNC_FAILED");fs::rename(tmp,path);
 int dir=open(path.parent_path().c_str(),O_RDONLY|O_DIRECTORY);require(dir>=0,"STORE.DIRECTORY_FAILED");ok=fsync(dir)==0;close(dir);require(ok,"STORE.DIRECTORY_SYNC_FAILED");
}
void pose(const Json & p){for(auto key:{"x","y","yaw"})require(p.at(key).is_number()&&std::isfinite(p.at(key).get<double>()),"STATION.INVALID_POSE");require(p.value("frame","")=="map","STATION.FRAME_MISMATCH");}
void syncDirectory(const fs::path & path){int fd=open(path.c_str(),O_RDONLY|O_DIRECTORY);require(fd>=0,"ASSET.SYNC_FAILED");bool ok=fsync(fd)==0;close(fd);require(ok,"ASSET.SYNC_FAILED");}
void validateDock(const Json & map,const Json & p){
 const fs::path dir=map.at("directory").get<std::string>();const auto yaml=map.at("map_yaml").get<std::string>();
 require(Catalog::hash(dir/yaml)==map.at("sha256").at(yaml),"ASSET.HASH_MISMATCH");
 auto config=YAML::LoadFile((dir/yaml).string());const auto image_name=config["image"].as<std::string>();
 require(Catalog::hash(dir/image_name)==map.at("sha256").at(image_name),"ASSET.HASH_MISMATCH");
 std::ifstream image(dir/config["image"].as<std::string>(),std::ios::binary);std::string header(4096,'\0');image.read(header.data(),header.size());header.resize(image.gcount());std::smatch match;
 require(std::regex_search(header,match,std::regex(R"(^P5\s+(?:#[^\n]*\n\s*)*(\d+)\s+(\d+)\s+255[ \r]?\n)")),"ASSET.INVALID_PGM");
 auto width=std::stoull(match[1]),height=std::stoull(match[2]);double resolution=config["resolution"].as<double>(),angle=config["origin"][2].as<double>();
 double dx=p.at("x").get<double>()-config["origin"][0].as<double>(),dy=p.at("y").get<double>()-config["origin"][1].as<double>();
 double x=std::floor((std::cos(angle)*dx+std::sin(angle)*dy)/resolution),y=std::floor((-std::sin(angle)*dx+std::cos(angle)*dy)/resolution);
 require(x>=0&&y>=0&&x<width&&y<height,"STATION.OUTSIDE_MAP");
 image.clear();image.seekg(match.length()+(height-1-static_cast<uint64_t>(y))*width+static_cast<uint64_t>(x));char pixel=0;image.get(pixel);require(bool(image),"ASSET.INVALID_PGM");
 double occupancy=static_cast<unsigned char>(pixel)/255.0;if(!config["negate"]||config["negate"].as<int>()==0)occupancy=1.0-occupancy;
 double threshold=config["free_thresh"]?config["free_thresh"].as<double>():0.196;require(std::isfinite(threshold)&&threshold>0&&threshold<1,"ASSET.INVALID_THRESHOLD");
 require(occupancy<threshold,"STATION.NOT_KNOWN_FREE");
}
std::string now(){return std::to_string(std::chrono::system_clock::now().time_since_epoch().count());}
}
std::string Catalog::hash(const fs::path & file){
 std::ifstream in(file,std::ios::binary);require(bool(in),"ASSET.READ_FAILED");
 auto ctx=std::unique_ptr<EVP_MD_CTX,decltype(&EVP_MD_CTX_free)>(EVP_MD_CTX_new(),EVP_MD_CTX_free);
 require(ctx&&EVP_DigestInit_ex(ctx.get(),EVP_sha256(),nullptr)==1,"ASSET.HASH_FAILED");std::array<char,65536> buf;
 while(in){in.read(buf.data(),buf.size());require(EVP_DigestUpdate(ctx.get(),buf.data(),in.gcount())==1,"ASSET.HASH_FAILED");}require(in.eof(),"ASSET.READ_FAILED");
 unsigned char digest[EVP_MAX_MD_SIZE];unsigned len=0;require(EVP_DigestFinal_ex(ctx.get(),digest,&len)==1,"ASSET.HASH_FAILED");std::ostringstream out;
 for(unsigned i=0;i<len;++i)out<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(digest[i]);
 return out.str();
}
bool Catalog::blocked(const Json & t){return !t.is_null()&&t.value("state","")!="COMMITTED"&&t.value("state","")!="ABORTED";}
Catalog::Catalog(fs::path root,fs::path imports):root_(fs::absolute(root)),imports_(fs::weakly_canonical(imports)){
 fs::create_directories(root_/"assets");lock_=open((root_/"catalog.lock").c_str(),O_CREAT|O_RDWR|O_NOFOLLOW,0600);
 if(lock_<0||flock(lock_,LOCK_EX|LOCK_NB)!=0){if(lock_>=0)close(lock_);lock_=-1;throw std::runtime_error("STORE.ALREADY_OWNED");}
 try {
 if(fs::exists(root_/"catalog.json"))data_=read(root_/"catalog.json");
 else data_={{"schema_version",1},{"revision",0},{"maps",Json::object()},{"stations",Json::object()},{"active_map",nullptr},{"transaction",nullptr},{"commands",Json::object()},{"history",Json::array()}};
 require(data_.at("schema_version")==1&&data_.at("maps").is_object()&&data_.at("commands").is_object(),"STORE.INVALID_SCHEMA");
 if(blocked(data_.at("transaction"))){auto next=data_;next["transaction"]["state"]="RECOVERY_REQUIRED";next["transaction"]["reason_code"]="RECOVERY.PROCESS_RESTART";commit(next);}
 }catch(...){flock(lock_,LOCK_UN);close(lock_);lock_=-1;throw;}
}
Catalog::~Catalog(){if(lock_>=0){flock(lock_,LOCK_UN);close(lock_);}}
void Catalog::commit(Json next){next["revision"]=data_.at("revision").get<uint64_t>()+1;auto text=next.dump(2);require(text.size()<=16*1024*1024,"STORE.CAPACITY");durable(root_/"catalog.json",text);data_=std::move(next);}
Json Catalog::importMap(const Json & p){
 auto id=identifier(p,"map_id");require(p.at("floor").is_number_integer()&&p.at("floor").get<double>()>=-20&&p.at("floor").get<double>()<=200,"MAP.INVALID_FLOOR");auto directory=fs::canonical(p.at("directory").get<std::string>());auto relative=directory.lexically_relative(imports_);
 require(!relative.empty()&&!relative.is_absolute()&&*relative.begin()!="..","ASSET.OUTSIDE_IMPORT_ROOT");
 require(!fs::is_symlink(directory/"manifest.json"),"ASSET.SYMLINK");
 auto manifest=read(directory/"manifest.json");
 require(manifest.at("format_version")==1&&manifest.at("backend")=="voxel_slam"&&manifest.at("world_frame")=="map","ASSET.FORMAT_UNSUPPORTED");
 require(manifest.value("final_pose_updates",0)>0&&manifest.value("keyframes",0)>0&&manifest.value("scans",0)>0,"MAP.NOT_FINALIZED");
 auto outcome=manifest.value("exploration_outcome","");require(outcome=="COMPLETED"||outcome=="CANCELED_PARTIAL","MAP.NOT_FINALIZED");
 const auto hashes=manifest.at("sha256");require(hashes.is_object()&&hashes.size()>=4&&hashes.size()<100000,"ASSET.INVALID_MANIFEST");
 const auto version=hash(directory/"manifest.json");auto final=root_/"assets"/id/version;auto stage=root_/"assets"/(".import_"+id);
 require(!fs::exists(stage),"ASSET.ORPHAN_IMPORT_REQUIRES_REVIEW");fs::create_directories(stage);
 try {
 uint64_t total=0;
 for(auto it=hashes.begin();it!=hashes.end();++it){fs::path rel=it.key();require(!rel.empty()&&!rel.is_absolute(),"ASSET.INVALID_PATH");for(const auto & part:rel)require(part!=".."&&part!=".","ASSET.INVALID_PATH");
 auto source=directory/rel;auto cursor=directory;for(const auto & part:rel){cursor/=part;require(!fs::is_symlink(cursor),"ASSET.SYMLINK");}
 require(fs::is_regular_file(source),"ASSET.NOT_FILE");total+=fs::file_size(source);require(total<=8ULL*1024*1024*1024,"ASSET.CAPACITY");
 require(hash(source)==it.value().get<std::string>(),"ASSET.HASH_MISMATCH");fs::create_directories((stage/rel).parent_path());fs::copy_file(source,stage/rel);
 require(hash(stage/rel)==it.value().get<std::string>(),"ASSET.COPY_MISMATCH");int fd=open((stage/rel).c_str(),O_RDONLY);require(fd>=0,"ASSET.SYNC_FAILED");bool synced=fsync(fd)==0;close(fd);require(synced,"ASSET.SYNC_FAILED");}
 auto inspection=astribot_s1_autonomy::inspectSession(directory);
 require(inspection.at("sha256")==hashes,"ASSET.MANIFEST_INCOMPLETE");
 auto yaml_name=manifest.at("map_yaml").get<std::string>();require(hashes.contains(yaml_name)&&hashes.contains("alidarState.txt"),"ASSET.MISSING_METADATA");
 auto map=YAML::LoadFile((stage/yaml_name).string());auto image=map["image"].as<std::string>();require(hashes.contains(image)&&!fs::path(image).is_absolute(),"ASSET.IMAGE_UNTRACKED");
 require(std::isfinite(map["resolution"].as<double>())&&map["resolution"].as<double>()>0&&map["origin"].size()==3,"ASSET.INVALID_GEOMETRY");
 for(auto v:map["origin"])require(std::isfinite(v.as<double>()),"ASSET.INVALID_GEOMETRY");
 std::ifstream original(directory/"manifest.json");std::string bytes((std::istreambuf_iterator<char>(original)),{});durable(stage/"manifest.json",bytes);
 require(hash(stage/"manifest.json")==version,"ASSET.COPY_MISMATCH");
 for(const auto & entry:fs::recursive_directory_iterator(stage))if(entry.is_directory())syncDirectory(entry.path());
 syncDirectory(stage);
 fs::create_directories(final.parent_path());if(fs::exists(final))fs::remove_all(stage);else fs::rename(stage,final);
 syncDirectory(final.parent_path());syncDirectory(root_/"assets");
 require(hash(final/"manifest.json")==version,"ASSET.HASH_MISMATCH");
 for(auto it=hashes.begin();it!=hashes.end();++it)require(hash(final/it.key())==it.value(),"ASSET.HASH_MISMATCH");
 Json result={{"map_id",id},{"version",version},{"floor",p.at("floor").get<int>()},{"name",p.value("name",id)},{"directory",final.string()},{"map_yaml",yaml_name},{"outcome",outcome},{"sha256",hashes}};
 require(result.at("name").get<std::string>().size()<=128,"MAP.INVALID_NAME");return result;
 }catch(...){fs::remove_all(stage);throw;}
}
Json Catalog::verifiedMap(const std::string & id)const{
 const auto map=data_.at("maps").at(id);fs::path dir=map.at("directory").get<std::string>();require(hash(dir/"manifest.json")==map.at("version"),"ASSET.HASH_MISMATCH");
 for(auto it=map.at("sha256").begin();it!=map.at("sha256").end();++it)require(hash(dir/it.key())==it.value(),"ASSET.HASH_MISMATCH");
 return map;
}
Json Catalog::command(const std::string & id,const std::string & op,const Json & p,const std::string & actor){
 require(!id.empty()&&id.size()<=128&&actor.size()<=128,"REQUEST.INVALID_ID");const auto signature=Json({op,p,actor}).dump();auto & commands=data_.at("commands");
 if(commands.contains(id)){require(commands.at(id).at("signature")==signature,"REQUEST.CONFLICT");return commands.at(id).at("result");}
 require(commands.size()<4096,"STORE.CAPACITY");require(p.at("expected_revision")==data_.at("revision"),"STATE.REVISION_MISMATCH");
 Json next=data_,result;
 if(op=="map_import"){
 require(!blocked(data_.at("transaction")),"MAP.TRANSACTION_ACTIVE");require(data_.at("maps").size()<32,"STORE.CAPACITY");auto map=importMap(p);auto key=map.at("map_id").get<std::string>();
 require(!data_.at("maps").contains(key)||data_.at("maps").at(key)==map,"MAP.IMMUTABLE_ID");next["maps"][key]=map;result=map;
 }else if(op=="station_put"){
 auto map_id=identifier(p,"map_id"),station_id=identifier(p,"station_id");const auto & map=data_.at("maps").at(map_id);require(p.at("map_version")==map.at("version"),"MAP.VERSION_MISMATCH");pose(p.at("dock_pose"));validateDock(map,p.at("dock_pose"));
 auto kind=p.at("kind").get<std::string>();require(kind=="shelf"||kind=="workstation"||kind=="standby"||kind=="handover","STATION.INVALID_KIND");
 if(p.contains("work_pose")&&!p.at("work_pose").is_null())pose(p.at("work_pose"));
 auto key=map_id+"/"+station_id;require(data_.at("stations").contains(key)||data_.at("stations").size()<128,"STORE.CAPACITY");auto versions=data_.at("stations").value(key,Json::array());require(versions.size()<1000,"STORE.CAPACITY");require(p.at("expected_station_version")==versions.size(),"STATION.VERSION_MISMATCH");
 result={{"station_id",station_id},{"map_id",map_id},{"map_version",map.at("version")},{"version",versions.size()+1},{"kind",kind},{"dock_pose",p.at("dock_pose")},{"work_pose",p.value("work_pose",Json())},{"actor",actor}};versions.push_back(result);next["stations"][key]=versions;
 }else if(op=="map_switch_begin"){
 require(!blocked(data_.at("transaction")),"MAP.TRANSACTION_ACTIVE");auto target=verifiedMap(identifier(p,"map_id"));require(p.at("map_version")==target.at("version"),"MAP.VERSION_MISMATCH");
 const bool transfer=p.value("manual_transfer",false);if(!data_.at("active_map").is_null())require(data_.at("active_map").at("floor")==target.at("floor")||transfer,"MAP.TRANSFER_REQUIRED");
 result={{"transaction_id",id},{"target",target},{"previous",data_.at("active_map")},{"state",transfer?"WAIT_TRANSFER":"LOAD_INTENT"},{"actor",actor},{"created_at",now()},{"manual_transfer",transfer},{"reason_code","MAP.INTENT_RECORDED"}};next["transaction"]=result;
 }else if(op=="map_transfer_confirm"||op=="map_abort"||op=="map_recover"){
 auto & t=next.at("transaction");require(!t.is_null()&&p.at("transaction_id")==t.at("transaction_id"),"MAP.TRANSACTION_MISMATCH");
 if(op=="map_abort"){require(t.at("state")=="WAIT_TRANSFER","MAP.CANNOT_ABORT_AFTER_TRANSFER");t["state"]="ABORTED";t["reason_code"]="MAP.ABORTED_BEFORE_LOAD";}
 else if(op=="map_recover"){require(t.at("state")=="RECOVERY_REQUIRED","RECOVERY.NOT_REQUIRED");verifiedMap(t.at("target").at("map_id"));t["state"]="LOAD_INTENT";t["reason_code"]="RECOVERY.EXPLICIT_REVALIDATION";}
 else{require(t.at("state")=="WAIT_TRANSFER","MAP.NOT_WAITING_TRANSFER");t["state"]="LOAD_INTENT";t["confirmed_by"]=actor;t["confirmed_at"]=now();}
 result=t;
 }else throw std::runtime_error("REQUEST.UNSUPPORTED");
 next["commands"][id]={{"signature",signature},{"result",result}};next["history"].push_back({{"command_id",id},{"operation",op},{"actor",actor},{"at",now()}});commit(next);return result;
}
void Catalog::transition(const std::string & tx,const std::string & state,const std::string & reason){
 auto next=data_;auto & t=next.at("transaction");require(!t.is_null()&&t.at("transaction_id")==tx,"MAP.TRANSACTION_MISMATCH");
 auto from=t.at("state").get<std::string>();require((state=="LOADING"&&from=="LOAD_INTENT")||(state=="COMMITTED"&&from=="LOADING")||(state=="RECOVERY_REQUIRED"&&blocked(t)),"MAP.INVALID_TRANSITION");
 t["state"]=state;t["reason_code"]=reason;t["updated_at"]=now();if(state=="COMMITTED")next["active_map"]=t.at("target");
 next["history"].push_back({{"transaction_id",tx},{"state",state},{"reason_code",reason},{"at",now()}});commit(next);
}
}
