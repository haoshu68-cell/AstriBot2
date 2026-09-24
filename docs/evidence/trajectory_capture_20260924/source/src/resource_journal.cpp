#include "astribot_s1_transport_native/resource_journal.hpp"
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <filesystem>
#include <stdexcept>
namespace astribot::transport {
namespace {
constexpr off_t CAPACITY=4*1024*1024;
void check_file(int fd){struct stat s{};if(fd<0||fstat(fd,&s)||!S_ISREG(s.st_mode)||s.st_uid!=geteuid()||s.st_nlink!=1)throw std::runtime_error("RESOURCE_FILE_INVALID");}
}
std::string canonical_domain(const std::string &raw) {
 if(raw.empty()||raw.size()>3||raw.find_first_not_of("0123456789")!=std::string::npos||std::stoi(raw)>232||raw!=std::to_string(std::stoi(raw)))throw std::invalid_argument("NONCANONICAL_ROS_DOMAIN");
 return raw;
}
ResourceJournal::ResourceJournal(const std::string &lock_path,const std::string &state_path) {
 try {
  lock_=open(lock_path.c_str(),O_RDWR|O_CREAT|O_CLOEXEC|O_NOFOLLOW,0600);check_file(lock_);
  if(flock(lock_,LOCK_EX|LOCK_NB))throw std::runtime_error("RESOURCES_BUSY");
  fd_=open(state_path.c_str(),O_RDWR|O_CREAT|O_APPEND|O_CLOEXEC|O_NOFOLLOW,0600);check_file(fd_);
  if(flock(fd_,LOCK_EX|LOCK_NB))throw std::runtime_error("RESOURCE_JOURNAL_BUSY");
  struct stat s{};if(fstat(fd_,&s)||s.st_size>CAPACITY)throw std::runtime_error("RESOURCE_JOURNAL_CAPACITY");
  std::string contents(static_cast<size_t>(s.st_size),'\0');size_t at=0;
  while(at<contents.size()){auto n=pread(fd_,contents.data()+at,contents.size()-at,at);if(n<0&&errno==EINTR)continue;if(n<=0)throw std::runtime_error("RESOURCE_JOURNAL_READ");at+=n;}
  if(!contents.empty()&&contents.back()!='\n')throw std::runtime_error("RESOURCE_JOURNAL_INCOMPLETE");
  size_t start=0;
  while(start<contents.size()){auto end=contents.find('\n',start);if(end-start>65536)throw std::runtime_error("RESOURCE_JOURNAL_RECORD_TOO_LARGE");restored_=nlohmann::json::parse(contents.substr(start,end-start));start=end+1;}
  // Both runtime generations share this recovery marker, as well as flock.
  // Existing native history cannot hide a later crashed legacy executor.
  struct stat legacy{};if(fstat(lock_,&legacy)||legacy.st_size>65536)throw std::runtime_error("LEGACY_RESOURCE_RECOVERY_REQUIRED");
  if(!legacy.st_size&&restored_)throw std::runtime_error("RESOURCE_MARKER_MISSING");
  if(legacy.st_size) {
   std::string prior(static_cast<size_t>(legacy.st_size),'\0');
   if(pread(lock_,prior.data(),prior.size(),0)!=static_cast<ssize_t>(prior.size()))throw std::runtime_error("LEGACY_RESOURCE_RECOVERY_REQUIRED");
   auto marker=nlohmann::json::parse(prior);
   if(!marker.contains("unconfirmed_executor")||!marker["unconfirmed_executor"].is_boolean())throw std::runtime_error("LEGACY_RESOURCE_RECOVERY_REQUIRED");
   if(marker["unconfirmed_executor"].get<bool>()) {
    if(!restored_||marker.value("native_authority","")!="astribot.resource/1"||marker.value("native_epoch","")!=restored_->value("epoch","")||marker.value("native_lease_id","")!=restored_->value("lease_id",""))throw std::runtime_error("LEGACY_RESOURCE_RECOVERY_REQUIRED");
    // A crash between the journal handoff and marker sync must stay blocked.
    if(restored_->at("phase")==0)(*restored_)["phase"]=5;
   }
  }
  auto parent=std::filesystem::path(state_path).parent_path();if(parent.empty())parent=".";
  int directory=open(parent.c_str(),O_RDONLY|O_DIRECTORY|O_CLOEXEC);if(directory<0)throw std::runtime_error("RESOURCE_DIRECTORY_UNAVAILABLE");
  int result=fsync(directory);close(directory);if(result)throw std::runtime_error("RESOURCE_DIRECTORY_SYNC");
 }catch(...){if(fd_>=0)close(fd_);if(lock_>=0)close(lock_);fd_=lock_=-1;throw;}
}
ResourceJournal::~ResourceJournal(){if(fd_>=0)close(fd_);if(lock_>=0)close(lock_);}
void ResourceJournal::append(const nlohmann::json &record) {
 if(failed_)throw std::runtime_error("RESOURCE_JOURNAL_PREVIOUS_FAILURE");
 try {
  auto line=record.dump()+"\n";struct stat s{};
  if(line.size()>65536||fstat(fd_,&s)||s.st_size+static_cast<off_t>(line.size())>CAPACITY)throw std::runtime_error("RESOURCE_JOURNAL_CAPACITY");
  size_t at=0;while(at<line.size()){auto n=write(fd_,line.data()+at,line.size()-at);if(n<0&&errno==EINTR)continue;if(n<=0)throw std::runtime_error("RESOURCE_JOURNAL_WRITE");at+=n;}
  if(fdatasync(fd_))throw std::runtime_error("RESOURCE_JOURNAL_SYNC");
  auto marker=nlohmann::json({{"unconfirmed_executor",record.at("phase")!=0},
    {"native_authority","astribot.resource/1"},{"native_epoch",record.value("epoch","")},
    {"native_lease_id",record.value("lease_id","")}}).dump();
  // The legacy reader is under the same lock. A partial marker after a crash
  // parses as an error and cannot authorize a new legacy task.
  size_t mark_at=0;while(mark_at<marker.size()){auto n=pwrite(lock_,marker.data()+mark_at,marker.size()-mark_at,mark_at);if(n<0&&errno==EINTR)continue;if(n<=0)throw std::runtime_error("RESOURCE_MARKER_WRITE");mark_at+=n;}
  if(fdatasync(lock_))throw std::runtime_error("RESOURCE_MARKER_SYNC");
  if(ftruncate(lock_,marker.size())||fdatasync(lock_))throw std::runtime_error("RESOURCE_MARKER_TRUNCATE_SYNC");
  restored_=record;
 }catch(...){failed_=true;throw;}
}
}
