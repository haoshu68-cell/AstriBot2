#include "astribot_s1_payload_state/ledger.hpp"
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <filesystem>
#include <cerrno>
#include <stdexcept>
namespace astribot::payload {
Journal::Journal(const std::string &path) {
  fd_=::open(path.c_str(),O_RDWR|O_CREAT|O_APPEND|O_CLOEXEC|O_NOFOLLOW,0600);
  if(fd_<0)throw std::runtime_error("cannot open payload journal");
  try {
    if(::flock(fd_,LOCK_EX|LOCK_NB)!=0)throw std::runtime_error("payload journal already owned");
    struct stat info{};
    if(::fstat(fd_,&info)!=0 || !S_ISREG(info.st_mode))throw std::runtime_error("payload journal is not a regular file");
    if(info.st_size) {char last=0;if(::pread(fd_,&last,1,info.st_size-1)!=1 || last!='\n')throw std::runtime_error("payload journal has incomplete tail; inspection required");}
    auto parent=std::filesystem::path(path).parent_path();if(parent.empty())parent=".";
    const int directory=::open(parent.c_str(),O_RDONLY|O_DIRECTORY|O_CLOEXEC);
    if(directory<0)throw std::runtime_error("payload journal parent unavailable");
    const int result=::fsync(directory);::close(directory);
    if(result!=0)throw std::runtime_error("cannot persist payload journal directory");
  }catch(...) {::close(fd_);fd_=-1;throw;}
}
Journal::~Journal() {if(fd_>=0)::close(fd_);}
void Journal::append(const nlohmann::json &record) {
  if(failed_)throw std::runtime_error("payload journal previously failed");
  const auto line=record.dump()+"\n";
  if(line.size()>1024*1024)throw std::runtime_error("payload journal record too large");
  struct stat info{};
  if(::fstat(fd_,&info)!=0 || info.st_size>200LL*1024*1024) {failed_=true;throw std::runtime_error("payload journal capacity exhausted");}
  std::size_t offset=0;
  while(offset<line.size()) {
    const auto n=::write(fd_,line.data()+offset,line.size()-offset);
    if(n<0 && errno==EINTR)continue;
    if(n<=0) {failed_=true;throw std::runtime_error("payload journal write failed");}
    offset+=static_cast<std::size_t>(n);
  }
  if(::fdatasync(fd_)!=0) {failed_=true;throw std::runtime_error("payload journal sync failed");}
}
} // namespace astribot::payload
