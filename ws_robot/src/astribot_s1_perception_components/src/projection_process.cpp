#include "astribot_s1_perception_components/projection_process.hpp"
#include "astribot_s1_perception_components/projection_wire.hpp"
#include <cerrno>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <poll.h>
#include <random>
#include <signal.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
extern char** environ;
namespace astribot::vision {
namespace {
using Clock=std::chrono::steady_clock;
auto after(double sec){return Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(sec));}
struct Failure:std::runtime_error {using std::runtime_error::runtime_error;};
}
struct ProjectionProcess::Impl {
  ProjectionProcessOptions options;
  mutable std::mutex status_mutex;
  ProjectionProcessStatus status_;
  std::atomic<bool> canceled{false};
  int socket{-1},pidfd{-1};pid_t pid{-1};
  uint64_t boot{0},epoch{0},request{0};uint32_t starts{0};
  Clock::time_point next_start{};
  explicit Impl(ProjectionProcessOptions o):options(std::move(o)) {
    const auto bounded=[](double t,double max){return std::isfinite(t)&&t>0&&t<=max;};
    if(options.executable.empty()||options.executable.front()!='/'||
       !bounded(options.startup_timeout_sec,30)||!bounded(options.request_timeout_sec,5)||
       !bounded(options.terminate_timeout_sec,1)||!bounded(options.kill_timeout_sec,1)||
       !std::isfinite(options.restart_backoff_sec)||options.restart_backoff_sec<0||options.restart_backoff_sec>30||
       options.max_restarts>10)throw std::invalid_argument("invalid projection worker policy");
    std::random_device random;boot=(uint64_t(random())<<32)|random();if(!boot)boot=1;
  }
  void state(const std::string& value,const std::string& reason) {
    std::lock_guard<std::mutex> lock(status_mutex);
    status_={value,reason,epoch,request,starts?starts-1:0,pid};
  }
  ProjectionProcessStatus status()const {std::lock_guard<std::mutex> lock(status_mutex);return status_;}
  void transfer(void* buffer,size_t bytes,bool writing,Clock::time_point deadline,const char* timeout) {
    auto* current=static_cast<char*>(buffer);
    while(bytes) {
      if(canceled.load())throw Failure("WORKER_CANCELED");
      if(Clock::now()>=deadline)throw Failure(timeout);
      const auto n=writing?send(socket,current,bytes,MSG_NOSIGNAL):recv(socket,current,bytes,0);
      if(n>0){current+=n;bytes-=size_t(n);continue;}
      if(n==0)throw Failure("WORKER_EXITED");
      if(errno==EINTR)continue;
      if(errno!=EAGAIN&&errno!=EWOULDBLOCK)throw Failure("WORKER_IO_FAILED");
      pollfd p{socket,short(writing?POLLOUT:POLLIN),0};
      // A cancellation never has to wait for the overall startup deadline.
      const int rc=poll(&p,1,2);
      if(rc<0&&errno!=EINTR)throw Failure("WORKER_IO_FAILED");
    }
    if(canceled.load())throw Failure("WORKER_CANCELED");
    if(Clock::now()>=deadline)throw Failure(timeout);
  }
  bool reap_until(Clock::time_point deadline)noexcept {
    do {
      int result=0;const auto child=waitpid(pid,&result,WNOHANG);
      if(child==pid)return true;
      if(child<0&&errno==ECHILD) {
        pollfd fd{pidfd,POLLIN,0};
        return pidfd>=0&&poll(&fd,1,0)==1&&(fd.revents&POLLIN);
      }
      if(child<0&&errno!=EINTR)return false;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }while(Clock::now()<deadline);
    return false;
  }
  bool stop()noexcept {
    if(socket>=0){shutdown(socket,SHUT_RDWR);close(socket);socket=-1;}
    if(pid<0)return true;
    if(pidfd>=0)syscall(SYS_pidfd_send_signal,pidfd,SIGTERM,nullptr,0);
    // If pidfd could not be opened, this unreaped direct child still owns its PID.
    else kill(pid,SIGTERM);
    bool exited=reap_until(after(options.terminate_timeout_sec));
    if(!exited) {
      if(pidfd>=0)syscall(SYS_pidfd_send_signal,pidfd,SIGKILL,nullptr,0);
      else kill(pid,SIGKILL);
      exited=reap_until(after(options.kill_timeout_sec));
    }
    if(exited){if(pidfd>=0)close(pidfd);pidfd=-1;pid=-1;}
    return exited;
  }
  void start() {
    if(canceled.load())throw Failure("WORKER_CANCELED");
    const auto previous=status();
    if(previous.state=="QUARANTINED")throw Failure("WORKER_QUARANTINED");
    if(pid>0)return;
    if(starts>options.max_restarts){state("QUARANTINED","WORKER_RESTART_BUDGET_EXHAUSTED");throw Failure("WORKER_RESTART_BUDGET_EXHAUSTED");}
    if(Clock::now()<next_start)throw Failure("WORKER_BACKOFF");
    ++starts;++epoch;state("STARTING","WORKER_STARTING");
    int pair[2];
    if(socketpair(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0,pair)<0)throw Failure("WORKER_SOCKET_FAILED");
    socket=pair[0];
    // Only the parent endpoint is nonblocking; the child may block freely.
    if(fcntl(socket,F_SETFL,fcntl(socket,F_GETFL)|O_NONBLOCK)<0){close(pair[1]);throw Failure("WORKER_SOCKET_FAILED");}
    const int child_fd=fcntl(pair[1],F_DUPFD_CLOEXEC,64);close(pair[1]);
    if(child_fd<0)throw Failure("WORKER_SOCKET_FAILED");
    posix_spawn_file_actions_t actions;
    int error=posix_spawn_file_actions_init(&actions);
    if(error){close(child_fd);throw Failure("WORKER_SPAWN_CONFIG_FAILED");}
    error=posix_spawn_file_actions_adddup2(&actions,child_fd,3);
    if(!error)error=posix_spawn_file_actions_addclose(&actions,child_fd);
    std::vector<std::string> args{options.executable};
    args.insert(args.end(),options.arguments.begin(),options.arguments.end());
    std::vector<char*> argv;for(auto& arg:args)argv.push_back(arg.data());argv.push_back(nullptr);
    if(!error)error=posix_spawn(&pid,options.executable.c_str(),&actions,nullptr,argv.data(),environ);
    posix_spawn_file_actions_destroy(&actions);close(child_fd);
    if(error){pid=-1;throw Failure("WORKER_SPAWN_FAILED");}
    pidfd=int(syscall(SYS_pidfd_open,pid,0));state("STARTING","WORKER_STARTING");
    if(pidfd<0)throw Failure("WORKER_PIDFD_UNAVAILABLE");
    wire::Header ready;
    transfer(&ready,sizeof(ready),false,after(options.startup_timeout_sec),"WORKER_STARTUP_TIMEOUT");
    if(!wire::envelope_valid(ready)||ready.op!=wire::Op::ready)throw Failure("WORKER_PROTOCOL_MISMATCH");
    state("READY","OK");
  }
  void fail(const std::string& reason) {
    if(reason=="WORKER_BACKOFF"||reason=="WORKER_QUARANTINED"||reason=="WORKER_RESTART_BUDGET_EXHAUSTED")return;
    state("STOPPING",reason);
    if(!stop()){state("QUARANTINED","WORKER_EXIT_UNCONFIRMED");return;}
    next_start=after(options.restart_backoff_sec);
    state(canceled.load()?"STOPPED":"BACKOFF",reason);
  }
};
ProjectionProcess::ProjectionProcess(ProjectionProcessOptions o):impl_(std::make_unique<Impl>(std::move(o))){}
ProjectionProcess::~ProjectionProcess(){cancel();if(!impl_->stop())impl_->state("QUARANTINED","WORKER_EXIT_UNCONFIRMED");if(impl_->pidfd>=0)close(impl_->pidfd);}
void ProjectionProcess::cancel()noexcept{impl_->canceled.store(true);}
ProjectionProcessStatus ProjectionProcess::status()const{return impl_->status();}
void ProjectionProcess::project(const DepthView& v,const ProjectionConfig& c,float* out){project_with_context(v,c,out,{});}
void ProjectionProcess::project_with_context(const DepthView& v,const ProjectionConfig& c,float* out,ProjectionContext context) {
  const auto count=output_points(v,c);if(!out||context.capture_ns<0)throw std::invalid_argument("invalid projection output/context");
  auto& p=*impl_;
  try {
    p.start();
    wire::Header request;request.boot=p.boot;request.epoch=p.epoch;request.request=++p.request;
    request.generation=context.generation;request.capture_ns=context.capture_ns;
    request.input_bytes=v.bytes;request.output_bytes=count*4*sizeof(float);
    request.width=v.width;request.height=v.height;request.step=v.step;request.floating=v.floating;request.big_endian=v.big_endian;
    request.decimation=c.decimation;request.fx=c.fx;request.fy=c.fy;request.cx=c.cx;request.cy=c.cy;request.min_depth=c.min_depth;request.max_depth=c.max_depth;
    p.state("RUNNING","OK");const auto deadline=after(p.options.request_timeout_sec);
    p.transfer(&request,sizeof(request),true,deadline,"WORKER_REQUEST_TIMEOUT");
    p.transfer(const_cast<uint8_t*>(v.data),v.bytes,true,deadline,"WORKER_REQUEST_TIMEOUT");
    wire::Header reply;p.transfer(&reply,sizeof(reply),false,deadline,"WORKER_REQUEST_TIMEOUT");
    if(!wire::same_request(reply,request))throw Failure("WORKER_PROTOCOL_MISMATCH");
    if(reply.op==wire::Op::error)throw Failure("WORKER_COMPUTE_FAILED");
    if(reply.op!=wire::Op::result)throw Failure("WORKER_PROTOCOL_MISMATCH");
    p.transfer(out,request.output_bytes,false,deadline,"WORKER_REQUEST_TIMEOUT");
    p.state("READY","OK");
  }catch(const Failure& e){p.fail(e.what());throw;}
}
std::string projection_worker_executable() {
  return (std::filesystem::read_symlink("/proc/self/exe").parent_path()/"depth_projection_worker").string();
}
}
