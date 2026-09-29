#include "astribot_s1_perception_components/projection_wire.hpp"
#include <unistd.h>
#include <sys/socket.h>
#include <signal.h>
#include <cerrno>
#include <cstring>
#include <thread>
using namespace astribot::vision;
bool transfer(void* data,size_t bytes,bool writing) {
  auto* p=static_cast<char*>(data);
  while(bytes){auto n=writing?send(3,p,bytes,MSG_NOSIGNAL):read(3,p,bytes);
    if(n<0&&errno==EINTR)continue;
    if(n<=0)return false;
    p+=n;bytes-=size_t(n);}
  return true;
}
int main(int argc,char**argv) {
  const std::string mode=argc>1?argv[1]:"good";
  if(mode=="startup_hang")for(;;)pause();
  wire::Header h;h.op=wire::Op::ready;
  if(!transfer(&h,sizeof(h),true))return 1;
  auto cpu=make_depth_projector("cpu");
  while(transfer(&h,sizeof(h),false)) {
    if(!wire::request_valid(h))return 2;
    std::vector<uint8_t> input(h.input_bytes);std::vector<float> output(h.output_bytes/sizeof(float));
    if(!transfer(input.data(),input.size(),false))return 3;
    if(mode=="hang"){signal(SIGTERM,SIG_IGN);for(;;)pause();}
    if(mode=="crash")_exit(7);
    if(mode=="slow")std::this_thread::sleep_for(std::chrono::milliseconds(100));
    cpu->project({input.data(),input.size(),h.width,h.height,h.step,bool(h.floating),bool(h.big_endian)},wire::config(h),output.data());
    h.op=wire::Op::result;
    if(mode=="wrong_epoch")++h.epoch;
    if(mode=="wrong_request")++h.request;
    if(mode=="wrong_boot")++h.boot;
    if(mode=="wrong_generation")++h.generation;
    if(mode=="wrong_capture")++h.capture_ns;
    if(mode=="oversized")h.output_bytes=wire::max_output_bytes+1;
    if(mode=="error"){h.op=wire::Op::error;std::strcpy(h.reason,"FIXTURE_CONTEXT_FAILED");}
    if(!transfer(&h,sizeof(h),true))return 4;
    if(mode=="truncated")_exit(9);
    if(h.op==wire::Op::result && !transfer(output.data(),output.size()*sizeof(float),true))return 5;
  }
  return 0;
}
