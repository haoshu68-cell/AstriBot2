#include "astribot_s1_perception_components/projection_wire.hpp"
#include <unistd.h>
#include <sys/socket.h>
#include <cerrno>
#include <cstring>
#include <cstdio>
namespace {
bool transfer(void* data,size_t bytes,bool writing) {
  auto* p=static_cast<char*>(data);
  while(bytes) {
    const auto n=writing?send(3,p,bytes,MSG_NOSIGNAL):read(3,p,bytes);
    if(n<0&&errno==EINTR)continue;
    if(n<=0)return false;
    p+=n;bytes-=size_t(n);
  }
  return true;
}
}
int main(int argc,char**argv) {
  using namespace astribot::vision;
  // CPU mode is an explicit verification/diagnostic backend, never fallback.
  if(argc!=3||std::string(argv[1])!="--backend")return 2;
  try {
    auto projector=make_depth_projector(argv[2]);
    wire::Header message;message.op=wire::Op::ready;
    if(!transfer(&message,sizeof(message),true))return 3;
    std::vector<uint8_t> input;std::vector<float> output;
    while(transfer(&message,sizeof(message),false)) {
      if(!wire::request_valid(message))return 4;
      input.resize(message.input_bytes);output.resize(message.output_bytes/sizeof(float));
      if(!transfer(input.data(),input.size(),false))return 5;
      try {
        projector->project({input.data(),input.size(),message.width,message.height,message.step,
          bool(message.floating),bool(message.big_endian)},wire::config(message),output.data());
      }catch(const std::exception& e) {
        message.op=wire::Op::error;std::snprintf(message.reason,sizeof(message.reason),"%s",e.what());
        transfer(&message,sizeof(message),true);
        // The parent owns termination. Do not reuse or destroy a possibly
        // poisoned CUDA context; even CUDA cleanup can block indefinitely.
        _exit(6);
      }
      message.op=wire::Op::result;
      if(!transfer(&message,sizeof(message),true)||!transfer(output.data(),output.size()*sizeof(float),true))return 7;
    }
  }catch(const std::exception& e){std::fprintf(stderr,"depth_projection_worker: %s\n",e.what());return 8;}
  return 0;
}
