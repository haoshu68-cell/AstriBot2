#include <gtest/gtest.h>
#include "astribot_s1_perception_components/projection_process.hpp"
#include <thread>
#include <future>
#include <signal.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <sys/wait.h>
// Fault seam at the OS boundary: simulate an owned child whose exit cannot be
// confirmed, without hanging a real GPU or leaving an unreaped test process.
static std::atomic<pid_t> unconfirmed_pid{-1};
extern "C" pid_t waitpid(pid_t pid,int* status,int options) {
  if(pid==unconfirmed_pid.load())return 0;
  return static_cast<pid_t>(syscall(SYS_wait4,pid,status,options,nullptr));
}
using namespace astribot::vision;
using namespace std::chrono_literals;
namespace {
ProjectionProcessOptions options(const std::string& mode) {
  ProjectionProcessOptions o;o.executable=PROJECTION_FIXTURE_PATH;o.arguments={mode};
  o.startup_timeout_sec=.12;o.request_timeout_sec=.06;o.restart_backoff_sec=0;
  return o;
}
void call(ProjectionProcess& p) {
  const uint8_t data[]={232,3,208,7};float out[8]{};
  p.project_with_context({data,4,2,1,4,false,false},{100,100,0,0,.2,5,1},out,{9,123000000});
  EXPECT_FLOAT_EQ(out[2],1);EXPECT_FLOAT_EQ(out[6],2);
}
}
TEST(ProjectionProcessContract, CpuFixturePreservesValuesAndReusesChild) {
  ProjectionProcess p(options("good"));call(p);auto a=p.status();call(p);auto b=p.status();
  EXPECT_EQ(a.pid,b.pid);EXPECT_GT(a.pid,0);EXPECT_EQ(a.worker_epoch,b.worker_epoch);EXPECT_EQ(b.requests,2u);
}
TEST(ProjectionProcessContract, HangTimesOutAndReapsWithoutBlockingShutdown) {
  const auto start=std::chrono::steady_clock::now();int64_t pid=-1;
  {ProjectionProcess p(options("hang"));EXPECT_THROW(call(p),std::runtime_error);pid=p.status().pid;
   EXPECT_EQ(p.status().reason,"WORKER_REQUEST_TIMEOUT");EXPECT_EQ(p.status().state,"BACKOFF");}
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count(),.7);
  EXPECT_EQ(pid,-1);
}
TEST(ProjectionProcessContract, StartupTimeoutDoesNotBecomeReady) {
  ProjectionProcess p(options("startup_hang"));EXPECT_THROW(call(p),std::runtime_error);
  EXPECT_EQ(p.status().reason,"WORKER_STARTUP_TIMEOUT");EXPECT_EQ(p.status().pid,-1);
}
TEST(ProjectionProcessContract, RejectsWrongIdentityAndOversizedResults) {
  for(const auto* mode:{"wrong_epoch","wrong_request","wrong_boot","wrong_generation","wrong_capture","oversized"}) {
    ProjectionProcess p(options(mode));EXPECT_THROW(call(p),std::runtime_error)<<mode;
    EXPECT_EQ(p.status().reason,"WORKER_PROTOCOL_MISMATCH")<<mode;
  }
}
TEST(ProjectionProcessContract, IncompleteAndFailedResultsNeverBecomeSuccess) {
  for(const auto* mode:{"truncated","error"}) {
    ProjectionProcess p(options(mode));EXPECT_THROW(call(p),std::runtime_error);
    EXPECT_EQ(p.status().state,"BACKOFF");EXPECT_EQ(p.status().pid,-1);
  }
}
TEST(ProjectionProcessContract, ConfirmedCrashRecoversWithNewChildAndEpoch) {
  ProjectionProcess p(options("good"));call(p);const auto first=p.status();
  ASSERT_EQ(kill(first.pid,SIGKILL),0);
  std::this_thread::sleep_for(10ms);
  EXPECT_THROW(call(p),std::runtime_error);EXPECT_EQ(p.status().state,"BACKOFF");
  EXPECT_NO_THROW(call(p));EXPECT_GT(p.status().worker_epoch,first.worker_epoch);
  EXPECT_NE(p.status().pid,first.pid);EXPECT_EQ(p.status().restarts,1u);
}
TEST(ProjectionProcessContract, UnconfirmedExitQuarantinesWithoutReplacement) {
  auto o=options("good");o.terminate_timeout_sec=.01;o.kill_timeout_sec=.01;
  int64_t child=-1;
  {
    ProjectionProcess p(o);call(p);child=p.status().pid;const auto epoch=p.status().worker_epoch;
    unconfirmed_pid.store(child);ASSERT_EQ(kill(child,SIGKILL),0);
    std::this_thread::sleep_for(10ms);EXPECT_THROW(call(p),std::runtime_error);
    EXPECT_EQ(p.status().state,"QUARANTINED");EXPECT_EQ(p.status().reason,"WORKER_EXIT_UNCONFIRMED");
    EXPECT_EQ(p.status().pid,child);EXPECT_THROW(call(p),std::runtime_error);
    EXPECT_EQ(p.status().worker_epoch,epoch);EXPECT_EQ(p.status().pid,child);
  }
  unconfirmed_pid.store(-1);int status=0;EXPECT_EQ(waitpid(child,&status,0),child);
}
TEST(ProjectionProcessContract, ProductionWorkerTransfersLargeFrameWithoutChangingValues) {
  auto o=options("");o.executable=PROJECTION_WORKER_PATH;o.arguments={"--backend","cpu"};
  o.request_timeout_sec=1.;ProjectionProcess p(o);
  std::vector<uint16_t> input(640*360,1500);std::vector<float> out(input.size()*4),reference(out.size());
  DepthView v{reinterpret_cast<uint8_t*>(input.data()),input.size()*2,640,360,1280,false,false};
  ProjectionConfig c{400,400,320,180,.2,5,1};
  make_depth_projector("cpu")->project(v,c,reference.data());p.project(v,c,out.data());
  EXPECT_EQ(out,reference);const auto pid=p.status().pid;p.project(v,c,out.data());EXPECT_EQ(p.status().pid,pid);
}
#ifdef ASTRIBOT_CUDA_PROJECTION
TEST(ProjectionProcessContract, ProductionCudaWorkerMatchesCpuAndReusesContext) {
  auto o=options("");o.executable=PROJECTION_WORKER_PATH;o.arguments={"--backend","cuda"};
  o.startup_timeout_sec=5.;o.request_timeout_sec=1.;ProjectionProcess p(o);
  std::vector<uint16_t> input(640*360,1500);std::vector<float> out(input.size()*4),reference(out.size());
  DepthView v{reinterpret_cast<uint8_t*>(input.data()),input.size()*2,640,360,1280,false,false};
  ProjectionConfig c{400,400,320,180,.2,5,1};make_depth_projector("cpu")->project(v,c,reference.data());
  p.project(v,c,out.data());const auto pid=p.status().pid;
  for(size_t i=0;i<out.size();++i)ASSERT_NEAR(out[i],reference[i],1.e-6)<<i;
  p.project(v,c,out.data());EXPECT_EQ(p.status().pid,pid);EXPECT_EQ(p.status().worker_epoch,1u);
}
#endif
TEST(ProjectionProcessContract, CrashRetriesAreBoundedAndAdvanceEpoch) {
  auto o=options("crash");o.max_restarts=1;ProjectionProcess p(o);
  EXPECT_THROW(call(p),std::runtime_error);auto epoch=p.status().worker_epoch;
  EXPECT_THROW(call(p),std::runtime_error);EXPECT_GT(p.status().worker_epoch,epoch);
  const auto last=p.status().worker_epoch;EXPECT_THROW(call(p),std::runtime_error);
  EXPECT_EQ(p.status().state,"QUARANTINED");EXPECT_EQ(p.status().worker_epoch,last);
}
TEST(ProjectionProcessContract, ExplicitCancelInterruptsWaitAndCannotRestart) {
  auto o=options("hang");o.request_timeout_sec=3.;ProjectionProcess p(o);
  auto future=std::async(std::launch::async,[&]{EXPECT_THROW(call(p),std::runtime_error);});
  const auto deadline=std::chrono::steady_clock::now()+1s;
  while(p.status().state!="RUNNING" && std::chrono::steady_clock::now()<deadline)std::this_thread::sleep_for(1ms);
  p.cancel();EXPECT_EQ(future.wait_for(500ms),std::future_status::ready);
  EXPECT_EQ(p.status().reason,"WORKER_CANCELED");EXPECT_THROW(call(p),std::runtime_error);
}
TEST(ProjectionProcessContract, InvalidConfigurationRejectedBeforeSpawn) {
  auto o=options("good");o.executable="relative";EXPECT_THROW(ProjectionProcess p(o),std::invalid_argument);
  o=options("good");o.request_timeout_sec=0;EXPECT_THROW(ProjectionProcess p(o),std::invalid_argument);
}
