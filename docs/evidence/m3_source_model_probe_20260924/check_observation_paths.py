"""Exercise exact extracted probe lambdas with deterministic stand-ins; no ROS/GPU."""
from pathlib import Path
import hashlib
import json
import subprocess

root = Path(__file__).resolve().parents[3]
source = root / 'tools/vision/m3_source_probe/probe.cpp'
text = source.read_text()
observe = text[text.index('  auto observe='):text.index('  try {\n    require(node->get_parameter')]
cleanup = text[text.index('  auto cleanup='):text.index('  const auto cleanup_until=')]
prelude = r'''
#include <nlohmann/json.hpp>
#include <array>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
using Json=nlohmann::json;
namespace rclcpp_action {enum class ResultCode {UNKNOWN,SUCCEEDED,ABORTED,CANCELED};}
using Code=rclcpp_action::ResultCode;
int64_t steady_ns(){return std::chrono::steady_clock::now().time_since_epoch().count();}
struct Time{int64_t nanoseconds()const{return 10000000000LL;}};
struct Node{Time now(){return {};}};
struct Handle{std::array<unsigned char,16>get_goal_id(){std::array<unsigned char,16>id{};id[15]=42;return id;}};
struct Client{bool fail=false;int cancels=0;void async_cancel_goal(const std::shared_ptr<Handle>&){++cancels;if(fail)throw std::runtime_error("synthetic_cancel_error");}};
struct Wrapped{Code code=Code::SUCCEEDED;};
struct Future{bool fail=false;Wrapped value;Wrapped get(){if(fail)throw std::runtime_error("synthetic_result_error");return value;}};
struct Pending{
  bool sent=true,rejected=false,cancel_sent=false,ready=false,flip_after_sample=false,poll_error=false;
  int samples=0;std::shared_ptr<Handle>handle=std::make_shared<Handle>();
  std::shared_ptr<Client>client=std::make_shared<Client>();Future result;
  void poll(){if(poll_error)throw std::runtime_error("synthetic_poll_error");}
  bool received(){++samples;bool sampled=ready;if(flip_after_sample){ready=true;flip_after_sample=false;}return sampled;}
};
void check(bool value,const char* why){if(!value)throw std::runtime_error(why);}
int main(){Json report;auto node=std::make_shared<Node>();int exit_code=0;
'''
cases = r'''
  int passed=0;
  {Pending p;p.flip_after_sample=true;check(!observe(p,"race"),"in-flight transition became terminal");
   check(p.samples==1&&!report["race"].contains("terminal_observed_steady_ns"),"multiple samples or invented terminal time");
   check(observe(p,"race")&&report["race"]["terminal_observed_steady_ns"].is_number_integer(),"next poll missing real time");++passed;}
  {Pending p;p.handle.reset();check(!observe(p,"pending")&&report["pending"]["state"]=="SENT_ACCEPTANCE_PENDING","acceptance pending mislabeled");++passed;}
  {Pending p;p.handle.reset();p.rejected=true;check(observe(p,"reject")&&report["reject"]["state"]=="REJECTED"&&!report["reject"].contains("terminal_observed_steady_ns"),"reject treated as model result");++passed;}
  {Pending p;p.ready=true;p.result.value.code=Code::UNKNOWN;check(!observe(p,"unknown")&&report["unknown"]["state"]=="RESULT_UNKNOWN","unknown treated as terminal");++passed;}
  {Pending p;p.ready=true;p.result.value.code=Code::ABORTED;check(observe(p,"abort")&&report["abort"]["terminal_observed_ros_ns"].is_number_integer(),"algorithm failure missing terminal time");
   check(report["abort"]["goal_uuid"]=="0000000000000000000000000000002a","goal uuid mismatch");++passed;}
  {Pending p,g;p.ready=true;p.result.fail=true;check(!cleanup(p,"exception")&&!cleanup(g,"peer"),"exception mislabeled terminal");
   check(p.client->cancels==1&&g.client->cancels==1&&report["exception"].contains("observe_error"),"exception skipped cancellation or peer");++passed;}
  {Pending p,g;p.client->fail=true;(void)cleanup(p,"cancel_exception");(void)cleanup(g,"cancel_peer");
   (void)cleanup(p,"cancel_exception");check(p.client->cancels==1&&g.client->cancels==1&&report["cancel_exception"].contains("cancel_error"),"cancel error hid peer or retried ambiguous send");++passed;}
  {Pending p;p.sent=false;p.handle.reset();check(observe(p,"not_run")&&report["not_run"]["state"]=="NOT_RUN"&&!report["not_run"].contains("goal_uuid"),"unsent goal provenance fabricated");++passed;}
  std::cout<<Json{{"checks",passed},{"passed",passed},{"failed",0},{"scope","exact extracted probe lambdas; synthetic interfaces; no ROS/GPU"},{"report",report}}.dump(2)<<'\n';
}
'''
folder = root / 'runs/m3_source_model_probe_20260924/observation_harness'
folder.mkdir(parents=True, exist_ok=True)
harness = folder / 'probe_observation.cpp'
harness.write_text(prelude + observe + cleanup + cases)
binary = folder / 'probe_observation'
compile_command = ['g++','-std=c++17','-Wall','-Wextra','-Wpedantic','-O0','-g','-fsanitize=undefined','-fno-sanitize-recover=all',str(harness),'-o',str(binary)]
subprocess.run(compile_command, check=True)
run = subprocess.run([str(binary)],check=True,text=True,capture_output=True)
result=json.loads(run.stdout)
result.update(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),harness_sha256=hashlib.sha256(harness.read_bytes()).hexdigest(),binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),compile_command=compile_command,stderr=run.stderr)
out=Path(__file__).with_name('observation_paths.json')
out.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:result[k] for k in ['checks','passed','failed','scope','source_sha256','stderr']},indent=2))
