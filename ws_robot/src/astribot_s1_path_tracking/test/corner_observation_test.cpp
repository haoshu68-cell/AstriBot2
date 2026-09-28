#include "corner_fixture.hpp"
#include "rcutils/logging.h"
#include <cstdio>
#include <cstdarg>
#include <string>
#include <vector>

std::vector<std::string> events;
void capture(const rcutils_log_location_t *,int,const char *,rcutils_time_point_value_t,
  const char * format,va_list * args)
{
  va_list copied;va_copy(copied,*args);
  char buffer[2048];std::vsnprintf(buffer,sizeof(buffer),format,copied);va_end(copied);
  if(std::string(buffer).find("CORNER_STATE ")!=std::string::npos) {events.emplace_back(buffer);}
}
int main()
{
  using namespace astribot_s1_path_tracking;using Peer=ThreePhaseControllerTestPeer;
  ThreePhaseController c;Peer::init(c);Peer::time(c,200.);
  RCLCPP_INFO(rclcpp::get_logger("offline_observation"),"logging initialization only; no ROS context");
  const auto previous=rcutils_logging_get_output_handler();rcutils_logging_set_output_handler(capture);
  int checks=0,failures=0;
  auto check=[&](bool ok,const char * name) {++checks;if(!ok){++failures;std::cerr<<"FAIL "<<name<<'\n';}};
  auto execution=Peer::managed(c);auto plan=path({{0,0},{1,0},{1,1}},100);
  c.setPlan(plan);execution->begin();Peer::commit(c);
  Peer::active(c,0);Peer::diagnosticPose(c);Peer::diagnosticTurn(c);
  check(!events.empty(),"stage transition emits diagnostic event");
  check(!events.empty() && events.back().find("execution=1 ")!=std::string::npos &&
    events.back().find("revision=1 ")!=std::string::npos && events.back().find("ros_s=200.")!=std::string::npos,
    "diagnostic carries explicit execution/revision/control time");
  const auto no_repeat=events.size();Peer::diagnosticTurn(c);
  check(events.size()==no_repeat,"same substate does not emit per-tick logs");
  const auto timer=Peer::timer(c);Peer::time(c,200.1);
  const auto before=events.size();plan.header.stamp.sec=101;c.setPlan(plan);Peer::commit(c);
  check(events.size()>before && events.back().find("state=TURNING ")!=std::string::npos &&
    events.back().find("revision=2 ")!=std::string::npos,"equivalent refresh emits same state in new route revision");
  check(c.phase()==Phase::kAlignCorner,"diagnostic refresh never changes control phase");
  check(Peer::timer(c)==timer,"refresh diagnostics preserve phase budget");
  const auto accepted_events=events.size();auto stale=plan;stale.header.stamp.sec=99;c.setPlan(stale);
  bool rejected=false;try{Peer::commit(c);}catch(const nav2_core::PlannerException &){rejected=true;}
  check(rejected && events.size()==accepted_events,"rejected refresh does not emit accepted-state event");
  Peer::time(c,200.2);c.deactivate();
  check(!events.empty() && events.back().find("state=IDLE ")!=std::string::npos &&
    events.back().find("active=0 ")!=std::string::npos,"deactivation invalidates last active corner event");
  check(events.back().find("pose_s=0.000000000 ")!=std::string::npos &&
    events.back().find("pose_valid=0 ")!=std::string::npos,"reset never exposes old pose as available");
  Peer::time(c,200.3);c.setPlan(plan);execution->begin();Peer::commit(c);Peer::active(c,0);Peer::diagnosticPose(c);Peer::diagnosticTurn(c);
  check(events.back().find("execution=2 ")!=std::string::npos,"new execution has distinct diagnostic identity");
  Peer::time(c,200.4);c.setPlan(nav_msgs::msg::Path{});Peer::commit(c);
  check(events.back().find("state=IDLE ")!=std::string::npos,"empty path invalidates corner state");
  c.setPlan(plan);Peer::commit(c);Peer::active(c,0);Peer::diagnosticPose(c);Peer::diagnosticTurn(c);
  Peer::time(c,200.5);c.cleanup();
  check(events.back().find("execution=2 ")!=std::string::npos &&
    events.back().find("state=IDLE ")!=std::string::npos,"cleanup invalidation belongs to retiring execution");
  std::size_t expected=1;bool monotonic=true;
  for(const auto & event:events) {
    const auto pos=event.find("event=");
    monotonic=monotonic && pos!=std::string::npos && std::stoull(event.substr(pos+6))==expected++;
  }
  check(monotonic,"event counter is monotonic across plans and resets");
  const auto before_disabled=events.size();ThreePhaseController disabled;Peer::init(disabled);
  Peer::disableCorners(disabled);disabled.deactivate();
  ThreePhaseController unconfigured;unconfigured.cleanup();
  check(events.size()==before_disabled,"disabled and unconfigured instances do not emit corner evidence");
  for(const auto & event:events) {std::cout<<event<<'\n';}
  rcutils_logging_set_output_handler(previous);
  std::cout<<"observation_checks="<<checks<<" failures="<<failures<<'\n';return failures?1:0;
}
