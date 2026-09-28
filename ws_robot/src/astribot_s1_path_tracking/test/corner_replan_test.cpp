#include "corner_fixture.hpp"

int main() {
  using namespace astribot_s1_path_tracking;
  using Peer=ThreePhaseControllerTestPeer;
  int failures=0;
  auto check=[&](bool ok,const char * name) {if(!ok){++failures;std::cerr<<name<<'\n';}};
  auto original=path({{0,0},{1,0},{1,1},{2,1}},100);
  {
    ThreePhaseController c;Peer::init(c);c.setPlan(original);Peer::active(c,0);
    auto stale=original;stale.header.stamp.sec=99;bool rejected=false;
    try {c.setPlan(stale);} catch(const nav2_core::PlannerException &) {rejected=true;}
    check(rejected,"stale plan must be rejected");
    check(Peer::plan(c).header.stamp.sec==100,"rejected path must not replace active plan");
  }
  {
    ThreePhaseController c;Peer::init(c);c.setPlan(original);Peer::active(c,1);
    bool rejected=false;
    try {c.setPlan(path({{1,.5},{1,1},{2,1}},101));}
    catch(const nav2_core::PlannerException &) {rejected=true;}
    check(rejected,"trimmed path without a robot pose must not inherit old cursor");
  }
  {
    ThreePhaseController c;Peer::init(c);c.setPlan(original);Peer::active(c,0);
    bool rejected=false;
    try {c.setPlan(path({{0,0},{1,0},{1,-1},{2,-1},{2,1}},101));}
    catch(const nav2_core::PlannerException &) {rejected=true;}
    check(rejected,"changed outgoing direction cannot reuse nearest active corner");
  }
  {
    ThreePhaseController c;Peer::init(c);c.setPlan(original);Peer::active(c,1);
    auto same=path({{0,0},{.5,0},{1,0},{1,.5},{1,1},{2,1}},101);
    c.setPlan(same);
    check(c.phase()==Phase::kAlignCorner && Peer::cursor(c)==1,
      "collinear resampling must retain ordered corner and phase");
  }
  {
    ThreePhaseController c;Peer::init(c);c.setPlan(original);Peer::active(c,1);Peer::idle(c);
    auto next=original;next.header.stamp.sec=101;c.setPlan(next);
    check(c.phase()==Phase::kAlignCorner && Peer::cursor(c)==1,
      "scheduler gaps are not new execution attempts");
  }
  {
    ThreePhaseController c;Peer::init(c);c.setPlan(original);Peer::active(c,0);
    auto changed=original;changed.header.stamp.sec=101;
    changed.poses.back().pose.orientation.z=std::sin(.5);
    changed.poses.back().pose.orientation.w=std::cos(.5);c.setPlan(changed);
    check(c.phase()!=Phase::kAlignCorner,"new terminal yaw must reset corner context");
  }
  {
    ThreePhaseController c;Peer::init(c);c.setPlan(original);Peer::active(c,1);
    Peer::observed(c,1.,.5);c.setPlan(path({{1,.5},{1,1},{2,1}},101));
    check(Peer::cursor(c)==0 && c.phase()!=Phase::kAlignCorner && Peer::settling(c),
      "valid trimmed route must reanchor after settling, never retain old index");
  }
  {
    ThreePhaseController c;Peer::init(c);c.setPlan(original);Peer::active(c,0);
    auto changed=original;changed.header.stamp.sec=101;changed.poses.back().pose.position.x+=.1;
    c.setPlan(changed);
    check(c.phase()!=Phase::kAlignCorner,"nearby new target must not be treated as a refresh");
  }
  {
    ThreePhaseController c;Peer::init(c);
    auto hold=Peer::hold(c,.97,0.,0.);
    check(hold.linear.x>0. && hold.linear.x<=.04 && std::abs(hold.linear.y)<1e-9,
      "corner drift needs bounded corrective translation");
    hold=Peer::hold(c,.97,0.,M_PI/2.);
    check(std::abs(hold.linear.x)<1e-9 && hold.linear.y<0.,
      "corner correction must transform world error into robot axes");
    hold=Peer::hold(c,.995,0.,0.);
    check(hold.linear.x==0. && hold.linear.y==0.,"corner position deadband must not hunt");
    hold=Peer::hold(c,.94,0.,0.);
    check(hold.angular.z==0.,"rotation pauses outside corner capture radius");
  }
  {
    ThreePhaseController c;Peer::init(c);
    c.setPlan(path({{0,0},{1,0},{1,1},{0,1},{0,0},{1,0},{1,-1}},100));
    check(Peer::approach(c,4,.9,0.),"self-intersection must project on current leg, not first visit");
  }
  {
    ThreePhaseController c;Peer::init(c);c.setPlan(original);
    Peer::completed(c,1);
    check(Peer::segment(c).poses.front().pose.position.x==1. &&
      Peer::segment(c).poses.front().pose.position.y==0.,
      "completed incoming leg must not remain in inner plan");
    Peer::completed(c,2);
    check(Peer::segment(c).poses.front().pose.position.y==1.,
      "last completed corner still trims final leg");
  }
  {
    ThreePhaseController c;Peer::init(c);
    c.setPlan(path({{0,0},{1,0},{1,.3}},100));
    Peer::approach(c,0,0.,0.);
    check(Peer::cursor(c)==1,"terminal fixture must actually skip a detected corner");
    check(Peer::segment(c).poses.front().pose.position.x==0.,
      "skipped terminal corner must retain untraversed incoming leg");
  }
  for (int fault=0;fault<3;++fault) {
    ThreePhaseController c;Peer::init(c);c.setPlan(original);Peer::active(c,0);
    Peer::observe(c,1.,0.,10.);bool rejected=false;
    try {Peer::observe(c,fault==0?1.8:1.,fault==1?.8:0.,fault==2?9.:10.05);}
    catch(const nav2_core::PlannerException &) {rejected=true;}
    check(rejected,"corner localization position/yaw/time discontinuity must stop");
  }
  {
    ThreePhaseController c;Peer::init(c);c.setPlan(original);Peer::active(c,0);
    bool rejected=false;
    try {Peer::observe(c,1.,0.,10.);Peer::observe(c,1.002,.02,10.05);}
    catch(const nav2_core::PlannerException &) {rejected=true;}
    check(!rejected,"ordinary corner feedback change must remain accepted");
  }
  {
    ThreePhaseController c;Peer::init(c);c.setPlan(original);Peer::active(c,0);
    Peer::observe(c,1.,0.,10.);Peer::recovered(c);bool rejected=false;
    try {Peer::observe(c,1.8,0.,10.05);}catch(const nav2_core::PlannerException &){rejected=true;}
    check(rejected,"localization jump during corner recovery must stop before terminal takeover");
  }
  {
    ThreePhaseController c;Peer::init(c);const auto command=Peer::captureBoundary(c);
    check(std::hypot(command.linear.x,command.linear.y)>=.02,
      "stopped 4.15cm outside corner must not retain sub-friction MPPI command");
    check(command.linear.x<0. && command.linear.y>0. && command.angular.z==0.,
      "capture recovery points toward corner without early rotation");
  }
  {
    auto command=ArrivalControllerTestPeer::limited(.01,false);
    check(std::abs(std::hypot(command.linear.x,command.linear.y)-.01)<1e-9 && command.angular.z==.4,
      "corner correction must obey external linear limit without altering turn law");
    command=ArrivalControllerTestPeer::limited(5.,true);
    check(std::abs(std::hypot(command.linear.x,command.linear.y)-.0175)<1e-9,
      "percentage limit uses nominal navigation speed");
    command=ArrivalControllerTestPeer::limited(0.,false);
    check(std::abs(std::hypot(command.linear.x,command.linear.y)-.05)<1e-9,
      "Nav2 zero limit reset must retain existing corner command");
  }
  {
    ThreePhaseController c;Peer::init(c);bool rejected=false;
    try {c.setPlan(path({{0,0},{2.2,0},{1.2,.02}},100));}
    catch(const nav2_core::PlannerException & e) {
      rejected=std::string(e.what()).find("CORNER_UNSUPPORTED_REVERSAL")!=std::string::npos;
    }
    check(rejected,"near-180 reversal outside configured range must reject before shortcutting");
  }
  {
    ThreePhaseController c;Peer::init(c);
    c.setPlan(path({{0,0},{1,0},{1,1},{0,1},{0,.1}},100));
    check(!Peer::terminal(c,0,0),"nearby final target cannot skip uncompleted loop");
    Peer::completed(c,3);
    check(Peer::terminal(c,0,.2),"last route leg may enter terminal capture");
  }
  {
    ThreePhaseController c;Peer::init(c);c.setPlan(original);Peer::active(c,1);
    check(!Peer::terminal(c,1.9,1.),"active corner cannot be interrupted by terminal takeover");
  }
  {
    ThreePhaseController c;Peer::init(c);c.setPlan(path({{0,0},{1,0},{1,.3}},100));
    check(!Peer::terminal(c,.9,0),"terminal short corner retains ordered remaining distance");
    Peer::approach(c,0,.9,0.);
    check(Peer::terminal(c,1.,.1),"short terminal leg remains reachable after corner skip");
  }
  {
    ThreePhaseController c;Peer::init(c);c.setPlan(path({{0,0},{1,0}},100));
    check(Peer::terminal(c,.8,0),"ordinary straight arrival remains accepted");
  }
  for (double yaw : {0.,1.5707963267948966,-1.5707963267948966}) {
    ThreePhaseController c;Peer::init(c);c.setPlan(path({{0,0},{0,0}},100,yaw));
    check(Peer::terminal(c,0,0),"zero-arc heading-only and already-at-goal paths retain terminal refinement");
  }
  for (bool angular : {false,true}) {
    ThreePhaseController c;Peer::init(c);c.setPlan(original);Peer::active(c,0);
    Peer::observe(c,1.,0.,10.);bool rejected=false;
    try {Peer::observe(c,angular?1.:1.8,angular?.8:0.,10.025,angular?0.:79.981883,angular?79.98:0.);}
    catch(const nav2_core::PlannerException & e) {
      rejected=std::string(e.what()).find("CORNER_LOCALIZATION_DISCONTINUITY")!=std::string::npos;
    }
    check(rejected,"feedback spike derived from pose jump cannot justify that same jump");
  }
  {
    ThreePhaseController c;Peer::init(c);c.setPlan(original);Peer::active(c,0);
    Peer::observe(c,1.,0.,10.);bool rejected=false;
    try {Peer::observe(c,1.4,.8,10.4,1.,2.);}
    catch(const nav2_core::PlannerException &) {rejected=true;}
    check(!rejected,"ordinary configured-speed translation and rotation remain accepted");
  }
  {
    ThreePhaseController c;Peer::init(c);c.setPlan(original);Peer::active(c,0);
    Peer::observe(c,1.,0.,10.);bool rejected=false;
    try {Peer::observe(c,1.002,.002,10.025,79.98,79.98);}
    catch(const nav2_core::PlannerException &) {rejected=true;}
    check(!rejected,"continuity check must test pose change, not reject velocity by itself");
  }
  return failures?1:0;
}
