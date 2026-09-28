#include "corner_fixture.hpp"

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  using namespace astribot_s1_path_tracking;using Peer=ThreePhaseControllerTestPeer;
  auto costmap=std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides({
      rclcpp::Parameter("plugins",std::vector<std::string>{}),
      rclcpp::Parameter("global_frame",std::string("map")),
      rclcpp::Parameter("robot_base_frame",std::string("base_link")),
      rclcpp::Parameter("track_unknown_space",false)}));
  if(costmap->on_configure(rclcpp_lifecycle::State())!=nav2_util::CallbackReturn::SUCCESS)
    throw std::runtime_error("corner fixture costmap configure failed");
  costmap->getCostmap()->resizeMap(100,100,.05,-2.5,-2.5);
  auto footprint=std::make_shared<geometry_msgs::msg::Polygon>();
  for(const auto &xy:std::vector<std::pair<float,float>>{{-.2f,-.2f},{.2f,-.2f},{.2f,.2f},{-.2f,.2f}}) {
    geometry_msgs::msg::Point32 p;p.x=xy.first;p.y=xy.second;footprint->points.push_back(p);
  }
  costmap->setRobotFootprintPolygon(footprint);
  int failures=0,checks=0;
  auto check=[&](bool ok,const char * name) {++checks;if(!ok){++failures;std::cerr<<"FAIL "<<name<<'\n';}};
  const auto original=path({{0,0},{1,0},{1,1},{2,1}},100);
  for (double gap : {0.1,0.49,0.5,0.51,1.0}) {
    ThreePhaseController c;Peer::init(c);c.setPlan(original);Peer::completed(c,1);Peer::active(c,1);
    Peer::gap(c,gap);auto stale=original;stale.header.stamp.sec=99;bool rejected=false;
    try {c.setPlan(stale);}catch(const nav2_core::PlannerException &){rejected=true;}
    check(rejected,"old plan must be rejected independent of scheduler gap");
    check(Peer::plan(c).header.stamp.sec==100 && Peer::cursor(c)==1 && c.phase()==Phase::kAlignCorner,
      "rejected old path preserves route, progress and phase");
    auto fresh=original;fresh.header.stamp.sec=101;c.setPlan(fresh);
    check(c.phase()==Phase::kAlignCorner && Peer::cursor(c)==1,
      "equivalent refresh after a gap must retain active corner");
  }
  {
    ArrivalController c;ArrivalControllerTestPeer::setup(c,costmap);c.setPlan(original);
    Peer::completed(c,1);Peer::active(c,1);auto fresh=original;fresh.header.stamp.sec=101;c.setPlan(fresh);
    check(c.phase()==Phase::kAlignCorner && Peer::cursor(c)==1,
      "P3/P4/P5 equivalent refresh cannot overwrite corner alignment");
  }
  {
    ArrivalController c;ArrivalControllerTestPeer::setup(c,costmap);auto execution=Peer::managed(c);
    c.setPlan(original);check(Peer::plan(c).poses.empty(),"setPlan waits for explicit execution generation");
    bool missing=false;try {Peer::commit(c);}catch(const nav2_core::PlannerException &){missing=true;}
    check(missing,"missing lifecycle owner cannot execute a staged path");
    execution->begin();Peer::commit(c);Peer::completed(c,1);Peer::active(c,1);
    auto refresh=original;refresh.header.stamp.sec=101;c.setPlan(refresh);Peer::commit(c);
    check(c.phase()==Phase::kAlignCorner && Peer::cursor(c)==1,"same generation refresh preserves corner");
    check(execution->generation()==1,"path refresh never creates an execution");
    // Model Nav2 order: setPlan -> progress reset -> first control tick.
    c.setPlan(original);execution->begin();Peer::commit(c);
    check(Peer::cursor(c)==0 && c.phase()==Phase::kAlignStart,
      "rapid cancel/restart of same cached route starts a new execution");
    check(Peer::plan(c).header.stamp.sec==100,"new execution may use its explicitly supplied cached path");
    c.setPlan(refresh);Peer::commit(c);Peer::active(c,0);Peer::gap(c,1.);
    c.setPlan(original);bool stale=false;try {Peer::commit(c);}catch(const nav2_core::PlannerException &){stale=true;}
    check(stale && Peer::plan(c).header.stamp.sec==101,"managed generation rejects late old plan after gap");
    bool retried=false;try {Peer::commit(c);}catch(const nav2_core::PlannerException &){retried=true;}
    check(retried,"failure_tolerance retry must not resume old route after rejected staged plan");
  }
  {
    ArrivalController c;ArrivalControllerTestPeer::setup(c,costmap);c.setPlan(original);
    Peer::completed(c,1);Peer::active(c,1);Peer::observed(c,1.,.5);
    const double timer=Peer::timer(c);c.setPlan(path({{1,.5},{1,1},{2,1}},101));
    check(Peer::settling(c) && Peer::cursor(c)==0,"changed route reanchors after a stop");
    check(Peer::timer(c)==timer,"outer takeover must not renew base replacement budget");
  }
  {
    ArrivalController c;ArrivalControllerTestPeer::setup(c,costmap);c.setPlan(original);
    Peer::active(c,0);Peer::observed(c,1.,.5);ArrivalControllerTestPeer::refine(c);
    c.setPlan(path({{1,.5},{1,1},{2,1}},101));
    check(!ArrivalControllerTestPeer::refining(c),"replacement route exits terminal refinement");
  }
  {
    int a=0,b=0;auto first=ControllerExecution::forNode(&a);auto same=ControllerExecution::forNode(&a);
    auto other=ControllerExecution::forNode(&b);first->begin();
    check(same->generation()==1 && other->generation()==0,"execution generations are scoped to one controller server");
  }
  for (double length : {.25,.3,.49,.5,.51}) {
    for (double sign : {-1.,1.}) {
      ThreePhaseController c;Peer::init(c);c.setPlan(path({{0,0},{length,0},{length,sign}},100));
      check(std::abs(Peer::heading(c))<1e-9,"start heading cannot look beyond first supported corner");
    }
  }
  for (double spacing : {.2,.249,.25,.251,.3}) {
    ThreePhaseController c;Peer::init(c);bool rejected=false;
    try {c.setPlan(path({{0,0},{1,0},{1,spacing},{2,spacing}},100));}
    catch(const nav2_core::PlannerException & error) {
      rejected=std::string(error.what()).find("CORNER_DENSE_GEOMETRY")!=std::string::npos;
    }
    check(rejected==(spacing<.25),"dense dogleg must have explicit unsupported result at minimum segment boundary");
  }
  {
    ThreePhaseController c;Peer::init(c);bool rejected=false;
    try {c.setPlan(path({{0,0},{.24,0},{.24,1}},100));}
    catch(const nav2_core::PlannerException & error) {
      rejected=std::string(error.what()).find("CORNER_SHORT_APPROACH")!=std::string::npos;
    }
    check(rejected,"too-short first approach must not silently become curve tracking");
  }
  {
    ThreePhaseController c;Peer::init(c);c.setPlan(original);Peer::capture(c);
    check(c.phase()==Phase::kCornerApproach,"one stationary source frame is not sustained corner stop evidence");
  }
  {
    ArrivalController c;ArrivalControllerTestPeer::setup(c,costmap);ArrivalControllerTestPeer::referenceBrake(c);c.setPlan(original);
    check(!Peer::earlyApproach(c,.60,.35),"corner does not enter before configured stop-distance budget");
    check(Peer::earlyApproach(c,.67,.35),"measured high speed starts corner braking earlier than fixed 25cm");
    double previous=0.;
    for(double speed : {0.,.06,.10,.20,.35}) {
      const double distance=ArrivalControllerTestPeer::stopDistance(c,speed);
      check(distance>=previous,"reference stop budget is monotone in measured speed");previous=distance;
    }
    ArrivalControllerTestPeer::hardwareBrake(c);
    check(ArrivalControllerTestPeer::stopDistance(c,.249)>.18,"hardware stop curve includes configured residual displacement");
    bool refused=false;try {ArrivalControllerTestPeer::stopDistance(c,.250);}catch(const nav2_core::PlannerException &){refused=true;}
    check(refused,"uncalibrated hardware speed cannot silently extrapolate stopping distance");
  }
  {
    ArrivalController c;ArrivalControllerTestPeer::setup(c,costmap);auto one=path({{1.,0.}},100);c.setPlan(one);
    ArrivalControllerTestPeer::refine(c);one.header.stamp.sec=101;c.setPlan(one);
    check(ArrivalControllerTestPeer::refining(c),"equivalent one-pose refresh must preserve terminal settling and budget");
    one.header.stamp.sec=99;bool refused=false;try {c.setPlan(one);}catch(const nav2_core::PlannerException &){refused=true;}
    check(refused,"one-pose routes retain stale revision rejection");
  }
  {
    ThreePhaseController c;Peer::init(c);c.setPlan(original);
    auto unversioned=original;unversioned.header.stamp.sec=0;c.setPlan(unversioned);
    auto old=original;old.header.stamp.sec=99;bool refused=false;
    try {c.setPlan(old);}catch(const nav2_core::PlannerException &){refused=true;}
    check(refused,"unstamped equivalent refresh must not erase accepted source watermark");
    c.setPlan(original);c.setPlan(unversioned);
    Peer::observed(c,1.,.5);refused=false;
    try {c.setPlan(path({{1,.5},{1,1},{2,1}},99));}catch(const nav2_core::PlannerException &){refused=true;}
    check(refused,"changed old route cannot bypass watermark through unstamped equivalent refresh");
  }
  for(bool align : {false,true}) {
    ArrivalController c;ArrivalControllerTestPeer::setup(c,costmap);Peer::time(c,200.);c.setPlan(original);
    Peer::observed(c,1.,.5);c.setPlan(path({{1,.5},{1,1},{2,1}},101));
    ArrivalGoalChecker checker;
    const auto command=ArrivalControllerTestPeer::corridorTick(c,checker,200.,align);
    check(std::hypot(command.linear.x,command.linear.y)==0. && command.angular.z==0.,
      "corridor policy motion cannot bypass replacement settling on one stationary frame");
    check(Peer::settling(c),"replacement settling remains pending before source window is full");
    bool moving=false;
    for(int i=1;i<=9;++i) {
      const auto next=ArrivalControllerTestPeer::corridorTick(c,checker,200.+i*.1,align);
      moving=moving || std::hypot(next.linear.x,next.linear.y)>0. || next.angular.z!=0.;
    }
    check(moving && !Peer::settling(c),"corridor policy resumes after sustained stop evidence");
  }
  {
    ArrivalController c;ArrivalControllerTestPeer::setup(c,costmap);Peer::time(c,200.);c.setPlan(original);
    Peer::observed(c,1.,.5);c.setPlan(path({{1,.5},{1,1},{2,1}},101));
    ArrivalGoalChecker checker;bool zero=true;
    for(int i=0;i<=8;++i) {
      const auto command=ArrivalControllerTestPeer::corridorTick(c,checker,200.+i*.1,false,true);
      zero=zero && std::hypot(command.linear.x,command.linear.y)==0. && command.angular.z==0.;
    }
    check(zero && Peer::settling(c),"HOLD keeps motion stopped and leaves replacement ownership intact");
    const auto resumed=ArrivalControllerTestPeer::corridorTick(c,checker,200.9,false);
    check(std::hypot(resumed.linear.x,resumed.linear.y)>0. && !Peer::settling(c),
      "valid source stop window from HOLD is reused without duplicate settling wait");
  }
  {
    ThreePhaseController c;Peer::init(c);c.setPlan(original);
    auto unversioned=original;unversioned.header.stamp.sec=0;c.setPlan(unversioned);
    bool refused=false;try {c.setPlan(path({{0.,0.},{3.,0.}},99));}
    catch(const nav2_core::PlannerException &){refused=true;}
    check(refused,"changing goal coordinates within an execution cannot erase source watermark");
  }
  {
    ArrivalController c;ArrivalControllerTestPeer::setup(c,costmap);Peer::disableCorners(c);
    auto execution=Peer::managed(c);c.setPlan(original);execution->begin();Peer::commit(c);
    ArrivalControllerTestPeer::refine(c);Peer::gap(c,1.);
    auto refresh=original;refresh.header.stamp.sec=101;c.setPlan(refresh);Peer::commit(c);
    check(ArrivalControllerTestPeer::refining(c),
      "explicit execution identity also preserves corner-disabled refinement across a scheduler gap");
    c.setPlan(original);execution->begin();Peer::commit(c);
    check(!ArrivalControllerTestPeer::refining(c),"corner-disabled explicit restart clears old refinement");
  }
  for (double gap : {.1,60.}) {
    ArrivalController c;ArrivalControllerTestPeer::setup(c,costmap);Peer::time(c,200.);
    c.setPlan(original);ArrivalGoalChecker checker;
    ArrivalControllerTestPeer::corridorTick(c,checker,200.,false,true);
    Peer::time(c,200.+gap);c.setPlan(path({{1.,.5},{1.,1.},{2.,1.}},101));
    ArrivalControllerTestPeer::corridorTick(c,checker,200.+gap,false);
    check(std::abs(Peer::timer(c)-(200.+gap))<1e-8,
      "replacement phase cannot inherit pause time from before its creation");
  }
  {
    ArrivalController c;ArrivalControllerTestPeer::setup(c,costmap);Peer::time(c,200.);
    c.setPlan(original);ArrivalGoalChecker checker;
    ArrivalControllerTestPeer::corridorTick(c,checker,201.,false,true);
    Peer::time(c,260.);auto refresh=original;refresh.header.stamp.sec=101;c.setPlan(refresh);
    ArrivalControllerTestPeer::corridorTick(c,checker,260.,false);
    check(std::abs(Peer::timer(c)-259.)<1e-8,
      "equivalent refresh preserves the active time spent before HOLD");
  }
  {
    ArrivalController c;ArrivalControllerTestPeer::setup(c,costmap);Peer::time(c,200.);
    c.setPlan(original);Peer::active(c,0);ArrivalGoalChecker checker;
    ArrivalControllerTestPeer::corridorTick(c,checker,201.,false,true);
    Peer::time(c,260.);c.setPlan(path({{1.,.5},{1.,1.},{2.,1.}},101));
    ArrivalControllerTestPeer::corridorTick(c,checker,260.,false);
    check(std::abs(Peer::timer(c)-259.)<1e-8,
      "active corner replacement retains prior active budget and excludes HOLD only once");
  }
  if(costmap->on_cleanup(rclcpp_lifecycle::State())!=nav2_util::CallbackReturn::SUCCESS)
    throw std::runtime_error("corner fixture costmap cleanup failed");
  costmap.reset();rclcpp::shutdown();
  std::cout<<"checks="<<checks<<" failures="<<failures<<'\n';return failures?1:0;
}
