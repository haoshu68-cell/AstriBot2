#include <limits>
#include <iostream>
#include "astribot_s1_path_tracking/corridor_refinement.hpp"
#include "astribot_s1_path_tracking/arrival_braking.hpp"
#include "astribot_s1_path_tracking/arrival_settling.hpp"

int main() {
  using namespace astribot_s1_path_tracking;
  int failed=0;
  auto check=[&](bool ok,const char * description) {
    if(!ok) {std::cerr<<description<<'\n';++failed;}
  };
  constexpr double yaw_tolerance=.0017453292519943296;
  check(directCandidateHeadingCompatible(M_PI-.03,-M_PI+.011,true),"offset approach generates swept straight candidate");
  check(!directCandidateHeadingCompatible(M_PI-.03,-M_PI+.011,false),"legacy candidate eligibility unchanged");
  check(!directCandidateHeadingCompatible(.051,0.,true),"candidate beyond approach window is rejected");
  check(!directCandidateHeadingCompatible(M_PI,0.,true),"opposed endpoint keeps SE2 semantics");
  // The observed failure entered REFINE with remaining translation and a
  // small yaw residual. It must not be confused with a stopped turn request.
  check(corridorRefinementReachable(0.,.008,false,yaw_tolerance),"moving approach can converge yaw");
  check(!corridorRefinementReachable(0.,.008,true,yaw_tolerance),"unmet stopped heading is rejected");
  check(corridorRefinementReachable(0.,.0005,true,yaw_tolerance),"aligned settled goal remains reachable");
  check(!corridorRefinementReachable(3.141592653589793,0.,false,yaw_tolerance),"inside turn-around is rejected");
  check(!corridorRefinementReachable(.051,.002,false,yaw_tolerance),"goal outside corridor heading is rejected");
  check(!corridorRefinementReachable(0.,std::numeric_limits<double>::quiet_NaN(),false,yaw_tolerance),"invalid pose is rejected");
  check(corridorRefinementReachable(0.,.003,false,yaw_tolerance),"braking residual waits for stopped evidence");
  check(!corridorRefinementReachable(0.,.003,true,yaw_tolerance),"settled residual is explicitly rejected");
  check(corridorTerminalWithinBounds(.0009,.098*3.141592653589793/180.,0.,0.,.002,yaw_tolerance),
    "observed post-braking pose meets the unchanged hard tolerance");
  check(!corridorTerminalWithinBounds(.0009,.098*3.141592653589793/180.,0.,.0001,.002,yaw_tolerance),
    "residual rotation must also fit the hard tolerance");
  check(!corridorTerminalWithinBounds(.0021,0.,0.,0.,.002,yaw_tolerance),"translation overshoot fails");
  // Replay a zero-command terminal coast: crossing XY tolerance while yaw is
  // still braking cannot reject or certify arrival before the source window.
  for (double final_error : {.0008,.003}) {
    ArrivalSettling<1> settling;settling.command(true,1.);
    for(int i=0;i<=100;++i) {
      const double t=1.+i*.02;
      const double error=final_error+std::max(0.,1.-i*.04)*.002;
      settling.observe(t,t,{error},.6,.001,.2*yaw_tolerance,true);
      if(i<25)check(!settling.evidence().stopped,"braking transient never certifies stopped");
    }
    check(settling.evidence().stopped,"fresh stationary window is obtained");
    check(corridorRefinementReachable(0.,final_error,true,yaw_tolerance)==(final_error<yaw_tolerance),
      "settled success and failure use measured final heading");
  }
  check(corridorAngularCorrectionAllowed(.05,.03),"coupled correction during measured translation");
  check(!corridorAngularCorrectionAllowed(.05,0.),"no turn from rest");
  check(!corridorAngularCorrectionAllowed(0.,.03),"no turn while translation command is coasting");
  check(!corridorAngularCorrectionAllowed(.004,.004),"no turn at near-stop command");
  check(!corridorAngularCorrectionAllowed(.02,.002),"no turn at near-stop measurement");
  check(corridorApproachHeadingGain(.1,.008,.008,.8,.002)==.8,"far approach keeps existing gain");
  check(corridorApproachHeadingGain(.011,.008,.004,.8,.002)>1.7,"reserve heading convergence before XY stop");
  check(corridorApproachHeadingGain(.003,.008,.004,.8,.002)==3.,"near approach gain remains bounded");
  check(corridorApproachHeadingGain(.003,0.,0.,.8,.002)==.8,"zero speed does not create heading urgency");
  // Bounded translating model: finish yaw before entering the terminal stop.
  // This only checks the coupled law, not Gazebo wheel/contact dynamics.
  double legacy_error=.012,coupled_error=.012;
  for(double remaining=.02;remaining>.002;remaining-=.008*.01) {
    legacy_error-=.8*legacy_error*.01;
    coupled_error-=corridorApproachHeadingGain(remaining,.008,.008,.8,.002)*coupled_error*.01;
  }
  check(legacy_error>yaw_tolerance && coupled_error<yaw_tolerance,"coupled approach preserves yaw correction distance");
  // Reproduce the observed failure mechanism: after a braking zero, the
  // translating wheels keep yaw unsettled. Independent COAST never restarts.
  ArrivalCoast<1> legacy,coupled;
  for(double request:{.02,0.,-.004,-.008}) {
    const double old=applyArrivalYawCoast(legacy,request,false,false);
    const double moving=applyArrivalYawCoast(coupled,request,false,true);
    if(request<0.)check(old==0. && moving<0.,"corridor drift stays correctable during translation");
  }
  check(!coupled.active() && legacy.active(),"legacy coast behavior retained");
  check(applyArrivalYawCoast(legacy,-.008,true,false)==0.,"legacy requires stopped window before restart");
  check(applyArrivalYawCoast(legacy,-.008,false,false)<0.,"legacy restarts after settling");
  return failed ? 1 : 0;
}
