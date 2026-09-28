#include "astribot_s1_path_tracking/corner_stop_evidence.hpp"
#include <cassert>
#include <iostream>

using astribot_s1_path_tracking::CornerStopEvidence;
bool sample(CornerStopEvidence & window,double stamp,double now,double x=0.,double yaw=0.) {
  return window.observe(stamp,now,x,0.,yaw,.6,.5,.02,.01,.01);
}
int main() {
  {
    CornerStopEvidence w;
    for(int i=0;i<30;++i) {assert(!sample(w,10.+i*.05,10.+i*.05));}
    w.command(0.,0.,0.,12.);
    for(int i=1;i<=12;++i) {assert(!sample(w,12.+i*.05,12.+i*.05));}
    assert(sample(w,12.65,12.65));
    assert(sample(w,12.70,12.70)); // no repeated settling timer
    w.command(.02,0.,0.,12.7);assert(!sample(w,12.75,12.75));
  }
  {
    CornerStopEvidence w;w.command(0.,0.,0.,10.);
    for(int i=0;i<30;++i) {assert(!sample(w,10.05,10.05+i*.02));}
  }
  {
    CornerStopEvidence w;w.command(0.,0.,0.,10.);
    for(int i=0;i<=6;++i) {sample(w,10.+i*.1,10.+i*.1);}
    assert(sample(w,10.6,10.61,.001)); // duplicate source is ignored
    assert(sample(w,10.5,10.62));     // out of order is ignored
    assert(sample(w,11.5,10.7));     // future source follows the steady stop window
    w.command(0.,0.,0.,12.);
    assert(sample(w,11.,12.));       // old source cannot replace latest evidence
  }
  {
    CornerStopEvidence w;w.command(0.,0.,0.,10.);
    for(int i=0;i<20;++i) {assert(!sample(w,10.+i*.1,10.+i*.1,i*.003));}
    w.command(0.,0.,0.,13.);
    for(int i=0;i<=6;++i) {sample(w,13.+i*.1,13.+i*.1,.06);}
    assert(sample(w,13.7,13.7,.06));
  }
  {
    CornerStopEvidence w;w.command(0.,0.,0.,10.);
    const double pi=std::acos(-1.);
    for(int i=0;i<=6;++i) {sample(w,10.+i*.1,10.+i*.1,0.,i<3?pi-.001:-pi+.001);}
    assert(sample(w,10.7,10.7,0.,-pi+.001));
    w.command(0.,0.,.1,10.7);assert(!sample(w,10.8,10.8));
  }
  {
    CornerStopEvidence w;w.command(0.,0.,0.,10.);
    for(int i=0;i<=6;++i) {sample(w,10.+i*.1,10.+i*.1);}
    assert(sample(w,10.7,10.7));
    assert(!sample(w,11.3,11.3)); // a new sparse window lacks three measured samples
    assert(!sample(w,9.,9.));    // clock rollback
  }
  std::cout<<"6 source-time stop-window scenario groups passed\n";
}
