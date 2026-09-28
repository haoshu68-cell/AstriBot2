#include <cassert>
#include <limits>
#include "astribot_s1_robot_geometry/lease_time.hpp"
using namespace astribot_s1_robot_geometry;
int main() {
  LeaseSample old{1000000000,1000000000,1300000000,.3,true,"a"};
  auto next=old;next.stamp_ns=next.limits_stamp_ns=1100000000;next.valid_until_ns=1400000000;
  LeaseSelector gate;
  assert(gate.select({old,next},1099000000)==0);
  assert(gate.select({old,next},1100000000)==1);
  assert(gate.select({old,next},1399999999)==1);
  assert(gate.select({old,next},1400000000)==-1);
  assert(gate.select({old,next},1100000000)==-1); // rollback latches invalid
  for (int change=0;change<4;++change) {
    LeaseSelector changed;auto sample=next;
    if(change==0)sample.context="b";
    if(change==1)sample.valid_until_ns=1150000000;
    if(change==2)sample.lease_s=.1;
    if(change==3)sample.allowed=false;
    assert(changed.select({old,sample},1099000000)==(change==3?1:-1));
  }
  LeaseSelector expired;next.stamp_ns=next.limits_stamp_ns=9000000000;next.valid_until_ns=9300000000;
  assert(expired.select({old,next},1299999999)==0);
  assert(expired.select({old,next},1300000000)==-1);
  for(double bad:{0.,-.1,.6,std::numeric_limits<double>::quiet_NaN()}) {
    LeaseSelector malformed;old.lease_s=bad;
    assert(malformed.select({old},1000000000)==-1);
  }
}
