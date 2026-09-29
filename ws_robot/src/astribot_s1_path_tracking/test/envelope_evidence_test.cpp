#include <cassert>
#include "astribot_s1_path_tracking/envelope_evidence.hpp"
using E=astribot_s1_path_tracking::EnvelopeEvidence;
auto message(int64_t stamp) {
 auto m=std::make_shared<E::Message>();m->header.stamp.sec=stamp/1000000000;
 m->header.stamp.nanosec=stamp%1000000000;m->limits.stamp=m->header.stamp;
 m->valid_until=m->header.stamp;m->header.frame_id="base";m->mode=E::Message::FIXED_POSTURE;
 m->coordinator_session_id="session";m->installed_geometry_hash="geometry";
 m->epoch=1;m->navigation_allowed=m->limits.transport_ready=true;
 return m;
}
int main() {
 E gate;const auto t=E::Wall::now();auto old=message(1000000000),next=message(9000000000);
 gate.accept(old,1000000000,t);gate.accept(next,1099000000,t);
 assert(gate.sample(1099000000).message==next);
 assert(E::navigationReason(gate.sample(1),"base",1,t+std::chrono::hours(1)).empty());
 assert(E::navigationReason(gate.sample(99000000000),"base",99000000000,t).empty());
 gate.accept(old,1,t);assert(gate.sample(1).message==next);
 auto negative=message(10000000000);negative->navigation_allowed=false;
 gate.accept(negative,1,t);assert(E::navigationReason(gate.sample(1),"base",1,t).find("REVOKED")!=std::string::npos);
 gate.accept(next,1,t);assert(gate.sample(1).message==negative);
 auto new_version=message(1);new_version->epoch=2;gate.accept(new_version,1,t);
 assert(gate.sample(1).message==new_version);
 assert(!E::sameExecution(*new_version,*negative));
 auto session=message(0);session->coordinator_session_id="restart";gate.accept(session,1,t);
 assert(gate.sample(1).message==session);
 assert(E::navigationReason({},"base",1,t)=="ENVELOPE_MISSING");
 assert(E::navigationReason(gate.sample(1),"wrong",1,t)=="ENVELOPE_FRAME_MISMATCH");
 auto conflict=std::make_shared<E::Message>(*session);conflict->installed_geometry_hash="changed";
 gate.accept(conflict,1,t);assert(!gate.sample(1).message);
}
