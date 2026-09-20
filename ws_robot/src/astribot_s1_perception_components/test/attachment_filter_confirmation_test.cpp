#include <iostream>
#include <stdexcept>
#include "astribot_s1_autonomy/attachment_filter_confirmation.hpp"

using astribot_s1_autonomy::AttachmentFilterConfirmation;
void check(bool value, const char * reason) {if (!value) throw std::runtime_error(reason);}
int main()
{
  AttachmentFilterConfirmation confirmation;
  auto valid = [&](double source, double wall, double snapshot=10.) {
      return confirmation.current("A",snapshot,true,10.,source,wall,.3);
    };
  check(!valid(10.,100.),"no confirmation before filtering");
  confirmation.applied("A",10.);
  check(valid(10.01,100.1),"first successful cloud");
  check(valid(10.1,100.7),"slow simulation preserves source-fresh confirmation");
  check(valid(10.2,101.3),"heartbeat does not require another cloud within 0.5 wall seconds");
  check(!valid(10.31,101.4),"heartbeat never refreshes acquisition time");
  check(!valid(10.32,101.5,10.32),"new snapshot cannot revive stale cloud");
  confirmation.applied("A",10.32);
  check(valid(10.33,101.6,10.32),"new successful cloud recovers");
  confirmation.invalidate();
  check(!valid(10.34,101.7,10.32),"invalid frame revokes confirmation");
  confirmation.applied("A",10.34);
  check(!confirmation.current("B",10.34,true,10.34,10.35,101.8,.3),"revision mismatch");
  confirmation.applied("A",10.34);
  check(!confirmation.current("A",10.34,true,10.35,10.36,101.9,.3),"cloud predates attachment change");
  confirmation.applied("A",10.36);
  check(!confirmation.current("A",10.36,false,10.36,10.37,102.,.3),"diff snapshot not authoritative");
  confirmation.applied("A",10.38);
  check(!valid(10.38,102.1,10.39),"future snapshot");
  confirmation.applied("A",10.40);
  check(!valid(10.39,102.2,10.39),"future cloud");
  confirmation.applied("A",10.4);
  check(!valid(10.4,102.3,9.8),"expired snapshot");
  confirmation.applied("A",10.4);
  check(valid(10.41,102.4,10.4),"resume before pause test");
  check(!valid(10.41,103.,10.4),"paused clock cannot heartbeat indefinitely");
  confirmation.applied("A",10.41);
  check(!valid(9.,103.1,9.),"clock rewind revokes old epoch");
  confirmation.applied("A",9.);
  check(confirmation.current("A",9.,true,9.,9.01,103.2,.3),"fresh filtering in new epoch recovers");
  check(!confirmation.current("A",9.,true,9.,NAN,103.3,.3),"nonfinite clock rejected");
  std::cout << "PASS 20 attachment confirmation source freshness, revision, invalidation and clock checks\n";
}
