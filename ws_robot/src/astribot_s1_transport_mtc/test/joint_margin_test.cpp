#include "astribot_s1_transport_mtc/joint_margin.hpp"
#include <limits>
#include <iostream>

int main() {
  using astribot_s1_transport_mtc::planningInterval;
  const auto joint3=planningInterval(-3.1,3.1,.1);
  if (joint3.contains(3.1) || joint3.contains(3.08286) || joint3.contains(-3.1) ||
      !joint3.contains(3.) || !joint3.contains(-3.) || !joint3.contains(-.6) ||
      joint3.contains(std::numeric_limits<double>::quiet_NaN())) return 1;
  for (const double margin:{0.,-.1,.21,std::numeric_limits<double>::infinity()}) {
    try {planningInterval(-3.1,3.1,margin);return 2;} catch(const std::invalid_argument&) {}
  }
  try {planningInterval(-.05,.05,.1);return 3;} catch(const std::invalid_argument&) {}
  std::cout << "Observed hard-stop branch rejected; interior and invalid-bound cases passed\n";
}
