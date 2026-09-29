#include "astribot_s1_navigation_policy_native/policy_profile.hpp"
#include <iostream>
#include <string>

// Test-only process boundary for comparing the complete resolved profile.
int main(int argc, char** argv) {
  if (argc != 3) return 2;
  try {
    std::cout << astribot::navigation::load_policy_profile(argv[1], std::string(argv[2]) == "true")
                     .dump() << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
