#include "astribot_operator_station/evidence.hpp"
#include <iostream>
#include <limits>
int main(int argc, char ** argv) {
  try {
    if (argc < 2 || argc > 3) throw std::runtime_error("Usage: inspect_incident EVENTS.jsonl [sequence]");
    auto events = astribot_operator_station::read_events(argv[1]);
    auto sequence = argc == 3 ? std::stoull(argv[2]) : std::numeric_limits<uint64_t>::max();
    std::cout << astribot_operator_station::parameters_at(events, sequence).dump(2) << '\n';
    return 0;
  } catch (const std::exception & e) {std::cerr << e.what() << '\n'; return 1;}
}
