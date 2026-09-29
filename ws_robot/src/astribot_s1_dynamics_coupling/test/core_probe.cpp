#include "astribot_s1_dynamics_coupling/core.hpp"
#include <iomanip>
#include <iostream>
#include <string>

double read_number() {
  std::string token;
  if (!(std::cin >> token)) throw std::invalid_argument("missing numeric input");
  return std::stod(token);  // Explicitly accepts the oracle's nan/inf spellings.
}

int main() {
  using namespace astribot_s1_dynamics_coupling;
  std::cout << std::setprecision(17);
  std::string op;
  while (std::cin >> op) {
    try {
      double a, b, c;
      if (op == "reach") {
        a = read_number(); b = read_number(); c = read_number();
        std::cout << reach_activity(a, b, c) << '\n';
      } else if (op == "xy") {
        a = read_number(); b = read_number();
        std::cout << horizontal_reach(a, b) << '\n';
      } else if (op == "scale") {
        a = read_number(); b = read_number();
        std::cout << scale_from_activity(a, b) << '\n';
      } else if (op == "joint" || op == "velocity") {
        std::size_t n, r;
        a = read_number(); std::cin >> n >> r;
        std::vector<std::string> names;
        std::vector<double> values(n), refs(r);
        for (std::size_t i = 0; i < n; ++i) {
          names.push_back("j" + std::to_string(i));
          values[i] = read_number();
        }
        for (auto & value : refs) value = read_number();
        auto positions = zip_map(names, values);
        std::cout << (op == "joint" ? joint_deviation_activity(positions, names, refs, a) :
          velocity_activity(positions, names, a)) << '\n';
      } else {
        return 2;
      }
    } catch (const std::exception &) {
      std::cout << "error\n";
    }
  }
}
