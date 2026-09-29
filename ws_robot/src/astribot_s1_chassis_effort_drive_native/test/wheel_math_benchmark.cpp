#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "astribot_s1_chassis_effort_drive_native/wheel_math.hpp"

int main(int argc, char **argv) {
  const int count = argc > 1 ? std::atoi(argv[1]) : 500000;
  const int repeats = argc > 2 ? std::atoi(argv[2]) : 7;
  std::vector<double> samples;
  samples.reserve(repeats);
  double checksum = 0.0;
  for (int repeat = 0; repeat < repeats; ++repeat) {
    astribot::chassis_effort::WheelLoop loop;
    checksum = 0.0;
    const auto start = std::chrono::steady_clock::now();
    for (int index = 0; index < count; ++index) {
      const auto result = loop.update(
          2.0 + static_cast<double>(index % 13) * 0.01, 0.2,
          0.4, 0.1, 0.0, 0.1, 1.0, 0.05, 15.0, 0.01);
      checksum += result[0] + result[1];
    }
    const auto finish = std::chrono::steady_clock::now();
    samples.push_back(std::chrono::duration<double>(finish - start).count());
  }
  std::sort(samples.begin(), samples.end());
  const double median = samples[samples.size() / 2];
  std::printf("direct_cpp %.6f s %.0f/s checksum=%.9e\n",
              median, static_cast<double>(count) / median, checksum);
  return 0;
}
