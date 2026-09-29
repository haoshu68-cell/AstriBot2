#include "astribot_graspnet_runtime/cloud_io.hpp"
#include <cassert>
#include <filesystem>
#include <fstream>
#include <limits>
#include <unistd.h>

using astribot_graspnet_runtime::read_cloud;
int main() {
  auto p = std::filesystem::temp_directory_path() /
    ("graspnet_io_test_" + std::to_string(getpid()) + ".bin");
  auto write = [&](const std::vector<float>& v) {
    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    f.write(reinterpret_cast<const char*>(v.data()), v.size() * sizeof(float));
  };
  auto rejected = [&]() {
    try { read_cloud(p.string()); } catch (const std::exception&) { return true; }
    return false;
  };
  write({}); assert(rejected());
  write({0, 0, 1}); assert(rejected());
  std::vector<float> data(2048 * 3, 0.0f);
  for (std::size_t i = 0; i < 2048; ++i) data[i*3+2] = 1.0f;
  write(data);
  auto result = read_cloud(p.string());
  assert(result.size() == 6144 && result[2] == 1.0f);
  data[50] = std::numeric_limits<float>::quiet_NaN();
  write(data); assert(rejected());
  data[50] = -1.0f;
  write(data); assert(rejected());
  { std::ofstream f(p, std::ios::binary | std::ios::app); f.put('x'); }
  assert(rejected());
  std::filesystem::remove(p);
}
