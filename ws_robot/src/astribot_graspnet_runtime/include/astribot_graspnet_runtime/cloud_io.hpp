#pragma once
#include <cmath>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace astribot_graspnet_runtime {
// Wire format: headerless little-endian IEEE754 float32 camera-optical XYZ, metres.
inline std::vector<float> read_cloud(const std::string& path) {
  static_assert(sizeof(float) == 4, "float32 wire protocol required");
  const std::uint32_t endian = 1;
  if (*reinterpret_cast<const unsigned char*>(&endian) != 1)
    throw std::runtime_error("only little-endian hosts are supported");
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  if (!stream) throw std::runtime_error("cannot open cloud input");
  const auto bytes = stream.tellg();
  constexpr std::streamoff min_bytes = 2048 * 3 * sizeof(float);
  constexpr std::streamoff max_bytes = 2000000 * 3 * sizeof(float);
  if (bytes < min_bytes || bytes > max_bytes || bytes % 12 != 0)
    throw std::runtime_error("cloud must contain 2048..2000000 XYZ float32 points");
  std::vector<float> values(static_cast<std::size_t>(bytes) / sizeof(float));
  stream.seekg(0);
  if (!stream.read(reinterpret_cast<char*>(values.data()), bytes))
    throw std::runtime_error("incomplete cloud read");
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (!std::isfinite(values[i])) throw std::runtime_error("nonfinite input");
    if (i % 3 == 2 && values[i] <= 0.0f)
      throw std::runtime_error("cloud must use optical frame with positive depth");
  }
  return values;
}
}  // namespace astribot_graspnet_runtime
