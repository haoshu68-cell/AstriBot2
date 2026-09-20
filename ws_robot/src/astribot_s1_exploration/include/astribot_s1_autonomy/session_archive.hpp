#pragma once
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <regex>
#include <array>
#include <openssl/evp.h>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

namespace astribot_s1_autonomy {
namespace fs = std::filesystem;
inline std::string sessionHash(const fs::path & file) {
  std::ifstream in(file, std::ios::binary);
  if (!in) throw std::runtime_error("Cannot read " + file.string());
  auto ctx = std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!ctx || EVP_DigestInit_ex(ctx.get(), EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("SHA256 initialization failed");
  std::array<char, 65536> block{};
  while (in) {in.read(block.data(), block.size());
    if (EVP_DigestUpdate(ctx.get(), block.data(), in.gcount()) != 1) throw std::runtime_error("SHA256 failed");}
  if (!in.eof()) throw std::runtime_error("File read failed");
  unsigned char hash[EVP_MAX_MD_SIZE]; unsigned length = 0;
  if (EVP_DigestFinal_ex(ctx.get(), hash, &length) != 1) throw std::runtime_error("SHA256 failed");
  std::ostringstream out;
  for (unsigned i = 0; i < length; ++i) out << std::hex << std::setw(2) << std::setfill('0') << unsigned(hash[i]);
  return out.str();
}
// Compatible with the existing slam_session.inspect_session manifest contract.
inline nlohmann::json inspectSession(const fs::path & directory) {
  const auto dir = fs::canonical(directory);
  const auto name = dir.filename().string();
  const auto yaml = dir / (name + ".yaml");
  const auto config = YAML::LoadFile(yaml.string());
  const auto image = fs::canonical(dir / config["image"].as<std::string>());
  if (image.parent_path() != dir) throw std::runtime_error("Map image escapes session directory");
  const auto resolution = config["resolution"].as<double>();
  if (!std::isfinite(resolution) || resolution <= 0 || config["origin"].size() != 3)
    throw std::runtime_error("Invalid map geometry");
  for (const auto & v : config["origin"]) if (!std::isfinite(v.as<double>())) throw std::runtime_error("Invalid origin");
  std::ifstream pgm(image, std::ios::binary);
  std::string header(4096, '\0'); pgm.read(header.data(), header.size()); header.resize(pgm.gcount());
  std::smatch match;
  const std::regex pattern(R"(^P5\s+(?:#[^\n]*\n\s*)*(\d+)\s+(\d+)\s+255[ \r]?\n)");
  if (!std::regex_search(header, match, pattern)) throw std::runtime_error("Invalid PGM header");
  const auto width = std::stoull(match[1]), height = std::stoull(match[2]);
  const auto bytes = fs::file_size(image);
  if (!width || !height || bytes < size_t(match.length()) ||
      (bytes - match.length()) / width != height || (bytes - match.length()) % width)
    throw std::runtime_error("Incomplete PGM pixels");
  const auto poses = dir / "alidarState.txt";
  std::ifstream pose_file(poses); std::string row; size_t scans = 0;
  while (std::getline(pose_file, row)) {
    std::istringstream stream(row); std::array<double, 26> values{};
    for (auto & v : values) if (!(stream >> v) || !std::isfinite(v)) throw std::runtime_error("Invalid scan pose");
    std::string extra; if (stream >> extra) throw std::runtime_error("Invalid scan pose width");
    double norm = 0; for (size_t i = 4; i < 8; ++i) norm += values[i] * values[i];
    if (std::abs(norm - 1.) > 1e-3) throw std::runtime_error("Invalid scan quaternion");
    ++scans;
  }
  if (!scans || !pose_file.eof()) throw std::runtime_error("Missing scan poses");
  nlohmann::json hashes = nlohmann::json::object(); size_t clouds = 0;
  for (const auto & file : {yaml, image, poses}) hashes[file.filename().string()] = sessionHash(file);
  for (const auto & entry : fs::directory_iterator(dir / "kf")) {
    if (entry.path().extension() != ".pcd") continue;
    const auto id = entry.path().stem().string();
    if (!std::regex_match(id, std::regex("[0-9]+")) || std::stoull(id) >= scans || entry.file_size() < 100)
      throw std::runtime_error("Invalid keyframe cloud");
    hashes["kf/" + entry.path().filename().string()] = sessionHash(entry.path()); ++clouds;
  }
  if (!clouds) throw std::runtime_error("No keyframes");
  return {{"format_version", 1}, {"backend", "voxel_slam"}, {"session", name},
    {"world_frame", "map"}, {"map_yaml", yaml.filename().string()}, {"keyframes", clouds},
    {"scans", scans}, {"sha256", hashes}};
}
inline void commitSession(const fs::path & dir, nlohmann::json result, size_t updates) {
  const auto target = dir / "manifest.json";
  if (fs::exists(target)) {
    std::ifstream in(target); nlohmann::json old; in >> old;
    for (auto it = result.begin(); it != result.end(); ++it)
      if (!old.contains(it.key()) || old[it.key()] != it.value()) throw std::runtime_error("Existing manifest conflict");
    return;
  }
  result["final_pose_updates"] = updates;
  std::ofstream out(dir / "manifest.json.tmp"); out << result.dump(2) << '\n'; out.close();
  if (!out) throw std::runtime_error("Cannot write manifest");
  fs::rename(dir / "manifest.json.tmp", target);
}
}
