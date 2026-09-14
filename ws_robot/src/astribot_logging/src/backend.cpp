// Non-ROS Python clients use a small C ABI, avoiding Python-version-specific bindings.
// ROS nodes retain rcl_logging_spdlog and its rosout / throttling behavior.
#include <spdlog/logger.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <unistd.h>

namespace {
std::mutex mutex;
std::vector<spdlog::sink_ptr> sinks;
std::unordered_map<std::string, std::shared_ptr<spdlog::logger>> loggers;
thread_local std::string last_error;
struct FileLog {
  std::string path;
  size_t max_bytes;
  size_t backups;
  std::shared_ptr<spdlog::logger> logger;

  void open() {
    auto sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(path, max_bytes, backups);
    logger = std::make_shared<spdlog::logger>(path, sink);
    logger->set_pattern("%v");
    logger->flush_on(spdlog::level::trace);
    logger->set_error_handler([](const std::string & error) {throw std::runtime_error(error);});
  }
};
std::unordered_map<std::string, FileLog> files;
}

// File-only sinks for launch and subprocess output. Never echo back into the pipe.
extern "C" int astribot_file_open(const char * path, size_t max_bytes, size_t backups)
{
  try {
    std::lock_guard<std::mutex> lock(mutex);
    if (files.count(path)) {return 0;}
    FileLog file{path, max_bytes, backups, nullptr};
    file.open();
    files.emplace(path, std::move(file));
    return 0;
  } catch (const std::exception & e) {last_error = e.what(); return -1;}
}

extern "C" int astribot_file_write(const char * path, const char * message)
{
  try {
    std::lock_guard<std::mutex> lock(mutex);
    auto & file = files.at(path);
    // A removed file must not silently keep receiving logs through an unlinked fd.
    if (!std::filesystem::is_regular_file(path)) {
      file.open();
      file.logger->warn("[astribot_logging] log file was removed; reopened (earlier content is unavailable)");
    }
    file.logger->info("{}", message);
    return 0;
  } catch (const std::exception & e) {last_error = e.what(); return -1;}
}

extern "C" const char * astribot_log_error() {return last_error.c_str();}

extern "C" int astribot_log_initialize(
  const char * directory, size_t max_bytes, size_t backups)
{
  try {
    std::lock_guard<std::mutex> lock(mutex);
    if (!sinks.empty()) {return 0;}
    std::filesystem::create_directories(directory);
    const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const auto file = std::filesystem::path(directory) /
      ("astribot_" + std::to_string(getpid()) + "_" + std::to_string(stamp) + ".log");
    auto console = std::make_shared<spdlog::sinks::stderr_sink_mt>();
    console->set_pattern("[%l] [%E.%f] [%n]: %v");
    sinks = {console};
    // A supervisor owns the only disk sink when process output is collected.
    const char * captured = std::getenv("ASTRIBOT_LOG_CAPTURE");
    if (!captured || std::string(captured) != "1") {
      auto disk = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        file.string(), max_bytes, backups);
      disk->set_pattern("[%l] [%E.%f] [%n]: %v");
      sinks.insert(sinks.begin(), disk);
    }
    return 0;
  } catch (const std::exception & e) {last_error = e.what(); return -1;}
}

extern "C" int astribot_log_write(const char * name, int level, const char * message)
{
  try {
    std::lock_guard<std::mutex> lock(mutex);
    if (sinks.empty()) {throw std::runtime_error("spdlog backend is not initialized");}
    auto & logger = loggers[name];
    if (!logger) {
      logger = std::make_shared<spdlog::logger>(name, sinks.begin(), sinks.end());
      logger->set_level(spdlog::level::trace);
      logger->set_error_handler([](const std::string & error) {
          throw std::runtime_error(error);
        });
      // No background worker and no changes to ROS's global spdlog registry.
      logger->flush_on(spdlog::level::trace);
    }
    const auto severity = level >= 50 ? spdlog::level::critical :
      level >= 40 ? spdlog::level::err : level >= 30 ? spdlog::level::warn :
      level >= 20 ? spdlog::level::info : spdlog::level::debug;
    logger->log(severity, "{}", message);
    return 0;
  } catch (const std::exception & e) {last_error = e.what(); return -1;}
}
