#include "astribot_s1_manipulation_perception/worker_process.hpp"
#include <cerrno>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <filesystem>
#include <signal.h>
#include <spawn.h>
#include <stdexcept>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
extern char **environ;
namespace astribot::inference {
JobDirectory::JobDirectory() {
  char name[] = "/tmp/astribot_inference_XXXXXX";
  auto *dir = mkdtemp(name);
  if (!dir)
    throw std::runtime_error("JOB_DIRECTORY_FAILED");
  path_ = dir;
}
JobDirectory::~JobDirectory() {
  std::error_code error;
  std::filesystem::remove_all(path_, error);
}
WorkerResult run_worker(const std::vector<std::string> &args,
                        const std::string &output, double timeout,
                        const std::function<bool()> &canceled) {
  WorkerResult result;
  const auto start = std::chrono::steady_clock::now();
  if (args.empty() || args.front().empty() || args.front()[0] != '/' ||
      !std::isfinite(timeout) || timeout <= 0) {
    result.reason = "INVALID_WORKER_CONFIG";
    return result;
  }
  int fd = open(output.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
  if (fd < 0) {
    result.reason = "WORKER_OUTPUT_FAILED";
    return result;
  }
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, fd, STDOUT_FILENO);
  posix_spawn_file_actions_adddup2(&actions, fd, STDERR_FILENO);
  posix_spawn_file_actions_addclose(&actions, fd);
  posix_spawnattr_t attr;
  posix_spawnattr_init(&attr);
  posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP);
  posix_spawnattr_setpgroup(&attr, 0);
  std::vector<char *> argv;
  for (const auto &s : args)
    argv.push_back(const_cast<char *>(s.c_str()));
  argv.push_back(nullptr);
  pid_t pid = -1;
  const int error = posix_spawn(&pid, args.front().c_str(), &actions, &attr,
                                argv.data(), environ);
  posix_spawn_file_actions_destroy(&actions);
  posix_spawnattr_destroy(&attr);
  close(fd);
  if (error) {
    result.reason = "WORKER_SPAWN_FAILED";
    return result;
  }
  int status = 0;
  for (;;) {
    result.elapsed_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();
    // Cancellation/deadline wins over simultaneous process completion.
    if (canceled() || result.elapsed_sec >= timeout) {
      result.reason =
          result.elapsed_sec >= timeout ? "INFERENCE_TIMEOUT" : "CANCELED";
      kill(-pid, SIGKILL);
      while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {
      }
      break;
    }
    auto state = waitpid(pid, &status, WNOHANG);
    if (state == pid) {
      result.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
      result.reason = result.exit_code == 0 ? "OK" : "WORKER_FAILED";
      break;
    }
    if (state < 0 && errno != EINTR) {
      kill(-pid, SIGKILL);
      while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {
      }
      result.reason = "WORKER_WAIT_FAILED";
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return result;
}
} // namespace astribot::inference
