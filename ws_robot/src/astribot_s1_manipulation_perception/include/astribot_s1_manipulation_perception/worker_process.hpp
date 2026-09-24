#pragma once
#include <functional>
#include <string>
#include <vector>
namespace astribot::inference {
class JobDirectory {
public:
  JobDirectory();
  ~JobDirectory();
  JobDirectory(const JobDirectory &) = delete;
  JobDirectory &operator=(const JobDirectory &) = delete;
  const std::string &path() const { return path_; }

private:
  std::string path_;
};
struct WorkerResult {
  std::string reason;
  int exit_code{-1};
  double elapsed_sec{0};
};
// Fixed argv, no shell. Child owns a private process group; always reaped on
// cancel.
WorkerResult run_worker(const std::vector<std::string> &argv,
                        const std::string &stdout_path, double timeout_sec,
                        const std::function<bool()> &canceled);
} // namespace astribot::inference
