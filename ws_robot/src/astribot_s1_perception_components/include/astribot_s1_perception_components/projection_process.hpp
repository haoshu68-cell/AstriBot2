#pragma once
#include "astribot_s1_perception_components/depth_projection.hpp"
#include <atomic>
#include <chrono>
#include <mutex>

namespace astribot::vision {
struct ProjectionContext {
  uint64_t generation{0};
  int64_t capture_ns{0};
};
struct ProjectionProcessOptions {
  std::string executable;
  std::vector<std::string> arguments;
  double startup_timeout_sec{3.0};
  double request_timeout_sec{0.20};
  double terminate_timeout_sec{0.05};
  double kill_timeout_sec{0.10};
  double restart_backoff_sec{0.1};
  uint32_t max_restarts{3};
};
struct ProjectionProcessStatus {
  std::string state{"STOPPED"},reason{"NOT_STARTED"};
  uint64_t worker_epoch{0},requests{0};
  uint32_t restarts{0};
  int64_t pid{-1};
};
class ProjectionProcess final:public DepthProjector {
public:
  explicit ProjectionProcess(ProjectionProcessOptions);
  ~ProjectionProcess()override;
  void project(const DepthView&,const ProjectionConfig&,float*)override;
  void project_with_context(const DepthView&,const ProjectionConfig&,float*,ProjectionContext);
  void cancel()noexcept override;
  ProjectionProcessStatus status()const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
std::string projection_worker_executable();
}
