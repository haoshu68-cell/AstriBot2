#include "astribot_s1_manipulation_perception/worker_process.hpp"
#include <chrono>
#include <gtest/gtest.h>
using namespace astribot::inference;
TEST(Worker, ExecutesWithoutShell) {
  JobDirectory d;
  auto r = run_worker(
      {"/usr/bin/printf", "%s", "literal $(touch /tmp/should_not_exist)"},
      d.path() + "/stdout", 1, [] { return false; });
  EXPECT_EQ(r.reason, "OK");
  EXPECT_EQ(r.exit_code, 0);
}
TEST(Worker, RejectsMissingExecutable) {
  JobDirectory d;
  auto r = run_worker({"/does/not/exist"}, d.path() + "/stdout", 1,
                      [] { return false; });
  EXPECT_EQ(r.reason, "WORKER_SPAWN_FAILED");
}
TEST(Worker, CancelsAndReapsOwnedChild) {
  JobDirectory d;
  auto start = std::chrono::steady_clock::now();
  auto r = run_worker({"/usr/bin/sleep", "10"}, d.path() + "/stdout", 2, [&] {
    return std::chrono::steady_clock::now() - start >
           std::chrono::milliseconds(80);
  });
  EXPECT_EQ(r.reason, "CANCELED");
  EXPECT_LT(r.elapsed_sec, 1.);
}
TEST(Worker, TimesOutAndReportsFailure) {
  JobDirectory d;
  auto r = run_worker({"/usr/bin/sleep", "10"}, d.path() + "/stdout", .08,
                      [] { return false; });
  EXPECT_EQ(r.reason, "INFERENCE_TIMEOUT");
  EXPECT_LT(r.elapsed_sec, 1.);
  r = run_worker({"/usr/bin/false"}, d.path() + "/stdout2", 1,
                 [] { return false; });
  EXPECT_EQ(r.reason, "WORKER_FAILED");
}
