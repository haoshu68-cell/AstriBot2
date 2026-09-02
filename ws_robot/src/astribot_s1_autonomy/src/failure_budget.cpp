// Copyright 2026 Astribot

#include "astribot_s1_autonomy/failure_budget.hpp"

namespace astribot_s1_autonomy
{

void ExplorationFailureBudget::onCandidatesSampled()
{
  sample_failures = 0;
  // 这里**故意**不动 validation_failures。见头文件顶部的 bug 记录。
}

bool ExplorationFailureBudget::onNoCandidateSampled()
{
  // 到顶就不再累加：实测日志出现过 `连续采样失败 5/4`（升级已经发生，
  // 但计数继续涨），读数超过分母会让人以为上限没生效。夹在上限上更诚实。
  if (sample_failures < max_sample_failures) {
    ++sample_failures;
  }
  return sample_failures >= max_sample_failures;
}

bool ExplorationFailureBudget::onAllCandidatesInvalid()
{
  if (validation_failures < max_validation_failures) {
    ++validation_failures;
  }
  return validation_failures >= max_validation_failures;
}

void ExplorationFailureBudget::onGoalDispatched()
{
  validation_failures = 0;
  // 采样预算不在这里清 —— 它已经在 onCandidatesSampled 清过了，
  // 而且下发成功并不能说明"以后一定采得到候选"。
}

void ExplorationFailureBudget::resetAll()
{
  sample_failures = 0;
  validation_failures = 0;
}

}  // namespace astribot_s1_autonomy
