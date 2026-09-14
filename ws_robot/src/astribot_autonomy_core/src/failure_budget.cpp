// Copyright 2026 Astribot

#include "astribot_s1_autonomy/failure_budget.hpp"

namespace astribot_s1_autonomy
{

void ExplorationFailureBudget::onCandidatesSampled()
{
  sample_failures = 0;
}

bool ExplorationFailureBudget::onNoCandidateSampled()
{
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
}

void ExplorationFailureBudget::resetAll()
{
  sample_failures = 0;
  validation_failures = 0;
}

}  // namespace astribot_s1_autonomy
