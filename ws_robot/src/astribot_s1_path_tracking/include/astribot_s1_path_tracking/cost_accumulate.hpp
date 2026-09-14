// Copyright 2026 Astribot. Apache-2.0.
#ifndef ASTRIBOT_S1_PATH_TRACKING__COST_ACCUMULATE_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__COST_ACCUMULATE_HPP_

#include <xtensor/xtensor.hpp>
#include <xtensor/xnoalias.hpp>

namespace astribot_s1_path_tracking
{
namespace cost_accumulate
{

template<class E>
inline bool accumulateInPlace(xt::xtensor<float, 1> & costs, const E & term)
{
  if (costs.shape(0) != term.shape(0)) {
    return false;
  }
  xt::noalias(costs) += term;
  return true;
}

}
}

#endif
