#include "astribot_s1_mapping/height_slice_grid.hpp"
#include <gtest/gtest.h>
using astribot_s1_mapping::HeightSliceGrid;
TEST(HeightSliceGrid, BoundariesAndUnknown) {
  HeightSliceGrid g(.05,0.,{.05,.25,.68,1.18,1.63,2.30});
  for(double h:{.05,.25,.68,1.18,1.63})g.add(1,0,h);
  for(auto n:g.counts)EXPECT_EQ(n,1u);
  g.add(1,0,.049);g.add(1,0,2.3);g.add(NAN,0,.5);
  EXPECT_EQ(g.below,1u);EXPECT_EQ(g.above,1u);EXPECT_EQ(g.nonfinite,1u);
  // No point creates a FREE cell; absent cells remain unknown at rasterization.
  EXPECT_EQ(g.occupied[0].count({0,0}),0u);
}
TEST(HeightSliceGrid, FarEndpointsSurviveAndGroundOffsetMatches) {
  HeightSliceGrid g(.05,-.125,{.05,.25,.68,1.18,1.63,2.30});
  g.add(1,0,.1-.125);g.add(5,0,.1-.125);g.add(2,0,.5-.125);
  EXPECT_EQ(g.occupied[0].size(),2u);EXPECT_EQ(g.occupied[1].size(),1u);
  g.add(-.001,-.001,.1-.125);EXPECT_EQ(g.occupied[0].count({-1,-1}),1u);
}
TEST(HeightSliceGrid, InvalidConfigurationFails) {
  EXPECT_THROW(HeightSliceGrid(0,0,{.05,.25}),std::invalid_argument);
  EXPECT_THROW(HeightSliceGrid(.05,NAN,{.05,.25}),std::invalid_argument);
  EXPECT_THROW(HeightSliceGrid(.05,0,{.25,.05}),std::invalid_argument);
}
