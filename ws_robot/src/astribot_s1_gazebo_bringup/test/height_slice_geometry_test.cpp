#include "astribot_s1_gazebo_bringup/height_slice_geometry.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace {
using namespace astribot::simulation;

SliceRaster raster(std::size_t width = 10, std::size_t height = 10) {
  return {width, height, 1., 0., 0., std::vector<std::int8_t>(width * height, 0)};
}

int at(const SliceRaster & grid, std::size_t x, std::size_t y) {
  return grid.data[y * grid.width + x];
}

void quad(std::vector<SliceTriangle> & triangles, SliceVertex a, SliceVertex b,
          SliceVertex c, SliceVertex d) {
  triangles.push_back({a, b, c});
  triangles.push_back({a, c, d});
}

std::vector<SliceTriangle> box(double x0, double x1, double y0, double y1,
                               double z0 = -2., double z1 = 2.) {
  std::vector<SliceTriangle> t;
  const SliceVertex a{x0,y0,z0}, b{x1,y0,z0}, c{x1,y1,z0}, d{x0,y1,z0};
  const SliceVertex e{x0,y0,z1}, f{x1,y0,z1}, g{x1,y1,z1}, h{x0,y1,z1};
  quad(t,a,d,c,b); quad(t,e,f,g,h);
  quad(t,a,b,f,e); quad(t,b,c,g,f); quad(t,c,d,h,g); quad(t,d,a,e,h);
  return t;
}

TEST(HeightSliceGeometry, SlantedTriangleClipsBeforeProjection) {
  auto grid = raster();
  std::vector<SliceTriangle> triangles{{SliceVertex{1.1,1.1,0.},
    SliceVertex{4.9,1.1,2.}, SliceVertex{1.1,4.9,2.}}};
  rasterize_mesh(triangles, 1., 1.5, grid);
  EXPECT_EQ(at(grid,1,1),0);
  EXPECT_EQ(at(grid,2,2),100);
  EXPECT_EQ(at(grid,4,4),0);
  EXPECT_EQ(at(grid,0,0),0);
}

TEST(HeightSliceGeometry, RotatedBoxDoesNotFillItsAxisAlignedBounds) {
  auto triangles = box(-2.,2.,-.4,.4);
  const double c = std::sqrt(.5);
  for (auto & t : triangles) for (auto & p : t) {
    const double x = p.x;
    p.x = 5. + c*(x-p.y); p.y = 5. + c*(x+p.y);
  }
  auto grid = raster();
  rasterize_mesh(triangles,-.25,.25,grid);
  EXPECT_EQ(at(grid,5,5),100);
  EXPECT_EQ(at(grid,3,6),0);
  EXPECT_EQ(at(grid,6,3),0);
}

TEST(HeightSliceGeometry, ClosedBoxFillsInteriorWithoutTopOrBottomInSlab) {
  auto grid = raster();
  rasterize_mesh(box(1.2,6.8,1.2,6.8),-.25,.25,grid);
  EXPECT_EQ(at(grid,3,3),100);
  EXPECT_EQ(at(grid,1,3),100);
  EXPECT_EQ(at(grid,0,3),0);
  EXPECT_EQ(at(grid,7,3),0);
}

TEST(HeightSliceGeometry, SeparateSolidsKeepTheirPassageFree) {
  auto triangles = box(.2,2.8,1.2,6.8);
  const auto second = box(5.2,7.8,1.2,6.8);
  triangles.insert(triangles.end(),second.begin(),second.end());
  auto grid = raster();
  rasterize_mesh(triangles,-.25,.25,grid);
  EXPECT_EQ(at(grid,1,3),100);
  EXPECT_EQ(at(grid,6,3),100);
  EXPECT_EQ(at(grid,3,3),0);
  EXPECT_EQ(at(grid,4,3),0);
}

TEST(HeightSliceGeometry, OverlappingDisconnectedShellsUseUnionNotXor) {
  auto triangles = box(.2,5.8,1.2,6.8);
  const auto second = box(2.2,7.8,2.2,7.8,-3.,3.);
  triangles.insert(triangles.end(),second.begin(),second.end());
  auto grid = raster();
  rasterize_mesh(triangles,-.25,.25,grid);
  EXPECT_EQ(at(grid,3,3),100);
  EXPECT_EQ(at(grid,1,3),100);
  EXPECT_EQ(at(grid,6,4),100);
}

TEST(HeightSliceGeometry, NestedDisconnectedShellIsConservativelyOccupied) {
  auto triangles = box(.2,8.8,.2,8.8,-3.,3.);
  const auto second = box(2.2,6.8,2.2,6.8,-2.,2.);
  triangles.insert(triangles.end(),second.begin(),second.end());
  auto grid = raster();
  rasterize_mesh(triangles,-.25,.25,grid);
  EXPECT_EQ(at(grid,4,4),100);
}

TEST(HeightSliceGeometry, ConnectedWatertightTubePreservesHole) {
  std::vector<SliceTriangle> triangles;
  const std::array<SliceVertex,4> outer{{{.2,.2,-2.},{8.8,.2,-2.},
    {8.8,8.8,-2.},{.2,8.8,-2.}}};
  const std::array<SliceVertex,4> inner{{{2.2,2.2,-2.},{6.8,2.2,-2.},
    {6.8,6.8,-2.},{2.2,6.8,-2.}}};
  for (unsigned i=0;i<4;++i) {
    const unsigned j=(i+1)%4;
    auto a=outer[i],b=outer[j],c=inner[i],d=inner[j];
    auto A=a,B=b,C=c,D=d;
    A.z=B.z=C.z=D.z=2.;
    quad(triangles,a,b,B,A); quad(triangles,c,C,D,d);
    quad(triangles,a,c,d,b); quad(triangles,A,B,D,C);
  }
  auto grid = raster();
  rasterize_mesh(triangles,-.25,.25,grid);
  EXPECT_EQ(at(grid,1,4),100);
  EXPECT_EQ(at(grid,7,4),100);
  EXPECT_EQ(at(grid,4,1),100);
  EXPECT_EQ(at(grid,4,4),0);
}

TEST(HeightSliceGeometry, OpenMeshContributesOnlyActualFaces) {
  std::vector<SliceTriangle> triangles;
  quad(triangles,{1.2,1.2,-2.},{1.2,6.8,-2.},{1.2,6.8,2.},{1.2,1.2,2.});
  quad(triangles,{6.8,1.2,-2.},{6.8,6.8,-2.},{6.8,6.8,2.},{6.8,1.2,2.});
  auto grid = raster();
  rasterize_mesh(triangles,-.25,.25,grid);
  EXPECT_EQ(at(grid,1,3),100);
  EXPECT_EQ(at(grid,6,3),100);
  EXPECT_EQ(at(grid,3,3),0);
}

TEST(HeightSliceGeometry, UnknownCellsRemainUnknownAndKnownOccupiedRemainOccupied) {
  auto grid = raster();
  grid.data[3*grid.width+3]=-1;
  grid.data[3*grid.width+1]=-1;
  grid.data[9*grid.width+9]=100;
  rasterize_mesh(box(1.2,6.8,1.2,6.8),-.25,.25,grid);
  EXPECT_EQ(at(grid,3,3),-1);
  EXPECT_EQ(at(grid,1,3),-1);
  EXPECT_EQ(at(grid,4,3),100);
  EXPECT_EQ(at(grid,9,9),100);
  EXPECT_EQ(at(grid,8,8),0);
}

TEST(HeightSliceGeometry, HeightIntervalIncludesLowerAndExcludesUpper) {
  std::vector<SliceTriangle> triangles{{SliceVertex{1.2,1.2,1.},
    SliceVertex{5.8,1.2,1.}, SliceVertex{1.2,5.8,1.}}};
  auto below = raster();
  rasterize_mesh(triangles,0.,1.,below);
  EXPECT_EQ(at(below,2,2),0);
  auto above = raster();
  rasterize_mesh(triangles,1.,2.,above);
  EXPECT_EQ(at(above,2,2),100);
}

TEST(HeightSliceGeometry, CellTouchingOnlyExcludedUpperEdgeRemainsFree) {
  std::vector<SliceTriangle> triangles{{SliceVertex{.2,.2,0.},
    SliceVertex{2.,.2,1.}, SliceVertex{2.,1.8,1.}}};
  auto grid = raster();
  rasterize_mesh(triangles,0.,1.,grid);
  EXPECT_EQ(at(grid,1,0),100);
  EXPECT_EQ(at(grid,2,0),0);
  EXPECT_EQ(at(grid,2,1),0);
}

TEST(HeightSliceGeometry, InfiniteSlabAndNonzeroGridOrigin) {
  auto grid = raster();
  grid.origin_x=-5.; grid.origin_y=-5.;
  rasterize_mesh(box(-3.8,1.8,-3.8,1.8),-.25,
                 std::numeric_limits<double>::infinity(),grid);
  EXPECT_EQ(at(grid,3,3),100);
  EXPECT_EQ(at(grid,0,0),0);
}

TEST(HeightSliceGeometry, InvalidBoundaryInputsAreExplicitErrors) {
  auto grid = raster();
  EXPECT_THROW(rasterize_mesh({},1.,1.,grid),std::invalid_argument);
  grid.data.pop_back();
  EXPECT_THROW(rasterize_mesh({},0.,1.,grid),std::invalid_argument);
  grid = raster();
  auto triangles = box(1.,2.,1.,2.);
  triangles[0][0].x=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(rasterize_mesh(triangles,0.,1.,grid),std::invalid_argument);
}
}  // namespace
