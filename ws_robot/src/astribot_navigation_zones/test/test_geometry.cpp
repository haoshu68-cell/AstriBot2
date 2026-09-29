#include "astribot_navigation_zones/geometry.hpp"
#include <gtest/gtest.h>
using namespace astribot_navigation_zones;
TEST(ZoneGeometry, ThinWallAndMargins) {
 auto zones=parseRegions(Json::array({{{"id","w"},{"name","wall"},{"type","wall"},{"enabled",true},{"points",{{0.,-2.},{0.,2.}}},{"width_m",.02},{"margin_m",.1}}}));
 EXPECT_TRUE(blocked(zones,0.,0.));EXPECT_TRUE(blocked(zones,.13,0.,.03));EXPECT_FALSE(blocked(zones,.2,0.));
 EXPECT_TRUE(sweptCircleBlocked(zones,-1,0,0,1,0,0,.2,2));
 EXPECT_FALSE(sweptCircleBlocked(zones,-1,0,0,-1,0,0,.2,.5));
}
TEST(ZoneGeometry, ConcavePolygonAndInvalidShapes) {
 Json z={{"id","room"},{"name","room"},{"type","polygon"},{"enabled",true},{"points",{{0.,0.},{2.,0.},{2.,1.},{1.,1.},{1.,2.},{0.,2.}}},{"margin_m",0.},{"width_m",0.}};
 auto regions=parseRegions(Json::array({z}));EXPECT_TRUE(blocked(regions,.5,1.5));EXPECT_FALSE(blocked(regions,1.5,1.5));EXPECT_TRUE(blocked(regions,1,1.5));
 z["points"]={{0,0},{2,2},{0,2},{2,0}};EXPECT_THROW(parseRegions(Json::array({z})),std::exception);
 z["type"]="wall";z["width_m"]=.1;z["points"]={{0,0},{0,0}};EXPECT_THROW(parseRegions(Json::array({z})),std::exception);
}
TEST(ZoneGeometry, DisabledAndCellIntersection) {
 Json z={{"id","w"},{"name","wall"},{"type","wall"},{"enabled",true},{"points",{{.025,0.},{.025,1.}}},{"width_m",.001},{"margin_m",0.}};
 auto r=parseRegions(Json::array({z}));EXPECT_TRUE(blocked(r,0.,.5,std::sqrt(2)*.05/2));
 z["enabled"]=false;EXPECT_FALSE(blocked(parseRegions(Json::array({z})),.025,.5));
}
