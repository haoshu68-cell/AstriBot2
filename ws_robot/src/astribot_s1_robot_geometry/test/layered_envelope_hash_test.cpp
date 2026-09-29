#include <gtest/gtest.h>
#include "astribot_s1_robot_geometry/layered_envelope.hpp"
using namespace astribot_s1_robot_geometry;
namespace {
struct Envelope {
  struct {std::string frame_id="base";} header;
  struct {double height_m=1.7;} limits;
  geometry_msgs::msg::Polygon installed_footprint;
  std::vector<astribot_navigation_msgs::msg::EnvelopeSlice> height_slices;
  double clearance_m=.1,ground_in_base_m=-.095;
  std::string height_profile_revision=sha256("fixture-profile"),height_geometry_hash;
};
Envelope fixture() {
  Envelope e;
  for(const auto &xy:Polygon2{{-.5,-.4},{.5,-.4},{.5,.4},{-.5,.4}}) {
    geometry_msgs::msg::Point32 p;p.x=xy[0];p.y=xy[1];e.installed_footprint.points.push_back(p);
  }
  const std::vector<double> edges{-.045,.155,.585,1.085,1.535,2.205};
  for(std::size_t i=1;i<edges.size();++i) {
    astribot_navigation_msgs::msg::EnvelopeSlice layer;layer.z_min_m=edges[i-1];layer.z_max_m=edges[i];
    if(i!=3)layer.footprint=e.installed_footprint;
    e.height_slices.push_back(layer);
  }
  e.height_geometry_hash=layeredGeometryHash(envelopePolygonPoints(e.installed_footprint),e.height_slices,
    e.header.frame_id,e.clearance_m,e.height_profile_revision,e.ground_in_base_m);
  return e;
}
TEST(LayeredEnvelopeHash, CompleteEnvelopeAndEmptyLayerValidate) {
  const auto e=fixture();EXPECT_NO_THROW(validateLayeredEnvelope(e));EXPECT_TRUE(e.height_slices[2].footprint.points.empty());
}
TEST(LayeredEnvelopeHash, EveryGeometryIdentityFieldIsCovered) {
  for(int change=0;change<8;++change) {
    auto e=fixture();
    if(change==0)e.installed_footprint.points[0].x-=.01F;
    if(change==1)e.height_slices[0].footprint.points[0].x-=.01F;
    if(change==2) {e.height_slices[0].z_max_m+=.01;e.height_slices[1].z_min_m+=.01;}
    if(change==3)e.ground_in_base_m-=.01;
    if(change==4)e.clearance_m+=.01;
    if(change==5)e.height_profile_revision=sha256("other");
    if(change==6)e.header.frame_id="different_base";
    if(change==7)e.height_slices[2].footprint=e.installed_footprint;
    EXPECT_THROW(validateLayeredEnvelope(e),std::invalid_argument);
  }
}
TEST(LayeredEnvelopeHash, OneWireUlpChangeCannotHideInsideOldMicrometreRounding) {
  auto e=fixture();auto &x=e.height_slices[0].footprint.points[0].x;x=std::nextafter(x,-INFINITY);
  EXPECT_THROW(validateLayeredEnvelope(e),std::invalid_argument);
}
TEST(LayeredEnvelopeHash, CanonicalVertexRotationAndWindingPreserveIdentity) {
  auto e=fixture();auto &polygon=e.installed_footprint.points;std::rotate(polygon.begin(),polygon.begin()+1,polygon.end());
  for(auto &layer:e.height_slices)std::reverse(layer.footprint.points.begin(),layer.footprint.points.end());
  EXPECT_NO_THROW(validateLayeredEnvelope(e));
}
TEST(LayeredEnvelopeHash, MissingOverlappingNonfiniteAndNonplanarLayersReject) {
  for(int change=0;change<8;++change) {
    auto e=fixture();
    if(change==0)e.height_slices.clear();
    if(change==1)e.height_slices[1].z_min_m+=.01;
    if(change==2)e.height_slices[1].z_min_m-=.01;
    if(change==3)e.height_slices[1].z_max_m=NAN;
    if(change==4)e.height_slices[0].footprint.points[0].z=.1F;
    if(change==5)e.height_slices[0].footprint.points[0].x=INFINITY;
    if(change==6)e.ground_in_base_m=NAN;
    if(change==7)e.height_profile_revision="not-a-file-sha256";
    EXPECT_THROW(validateLayeredEnvelope(e),std::invalid_argument);
  }
}
} // namespace
