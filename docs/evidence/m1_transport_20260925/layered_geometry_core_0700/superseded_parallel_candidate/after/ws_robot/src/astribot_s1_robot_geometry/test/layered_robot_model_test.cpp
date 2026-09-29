#include "astribot_s1_robot_geometry/robot_model.hpp"
#include <gtest/gtest.h>

using namespace astribot_s1_robot_geometry;
namespace {
const std::string box_urdf=R"(<robot name="layers"><link name="base"><collision><origin xyz="0 0 .5"/><geometry><box size=".4 .2 .5"/></geometry></collision></link></robot>)";
const std::string arm_urdf=R"(<robot name="arm"><link name="base"/><link name="arm"><collision><origin xyz=".5 0 0"/><geometry><box size=".4 .2 .2"/></geometry></collision></link><joint name="pitch" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 1"/><axis xyz="0 1 0"/><limit lower="-3" upper="3"/></joint></robot>)";

// Independent half-plane oracle for points known analytically to lie in a body.
void expectContains(const Polygon2 &polygon,double x,double y) {
  ASSERT_GE(polygon.size(),3u);
  for(std::size_t i=0;i<polygon.size();++i) {
    const auto &a=polygon[i],&b=polygon[(i+1)%polygon.size()];
    EXPECT_GE((b[0]-a[0])*(y-a[1])-(b[1]-a[1])*(x-a[0]),-1e-11);
  }
}
void expectLegacyEqual(const RobotGeometry &a,const RobotGeometry &b) {
  EXPECT_EQ(a.physical,b.physical);EXPECT_EQ(a.reserved,b.reserved);
  EXPECT_EQ(a.height,b.height);EXPECT_EQ(a.z_min,b.z_min);
  ASSERT_EQ(a.slices.size(),b.slices.size());
  for(std::size_t i=0;i<a.slices.size();++i) {
    EXPECT_EQ(a.slices[i].z_min,b.slices[i].z_min);EXPECT_EQ(a.slices[i].z_max,b.slices[i].z_max);
    EXPECT_EQ(a.slices[i].footprint,b.slices[i].footprint);
  }
}

TEST(LayeredRobotModel, DefaultAndExplicitLayersLeaveLegacyResultsExactlyUnchanged) {
  const RobotModel model(arm_urdf,"base");
  Transform pose=Transform::Identity();pose(0,3)=.8;pose(1,3)=.3;pose(2,3)=.4;
  const std::vector<Shape> payload{Shape("arm","box",{.3,.4,.5},pose)};
  for(double angle:{-.7,0.,.8,pi/2})for(double error:{0.,.003,.02}) {
    const JointValues q{{"pitch",angle}},errors{{"pitch",error}};
    const auto original=model.geometry(q,errors,payload);
    const auto empty=model.geometry(q,errors,payload,.01,{.31,.31},{});
    const auto layered=model.geometry(q,errors,payload,.01,{.31,.31},{-.5,0.,.5,1.,1.5,3.});
    EXPECT_TRUE(original.layered_slices.empty());EXPECT_TRUE(empty.layered_slices.empty());
    expectLegacyEqual(original,empty);expectLegacyEqual(original,layered);
    ASSERT_EQ(layered.layered_slices.size(),5u);
  }
}

TEST(LayeredRobotModel, BoxTouchingBothBoundariesBelongsToAdjacentLayersAndEmptyLayerRemains) {
  const RobotModel model(box_urdf,"base");
  const auto g=model.geometry({},{},{},0.,{.31,.31},{0.,.25,.5,.75,1.,1.25});
  ASSERT_EQ(g.layered_slices.size(),5u);
  const Polygon2 expected{{-.2,-.1},{.2,-.1},{.2,.1},{-.2,.1}};
  for(std::size_t i=0;i<4;++i)EXPECT_EQ(g.layered_slices[i].footprint,expected);
  EXPECT_TRUE(g.layered_slices[4].footprint.empty());
  EXPECT_EQ(g.layered_slices[4].z_min,1.);EXPECT_EQ(g.layered_slices[4].z_max,1.25);
  EXPECT_EQ(g.z_min,.25);EXPECT_EQ(g.height,.75);
}

TEST(LayeredRobotModel, BoundaryMinusExactPlusPreserveClosedIntersection) {
  const RobotModel model(R"(<robot name="fixture"><link name="base"><collision><origin xyz="0 0 -1"/><geometry><sphere radius=".1"/></geometry></collision></link></robot>)","base");
  // Binary-exact dimensions and 2^-40 m delta isolate comparison semantics.
  const double delta=std::ldexp(1.,-40);
  for(double shift:{-delta,0.,delta}) {
    Transform pose=Transform::Identity();pose(2,3)=.5625+shift;
    auto g=model.geometry({},{},{Shape("base","box",{.125,.125,.125},pose)},0.,{.31,.31},{.25,.5,.75});
    EXPECT_EQ(g.layered_slices[0].footprint.empty(),shift>0.);
    EXPECT_FALSE(g.layered_slices[1].footprint.empty());
    pose(2,3)=.4375+shift;
    g=model.geometry({},{},{Shape("base","box",{.125,.125,.125},pose)},0.,{.31,.31},{.25,.5,.75});
    EXPECT_FALSE(g.layered_slices[0].footprint.empty());
    EXPECT_EQ(g.layered_slices[1].footprint.empty(),shift<0.);
  }
}

TEST(LayeredRobotModel, SphereProjectionConservativelyContainsAnalyticCircle) {
  const RobotModel model(R"(<robot name="sphere"><link name="base"><collision><origin xyz=".7 -.4 .5"/><geometry><sphere radius=".125"/></geometry></collision></link></robot>)","base");
  const auto g=model.geometry({},{},{},0.,{.31,.31},{0.,.25,.375,.5,.625,.75,1.});
  EXPECT_TRUE(g.layered_slices.front().footprint.empty());EXPECT_TRUE(g.layered_slices.back().footprint.empty());
  for(std::size_t layer=1;layer<=4;++layer)for(int i=0;i<128;++i) {
    const double a=2*pi*i/128;expectContains(g.layered_slices[layer].footprint,.7+.125*std::cos(a),-.4+.125*std::sin(a));
  }
  EXPECT_EQ(g.z_min,.375);EXPECT_EQ(g.height,.625);
}

TEST(LayeredRobotModel, TiltedCylinderIncludesAnalyticRimProjectionAndHeight) {
  const RobotModel model(R"(<robot name="cylinder"><link name="base"><collision><origin xyz=".2 -.3 .75" rpy="0 .7853981633974483 0"/><geometry><cylinder radius=".125" length=".5"/></geometry></collision></link></robot>)","base");
  const auto g=model.geometry({},{},{},0.,{.31,.31},{0.,.4,.5,.7,.8,1.,1.1,1.5});
  const double c=std::sqrt(.5),extent=(.125+.25)*c;
  EXPECT_NEAR(g.z_min,.75-extent,1e-14);EXPECT_NEAR(g.height,.75+extent,1e-14);
  EXPECT_TRUE(g.layered_slices.front().footprint.empty());EXPECT_TRUE(g.layered_slices.back().footprint.empty());
  for(std::size_t layer=1;layer<=5;++layer)for(int i=0;i<128;++i)for(double z:{-.25,.25}) {
    const double a=2*pi*i/128;expectContains(g.layered_slices[layer].footprint,.2+c*.125*std::cos(a)+c*z,-.3+.125*std::sin(a));
  }
}

TEST(LayeredRobotModel, RotatingJointChangesLayerMembershipAndProjectedBounds) {
  const RobotModel model(arm_urdf,"base");
  const auto g=model.geometry({{"pitch",pi/2}},{{"pitch",0.}},{},0.,{.31,.31},{0.,.25,.5,.75,1.});
  EXPECT_NEAR(g.z_min,.3,1e-14);EXPECT_NEAR(g.height,.7,1e-14);
  EXPECT_TRUE(g.layered_slices[0].footprint.empty());EXPECT_TRUE(g.layered_slices[3].footprint.empty());
  for(std::size_t layer:{1u,2u})for(double x:{-.1,.1})for(double y:{-.1,.1})expectContains(g.layered_slices[layer].footprint,x,y);
  const auto horizontal=model.geometry({{"pitch",0.}},{{"pitch",0.}},{},0.,{.31,.31},{0.,.25,.5,.75,1.,1.25});
  for(std::size_t layer:{0u,1u,2u})EXPECT_TRUE(horizontal.layered_slices[layer].footprint.empty());
  EXPECT_FALSE(horizontal.layered_slices[3].footprint.empty());EXPECT_FALSE(horizontal.layered_slices[4].footprint.empty());
}

TEST(LayeredRobotModel, OffsetPayloadUsesJointTransformAndRemainsSeparateFromArmLayers) {
  const RobotModel model(arm_urdf,"base");
  Transform pose=Transform::Identity();pose(0,3)=1.;pose(1,3)=.4;pose(2,3)=.3;
  const auto g=model.geometry({{"pitch",pi/2}},{{"pitch",0.}},{Shape("arm","box",{.2,.4,.6},pose)},0.,{.31,.31},{-.25,0.,.25,.5,.75,1.});
  EXPECT_NEAR(g.z_min,-.1,1e-14);EXPECT_NEAR(g.height,.7,1e-14);
  for(std::size_t layer:{0u,1u})for(double x:{0.,.6})for(double y:{.2,.6})expectContains(g.layered_slices[layer].footprint,x,y);
  for(std::size_t layer:{2u,3u})for(double x:{-.1,.1})for(double y:{-.1,.1})expectContains(g.layered_slices[layer].footprint,x,y);
  EXPECT_TRUE(g.layered_slices[4].footprint.empty());
}

TEST(LayeredRobotModel, PaddingAndHoldErrorExpandHeightAndLayerFootprint) {
  const RobotModel model(R"(<robot name="slide"><link name="base"/><link name="body"><collision><geometry><box size=".2 .2 .125"/></geometry></collision></link><joint name="slide" type="prismatic"><parent link="base"/><child link="body"/><origin xyz="0 0 .5"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint></robot>)","base");
  const std::vector<double> edges{.4,.42,.59,.61};
  const auto g=model.geometry({{"slide",0.}},{{"slide",.02}},{},.01,{.31,.31},edges);
  EXPECT_NEAR(g.z_min,.4075,1e-14);EXPECT_NEAR(g.height,.5925,1e-14);
  for(const auto &layer:g.layered_slices) {
    expectContains(layer.footprint,.13,0.);expectContains(layer.footprint,-.13,0.);
    expectContains(layer.footprint,0.,.13);expectContains(layer.footprint,0.,-.13);
  }
  const auto no_error=model.geometry({{"slide",0.}},{{"slide",0.}},{},0.,{.31,.31},edges);
  EXPECT_TRUE(no_error.layered_slices.front().footprint.empty());EXPECT_TRUE(no_error.layered_slices.back().footprint.empty());
}

TEST(LayeredRobotModel, LimitedCoverageDoesNotClampGlobalHeightOrMinimum) {
  const RobotModel model(box_urdf,"base");
  Transform high=Transform::Identity(),low=Transform::Identity();high(2,3)=2.4;low(2,3)=-.5;
  const std::vector<double> edges{-.045,.155,.585,1.085,1.535,2.205};
  const auto g=model.geometry({},{},{Shape("base","box",{.2,.2,.2},high),Shape("base","sphere",{.1},low)},0.,{.31,.31},edges);
  EXPECT_NEAR(g.height,2.5,1e-14);EXPECT_NEAR(g.z_min,-.6,1e-14);
  EXPECT_GT(g.height,edges.back());EXPECT_LT(g.z_min,edges.front());
  EXPECT_TRUE(g.layered_slices.back().footprint.empty());
  EXPECT_GE(g.slices.back().z_max,g.height);EXPECT_LE(g.slices.front().z_min,g.z_min);
}

TEST(LayeredRobotModel, NarrowExplicitLayersDoNotCopyCoarseSliceOvercoverage) {
  const RobotModel model(R"(<robot name="narrow"><link name="base"><collision><origin xyz="0 0 .12"/><geometry><box size=".2 .2 .02"/></geometry></collision></link></robot>)","base");
  const auto g=model.geometry({},{},{},0.,{.31,.31},{0.,.1,.14,.2,.25});
  ASSERT_EQ(g.slices.size(),1u);EXPECT_EQ(g.slices.front().z_min,0.);EXPECT_EQ(g.slices.front().z_max,.25);
  EXPECT_TRUE(g.layered_slices[0].footprint.empty());EXPECT_FALSE(g.layered_slices[1].footprint.empty());
  EXPECT_TRUE(g.layered_slices[2].footprint.empty());EXPECT_TRUE(g.layered_slices[3].footprint.empty());
}

TEST(LayeredRobotModel, InvalidEdgesFailAndSubsequentValidCallRecovers) {
  const RobotModel model(box_urdf,"base");
  const double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity();
  for(const std::vector<double> &edges:std::vector<std::vector<double>>{{0.},{0.,0.},{1.,0.},{0.,1.,.5},{0.,nan},{-inf,1.},{0.,inf}})
    EXPECT_THROW(model.geometry({},{},{},0.,{.31,.31},edges),std::invalid_argument);
  const auto g=model.geometry({},{},{},0.,{.31,.31},{0.,1.});
  ASSERT_EQ(g.layered_slices.size(),1u);EXPECT_FALSE(g.layered_slices.front().footprint.empty());
}
} // namespace
