#include "astribot_s1_robot_geometry/geometry_state_core.hpp"
#include "astribot_s1_robot_geometry/joint_snapshot.hpp"
#include "astribot_s1_robot_geometry/source_time.hpp"
#include <iostream>
using namespace astribot_s1_robot_geometry;
int main() {
  const auto expect_reject=[](auto f,const std::string &reason) {
    try {f();}catch(const std::invalid_argument &e) {if(std::string(e.what()).find(reason)!=std::string::npos)return;throw;}
    throw std::runtime_error("expected rejection: "+reason);
  };
  GeometryWorkContext context{1,2,"scene",3};
  const auto check=[&](int64_t now,const GeometryWorkContext &current) {
    validateGeometryCompletion(context,current,true,1000000000,1300000000,now,1.7,2.2,1000000000,"scene",.1);
  };
  check(1150000000,context);
  expect_reject([&]{check(1300000000,context);},"SOURCE_LEASE");
  expect_reject([&]{check(900000000,context);},"SOURCE_LEASE");
  auto changed=context;changed.input_generation++;
  expect_reject([&]{check(1150000000,changed);},"INPUT_CHANGED");
  changed=context;changed.clock_epoch++;
  expect_reject([&]{check(1150000000,changed);},"INPUT_CHANGED");
  changed=context;changed.model_generation++;
  expect_reject([&]{check(1150000000,changed);},"INPUT_CHANGED");
  changed=context;changed.attachment_revision="different";
  expect_reject([&]{check(1150000000,changed);},"INPUT_CHANGED");
  expect_reject([&]{validateGeometryCompletion(context,context,true,1000000000,1300000000,1150000000,1.7,1.6,1000000000,"scene",.1);},"EXCEEDS_CONFIGURED_HEIGHT");
  expect_reject([&]{validateGeometryCompletion(context,context,true,1000000000,1300000000,1150000000,1.7,2.2,1000000000,"other",.1);},"FILTER_UNCONFIRMED");
  expect_reject([&]{validateGeometryCompletion(context,context,true,1000000000,1300000000,1150000000,1.7,2.2,1000000000,"scene",.500001);},"FILTER_UNCONFIRMED");
  expect_reject([&]{validateGeometryCompletion(context,context,true,1000000000,1300000000,1150000000,1.7,std::nullopt,1000000000,"scene",.1);},"CONFIGURATION_UNAVAILABLE");
  nlohmann::json coverage{{"input_cloud_topic","/map_scan"},{"base_frame","base"},{"enable_outlier_filter",false}};
  int index=0;
  for(const auto *slice:{"low_obstacle","main_nav","torso_high","overhead"}) {
    const auto prefix=std::string("slices.")+slice+".";coverage[prefix+"enabled"]=true;
    coverage[prefix+"z_min"]=index==0?-.03:index*.6;coverage[prefix+"z_max"]=(index+1)*.6;coverage[prefix+"min_points"]=1;++index;
  }
  if(std::abs(projectionCeiling(coverage,"base")-2.4)>1e-12)throw std::runtime_error("projection ceiling changed");
  auto invalid=coverage;invalid["slices.main_nav.z_min"]=1.;
  expect_reject([&]{projectionCeiling(invalid,"base");},"GAP_OR_INVALID_RANGE");
  invalid=coverage;invalid["enable_outlier_filter"]=true;
  expect_reject([&]{projectionCeiling(invalid,"base");},"SPARSE_OBSTACLES");
  invalid=coverage;invalid["slices.overhead.min_points"]=2;
  expect_reject([&]{projectionCeiling(invalid,"base");},"SLICE_INCOMPLETE");
  JointSnapshot snapshot({"a","b"});
  snapshot.receive({"a","b"},{1.,2.},1000000000,1000000000);
  snapshot.receive({"a"},{1.1},1100000000,1000000000);
  const auto before=snapshot.snapshot(1000000000);
  if(std::get<0>(before).at("a")!=1. || std::get<2>(before)!=1300000000)throw std::runtime_error("future data renewed source");
  const auto after=snapshot.snapshot(1100000000);
  if(std::get<0>(after).at("a")!=1.1 || std::get<2>(after)!=1300000000)throw std::runtime_error("partial data renewed source");
  expect_reject([&]{snapshot.snapshot(1400000000);},"STALE");
  expect_reject([&]{snapshot.snapshot(900000000);},"INCOMPLETE");
  if(snapshot.epoch()!=1)throw std::runtime_error("clock rollback epoch unchanged");
  if(selectSourceSample({700000000,1010000000},1000000000,300000000)!=1 ||
     selectSourceSample({699999999,1010000001},1000000000,300000000)!=-1)throw std::runtime_error("source boundary changed");
  std::cout<<"geometry completion source identity and lease gates passed\n";
}
