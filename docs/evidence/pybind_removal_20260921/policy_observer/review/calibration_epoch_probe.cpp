#include "astribot_s1_navigation_policy_native/policy_health.hpp"
#include <cstdint>
#include <iostream>
using namespace astribot::navigation::policy;
int main(){CameraCalibrationRegistry calibrations;const std::array<double,9> k{500,0,320,0,500,240,0,0,1};
calibrations.register_calibration(CameraCalibration("cam","camera",INT64_MAX,640,480,k,"plumb_bob",{}));
const auto old=calibrations.records().find("cam");const auto epoch=old->second.calibration_epoch;
CameraCalibration candidate("cam","camera",epoch,800,480,k,"plumb_bob",{});
if(!(candidate==old->second))calibrations.register_calibration(CameraCalibration("cam","camera",epoch+1,800,480,k,"plumb_bob",{}));
std::cout<<calibrations.records().at("cam").calibration_epoch<<'\n';}
