#pragma once
namespace astribot::payload {
// Enclose the existing kinematic source bounds: 5 mm translation and 0.01 rad
// orientation about the model origin. Inputs describe nominal geometry.
inline double kinematic_payload_margin(double nominal_radius,double origin_distance) {
 return .005+.01*(origin_distance+nominal_radius);
}
}  // namespace astribot::payload
