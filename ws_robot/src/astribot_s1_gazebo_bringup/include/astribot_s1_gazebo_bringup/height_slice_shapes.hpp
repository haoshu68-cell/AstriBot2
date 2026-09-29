#pragma once

#include <ignition/math/Pose3.hh>
#include <sdf/Geometry.hh>

#include "astribot_s1_gazebo_bringup/height_slice_geometry.hpp"

namespace astribot::simulation {

// Return the actual BOX or MESH collision surfaces in map coordinates.
// Unsupported geometry, submesh selection, and missing/invalid meshes throw.
std::vector<SliceTriangle> collision_triangles(
  const sdf::Geometry & geometry,
  const ignition::math::Pose3d & map_from_collision);

}  // namespace astribot::simulation
