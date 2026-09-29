#pragma once

#include <opencv2/core.hpp>

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace astribot::object_pose {

struct BoxPrimitive {
  cv::Vec3d center_m;
  cv::Vec3d size_m;
};

struct Options {
  std::size_t min_points{60};
  std::size_t max_points{12000};
  double relative_sampling_step{0.075};
  double relative_distance_step{0.06};
  double relative_scene_sample_step{0.25};
  std::size_t max_candidates{32};
  std::size_t refinement_points{600};
  double inlier_distance_m{0.008};
  double max_rmse_m{0.005};
  double min_model_coverage{0.35};
  double min_scene_coverage{0.65};
  double min_visible_model_coverage{0.75};
  double min_normal_support_fraction{0.05};
  double ambiguity_coverage_margin{0.035};
  double ambiguity_rmse_margin_m{0.001};
  double distinct_translation_m{0.015};
  double distinct_rotation_rad{0.20};
  int normal_neighbors{18};
  bool continuous_symmetry{false};
  // Object-frame proper rotations. Identity is implicit. Finite equivalent poses
  // are T_camera_object * S_object_object. No unique orientation is claimed.
  std::vector<cv::Matx33d> symmetry_rotations;
  // Registered CAD geometry, never measured/simulator object pose. If present,
  // candidates are scored against analytically visible union surfaces.
  std::vector<BoxPrimitive> visibility_boxes;
};

struct Candidate {
  cv::Matx44d camera_from_object{cv::Matx44d::eye()};
  double model_coverage{0.0};
  double scene_coverage{0.0};
  double visible_model_coverage{std::numeric_limits<double>::quiet_NaN()};
  std::size_t visible_model_points{0};
  std::size_t observable_normal_directions{0};
  double rmse_m{std::numeric_limits<double>::infinity()};
  std::size_t votes{0};
};

struct Result : Candidate {
  bool success{false};
  std::string reason;
  bool ambiguous{false};
  bool symmetry_equivalent{false};
  std::vector<Candidate> candidates;
};

// All coordinates are metres. Model is Nx6 CV_32F (object xyz, outward normals).
// Scene is segmented object points, Nx6 (camera xyz/outward normals), or Nx3
// with local PCA normals oriented toward the camera origin. No pose seed or
// simulator data enters this interface. Invalid input is returned as failure.
Result estimate(const cv::Mat& model_xyz_normals, const cv::Mat& scene_xyz_or_normals,
                const Options& options = {});

std::vector<cv::Matx33d> boxSymmetryRotations();

}  // namespace astribot::object_pose
