#include "astribot_s1_robot_geometry/fusion_snapshot.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace astribot_s1_robot_geometry;

static void check(bool ok, const char * msg) { if (!ok) throw std::runtime_error(msg); }
static void close(double a, double b, double tol, const char * msg) {
  check(std::abs(a - b) <= tol, msg);
}

static SnapshotTrackInput moving_track(int64_t capture, double vx, double vy, double vz) {
  SnapshotTrackInput t;
  t.capture_ns = capture;
  t.center[0] = 0.; t.center[1] = 0.; t.center[2] = 0.;
  t.size[0] = 1.; t.size[1] = 1.; t.size[2] = 1.;
  t.track_velocity[0] = vx; t.track_velocity[1] = vy; t.track_velocity[2] = vz;
  t.spatial_occupancy = false; t.velocity_observable = true; t.has_velocity_covariance = false;
  return t;
}

int main() {
  SnapshotParams p;  // defaults match simulation.json
  std::vector<SnapshotTrackInput> in;
  std::vector<SnapshotTrackOutput> out;

  // Empty input produces empty output.
  snapshotTracks(in, 1000, p, out);
  check(out.empty(), "empty input");

  // Non-finite geometry is rejected, matching the frozen contracts.
  SnapshotTrackInput bad = moving_track(0, 1., 0., 0.);
  bad.center[0] = std::nan("");
  in = {bad};
  bool threw = false;
  try { snapshotTracks(in, 1000, p, out); } catch (const std::invalid_argument &) { threw = true; }
  check(threw, "NaN center must be rejected");
  in.clear();

  // Future capture clamps age to zero: geometry and covariance unchanged.
  in = {moving_track(2000, 1., 0., 0.)};
  snapshotTracks(in, 1000, p, out);
  close(out[0].center[0], 0., 1e-12, "future capture keeps center");
  close(out[0].covariance[0], 0., 1e-12, "future capture keeps covariance");
  close(out[0].velocity[0], 1., 1e-12, "future capture is not old");
  in.clear();

  // Old track: prediction velocity zeroed, geometry still translated by
  // min(age, memory) * track velocity, covariance inflated by min(age,3)^2*0.04.
  in = {moving_track(0, 1., 0., 0.)};
  snapshotTracks(in, 2000000000LL, p, out);  // age = 2.0 s > track_memory_s = 1.0
  close(out[0].center[0], 1., 1e-9, "old track translates by 1.0 s");
  close(out[0].velocity[0], 0., 1e-12, "old track prediction velocity zeroed");
  close(out[0].covariance[0], 0.16, 1e-9, "old track covariance inflation");
  close(out[0].variance, 0.01, 1e-12, "moving track variance");
  in.clear();

  // Fresh moving track: full velocity, half-second translation.
  in = {moving_track(0, 1., 0., 0.)};
  snapshotTracks(in, 500000000LL, p, out);  // age = 0.5 s
  close(out[0].center[0], 0.5, 1e-9, "fresh track translates by 0.5 s");
  close(out[0].velocity[0], 1., 1e-12, "fresh track keeps velocity");
  close(out[0].covariance[0], 0.01, 1e-9, "fresh track covariance inflation");
  in.clear();

  // Spatial occupancy: never moved, variance forced to zero.
  SnapshotTrackInput occ = moving_track(0, 1., 0., 0.);
  occ.spatial_occupancy = true; occ.velocity_observable = false;
  in = {occ};
  snapshotTracks(in, 500000000LL, p, out);
  close(out[0].center[0], 0., 1e-12, "occupied cell is not moved");
  close(out[0].covariance[0], 0., 1e-12, "occupied cell covariance unchanged");
  close(out[0].variance, 0., 1e-12, "occupied cell variance zeroed");
  in.clear();

  // occupancy_only mirrors the reference's `velocity_m_s is None` check, so the
  // kernel must consult has_velocity_m_s rather than has_velocity_covariance.
  // (The Python contract always pairs them, so this input state is not reachable
  // through the contracts; it guards the mapping against a regressive
  // "simplification" to has_velocity_covariance.)
  SnapshotTrackInput meas_nocov = moving_track(0, 1., 0., 0.);
  meas_nocov.velocity_observable = false;
  meas_nocov.has_velocity_m_s = true;
  meas_nocov.has_velocity_covariance = false;
  in = {meas_nocov};
  snapshotTracks(in, 500000000LL, p, out);  // age = 0.5 s
  close(out[0].center[0], 0.5, 1e-9, "measured velocity still translates");
  close(out[0].covariance[0], 0.01, 1e-9, "non-occupancy keeps age_variance floor");
  in.clear();

  // Region relevance: a track far from the region is dropped from the model.
  SnapshotTrackInput far = moving_track(0, 0., 0., 0.);
  far.center[0] = 100.; far.center[1] = 100.;
  in = {far};
  SnapshotParams pr; pr.has_region = true; pr.region_x = 0.; pr.region_y = 0.; pr.region_travel = 1.;
  snapshotTracks(in, 1000, pr, out);
  check(!out[0].relevant, "far track is irrelevant to region");
  in.clear();

  SnapshotTrackInput near = moving_track(0, 0., 0., 0.);
  near.center[0] = 0.1; near.center[1] = 0.;
  in = {near};
  snapshotTracks(in, 1000, pr, out);
  check(out[0].relevant, "near track is relevant to region");

  std::cout << "PASS: empty, NaN rejection, future, old, fresh, occupied, measured-no-cov, region\n";
  return 0;
}
