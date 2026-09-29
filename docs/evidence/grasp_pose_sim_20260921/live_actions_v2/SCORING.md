# Live Pose Action independent scoring

Five new live RGB-D captures passed the fixed 20 mm / 10 degree threshold.
Each had 2.646222 mm translation error and 0.581104 degrees SO(3) rotation
error, with no symmetry relaxation. All five Action calls succeeded. This is
the static `clear` simulation fixture, not a multi-scene or hardware claim.

- `live_pose_v2_scored.json`: original Action report, exact capture-time TF,
  CameraInfo, health, source epoch, calibration/model revisions and results.
- `live_pose_v2_scored_inputs/`: five actual masked RGB-D point clouds.
- `live_pose_v2_scored_truth.pbtxt.gz`: lossless raw Gazebo pose transport,
  recorded independently while the RGB-D-only Action client ran.
- `live_pose_v2_scored_scores.json`: per-frame independent truth, errors,
  acceptance thresholds, provenance and original report/truth hashes.
- `live_pose_v2_scored_offline_checks.json`: 11 passed offline checks, including
  refusal of the old report without TF, missing/wrong/latest TF, wrong result
  frame/epoch, unbracketed or moving truth, and truth-fed estimator inputs.
- `live_pose_v2_scored_manifest.json`: archive hashes, including decompressed
  raw truth. Original report paths refer to `runs/grasp_pose_sim_20260921/action`;
  corresponding archived files retain the same basenames and bytes.

The five distinct capture stamps are 3686.0, 3687.6, 3689.1, 3690.7 and 3692.3
seconds of simulation time. The 555 measured transport samples show unchanged
object and robot root poses throughout the batch. Every capture has a 17 ms
truth bracket. At each image stamp, ROS TF provides `odom` from camera and
`odom` from `astribot_torso_base`. The independently measured world robot pose
anchors world from odom; its discrepancy is 0 m and below 1e-12 degrees.
Camera from object truth is computed only in this separate scorer, after
inference. No target pose or truth is supplied to the Action client, mask,
point cloud projection, registration initializer or estimator.

Live wall latencies were 1.542–1.603 seconds; input age before sending was
45–124 ms. The scene and camera were unchanged, so identical rendered inputs
and identical deterministic estimates across fresh stamps are expected.
The segmentation is the declared HSV magenta validation fixture, not YOLO.
No robot movement or grasp execution occurred.

Offline reproduction from the repository root (choose a new output path):

```bash
python3 tools/vision/score_live_pose.py \
  --report docs/evidence/grasp_pose_sim_20260921/live_actions_v2/live_pose_v2_scored.json \
  --truth-stream docs/evidence/grasp_pose_sim_20260921/live_actions_v2/live_pose_v2_scored_truth.pbtxt.gz \
  --output /tmp/live_pose_v2_scored_recomputed.json
```

Only the new report can support these live error claims. The earlier ten-frame
`live_pose_v2_final.json` report has no capture-time TF and is explicitly rejected
for independent scoring. Missing TF or insufficient independent truth refuses
scoring; unsuccessful Action results cannot count as accepted accuracy samples.
