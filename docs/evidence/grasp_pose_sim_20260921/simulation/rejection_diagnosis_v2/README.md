# Frozen v2 rejection diagnosis — 2026-09-21

No registration source, runtime default, Action interface, health gate or frozen binary was changed. `pose_v3` is a diagnostic artifact directory only; it does not contain a promoted v3 implementation.

## Result

The same seven measured RGB-D clouds still produce two accepted and five rejected results. One bounded expansion from 32 to 256 PPF candidate refinement slots found no additional candidate satisfying all fixed gates. All seven highest-ranked candidate metrics remained unchanged. This does not prove a globally correct pose cannot exist; it rules out the tested candidate-budget limit as the cause and provides no evidence justifying an algorithm change.

Five failures have visible CAD coverage below 75%. `tilted` and `close` additionally have only one supported normal direction. Scene coverage and residual error already pass for these candidates, so lowering residuals alone cannot resolve the rejection. The aggregate `no_acceptable_pose` occurs before the normal-diversity check and hides that additional failure in these two scenes.

## Per-scene evidence

Every geometric score below is conditional on the highest-ranked estimated candidate, including rejected candidates. It is not a ground-truth measurement and must not be interpreted as a released pose. Total / visible coverage are fractions of all CAD samples / CAD samples predicted visible after analytic CAD self-occlusion respectively. Normal directions require at least max(20, 5% of scene points) support each. The input normal cone is a descriptive statistic: fraction within 15 degrees of the largest 0.1-rounded normal bin, not an added acceptance rule. All input rows are finite.

“Foreground” is the fraction of predicted visible CAD samples whose measured depth is more than 8 mm closer and lies outside the target mask. “Invalid” is the fraction projecting to missing depth. Both denominators are all candidate-predicted visible samples. These fractions are evidence of external foreground under the candidate hypothesis; they do not establish a simulator-level occlusion fraction independent of the candidate.

| Scene | Points | Optical z (m) | Dominant normal cone | Total / visible coverage | Supported directions | Scene coverage | RMSE (mm) | Foreground | Invalid depth | Result |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| near | 2008 | 0.599–0.712 | 69.8% | 11.3% / 41.5% | 2 | 94.5% | 2.42 | 27.6% | 21.1% | reject |
| tilted | 2389 | 0.717–0.817 | 97.0% | 11.8% / 33.2% | 1 | 89.5% | 2.84 | 38.6% | 16.5% | reject |
| yawed | 2186 | 0.754–0.846 | 55.2% | 25.0% / 51.0% | 3 | 99.8% | 2.39 | 27.5% | 0.8% | reject |
| far | 1688 | 1.053–1.221 | 56.1% | 33.6% / 76.7% | 3 | 100.0% | 2.41 | 21.4% | 0.2% | accept |
| occluded | 1186 | 0.621–0.742 | 51.2% | 8.7% / 32.2% | 2 | 89.0% | 2.39 | 38.5% | 18.9% | reject |
| close | 5844 | 0.507–0.525 | 99.5% | 14.1% / 71.0% | 1 | 91.8% | 2.42 | 24.6% | 3.8% | reject |
| clear | 2556 | 0.909–1.047 | 57.3% | 36.9% / 76.8% | 3 | 99.9% | 2.43 | 20.7% | 0.3% | accept |

The `near` and `occluded` hypotheses have about 49% and 57% combined foreground/missing depth. In `yawed`, foreground alone accounts for 27.5% of predicted visible surface and a further 15.5% projects near valid target depth but lacks an accepted nearest-neighbor/normal match. These classifications are kept separate instead of attributing every unsupported sample to occlusion. `tilted` and `close` are overwhelmingly single-normal observations. No existing acceptance gate is relaxed for any of these cases.

## Fixed gates and search comparison

Registration used the unchanged registered three-box union geometry, visible coverage >= 0.75, scene coverage >= 0.65, RMSE <= 0.005 m, visible model samples >= 60 and >= 2 supported normal directions. The input sampling, refinement budget (600 points), normal agreement, distinct-pose checks and ambiguity checks were unchanged. `max_candidates` limits PPF hypotheses before additional ambiguity challenges; therefore the total reported candidate count can exceed that setting.

| Scene | Candidates with budget 32 | Candidates with budget 256 | Gate-passing candidates after expansion |
|---|---:|---:|---:|
| near | 20 | 20 | 0 |
| tilted | 34 | 37 | 0 |
| yawed | 29 | 44 | 0 |
| far | 30 | 34 | 1 |
| occluded | 22 | 22 | 0 |
| close | 26 | 26 | 0 |
| clear | 32 | 36 | 3 |

Expanded runs took 1.29–1.73 seconds each on this host. These are offline diagnostic latencies, not live Action end-to-end deadlines.

## Independent accuracy evidence

The existing independent v2 scoring report gives `far` 2.942 mm / 0.577 degrees and `clear` 2.093 mm / 0.435 degrees. Both satisfy the fixed 20 mm / 10 degree scoring threshold. Source: `docs/evidence/grasp_pose_sim_20260921/simulation/pose_results_visibility_v2/summary.json`. Rejected candidate transforms have not been independently scored in this diagnostic pass and are not presented as accurate estimates. No ground-truth files or pose matrices were read by this diagnosis or supplied to registration. There is no red-to-green algorithm claim: all five prior refusals remain explicit, while the two accepted cases remain accepted.

## Suggested diagnostics, not an interface change

Keep the aggregate registration outcome and attach a list of directly measured failed gates with `field`, `observed`, `limit`, and `direction`. Useful labels are `visible_model_coverage_low`, `scene_coverage_low`, `rmse_too_high`, `insufficient_visible_model_samples`, and `insufficient_normal_diversity`. An empty hypothesis set can be distinguished as `no_pose_candidates`. Evaluate all gates for the reported diagnostic candidate so a coverage failure does not hide missing normal diversity. Record the number of candidates satisfying all gates to distinguish ranking problems from an empty acceptable set.

Do not infer `occluded`, `wrong_model`, or `pose_inaccurate` from low coverage alone. The core currently receives only point clouds, so it cannot independently establish the foreground-depth classifications in this report. Those require the recorded raw depth, target mask, measured intrinsics, and an explicitly stated candidate hypothesis. These are proposals only; the API and JSON schema are unchanged.

## Provenance and reproduction

Frozen CLI SHA256: `6fd740dccde2b7cf3fdbc009e0402a841b8b030fd76a0273ecc99925913230dc`.

Frozen library SHA256: `2eab393e88e7541252a833e7daeb06002a37019c523c500ae3c912938b3453e2`.

Model SHA256: `4eaf790018ab4e72bf9dc8ce24b8e296b32844235b99ca566af6c8921c133429`.

Visibility geometry SHA256: `a8a43181fe2378ca583f6517e07baacb74f466236b6aa7bdedf841728e6a819f`.

`diagnostic_summary.json` contains input hashes, input statistics, gate comparisons and projection fractions. `*_v2_candidates.json` and `*_wide_candidates.json` retain all candidate metrics, including diagnostic-only rejected transforms. `*_visible_missing.json` lists each unsupported CAD sample and its image-space class. Never substitute those rejected transforms for the public CLI result, which returns no pose on rejection.

The diagnostic helper links the frozen registration library and adds no search logic:

```bash
c++ -std=c++17 pose_candidate_diagnostic.cpp \
  -I/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_object_pose_core/include \
  $(pkg-config --cflags --libs opencv4) \
  -L/home/yjh/WorkSpace/astribot_sdk_ros2/runs/grasp_pose_sim_20260921/ros_ws/install/astribot_object_pose_core/lib \
  -Wl,-rpath,/home/yjh/WorkSpace/astribot_sdk_ros2/runs/grasp_pose_sim_20260921/ros_ws/install/astribot_object_pose_core/lib \
  -lastribot_object_pose_registration -o /tmp/pose_candidate_diagnostic
/tmp/pose_candidate_diagnostic MODEL.xyz SCENE.xyz VISIBILITY.json OUTPUT.json 256
python3 pose_visibility_diagnosis.py
```

The helper is an offline evidence tool outside the core package. The projection script reads only `scene.xyz`, `depth.npy`, `mask.png`, `camera_info.json`, known CAD/geometry and saved candidate outputs. It does not read object truth or use image observations to change any registered pose, threshold, or score.
