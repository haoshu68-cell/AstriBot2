# Stage 6B: lossless integer observer checkpoint

This closes the previously recorded JSON calibration-epoch/image-size integer differences. It does not retire the production observer/controller or any remaining binding. Lone surrogate strings, Stamp domains and the complete P2/P3/P4/P5/H2 controller remain separate work.

## Implementation and authority

The native observer uses exact signed arbitrary integers for the selected calibration/image/budget fields, exact integer-or-double pixel comparisons and lossless JSON token handling. It checks the health epoch against uint64 only at the original output-assignment step, after callback state mutations. PYTHONINTMAXSTRDIGITS is validated before ROS initialization, preserving invalid-environment startup failure. The shared typed core and adapter authority/mutation-order evidence are in ../policy_json_integer_contracts and ../policy_observation_adapters. No metric geometry or freshness threshold is relaxed.

The full isolated source tree is identified by source_manifest.json: Git 51132768 plus 33 scoped policy file changes. CMake/package.xml include only the behavior fragment and Boost addition; concurrent navigation-zones changes are excluded. Geometry source and the fixed-envelope profile come from that Git record. The old Python geometry extension was rebuilt from the archived geometry/baseline_sources, not taken from the shared build. Its new binary hash and exact rebuild files are in oracle_rebuild_manifest.json. Rebuilding can change its ELF hash; this is not the earlier ELF from the 1b51e14e experiment.

## Fresh validation

* Native-only geometry and policy configure/build/install succeeded; 13/13 CTest passed. This includes the existing suites and the new behavior/path-evidence core wrappers. Counts repeated through different runners are not independent cases.
* Installed observer: 41 scenarios on each implementation, **82/82 pytest cases**. Added integer cases include 2**63/UINT64_MAX calibration epochs, 2**64+1/400-digit image dimensions, exact integer versus double pixel bounds, and health-output failure above UINT64_MAX.
* The too-large epoch is accepted at ingress, then fails at health output, with no subsequent normal result. Repeated same-stamp clock messages wake the Humble executor so it retrieves the callback exception; this does not advance simulated time or change the production error stage.
* Six installed native ELF files have resolved ldd dependencies and no libpython/pybind linkage; this selected build installs no Python source. Three malformed PYTHONINTMAXSTRDIGITS settings fail before ROS initialization on both sides. This does not certify the whole repository dependency gate.
* P2 YieldPolicy live-profile refresh and full semantic path identity have separate Release/UBSan evidence in ../policy_behavior. No complete controller runtime is delivered here.

## Paired performance

Four AB/BA pairs per workload, 20 warmup and 100 measured frames per process: **1,600 measured frames**, all semantic outputs compared (timing fields excluded; floats rel=2e-12, abs=3e-12). Values below are the median of four per-process summaries. Driver and observer CPU affinities are recorded in benchmark/summary.json. The machine is shared, not an otherwise idle dedicated host.

| Tracks | Metric | Python | C++ | Reduction |
|---:|---|---:|---:|---:|
| 8 | tick P50 ms | 1.617309 | 0.165341 | 89.78% |
| 8 | tick P95 ms | 1.752990 | 0.194723 | 88.89% |
| 8 | whole-process CPU ms/frame | 6.700000 | 0.800000 | 88.06% |
| 8 | publish-to-result P95 ms | 5.881981 | 0.820432 | 86.05% |
| 8 | RSS MiB | 62.382812 | 27.818359 | 55.41% |
| 64 | tick P50 ms | 3.625544 | 0.501341 | 86.17% |
| 64 | tick P95 ms | 3.943359 | 0.589311 | 85.06% |
| 64 | whole-process CPU ms/frame | 14.050000 | 1.900000 | 86.48% |
| 64 | publish-to-result P95 ms | 7.994331 | 1.036258 | 87.04% |
| 64 | RSS MiB | 63.929688 | 28.892578 | 54.81% |

Tick timing excludes the preceding vision subscription callback. Whole-process CPU includes ROS callbacks, excludes startup, and uses /proc CPU ticks over the measured window. External delay starts at input clock publication, after the fixture drains preceding sensor/vision inputs. RSS is a short-window endpoint, not a leak guarantee. Raw runs include worst delays and RSS growth; do not infer that every individual frame is faster. No controller, Gazebo closed-loop, hardware, long-term soak or safety-stop acceptance is claimed.

## Recovery and retained limitations

The prior tool session lost its /tmp directory and process handles on interruption. Earlier unarchived runtime logs are not evidence for this checkpoint. The final 13/13 and 82/82 logs and benchmark here were regenerated in a persistent ignored runs directory and copied into this evidence tree. Historical core RED/sanitizer files already archived before interruption remain unchanged. The independent oracle rebuild initially used the wrong TinyXML2 CMake package name; both initial configure failure and corrected result are retained. That was a fixture build error.

Before the fresh run, the ROS fixture was tightened to wait for an actual parameter-service response, seed the simulated clock, and then advance one complete timer period. Publisher-authority checking waits for graph discovery while retaining its strict allowed-publisher assertion. These address the earlier observed startup/graph races without changing node code or weakening the behavior assertions. Prior startup-failure logs that were only in lost /tmp cannot be reconstructed and are not presented as archived evidence.

The frozen native source remains under runs/cpp_migration_20260921/stage6b_integer_recovery/source for local inspection. The Git checkpoint, hashes and reproducible recipe are the durable identity; run directories and binary caches are not production entries. Use reproduce.sh from the corresponding recorded commit with new owned build/install paths.
