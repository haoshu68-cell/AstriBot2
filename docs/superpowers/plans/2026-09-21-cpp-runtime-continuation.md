# Remaining runtime C++ migration implementation plan

> **For agentic workers:** Use `superpowers:executing-plans` for local integration and `superpowers:dispatching-parallel-agents` for the two independent runtime packages. Continue the already authorized migration; no additional approval gate is required.

**Goal:** Retire validated Python runtime entries, migrate the remaining runtime incrementally, and preserve reproducible behavioral/performance evidence and Git checkpoints.

**Architecture:** Native ROS adapters retain existing topics, services, actions, parameters, ownership and state transitions. Historical Python implementations are validation inputs only. The current bridge, policy and geometry binding consumers remain visible until their replacements pass validation.

**Tech Stack:** ROS 2 Humble, C++17/rclcpp/rclcpp_action, ament_cmake, pytest/rclpy test drivers, Git.

**Spec:** User instructions in this task; `docs/PYTHON_CPP_REMAINING_20260921.md`, `docs/PYBIND_REMOVAL_20260921.md` and `.agents/skills/robot-runtime-cpp/SKILL.md`.

## Global constraints

- Launch/configuration and validation may remain Python. Migrated production roles have one C++ entry and no Python fallback.
- Preserve core logic, cancellation ownership, source timestamps/leases, clock behavior, and all existing protection thresholds. Report inherited behavior defects separately.
- Build/install in owned temporary directories. Do not alter shared installations, running simulations, hardware or other tasks' modifications.
- Record each validated step on `codex/pybind-migration-20260921` using the existing private index; preserve shared HEAD and real index.
- SDK headers/protocol are still unavailable; do not invent a native device ABI or equate fake-port tests with hardware acceptance.

## Review focus

- Python source discovery and symlink installs must not reintroduce retired modules.
- Test subprocesses must load the intended oracle, rather than silently picking an old shared installation.
- Coupling: missing/stale TF or joint input, empty arrays, time rollback, nonfinite values, parameter consumption and smoothing sequence.
- Arbiter: delayed goal acceptance/result, rejected cancellation, cancel ACK before terminal, repeated replacement and late feedback.
- Performance: alternate Python/C++ runs, retain raw samples and process identity, separate callback/service latency from robot closed-loop behavior.

## Task 1: Retire already replaced reference modules from production packages

**Files:** 17 modules listed in the inventory; `tools/migration/python_reference/`; `tools/migration/test/test_retired_python_packaging.py`; affected validation drivers/CMake; `tools/robot/sync_metrics_to_robot.sh`.

- [x] Add a failing test that requires each retired source module to be absent from its production package and present only under the explicit validation reference root.
- [x] Move byte-identical reference modules to `tools/migration/python_reference/<original_package>/`; provide test-only namespace initialization without any console or runtime entry.
- [x] Update validation environments and explicit file-path consumers. Keep geometry's explicitly frozen baseline precedence and historical evidence unchanged.
- [x] Check fresh normal installations, source package discovery and references; rerun affected regression suites.
- [x] Record the source/reference move, validation and limitations in a dedicated Git checkpoint.

## Task 2: Native arm/chassis Twist coupling

**Files:** `ws_robot/src/astribot_s1_dynamics_coupling/`; dedicated evidence subdirectory `dynamics_coupling/`.

- [x] Freeze Python oracle and interface contract before implementation.
- [x] Convert the independent package to a C++ executable, keeping launch/configuration contract and scaling algorithm.
- [x] Exercise identical joint/TF/Twist streams, parameter changes, stale/empty/malformed cases and repeated lifecycle runs.
- [x] Quantify paired latency, CPU and RSS with raw measurements and explicit scenario limits.
- [x] Remove production Python implementation/console registration after checks; review and record in Git.

## Task 3: Native navigation task arbiter

**Files:** new `ws_robot/src/astribot_s1_task_arbiter_native/`; old policy task entry and test oracle; navigation launch/dependency wiring; dedicated evidence subdirectory `task_arbiter/`.

- [x] Freeze current protocol/state behavior and Python oracle.
- [x] Implement native action/service adapter with the same single-child ownership and terminal-result cancellation barrier.
- [x] Compare fake-Nav2 traces across success, failure, busy/replace, cancel rejection, delayed acceptance/results, unavailable backend and parameter boundaries.
- [x] Measure isolated action overhead/CPU/RSS and repeated-run stability without implying full navigation acceptance.
- [x] Select only the native executable, remove the old Python production entry and record validated changes in Git.

## Task 4: Continue the full remaining inventory

- [x] Refresh the AST/source/entry inventory after these changes and keep the previous snapshot separately.
- [x] Review remaining live binding imports before selecting subsequent policy, transport, perception, supervision and SDK migration units.
- [ ] Keep the full project goal open until all runtime consumers and required validation are complete; report blockers precisely without calling partial work full migration.

## Task 5: Native map/odom decomposition and cross-domain map relay

Pre-flight: these two nodes share only the new `astribot_s1_perception_native`
build package and perception launch/setup wiring. The map/odom worker owns its
source, core, tests and `cmake/map_odom.cmake`; the root owns relay, top-level
CMake, manifests and final entry switching. No control-topic publisher is added.

- [x] Freeze the three current Python source files byte-for-byte under test/reference.
- [x] Map/odom: preserve SE(2) composition, height difference, latest TF lookup,
  source-age and future-stamp rules, ROS timers, startup watchdog, jump/tilt
  accounting, parameter types and snapshot consumption. Same-input core and
  isolated TF/clock replay must cover both missing edges and recovery.
- [x] Relay: preserve independent local/remote contexts and domains, one-way
  unchanged OccupancyGrid forwarding, transient-local/reliable depth 1, topic
  remaps, initial-map wall timeout, equal-domain rejection and orderly teardown.
  Verify latched startup/late subscriber, updates, empty/malformed map metadata,
  no reverse/control forwarding, no-message timeout and parameter boundaries.
- [x] Quantify paired ROS latency/CPU/RSS with raw samples and short lifecycle
  repetition. Separate DDS proxy overhead from localization/navigation quality.
- [x] Switch only validated entries to C++; remove their Python console and
  production modules, preserve launch policy, clean-install and regression check.
- [x] Independent review, evidence archive, scoped Git checkpoint and refreshed
  inventory. No shared install, Gazebo, hardware, network robot domain, or shared
  process cleanup. Isolated fixtures use explicit owned contexts/domains.


## Task 6: Complete both policy runtime entries and retire navigation bindings

- [x] Stage 6A: typed contracts, persistent fusion, health/calibration, polygon
  continuous sweep and world-risk cores. 8/8 integrated CTest, 126 differential
  tests, 1783 additional risk scenarios and sanitizer checks passed. Preserve
  norm threshold, signed-zero and derived-overflow regressions. Four paired risk
  microbenchmark runs recorded; neither runtime entry nor binding retired here.
- [x] Stage 6B candidate checkpoint `1b51e14e`: native observer, envelope and observation adapters; scoped 11 CTest, 34 paired ROS scenarios, installed 34 cases, 1600 performance frames. This is not entry acceptance: JSON bigint and lone-surrogate differences are recorded and remain to fix.
- [x] Integer/contracts checkpoint `b7a2240b`: 13 CTest, installed 82 paired ROS cases, 1600 matched benchmark frames; exact image bounds and output-stage uint64/environment checks.
- [x] Strings checkpoint `f4feccdf`: lossless surrogate/NUL JSON plus original health/TF conversion order; 14 CTest, 108 paired installed ROS cases, 47+28 sanitizer cases and 1600 matched performance frames. No entry/binding retirement.
- [ ] Complete remaining Stamp domains and failure-stage gaps from `policy_stamp_boundary_review/README.md`, preserving original transaction and output failure stages.
- [ ] Follow `docs/CPP_POLICY_MIGRATION_NEXT_20260921.md`: native observation/world
  state, P2, P3, P4/P5, H2; reuse validated native kernels and preserve ownership.
- [ ] Build pure and ROS protocol differential/clock/lease/cancel/geometry tests
  for every stage, then quantify paired performance and integrated scenarios.
- [ ] Retire both old entries and shared production modules only after complete
  consumer replacement; delete navigation binding definitions/build/dependencies.
- [ ] Keep geometry binding retirement tied to the separate complete transport
  consumer migration. Track external adapter plugins explicitly; no Python fallback.

Task 5 complete at `80a70bde`: whole-package 4/4 CTest (34 core, 64 TF ROS,
46 relay ROS pytest), fresh package/ELF audit, installed smoke and paired
performance. Raw failures and tail outliers retained. Full project goal remains open.
