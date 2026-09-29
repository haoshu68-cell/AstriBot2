# Navigation zones implementation plan

Execution: inline using executing-plans. User requests design then implementation. Preserve dirty workspace, no commits or shared runtime restart; isolated /tmp/astribot_zones_{build,install}.

Goal: RViz-persisted virtual walls and keep-out regions applied to exploration and both navigation costmaps, shared simulation/hardware code with explicit evidence limits.
Architecture and requirements: docs/VIRTUAL_WALLS_AND_KEEP_OUT_PLAN_20260921.md.

- [x] New astribot_navigation_zones: geometry.hpp/cpp, snapshot.hpp/cpp, store.hpp/cpp; tests for geometry, replay, staleness, revision and persistence.
- [x] zone_server.cpp: map/runtime context, authorisation and standstill, transactional replace, consumers' version ACKs, persistence/review of imported mapping zones. Isolated fake dependencies tests.
- [x] zone_layer.hpp/cpp/plugin XML: global/local TF-aware conservative rasterisation, non-clearable layer, stale fails closed. Real LayeredCostmap tests.
- [x] Modify exploration coordinator: filtered map copy, version reset/cancel, readiness and boundary-limited completion. Modify native arbiter and final protection for version cancellation and braking footprint checks.
- [x] zone_page.hpp/cpp and zone_point_tool.cpp: exclusive clicks, wall/rectangle/polygon drafts, table/list save/enable/delete/load, explicit effective state. Integrate workstation and backend services/status/version fields.
- [x] Common navigation launch and RPP/MPPI config; recorder and RViz displays; durable-data operation manual.
- [x] Build affected packages in isolated overlay; run focused and existing regressions; save evidence and distinguish no real hardware acceptance.

Progress: geometry/state/server/layer, map manifest integration, exploration/arbiter/protection/route integration, RViz and gateway, startup and recorder implemented. Build first full pass 8 packages succeeded; final changes and regressions still in progress.
Ruling: preserve current dirty checkout and use isolated build/install, because unrelated ongoing migrations must remain intact; no automatic commit or shared stack restart.
Ruling: raw SLAM data remains untouched; freeze constraints in existing map manifest before finish; post-SAVED edits blocked to avoid stale archive silently overwriting newer edits. Tradeoff: edits after save require loading the map first.
Ruling: geometry/current-map integration tests are isolated C++ ROS tests; no live Gazebo or hardware acceptance claimed.
Fresh review (zones_review): two Important findings, no Critical/Minor. Rejected competing route requests could replace active zone token; fixed by accepting token only with accepted route. SAVED mapping-context edits could vanish on import; fixed by making finalized mapping contexts read-only. Regression coverage added for between-leg changes and SAVED edit refusal. Reviewer excluded unrelated migrations and physical/live UI claims; these remain outside current evidence.

Verification complete for implementation scope: 8 C++ packages + navigation launch/config package built in isolated overlay. 35/35 CTest entries passed, 97 recorded GTest cases (native/differential entries counted separately). Evidence: docs/evidence/navigation_zones_20260921. No live Gazebo/real-hardware acceptance. Real Ogre click test remains future validation; Qt widget drawing/protocol tests and screenshot passed.
Ruling: the standalone SLAM-only mapping_validation harness (launch_navigation=false, no zone server) explicitly disables zone dependency; it cannot validate zone functionality. Production common navigation and owned_mapping enable it.
No commits, pushes, shared installation replacement, runtime restarts or real robot commands performed.
