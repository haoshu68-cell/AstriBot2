# Canonical fixed-base scene contract

The planner input, actual-payload revalidation, and native binding now share one C++ canonicalScene contract. It accepts complete fixed-base scenes: world objects and map outer frame are astribot_torso_base; the producer's empty inner map frame is retained; attachments must be link-local on known robot-model links; multiDOF state is rejected. These checks precede omission of the authorized payload. Only unused external fixed transforms are then removed. Geometry, collision policy, padding, scale, and map origin remain exactly compared without numeric tolerances.

MTC 5/5 CTest and native binding 5/5 checks pass. Original payload tests are retained with the obsolete unused-TF rejection case replaced by a real octomap-origin-change rejection. Four payload and two native behavior failures were observed before the fix. Root independently reran MTC 5/5. An initial test used an old shadowing library; final logs bind the new installed library explicitly and retain that failure.

The READY scene01 actual native pre/post query pair was not saved. Two available pre-dispatch snapshots reproduce the unused-TF issue, but do not alone prove that runtime rejection's full cause. Actual first-stage execution is still pending. Root is separately building the frozen first-stage source with bounded raw CDR capture. Default empty known_links accepts no attachments; full native execution must supply links parsed from the actual URDF. This change does not relax world/attachment changes into occupancy-only revalidation or alter joint margins.

No changes to the working full-action hold_executor or its CMake were included. Installed runtime identities and exact replay commands are in result.json and reproduce.bash.
