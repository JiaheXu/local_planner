# Astribot Local Planner Implementation Plan

> Implement inline using superpowers:executing-plans. Existing checkout changes are preserved; no robot commands or SDK startups.

**Goal:** Front-LiDAR local collision avoidance using FAST-LIO pose and the existing XPolicy owner.

**Architecture:** Astribot Python adapter/follower plus the supplied C++ lattice; private scan/goal/path topics and optional `/cmd_vel` output. No Nav2.

**Spec:** `docs/superpowers/specs/2026-11-21-astribot-local-planner-design.md`

## Tasks

1. Add failing offline tests for point-cloud mount/height conversion, map-relative following, 3 cm arrival, obstacles, stale path and cancel. Implement the pure core in `real_world/robots/astribot_s1/local_planner_core.py`.
2. Patch external `localPlanner.cpp` to pair stamped chassis goals/scans, publish source-stamped paths, and reject missing/cancelled goals. Add ROS adapter `local_planner_node.py`; no SDK/enable calls.
3. Build through `scripts/setup_astribot_local_planner.sh`, launch through `scripts/astribot_local_planner.sh`, add RViz path visualization and current operator documentation. Default standalone launch is shadow.
4. Run focused pytest, build/syntax checks, and isolated real-planner ROS synthetic tests. Test forward/blocked/cancel/stale flows without hardware.

## Review focus

Path/goal epoch mismatches, old source timestamps, malformed cloud layouts, nominal footprint scale, and another `/cmd_vel` publisher must fail closed. Preserve all unrelated source changes and the existing joystick selector.

## Execution rulings

The user requested implementation and supplied the planner; use the existing language/algorithm and proceed with independent work while the RViz interface clarification is pending. No extra design approval round or commits. The legacy follower remains available but is excluded from the Astribot launch.

User subsequently confirmed RViz goal/cancel. Tasks 1–3 implemented; external
source patched and package compiled. Task 4: focused checks and actual lattice
ROS smoke passed before review. Independent review flagged all-zero cloud
placeholders and out-of-order paths. Both have failing regression tests followed
by fixes; final checks include the updated suite and ROS smoke.

Final verification: 226 focused pytest tests passed; the updated C++ planner
and RViz plugin built, RViz ctest passed (1/1), shell/Python syntax and launch
`--check` passed. The isolated domain-227 ROS smoke passed all six scenarios,
including 0.2 m arrival at synthetic FAST-LIO x=0.170 m. Hardware untested.
All four tasks complete; no commits or live robot starts were performed.
