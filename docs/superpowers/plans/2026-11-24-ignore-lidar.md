# MPC without LiDAR implementation plan

**Goal:** Add an explicit Python MPC test option that requires odometry but no collision cloud.

**Architecture:** `--ignore-lidar` passes a boolean through XML/Python launch. The coordinator omits cloud subscriptions and cloud readiness/publisher checks, sends empty-obstacle MPC requests from validated advancing odometry, and retains goal sequencing, owner gating, command expiry, cancellation and bounds. Default behavior remains collision enabled. FAST-LIO still needs its own sensors upstream.

**Verification:** Add failing tests for launcher selection, readiness without clouds, owner/odometry rejection and odometry-triggered requests; implement the option; run package tests and a synthetic shadow-only MPC scenario with no cloud publisher. Update operator documentation and install. No live robot startup or commands.

**Review focus:** Default obstacle gates unchanged; unsupported backends rejected; bad odometry cancels; cancelled goals cannot trigger requests; no cloud subscriptions or requests in the test scenario.
