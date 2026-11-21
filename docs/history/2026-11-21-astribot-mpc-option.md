# Astribot selectable MPC drive stage

User requests MPC as the default for stage two, retaining rotate–drive–rotate
and a selectable library backend. Implement inline, no robot starts or enable.

1. Implement `local_planner_mpc.py`: 20-step, 0.1 s planar velocity MPC using
   installed SciPy SLSQP with analytic cost/constraint derivatives. Optimize
   chassis-frame vx/vy, hold yaw during the drive stage, and retain the existing
   stage controller for initial/final rotations. Max L1 translation 0.05 m/s
   (conservatively within the Euclidean cap), component acceleration 0.14 m/s²,
   nominal 0.55 m obstacle clearance. Closest 24 points supply conservative
   separating halfplanes; validate the predicted path against all points.
2. Integrate 10 Hz solves with 20 Hz command/freshness checks. Reset optimizer
   state on goal/cancel/stage changes; refresh readiness after solves; expired
   solutions, unsuccessful/over-budget solves or invalid results emit zero.
   Extend the independent command envelope to vx/vy/yaw. No global detour guarantee.
3. Default `planner_backend: mpc` in local_planner.yml; support `--planner=library`
   and launch argument selection. MPC launches no lattice process and requires
   no external local_planner package. Desktop inherits the configured default.
4. Verify geometry/bounds/obstacles/reset/timeouts with focused pytest; run
   synthetic ROS rotate–drive–rotate tests for both backends, plus lateral MPC
   movement. Benchmark representative clear/obstacle solves with actual local
   dependencies. Update operator docs with measured timings and limitations.

Design choice: use the already installed SciPy numerical solver, avoiding a new
deployment dependency. Initial solve wall-time budget 40 ms, final value subject
to measured results. A timeout is detected during iterations and on return; this
is not a hard real-time interrupt. Existing owner command expiry remains active.

Completed: default MPC and explicit library backend wired through the same
desktop launcher. All 253 focused tests, nine MPC and eight library synthetic ROS
scenarios passed. The 300-solve benchmark maximum was 18.645 ms, so the 40 ms
acceptance budget remains. Review-driven new-cloud cache revalidation has two
regression tests. Operator docs include selection, measured timings and limits.
No hardware execution was performed.
