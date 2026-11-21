# Move MPC into the supplied C++ local_planner package

User correction: planning belongs in `ros2_ws/src/local_planner`, not a Python
optimizer under XPolicy. Keep the requested 0.1 m/s MPC default.

1. Add an Eigen-based planar MPC and standalone C++ regression executable in
   the existing package. Preserve the 20 x 0.1 s horizon, speed/acceleration and
   collision constraints. Solve the convex QP with bounded ADMM iterations,
   residual checks and the existing 40 ms acceptance limit. No Python solver.
2. Add typed MPC request/result messages and a backend branch in the existing
   `localPlanner` executable. Requests include source stamp, goal epoch, goal,
   previous accepted velocity and chassis obstacle points. Results include
   command, prediction and explicit failure. MPC does not load lattice assets.
3. Keep XPolicy's sensor conversion, goal-stage sequencing and owner safety
   adapter. Replace its optimization call with request/result transport;
   reject stale, mismatched, invalid or late proposals. Preserve immediate
   cancellation, new-cloud collision validation and exactly one SDK owner.
   Remove the Python MPC implementation. Launch C++ for either backend.
4. Rebuild the existing package without launching hardware. Run C++ tests,
   focused adapter tests and isolated ROS tests for both backends; update the
   reproducible package patch and current operator documentation. Review the
   typed boundary, cancellation races, failures and missing planner output.

Canonical new implementation lives in the external package; XPolicy keeps its
reproducible patch and launcher. All existing user modifications are preserved.

Completed: C++ package rebuilt and installed; Python optimizer removed; default
C++ MPC and optional C++ library wired through the desktop launcher. CTest,
259 focused pytest cases and 19 isolated ROS scenarios passed. Final review
findings have regression coverage. 300 native solves completed within 40 ms
(maximum 13.250 ms on this PC, without live ROS sensor load).

Additional operator correction: positive ROS angular.z reportedly turned right.
Removed the existing SDK yaw inversion (`sdk_signs` now `[1,1,1]`) at the sole
owner boundary, with positive/negative yaw unit tests and updated current docs.
Physical direction and navigation remain unverified; no hardware run was made.
