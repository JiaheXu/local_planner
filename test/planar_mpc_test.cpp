#include "local_planner/planar_mpc.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace local_planner;
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
int main() {
  PlanarMpc solver;
  auto r = solver.solve({1., 0.}, {}, {0., 0.});
  require(r.success && !r.blocked, r.error.c_str());
  require(r.velocity.x() > 0. && std::abs(r.velocity.y()) < 1e-5, "forward command");
  require(std::abs(r.controls.back().x() - .1) < 1e-5, "0.1 m/s speed");
  Eigen::Vector2d last = Eigen::Vector2d::Zero(), position = last;
  for (size_t i = 0; i < r.controls.size(); ++i) {
    const auto& u = r.controls[i];
    require(u.lpNorm<1>() <= .100001, "speed cap");
    require((u-last).lpNorm<Eigen::Infinity>() <= .014001, "acceleration cap");
    position += .1*u;
    require((position-r.path[i+1]).norm() < 1e-7, "prediction integration");
    last = u;
  }
  solver.reset();
  r = solver.solve({0., 1.}, {}, {0., 0.});
  require(r.success && r.velocity.y() > 0. && std::abs(r.velocity.x()) < 1e-5, "sideways");
  solver.reset();
  r = solver.solve({1., 1.}, {}, {0., 0.});
  require(r.success && r.velocity.x() > 0. && r.velocity.y() > 0., "diagonal");
  solver.reset();
  r = solver.solve({1., 0.}, {{.45, 0.}}, {0., 0.});
  require(r.success && r.blocked && r.velocity.norm() == 0., "blocked stop");
  r = solver.solve({1., 0.}, {{.60, 0.}}, {0., 0.});
  require(r.success, r.error.c_str());
  for (const auto& p : r.path) require((p-Eigen::Vector2d(.6,0.)).norm() >= .54999, "clearance");
  solver.reset();
  r = solver.solve({-1., 0.}, {}, {0., 0.});
  require(r.success && r.velocity.x() < 0., "reset old goal");
  require(!solver.solve({std::numeric_limits<double>::quiet_NaN(),0.}, {}, {0.,0.}).success, "invalid goal");
  require(!solver.solve({1.,0.}, {{0.,std::numeric_limits<double>::infinity()}}, {0.,0.}).success, "invalid point");
  require(!solver.solve({1.,0.}, {}, {.11,0.}).success, "invalid previous velocity");
  PlanarMpc deadline(1e-12);
  require(!deadline.solve({1.,0.}, {}, {0.,0.}).success, "deadline must reject result");
  PlanarMpc iterations(.04, 1);
  require(!iterations.solve({1.,0.}, {}, {0.,0.}).success, "nonconvergence must reject result");
  std::cout << "PASS: C++ MPC forward/lateral/diagonal, bounds, obstacles, reset, invalid inputs, timeout, nonconvergence\n";
}
