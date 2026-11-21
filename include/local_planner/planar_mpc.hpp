#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

namespace local_planner {
using Point = Eigen::Vector2d;
using Points = std::vector<Point>;
struct MpcResult {
  bool success = false, blocked = false;
  std::string error;
  Point velocity = Point::Zero();
  Points path, controls;
  double solve_ms = 0.;
};

// Small convex velocity MPC, using Eigen only. No ROS/SDK/hardware operations.
// ADMM solves min .5*u'H*u+q'u subject to A*u<=b. This implementation is not OSQP.
class PlanarMpc {
 public:
  static constexpr int horizon = 20, variables = 40;
  static constexpr double dt = .1, speed = .1, clearance = .55, acceleration = .14;
  explicit PlanarMpc(double deadline_s=.04, int max_iterations=4000)
      : deadline_(deadline_s), max_iterations_(max_iterations) {
    B_ = Eigen::MatrixXd::Zero(variables, variables);
    D_ = Eigen::MatrixXd::Identity(variables, variables);
    Eigen::VectorXd weights(variables);
    for (int i=0; i<horizon; ++i) {
      weights.segment<2>(2*i).setConstant(i == horizon-1 ? 15. : 5.);
      for (int j=0; j<=i; ++j) B_.block<2,2>(2*i,2*j) = dt*Eigen::Matrix2d::Identity();
      if (i) D_.block<2,2>(2*i,2*i-2) = -Eigen::Matrix2d::Identity();
    }
    WB_ = weights.asDiagonal()*B_;
    H_ = 2.*(B_.transpose()*WB_ + .02*D_.transpose()*D_
             + .001*Eigen::MatrixXd::Identity(variables,variables));
    reset();
  }
  void reset() { warm_ = Eigen::VectorXd::Zero(variables); }
  MpcResult solve(const Point& goal, const Points& points, const Point& previous) {
    const auto start = Clock::now();
    MpcResult result;
    auto elapsed = [&]() { return std::chrono::duration<double>(Clock::now()-start).count(); };
    auto fail = [&](const std::string& error) {
      reset(); result = MpcResult(); result.error = error; result.solve_ms = elapsed()*1000.; return result;
    };
    if (!std::isfinite(deadline_) || deadline_<=0. || !goal.allFinite() || !previous.allFinite()
        || goal.norm()>10. || previous.lpNorm<1>()>speed+1e-6 || points.size()>500000)
      return fail("invalid MPC input");
    std::vector<std::pair<double,size_t>> nearest;
    bool blocked = false;
    for (size_t i=0; i<points.size(); ++i) {
      if (!points[i].allFinite()) return fail("nonfinite MPC obstacle");
      const double distance = points[i].norm();
      blocked = blocked || distance<=clearance;
      if (distance<=clearance+speed*dt*horizon+.02) nearest.emplace_back(distance,i);
    }
    if (elapsed()>deadline_) return fail("MPC deadline exceeded");
    if (blocked) {
      reset(); result.success=true; result.blocked=true; result.path.push_back(Point::Zero());
      result.solve_ms=elapsed()*1000.; return result;
    }
    const size_t count = std::min<size_t>(24, nearest.size());
    std::partial_sort(nearest.begin(), nearest.begin()+count, nearest.end());
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(160+20*count, variables);
    Eigen::VectorXd b(A.rows()), d0 = Eigen::VectorXd::Zero(variables), target(variables);
    d0.head<2>()=previous;
    Eigen::Matrix<double,4,2> signs;
    signs << 1.,1., 1.,-1., -1.,1., -1.,-1.;
    for (int i=0; i<horizon; ++i) {
      target.segment<2>(2*i)=goal;
      A.block<4,2>(4*i,2*i)=signs;
    }
    b.head(80).setConstant(speed);
    A.middleRows(80,40)=D_; A.middleRows(120,40)=-D_;
    b.segment(80,40)=Eigen::VectorXd::Constant(variables,acceleration*dt)+d0;
    b.segment(120,40)=Eigen::VectorXd::Constant(variables,acceleration*dt)-d0;
    for (size_t j=0; j<count; ++j) {
      const Point normal=points[nearest[j].second]/nearest[j].first;
      for (int i=0; i<horizon; ++i) {
        A.row(160+20*j+i)=normal.transpose()*B_.middleRows(2*i,2);
        b(160+20*j+i)=nearest[j].first-clearance;
      }
    }
    // Row normalization improves ADMM conditioning without changing feasible space.
    for (int i=0; i<A.rows(); ++i) { double scale=A.row(i).norm(); A.row(i)/=scale; b(i)/=scale; }
    const Eigen::VectorXd q=-2.*(WB_.transpose()*target+.02*D_.transpose()*d0);
    const Eigen::MatrixXd ata=A.transpose()*A;
    double rho=1.;
    Eigen::LDLT<Eigen::MatrixXd> factor;
    auto refactor = [&]() { factor.compute(H_+rho*ata); };
    refactor();
    Eigen::VectorXd u=warm_, y=Eigen::VectorXd::Zero(A.rows()), z=(A*u).cwiseMin(b);
    bool converged=false;
    for (int iteration=0; iteration<max_iterations_; ++iteration) {
      if (elapsed()>deadline_) return fail("MPC deadline exceeded");
      u=factor.solve(-q+A.transpose()*(rho*z-y));
      if (!u.allFinite()) return fail("MPC numerical failure");
      const Eigen::VectorXd au=A*u;
      const Eigen::VectorXd relaxed=1.6*au-.6*z;
      z=(relaxed+y/rho).cwiseMin(b);
      y+=rho*(relaxed-z);
      if (iteration%10==0) {
        const double primal=(au-z).lpNorm<Eigen::Infinity>();
        const double dual=(H_*u+q+A.transpose()*y).lpNorm<Eigen::Infinity>();
        if (primal<1e-7 && dual<1e-6) { converged=true; break; }
        if (iteration>0 && iteration%50==0) {
          double next=std::max(.001,std::min(1000.,rho*std::sqrt((primal+1e-12)/(dual+1e-12))));
          if (next>2.*rho || next<.5*rho) { rho=next; refactor(); }
        }
      }
    }
    if (!converged || (A*u-b).maxCoeff()>1e-6) return fail("MPC did not converge to feasible solution");
    result.path.push_back(Point::Zero());
    const Eigen::VectorXd predicted=B_*u;
    for (int i=0; i<horizon; ++i) {
      result.controls.push_back(u.segment<2>(2*i));
      result.path.push_back(predicted.segment<2>(2*i));
    }
    // Check every return, including those omitted from the bounded QP.
    for (const auto& point : points) for (const auto& center : result.path) {
      if ((point-center).squaredNorm()<(clearance-1e-5)*(clearance-1e-5)) {
        reset(); result=MpcResult(); result.success=true; result.blocked=true;
        result.path.push_back(Point::Zero());
        if (elapsed()>deadline_) return fail("MPC deadline exceeded");
        result.solve_ms=elapsed()*1000.; return result;
      }
    }
    if (elapsed()>deadline_) return fail("MPC deadline exceeded");
    warm_.head(variables-2)=u.tail(variables-2); warm_.tail<2>()=u.tail<2>();
    result.velocity=u.head<2>();
    if (result.velocity.lpNorm<1>()>speed) result.velocity*=speed/result.velocity.lpNorm<1>();
    result.success=true; result.solve_ms=elapsed()*1000.;
    return result;
  }
 private:
  using Clock=std::chrono::steady_clock;
  double deadline_;
  int max_iterations_;
  Eigen::MatrixXd B_, D_, WB_, H_;
  Eigen::VectorXd warm_;
};
}  // namespace local_planner
