#pragma once
#include "local_planner/planar_mpc.hpp"
#include <Eigen/Geometry>

namespace local_planner {
struct PoseMpcResult {
  bool success=false;
  std::string error;
  Point velocity=Point::Zero();
  double yaw_rate=0., solve_ms=0.;
  std::vector<Eigen::Vector3d> poses, controls;
};

// Obstacle-free body-velocity MPC. One sequential-QP linearization per request;
// return the nonlinear rollout, not the affine approximation, for validation.
class PoseMpc {
 public:
  enum { horizon=20, variables=60 };
  explicit PoseMpc(double deadline=.04): deadline_(deadline) {}
  static Eigen::Vector3d step(const Eigen::Vector3d& p,const Eigen::Vector3d& u) {
    Eigen::Vector3d next=p;
    next.head<2>()+=.1*(Eigen::Rotation2Dd(p.z())*u.head<2>());
    next.z()+=.1*u.z();
    return next;
  }
  static double xy_cap(double distance) {return distance<=.03 ? 0. : std::min(.1,.6*distance);}
  static double yaw_cap(double distance,double error) {
    return std::abs(error)<=.05 ? 0. : std::min({.1,.03+.07*std::min(distance/.3,1.),.8*std::abs(error)});
  }
  PoseMpcResult solve(const Point& goal,double goal_yaw,const Point& previous,double previous_yaw) {
    using Clock=std::chrono::steady_clock;
    const auto start=Clock::now();
    auto elapsed=[&]() {return std::chrono::duration<double>(Clock::now()-start).count();};
    auto fail=[&](const std::string& error) {PoseMpcResult r;r.error=error;r.solve_ms=1000.*elapsed();return r;};
    if(!goal.allFinite() || goal.norm()>10. || !previous.allFinite() || previous.lpNorm<1>()>.100001
       || !std::isfinite(goal_yaw) || !std::isfinite(previous_yaw) || std::abs(previous_yaw)>.100001
       || !std::isfinite(deadline_) || deadline_<=0.)return fail("invalid pose MPC input");
    goal_yaw=std::atan2(std::sin(goal_yaw),std::cos(goal_yaw));
    Eigen::Vector3d prev(previous.x(),previous.y(),previous_yaw), nominal=Eigen::Vector3d::Zero();
    Eigen::MatrixXd prediction=Eigen::MatrixXd::Zero(variables,variables);
    Eigen::MatrixXd sensitivity=Eigen::MatrixXd::Zero(3,variables);
    Eigen::VectorXd offset=Eigen::VectorXd::Zero(variables),weights(variables),target(variables);
    Eigen::Vector3d affine=Eigen::Vector3d::Zero();
    Eigen::MatrixXd D=Eigen::MatrixXd::Identity(variables,variables);
    Eigen::MatrixXd A=Eigen::MatrixXd::Zero(240,variables);
    Eigen::VectorXd bounds(240),d0=Eigen::VectorXd::Zero(variables);
    d0.head<3>()=prev;
    Eigen::Matrix<double,4,2> signs;signs<<1.,1.,1.,-1.,-1.,1.,-1.,-1.;
    const double distance=goal.norm();
    for(int i=0;i<horizon;++i) {
      // Linearize p+=R(theta)*v*dt around constant previous body velocity.
      const double c=std::cos(nominal.z()),s=std::sin(nominal.z());
      Eigen::Matrix3d F=Eigen::Matrix3d::Identity(),G=Eigen::Matrix3d::Zero();
      F(0,2)=.1*(-s*prev.x()-c*prev.y());F(1,2)=.1*(c*prev.x()-s*prev.y());
      G(0,0)=.1*c;G(0,1)=-.1*s;G(1,0)=.1*s;G(1,1)=.1*c;G(2,2)=.1;
      Eigen::Vector3d bias=Eigen::Vector3d::Zero();
      bias.head<2>()=-F.block<2,1>(0,2)*nominal.z();
      sensitivity=F*sensitivity; sensitivity.block<3,3>(0,3*i)+=G;
      affine=F*affine+bias;
      prediction.middleRows(3*i,3)=sensitivity;offset.segment<3>(3*i)=affine;
      target.segment<3>(3*i)=Eigen::Vector3d(goal.x(),goal.y(),goal_yaw);
      weights.segment<3>(3*i)=Eigen::Vector3d(5.,5.,2.)*(i==horizon-1?3.:1.);
      if(i)D.block<3,3>(3*i,3*i-3)=-Eigen::Matrix3d::Identity();
      A.block<4,2>(6*i,3*i)=signs;
      A(6*i+4,3*i+2)=1.; A(6*i+5,3*i+2)=-1.;
      // A tightening envelope must still admit acceleration-bounded braking.
      const double time=.1*(i+1);
      const double braking_xy=std::max(0.,std::abs(prev.x())-.14*time)+std::max(0.,std::abs(prev.y())-.14*time);
      const double braking_yaw=std::max(0.,std::abs(prev.z())-.2*time);
      bounds.segment<4>(6*i).setConstant(std::max(xy_cap(distance),braking_xy));
      bounds.segment<2>(6*i+4).setConstant(std::max(yaw_cap(distance,goal_yaw),braking_yaw));
      nominal=step(nominal,prev);
    }
    A.middleRows(120,60)=D;A.bottomRows(60)=-D;
    for(int i=0;i<variables;++i) {
      const double delta=i%3==2?.02:.014;
      bounds(120+i)=delta+d0(i);bounds(180+i)=delta-d0(i);
    }
    const Eigen::MatrixXd weighted=weights.asDiagonal()*prediction;
    // Penalize XY and angular acceleration in the same optimization.
    const Eigen::MatrixXd H=2.*(prediction.transpose()*weighted+.2*D.transpose()*D
                                +.001*Eigen::MatrixXd::Identity(variables,variables));
    const Eigen::VectorXd q=2.*(weighted.transpose()*(offset-target)-.2*D.transpose()*d0);
    for(int i=0;i<A.rows();++i) {const double n=A.row(i).norm();A.row(i)/=n;bounds(i)/=n;}
    const Eigen::MatrixXd ata=A.transpose()*A;
    double rho=1.;Eigen::LDLT<Eigen::MatrixXd> factor;
    auto refactor=[&]() {factor.compute(H+rho*ata);};refactor();
    Eigen::VectorXd u=Eigen::VectorXd::Zero(variables),y=Eigen::VectorXd::Zero(A.rows());
    Eigen::VectorXd z=(A*u).cwiseMin(bounds);
    bool converged=false;
    for(int iteration=0;iteration<4000;++iteration) {
      if(elapsed()>deadline_)return fail("pose MPC deadline exceeded");
      u=factor.solve(-q+A.transpose()*(rho*z-y));
      if(!u.allFinite())return fail("pose MPC numerical failure");
      const Eigen::VectorXd au=A*u,relaxed=1.6*au-.6*z;
      z=(relaxed+y/rho).cwiseMin(bounds);y+=rho*(relaxed-z);
      if(iteration%10==0) {
        const double primal=(au-z).lpNorm<Eigen::Infinity>();
        const double dual=(H*u+q+A.transpose()*y).lpNorm<Eigen::Infinity>();
        if(primal<1e-7 && dual<1e-6){converged=true;break;}
        if(iteration>0 && iteration%50==0) {
          const double next=std::max(.001,std::min(1000.,rho*std::sqrt((primal+1e-12)/(dual+1e-12))));
          if(next>2.*rho || next<.5*rho){rho=next;refactor();}
        }
      }
    }
    if(!converged || (A*u-bounds).maxCoeff()>1e-6)return fail("pose MPC did not converge");
    PoseMpcResult result;Eigen::Vector3d p=Eigen::Vector3d::Zero();result.poses.push_back(p);
    for(int i=0;i<horizon;++i) {
      Eigen::Vector3d control=u.segment<3>(3*i);
      // Remove numerical overshoot at hard speed boundaries.
      const double l1=control.head<2>().lpNorm<1>();
      if(l1>.1)control.head<2>()*=.1/l1;
      control.z()=std::max(-.1,std::min(.1,control.z()));
      result.controls.push_back(control);p=step(p,control);result.poses.push_back(p);
    }
    if(elapsed()>deadline_)return fail("pose MPC deadline exceeded");
    result.velocity=result.controls.front().head<2>();result.yaw_rate=result.controls.front().z();
    result.success=true;result.solve_ms=1000.*elapsed();return result;
  }
 private:
  double deadline_;
};
} // namespace local_planner
