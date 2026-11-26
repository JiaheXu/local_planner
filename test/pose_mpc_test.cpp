#include "local_planner/pose_mpc.hpp"
#include <iostream>
#include <stdexcept>
using namespace local_planner;
void check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
int main() {
  PoseMpc solver;
  Points reference(20,Point(0.,.2));
  auto guided=solver.solve({1.,0.},.4,{0.,0.},0.,reference);
  check(guided.success && guided.velocity.y()>.001 && std::abs(guided.velocity.x())<.001,
        "XY reference must guide translation while retaining final yaw");
  check(guided.yaw_rate>0.,"reference tracking keeps joint yaw");
  check(!solver.solve({1.,0.},0.,{0.,0.},0.,Points(19,Point::Zero())).success,
        "reject incomplete reference horizon");
  reference[4].x()=NAN;
  check(!solver.solve({1.,0.},0.,{0.,0.},0.,reference).success,"reject invalid reference sample");
  auto far=solver.solve({.5,.2},1.,{0.,0.},0.);
  check(far.success,far.error.c_str());
  check(far.velocity.norm()>0. && far.yaw_rate>0.,"translate and turn together");
  check(far.poses.size()==21 && far.controls.size()==20,"full pose horizon");
  Eigen::Vector3d previous=Eigen::Vector3d::Zero(), pose=previous;
  for(size_t i=0;i<far.controls.size();++i) {
    const auto& u=far.controls[i];
    check(u.head<2>().lpNorm<1>()<=.100001 && std::abs(u.z())<=.100001,"speed bounds");
    check((u-previous).head<2>().lpNorm<Eigen::Infinity>()<=.014001 && std::abs(u.z()-previous.z())<=.020001,"acceleration bounds");
    pose=PoseMpc::step(pose,u);
    check((pose-far.poses[i+1]).norm()<1e-8,"nonlinear trajectory agrees with controls");
    previous=u;
  }
  auto near=solver.solve({.05,0.},.08,{0.,0.},0.);
  check(near.success,near.error.c_str());
  check(near.controls.back().head<2>().norm()<far.controls.back().head<2>().norm(),"near goal translates slower");
  check(std::abs(near.controls.back().z())<std::abs(far.controls.back().z()),"near goal turns slower");
  auto turn=solver.solve({0.,0.},1.,{0.,0.},0.);
  check(turn.success && turn.velocity.norm()<1e-6 && turn.yaw_rate>0.,"finish orientation at target position");
  auto wrap=solver.solve({0.,0.},2.*M_PI-.2,{0.,0.},0.);
  check(wrap.success && wrap.yaw_rate<0.,"shortest heading wrap");
  check(!solver.solve({0.,0.},NAN,{0.,0.},0.).success,"reject nonfinite yaw");
  check(!solver.solve({0.,0.},1.,{0.,0.},.2).success,"reject previous yaw overspeed");
  PoseMpc deadline(1e-12);
  check(!deadline.solve({.1,0.},1.,{0.,0.},0.).success,"deadline fails closed");
  // Receding-horizon loop with actual body translation while turning.
  pose.setZero();previous.setZero();
  const Eigen::Vector3d target(.3,-.15,.4);
  double maximum_ms=0.; bool arrived=false;
  for(int k=0;k<250;++k) {
    Eigen::Rotation2Dd rotation(-pose.z());
    Point goal=rotation*(target.head<2>()-pose.head<2>());
    const double angle=std::atan2(std::sin(target.z()-pose.z()),std::cos(target.z()-pose.z()));
    if(goal.norm()<=.03 && std::abs(angle)<=.05) {arrived=true;break;}
    auto r=solver.solve(goal,angle,previous.head<2>(),previous.z());
    check(r.success,r.error.c_str());maximum_ms=std::max(maximum_ms,r.solve_ms);
    previous=r.controls.front();
    if(goal.norm()<=.03)previous.head<2>().setZero();
    if(std::abs(angle)<=.05)previous.z()=0.;
    pose=PoseMpc::step(pose,previous);
  }
  check(arrived,"closed-loop pose arrival");
  std::cout<<"PASS: pose MPC bounds, coupled rollout, slowdown, rotation, wrap, convergence; max_ms="<<maximum_ms<<"\n";
}
