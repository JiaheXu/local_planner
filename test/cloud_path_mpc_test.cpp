#include "local_planner/cloud_path_mpc.hpp"
#include <iostream>
#include <stdexcept>
using namespace local_planner;
void require(bool b,const char* message) {if(!b)throw std::runtime_error(message);}
int main() {
  CloudPoseMpc solver; // No path assets or candidate library exist in this API.
  auto plain=PoseMpc().solve({2.,0.},.5,Point::Zero(),0.);
  auto clear=solver.solve({2.,0.},.5,{},Point::Zero(),0.);
  require(plain.success && clear.success && !clear.blocked,"empty cloud direct pose solve");
  require((plain.velocity-clear.velocity).norm()<1e-10 && plain.yaw_rate==clear.yaw_rate,
          "direct goal output equals ordinary pose MPC");
  Points cylinder;
  for(int i=0;i<60;++i)cylinder.emplace_back(1.2+.2*std::cos(i*M_PI/30),.2*std::sin(i*M_PI/30));
  auto distant=solver.solve({2.,0.},.5,cylinder,Point::Zero(),0.);
  require(distant.success && !distant.blocked,"distant obstacle outside rollout is clear");
  require((distant.velocity-plain.velocity).norm()<1e-10 && distant.yaw_rate==plain.yaw_rate,
          "obstacles do not select detour references or alter optimization");
  for(size_t i=0;i<plain.poses.size();++i)
    require((plain.poses[i]-distant.poses[i]).norm()<1e-10,"same direct-goal trajectory");
  for(const Points& cloud:std::vector<Points>{{{.54,0.}},{{.65,0.}}}) {
    auto stop=solver.solve({2.,0.},.5,cloud,Point::Zero(),0.);
    require(stop.success && stop.blocked && stop.poses.size()==1 && stop.velocity.norm()==0.
            && stop.yaw_rate==0.,"close footprint or unsafe rollout stops all axes");
  }
  Points beyond24(24,Point(-.7,0.));beyond24.emplace_back(.65,0.);
  require(solver.solve({2.,0.},0.,beyond24,Point::Zero(),0.).blocked,
          "all cloud points participate in stop checks");
  for(const Point& goal:Points{{0.,2.},{-2.,0.}}) {
    auto omni=solver.solve(goal,-.5,{{3.5,0.}},Point::Zero(),0.);
    require(omni.success && !omni.blocked && omni.velocity.dot(goal)>0. && omni.yaw_rate<0.,
            "sideways and reverse direct goals retain simultaneous yaw");
  }
  auto near=solver.solve({.05,0.},.08,{{-1.,0.}},Point::Zero(),0.);
  require(near.success && !near.blocked && near.controls.back().head<2>().norm()<.031,
          "goal distance slows translation");
  require(!solver.solve({2.,0.},0.,{{NAN,0.}},Point::Zero(),0.).success,"invalid point rejected");
  require(!solver.solve({2.,0.},0.,{{4.1,0.}},Point::Zero(),0.).success,"out of range rejected");
  require(!solver.solve({2.,0.},0.,Points(500001,Point(1.,0.)),Point::Zero(),0.).success,"oversized cloud rejected");
  require(!CloudPoseMpc(1e-12).solve({2.,0.},0.,{},Point::Zero(),0.).success,"combined deadline enforced");
  Points arc{{.01,.550025}};CloudObstacleIndex index(arc);
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  require(index.clear({Point::Zero(),Point(.01,0.)},deadline)==1,"Euler chord fixture clear");
  require(index.clear_controls({Eigen::Vector3d(.1,0.,.1)},deadline)==0,"held Twist arc collision rejected");
  auto dense=solver.solve({2.,0.},.5,Points(10000,Point(1.2,0.)),Point::Zero(),0.);
  require(dense.success && !dense.blocked && dense.solve_ms<=40.,"dense cloud within acceptance budget");
  std::cout<<"PASS direct-goal cloud MPC: no selector/assets, unchanged optimization, stop checks, yaw, slowdown; dense_ms="<<dense.solve_ms<<"\n";
}
