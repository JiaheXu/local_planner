#include "local_planner/planar_mpc.hpp"
#include <iostream>
#include <iomanip>
int main() {
  std::cout << "scenario,t,x,y,vx,vy,blocked,solve_ms\n" << std::setprecision(12);
  for (int scenario=0;scenario<3;++scenario) {
    local_planner::PlanarMpc solver;
    local_planner::Points obstacles;
    if(scenario==1) obstacles.emplace_back(.65,0.);
    if(scenario==2) obstacles.emplace_back(.45,0.);
    auto result=solver.solve({.30,.12},obstacles,{0.,0.});
    if(!result.success){std::cerr<<result.error; return 1;}
    for(size_t i=0;i<result.path.size();++i) {
      auto v=i<result.controls.size()?result.controls[i]:local_planner::Point::Zero().eval();
      std::cout<<scenario<<','<<i*.1<<','<<result.path[i].x()<<','<<result.path[i].y()<<','
        <<v.x()<<','<<v.y()<<','<<result.blocked<<','<<result.solve_ms<<'\n';
    }
  }
}
