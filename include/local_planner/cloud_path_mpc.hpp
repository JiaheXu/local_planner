#pragma once
#include "local_planner/pose_mpc.hpp"
#include <array>
namespace local_planner {
struct CloudPoseMpcResult:PoseMpcResult {bool blocked=false;};

// Direct-goal pose MPC with post-solve cloud collision checks only.
// No candidate assets, path selector or obstacle constraints in the QP.
// Retain every point. Occupied cells only accelerate exact segment checks; they
// do not replace returns with voxel centres or certify unobserved space.
class CloudObstacleIndex {
 public:
  using Clock=std::chrono::steady_clock;
  enum {width=81};
  const Points& points;
  std::array<std::vector<int>,width*width> bins;
  explicit CloudObstacleIndex(const Points& input):points(input) {
    for(size_t i=0;i<input.size();++i)bins[index(input[i].x())*width+index(input[i].y())].push_back(int(i));
  }
  static int index(double v) {return std::max(0,std::min(width-1,int(std::floor((v+4.)/.1))));}
  static double distance2(const Point& p,const Point& a,const Point& d,double length2) {
    const double t=length2>0.?std::max(0.,std::min(1.,(p-a).dot(d)/length2)):0.;
    return (p-a-t*d).squaredNorm();
  }
  int clear_controls(const std::vector<Eigen::Vector3d>& controls,Clock::time_point deadline) const {
    Points path{Point::Zero()};Eigen::Vector3d p=Eigen::Vector3d::Zero();
    for(const auto& u:controls)for(int sub=0;sub<10;++sub) {
      const double turn=.01*u.z();Point delta;
      if(std::abs(u.z())<1e-10)delta=.01*u.head<2>();
      else {
        const double s=std::sin(turn)/u.z(),c=(1.-std::cos(turn))/u.z();
        delta=Point(s*u.x()-c*u.y(),c*u.x()+s*u.y());
      }
      p.head<2>()+=Eigen::Rotation2Dd(p.z())*delta;p.z()+=turn;
      path.push_back(p.head<2>());
    }
    // At |v|<=.1, |w|<=.1, each 10 ms arc deviates from its chord by
    // at most v*w*dt^2/8 = 1.25e-7 m; round outward for numerical error.
    return clear(path,deadline,.5500002);
  }
  // -1 means deadline; 0 collision; 1 clear. Check whole segments, not vertices.
  int clear(const Points& path,Clock::time_point deadline,double radius=.55) const {
    for(size_t k=1;k<path.size();++k) {
      if(Clock::now()>=deadline)return -1;
      const Point& a=path[k-1];const Point d=path[k]-a;const double length2=d.squaredNorm();
      const double clearance=radius;
      int xmin=index(std::min(a.x(),path[k].x())-clearance),xmax=index(std::max(a.x(),path[k].x())+clearance);
      int ymin=index(std::min(a.y(),path[k].y())-clearance),ymax=index(std::max(a.y(),path[k].y())+clearance);
      for(int x=xmin;x<=xmax;++x)for(int y=ymin;y<=ymax;++y) {
        const auto& ids=bins[x*width+y];if(ids.empty())continue;
        const double centre2=distance2(Point(-3.95+.1*x,-3.95+.1*y),a,d,length2);
        if(centre2>(clearance+.071)*(clearance+.071))continue;
        if(centre2<(clearance-.071)*(clearance-.071))return 0;
        size_t visited=0;
        for(int id:ids) {
          if((++visited%1024)==0 && Clock::now()>=deadline)return -1;
          if(distance2(points[id],a,d,length2)<clearance*clearance)return 0;
        }
      }
    }
    return Clock::now()>=deadline?-1:1;
  }
};

class CloudPoseMpc {
 public:
  explicit CloudPoseMpc(double deadline=.04):deadline_(deadline) {}
  CloudPoseMpcResult solve(const Point& goal,double yaw,const Points& obstacles,const Point& previous,double previous_yaw,
                           std::chrono::steady_clock::time_point start=std::chrono::steady_clock::now()) {
    using Clock=std::chrono::steady_clock;
    const auto deadline=start+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(deadline_));
    auto ms=[&]() {return 1000.*std::chrono::duration<double>(Clock::now()-start).count();};
    auto fail=[&](const std::string& error) {CloudPoseMpcResult r;r.error=error;r.solve_ms=ms();return r;};
    auto stop=[&](const std::string& reason) {if(Clock::now()>=deadline)return fail("cloud pose MPC deadline exceeded");CloudPoseMpcResult r;r.success=true;r.blocked=true;r.error=reason;r.poses.push_back(Eigen::Vector3d::Zero());r.solve_ms=ms();return r;};
    if(!std::isfinite(deadline_) || deadline_<=0. || !goal.allFinite() || goal.norm()>10.
       || !previous.allFinite() || previous.lpNorm<1>()>.100001 || !std::isfinite(yaw)
       || !std::isfinite(previous_yaw) || std::abs(previous_yaw)>.100001 || obstacles.size()>500000)
      return fail("invalid cloud pose MPC input");
    bool close=false;
    for(size_t i=0;i<obstacles.size();++i) {
      const auto& p=obstacles[i];
      if(!p.allFinite() || p.squaredNorm()>16.000001)return fail("invalid cloud obstacle point");
      close=close || p.squaredNorm()<.55*.55;
      if(i%1024==0 && Clock::now()>=deadline)return fail("cloud pose MPC deadline exceeded");
    }
    if(Clock::now()>=deadline)return fail("cloud pose MPC deadline exceeded");
    if(close)return stop("obstacle inside clearance");
    CloudObstacleIndex index(obstacles);
    const double remaining=std::chrono::duration<double>(deadline-Clock::now()).count();
    if(remaining<=0.)return fail("cloud pose MPC deadline exceeded");
    auto solved=PoseMpc(remaining).solve(goal,yaw,previous,previous_yaw);
    if(!solved.success)return fail(solved.error);
    Points rollout;rollout.reserve(solved.poses.size());for(const auto& p:solved.poses)rollout.push_back(p.head<2>());
    const int clear=index.clear(rollout,deadline);
    if(clear<0)return fail("cloud pose MPC deadline exceeded");
    if(!clear)return stop("MPC rollout crosses clearance");
    const int swept=index.clear_controls(solved.controls,deadline);
    if(swept<0)return fail("cloud pose MPC deadline exceeded");
    if(!swept)return stop("held body Twist crosses clearance");
    CloudPoseMpcResult result;static_cast<PoseMpcResult&>(result)=std::move(solved);
    result.solve_ms=ms();
    if(result.solve_ms>1000.*deadline_)return fail("cloud pose MPC deadline exceeded");
    return result;
  }
 private:
  double deadline_;
};
}
