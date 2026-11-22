#pragma once

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace astribot {
using Point = Eigen::Vector2d;
using Pose = Eigen::Vector3d;
using Path = std::vector<Point>;
using Mount = Eigen::Affine3d;

inline Mount load_mount(bool rear) {
  auto cfg = YAML::LoadFile(ament_index_cpp::get_package_share_directory("local_planner") +
                            "/config/astribot_geometry.yaml");
  if (rear && cfg["rear_cloud_coordinates"].as<std::string>() != "front_lidar")
    throw std::invalid_argument("Unqualified rear cloud coordinates; expected front_lidar contract");
  const auto mount = cfg["base_from_front_lidar"];
  const auto xyz = mount["xyz"].as<std::vector<double>>();
  const auto q = mount["xyzw"].as<std::vector<double>>();
  if (xyz.size() != 3 || q.size() != 4) throw std::invalid_argument("Invalid front mount dimensions");
  const Eigen::Quaterniond quat(q[3],q[0],q[1],q[2]);
  const Pose translation(xyz[0],xyz[1],xyz[2]);
  if (!translation.allFinite() || !quat.coeffs().allFinite() || std::abs(quat.norm()-1.) > 1e-5)
    throw std::invalid_argument("Invalid front mount: expected metres and unit xyzw quaternion");
  return Mount(Eigen::Translation3d(translation) * quat.normalized());
}

inline Path obstacles(const sensor_msgs::msg::PointCloud2 &cloud, const Mount &mount) {
  const uint64_t size = uint64_t(cloud.width)*cloud.height;
  if (!size || size > 500000 || cloud.point_step < 12 ||
      cloud.row_step < uint64_t(cloud.width)*cloud.point_step ||
      cloud.data.size() != uint64_t(cloud.height)*cloud.row_step)
    throw std::invalid_argument("invalid/empty PointCloud2 layout");
  std::array<uint32_t,3> offsets;
  for (int i=0; i<3; ++i) {
    auto name = std::string(1,"xyz"[i]);
    auto field = std::find_if(cloud.fields.rbegin(),cloud.fields.rend(),
                              [&](const auto &f){return f.name == name;});
    if (field == cloud.fields.rend() || field->datatype != 7 || field->count != 1 ||
        field->offset > cloud.point_step-4) throw std::invalid_argument("expected scalar FLOAT32 XYZ fields");
    offsets[i] = field->offset;
  }
  Path result;
  bool usable = false;
  for (uint32_t row=0; row<cloud.height; ++row) {
    for (uint32_t col=0; col<cloud.width; ++col) {
      const auto *point = cloud.data.data()+size_t(row)*cloud.row_step+size_t(col)*cloud.point_step;
      Pose p;
      for (int i=0; i<3; ++i) {
        const auto *bytes = point+offsets[i];
        uint32_t bits = 0;
        for (int b=0; b<4; ++b) bits |= uint32_t(bytes[cloud.is_bigendian ? 3-b : b]) << (8*b);
        float f;
        std::memcpy(&f,&bits,4);
        p[i] = f;
      }
      if (!p.allFinite() || p.norm()<.05) continue;
      usable = true;
      p = mount*p;
      double distance = p.head<2>().norm();
      if (p.z()>=.03 && p.z()<=1.8 && distance>=.30 && distance<=4.) result.emplace_back(p.head<2>());
    }
  }
  if (!usable) throw std::invalid_argument("cloud contains no usable sensor returns");
  return result;
}

inline Point relative_goal(const Point &goal, const Pose &pose) {
  return Eigen::Rotation2Dd(-pose.z())*(goal-pose.head<2>());
}
inline Point world_point(const Point &local, const Pose &pose) {
  return Eigen::Rotation2Dd(pose.z())*local+pose.head<2>();
}
inline bool collision_free(double vx,double vy,double wz,const Path &points,double clearance=.4,double horizon=2.) {
  for (double value : {vx,vy,wz,clearance,horizon}) if (!std::isfinite(value)) return false;
  if (clearance<=0 || horizon<=0) return false;
  for (int i=0;i<=20;++i) {
    double t=horizon*i/20., a=wz*t;
    Point center(vx*t,vy*t);
    if (std::abs(wz)>=1e-8) center = Point(vx*std::sin(a)+vy*(std::cos(a)-1),
                                            vx*(1-std::cos(a))+vy*std::sin(a))/wz;
    for (const Point &p : points) if (!p.allFinite() || (p-center).norm()<=clearance) return false;
  }
  return true;
}
constexpr double never = -std::numeric_limits<double>::infinity();
inline bool within(double value,double low,double high) { return low<=value && value<=high; }
inline void require(bool valid,const std::string &reason) {
  if (!valid) throw std::invalid_argument(reason);
}
inline int64_t stamp_ns(const builtin_interfaces::msg::Time &time) {
  return int64_t(time.sec)*1000000000+time.nanosec;
}
struct Navigation {
  int64_t reference_stamp=0,anchor_stamp=0,stamp=0;
  double reference_received=never,anchor_received=never,stable_since=never,received=never;
  double pc_minus_source_ms=0;
  bool qualified=false,clock_fault=false,reference_ready=false;
  std::string reference_error,error;
  Pose pose=Pose::Zero();

  void reference(int64_t next,double now) {
    if (clock_fault) return;
    if (!std::isfinite(now) || now<reference_received) {
      clock_fault=true;
      reference_ready=false;
      reference_error="odometry reference monotonic clock regressed/invalid";
      return;
    }
    if (next<=reference_stamp) {
      clock_fault=reference_stamp && next<reference_stamp;
      reference_ready=false;
      stable_since=never;
      reference_error="non-advancing odometry clock reference";
      return;
    }
    if (!anchor_stamp) { anchor_stamp=next; anchor_received=now; }
    if (now-reference_received>.25) { reference_ready=false; stable_since=never; }
    double skew=(next-anchor_stamp)/1e9-(now-anchor_received);
    reference_stamp=next;
    reference_received=now;
    if (skew>.25) {
      if (qualified) {
        clock_fault=true;
        reference_ready=false;
        reference_error="odometry clock jumped ahead of established reference; restart after resolving timing";
        return;
      }
      anchor_stamp=next;
      anchor_received=now;
      stable_since=never;
    } else if (skew<-.25) {
      reference_ready=false;
      stable_since=never;
      reference_error="odometry delivery delayed relative to established reference";
      return;
    }
    if (stable_since==never) stable_since=now;
    reference_ready=now-stable_since>=.2-1e-9;
    qualified=qualified || reference_ready;
    reference_error=reference_ready ? "" : "waiting for 0.2 s of stable advancing odometry reference";
  }
  std::string reference_problem(double now) const {
    if (!reference_error.empty()) return reference_error;
    if (!reference_ready) return "waiting for advancing odometry clock reference";
    if (!within(now-reference_received,0,.25)) return "stale odometry clock reference";
    return "";
  }
  void receive(const nav_msgs::msg::Odometry &msg,double now,int64_t wall_ns) {
    try {
      require(msg.header.frame_id=="map" && msg.child_frame_id=="astribot_viz_chassis",
              "expected map -> astribot_viz_chassis");
      auto next=stamp_ns(msg.header.stamp);
      const auto &p=msg.pose.pose.position;
      const auto &q=msg.pose.pose.orientation;
      Eigen::Quaterniond orientation(q.w,q.x,q.y,q.z);
      require(Pose(p.x,p.y,p.z).allFinite() && orientation.coeffs().allFinite(),"non-finite odometry pose");
      require(std::abs(orientation.norm()-1)<=.01,"invalid odometry quaternion");
      auto rotation=orientation.normalized().toRotationMatrix();
      reference(next,now);
      auto problem=reference_problem(now);
      require(problem.empty(),problem);
      pc_minus_source_ms=(wall_ns-next)/1e6;
      pose=Pose(p.x,p.y,std::atan2(rotation(1,0),rotation(0,0)));
      received=now;
      stamp=next;
      error.clear();
    } catch (const std::invalid_argument &e) { error=e.what(); }
  }
  Pose ready(double now) const {
    auto problem=reference_problem(now);
    if (problem.empty()) problem=error;
    require(problem.empty() && within(now-received,0,.25),
            "Missing/invalid/stale FAST-LIO navigation pose: "+
            (problem.empty() ? "requires fresh chassis odometry" : problem));
    return pose;
  }
};
struct CollisionCloud {
  static constexpr double max_age_s=2.;
  static constexpr int64_t max_age_ns=2000000000LL;
  std::string side;
  Mount mount;
  Navigation &clock;
  int64_t stamp=0;
  double received=never;
  bool fault=false;
  std::string error;
  Path points;
  CollisionCloud(std::string name,Mount transform,Navigation &reference):
    side(name),mount(transform),clock(reference),error("waiting for "+name+" cloud") {}
  void receive(const sensor_msgs::msg::PointCloud2 &msg,double now) {
    try {
      require(!fault,"cloud clock fault latched; restart after checking timing");
      auto next=stamp_ns(msg.header.stamp);
      if (next<=stamp) { fault=true; throw std::invalid_argument("cloud timestamp did not advance"); }
      require(msg.header.frame_id=="livox_frame","unqualified cloud frame; expected livox_frame");
      auto problem=clock.reference_problem(now);
      require(problem.empty(),problem);
      require(within(clock.reference_stamp-next,-1e8,max_age_ns),"cloud timestamp stale/future relative to odometry");
      points=obstacles(msg,mount);
      stamp=next;
      received=now;
      error.clear();
    } catch (const std::invalid_argument &e) { error=side+": "+e.what(); }
  }
  std::string problem(double now,int64_t odom_stamp) const {
    if (!error.empty()) return error;
    if (fault || !within(now-received,0,max_age_s)) return side+" cloud stale or clock fault";
    auto issue=clock.reference_problem(now);
    if (!issue.empty()) return side+": "+issue;
    if (!within(clock.reference_stamp-stamp,-1e8,max_age_ns)) return side+" cloud source time is stale/future";
    if (std::abs(odom_stamp-stamp)>max_age_ns) return side+" cloud and FAST-LIO pose are not time-aligned";
    return "";
  }
};
struct GoalSequence {
  Point goal=Point::Zero();
  double target_yaw=0.;
  std::string phase="idle";
  void begin(Point next,double yaw) {
    require(next.allFinite() && std::isfinite(yaw),"invalid goal position or yaw");
    goal=next;
    target_yaw=yaw;
    phase="rotate_to_goal";
  }
  void cancel() { phase="idle"; }
  std::pair<double,bool> update(const Pose &pose) {
    if (phase=="idle") return {0,false};
    require(pose.allFinite(),"invalid navigation pose");
    Point delta=goal-pose.head<2>();
    bool near=delta.norm()<=.03;
    if (phase=="rotate_to_yaw" && !near) { phase="rotate_to_goal"; return {0,false}; }
    if (near) phase="rotate_to_yaw";
    if (phase=="drive") return {0,false};
    double desired=near ? target_yaw : std::atan2(delta.y(),delta.x());
    double error=std::atan2(std::sin(desired-pose.z()),std::cos(desired-pose.z()));
    if (std::abs(error)<=.05) { phase=near ? "done" : "drive"; return {0,near}; }
    return {std::max(-.1,std::min(.1,error)),false};
  }
};
struct PathSession {
  bool active=false;
  Point goal=Point::Zero();
  Path path;
  int64_t epoch=0,last_stamp=0;
  double received=never;
  std::string reason="idle";
  void cancel(std::string why) { active=false; path.clear(); received=never; reason=why; }
  void begin(Point next,int64_t stamp) {
    cancel("waiting for a new path");
    active=true; goal=next; epoch=last_stamp=stamp;
  }
  bool accept(int64_t stamp,Path next,double now) {
    if (!active || stamp<=last_stamp ||
        std::any_of(next.begin(),next.end(),[](const Point &p){ return !p.allFinite(); })) return false;
    last_stamp=stamp;
    path=std::move(next);
    received=now;
    reason=path.size()>=2 ? "following" : "blocked: no local path";
    return true;
  }
  bool valid(double now) const { return active && within(now-received,0,.3); }
};
inline Point follow(const Point &goal,const Pose &pose,const Path &path) {
  if ((goal-pose.head<2>()).norm()<=.03 || path.size()<2) return Point::Zero();
  size_t nearest=0;
  double distance=std::numeric_limits<double>::infinity();
  for (size_t i=0;i<path.size();++i) {
    if (!path[i].allFinite()) return Point::Zero();
    double d=(path[i]-pose.head<2>()).norm();
    if (d<distance) { nearest=i; distance=d; }
  }
  auto target=std::find_if(path.begin()+nearest,path.end(),[&](const Point &p){
    return (p-pose.head<2>()).norm()>=.15;
  });
  Point local=relative_goal(target==path.end()?path.back():*target,pose);
  double angle=std::atan2(local.y(),local.x());
  return Point(std::abs(angle)<=.35 ? .05 : 0.,std::max(-.1,std::min(.1,angle)));
}
}  // namespace astribot
