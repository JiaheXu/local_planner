#include "local_planner/astribot_core.hpp"
#include <iostream>

using namespace astribot;
namespace {
void check(bool result,const char *reason) { if (!result) throw std::runtime_error(reason); }
template<class F> void rejects(F f,const char *reason) {
  try { f(); } catch (const std::invalid_argument &) { return; }
  throw std::runtime_error(reason);
}
nav_msgs::msg::Odometry odometry(int64_t stamp) {
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id="map";
  odom.child_frame_id="astribot_viz_chassis";
  odom.header.stamp.sec=stamp/1000000000;
  odom.header.stamp.nanosec=stamp%1000000000;
  odom.pose.pose.orientation.w=1.;
  return odom;
}
void sample(Navigation &nav,int source_ms,double receipt_ms) {
  const int64_t stamp=1000000000LL+source_ms*1000000LL;
  nav.receive(odometry(stamp),receipt_ms/1000.,stamp-37000000000LL);
}
Navigation qualified() {
  Navigation nav;
  for (int ms=0;ms<200;ms+=50) {
    sample(nav,ms,ms);
    rejects([&]{nav.ready(ms/1000.);},"premature odometry qualification");
  }
  sample(nav,200,200);
  check(nav.ready(.2).isZero() && nav.pc_minus_source_ms==-37000.,"odometry-only qualification");
  return nav;
}
void timing() {
  auto nav=qualified();
  check(nav.ready(.45).isZero(),"250ms receipt boundary");
  rejects([&]{nav.ready(.450001);},"expired odometry accepted");
  sample(nav,200,210);
  check(!nav.reference_error.empty() && nav.reference_received==.2 && !nav.clock_fault,"duplicate odometry");
  for (int ms=220;ms<420;ms+=10) {
    sample(nav,ms,ms);
    check(!nav.reference_ready,"premature duplicate recovery");
  }
  sample(nav,420,420);
  check(nav.reference_ready,"duplicate recovery");
  for (int ms=710;ms<=1210;ms+=50) {
    sample(nav,ms-280,ms);
    check(!nav.reference_ready && !nav.clock_fault,"delay rebased into freshness");
  }
  for (int ms=1260;ms<=1460;ms+=50) sample(nav,ms,ms);
  check(nav.reference_ready && !nav.clock_fault,"catchup recovery");
  sample(nav,1800,1470);
  sample(nav,1810,1480);
  check(nav.clock_fault && !nav.reference_ready,"jump not latched");
  nav=qualified();
  sample(nav,190,210);
  check(nav.clock_fault,"source regression");
  nav=qualified();
  sample(nav,210,190);
  check(nav.clock_fault,"receipt regression");
  nav=qualified();
  auto msg=odometry(1210000000);
  msg.pose.pose.orientation.w=2.;
  nav.receive(msg,.21,0);
  rejects([&]{nav.ready(.21);},"invalid odometry accepted");
  check(nav.stamp==1200000000 && nav.reference_stamp==nav.stamp,"invalid odometry advanced reference");
}
sensor_msgs::msg::PointCloud2 make_cloud(bool big) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id="livox_frame";
  cloud.header.stamp.nanosec=700000000;
  cloud.width=1; cloud.height=2; cloud.row_step=24; cloud.point_step=16;
  cloud.is_bigendian=big;
  cloud.data.resize(48,255);
  for (int col=0;col<3;++col) {
    sensor_msgs::msg::PointField field;
    field.name=std::string(1,"xyz"[col]); field.count=1; field.datatype=7; field.offset=4+4*col;
    cloud.fields.push_back(field);
    for (int row=0;row<2;++row) {
      float value=col==0 ? 1.f+row : (col==1 ? 0.f : .1f);
      uint32_t bits;
      std::memcpy(&bits,&value,4);
      for (int b=0;b<4;++b)
        cloud.data[row*24+field.offset+b]=bits>>(8*(big?3-b:b));
    }
  }
  return cloud;
}
void cloud_contract() {
  for (bool big : {true,false}) {
    auto cloud=make_cloud(big);
    auto points=obstacles(cloud,Mount::Identity());
    check(points.size()==2 && points[0]==Point(1,0) && points[1]==Point(2,0),"cloud stride/endian");
    Navigation nav;
    for(int ms=0;ms<=200;ms+=50)
      nav.receive(odometry(2500000000LL+ms*1000000LL),ms/1000.,0);
    CollisionCloud sample("rear",Mount::Identity(),nav);
    sample.receive(cloud,.2);
    check(sample.problem(.2,2700000000LL).empty(),"2s inclusive");
    check(!sample.problem(.2,2700000001LL).empty(),"pose/cloud separation");
    nav.receive(odometry(2700000001LL),.200000001,0);
    check(!sample.problem(.200000001,2700000000LL).empty(),"expired cached source");
    sample.receive(cloud,.21);
    check(sample.fault,"nonadvancing cloud must latch");
    cloud.fields[0].offset=14;
    rejects([&]{obstacles(cloud,Mount::Identity());},"invalid schema accepted");
    cloud=make_cloud(big);
    std::fill(cloud.data.begin(),cloud.data.end(),0);
    rejects([&]{obstacles(cloud,Mount::Identity());},"zero cloud treated as clear");
    cloud.data.pop_back();
    rejects([&]{obstacles(cloud,Mount::Identity());},"truncated cloud accepted");
  }
  auto front=load_mount(false),rear=load_mount(true);
  check(front.matrix().isApprox(rear.matrix()),"rear cloud double transform");
  check(front.translation().isApprox(Pose(.17393,.16893,.170397)),"mount translation");
  check((front.linear()*Pose::UnitX()).isApprox(Pose(std::sqrt(.5),-std::sqrt(.5),0),1e-6),"mount yaw");
}
void motion() {
  GoalSequence seq;
  seq.begin(Point(0,1),0.);
  check(seq.update(Pose::Zero()).first==.1 && seq.phase=="rotate_to_goal","initial turn");
  seq.update(Pose(0,0,M_PI_2));
  check(seq.phase=="drive","drive transition");
  check(seq.update(Pose(0,1,M_PI_2)).first==-.1 && seq.phase=="rotate_to_yaw","final turn");
  seq.update(Pose(0,.9,0));
  check(seq.phase=="rotate_to_goal","position drift");
  check(seq.update(Pose(0,.98,.04)).second,"arrival tolerance");
  seq.cancel();
  check(seq.phase=="idle","cancel");
  rejects([&]{seq.begin(Point::Zero(),NAN);},"nonfinite goal");
  PathSession session;
  session.begin(Point(1,0),10);
  check(!session.accept(10,{},0),"old epoch");
  check(session.accept(12,{Point(1,0),Point(2,0)},0),"current result");
  check(!session.accept(11,{},.1) && !session.accept(13,{Point(NAN,0)},.2),"invalid result");
  check(session.valid(.3) && !session.valid(.301),"request age");
  session.cancel("operator cancelled");
  check(!session.accept(14,{},.2),"cancelled result");
  Pose pose(2,2,M_PI_2);
  auto xy=world_point(Point(1,0),pose);
  check(relative_goal(xy,pose).isApprox(Point(1,0)),"world path anchor");
  check(follow(xy,pose,{pose.head<2>(),xy}).isApprox(Point(.05,0),1e-8),"follow transformed path");
  check(!collision_free(.05,0,0,{Point(.49,0)}) && collision_free(.05,0,0,{Point(.51,0)}),"sweep");
  check(!collision_free(0,0,.1,{Point(.4,0)}) && !collision_free(0,NAN,0,{}),"rotation envelope");
}
}
int main() {
  try { timing(); cloud_contract(); motion(); }
  catch (const std::exception &e) { std::cerr<<e.what()<<'\n'; return 1; }
  std::cout<<"Astribot timing, cloud, geometry, and motion contracts passed\n";
  return 0;
}
