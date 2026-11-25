// ROS-only command coordinator. XPolicy remains the sole SDK/hardware owner.
#include "local_planner/astribot_core.hpp"
#include "local_planner/msg/mpc_request.hpp"
#include "local_planner/msg/mpc_result.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <csignal>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <set>

namespace {
using Json=nlohmann::json;
using astribot::Point;using astribot::Pose;using astribot::Path;
using Request=local_planner::msg::MpcRequest;using Result=local_planner::msg::MpcResult;
const std::string PREFIX="/astribot/local_planner",FRAME="astribot_viz_chassis";
const std::string ODOM="/fast_lio/chassis/odometry",BASE="/astribot/base/status";
const std::array<std::string,2> CLOUD={{"/sdk/astribot_lidar/front_scan","/sdk/astribot_lidar/back_scan"}};
const std::array<std::string,2> SIDE={{"front","rear"}};
// 10 Hz cloud requests need scan jitter + processing + a 50 ms dispatch tick.
// Keep reply acceptance stricter than the lifetime of an accepted command.
constexpr double MPC_RESULT_TIMEOUT_S=.15,MPC_COMMAND_TIMEOUT_S=.25;
double steady(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
int64_t stamp(const builtin_interfaces::msg::Time& t){return int64_t(t.sec)*1000000000LL+t.nanosec;}
void check(bool ok,const std::string& why){if(!ok)throw std::invalid_argument(why);}
bool finite(std::initializer_list<double> values){for(double v:values)if(!std::isfinite(v))return false;return true;}
Json nullable(const std::string& s){return s.empty()?Json(nullptr):Json(s);}
bool jbool(const Json& s,const char* k,bool expected){return s.contains(k)&&s[k].is_boolean()&&s[k].get<bool>()==expected;}
bool jstring(const Json& s,const char* k,const char* expected){return s.contains(k)&&s[k].is_string()&&s[k].get<std::string>()==expected;}
volatile std::sig_atomic_t shutdown_requested=0;
void request_shutdown(int){shutdown_requested=1;}
}

class AstribotCoordinator : public rclcpp::Node {
 public:
  explicit AstribotCoordinator(const rclcpp::NodeOptions& options=rclcpp::NodeOptions(),
      std::function<double()> monotonic=steady)
      :Node("astribot_local_planner",options),monotonic_(std::move(monotonic)) {
    dispatch_=declare_parameter<bool>("dispatch",false);
    backend_=declare_parameter<std::string>("planner_backend","mpc");
    mode_=declare_parameter<std::string>("collision_lidar","front");
    check(backend_=="mpc"||backend_=="library","planner_backend must be mpc or library");
    check(mode_=="front"||mode_=="rear"||mode_=="auto","collision_lidar must be front, rear or auto");
    clouds_[0].reset(new astribot::CollisionCloud("front",astribot::load_mount(false),nav_));
    clouds_[1].reset(new astribot::CollisionCloud("rear",astribot::load_mount(true),nav_));
    output_=dispatch_?"/cmd_vel":PREFIX+"/cmd_vel_shadow";
    commands_=create_publisher<geometry_msgs::msg::Twist>(output_,1);
    scans_=create_publisher<sensor_msgs::msg::LaserScan>(PREFIX+"/scan",1);
    goals_=create_publisher<geometry_msgs::msg::PointStamped>(PREFIX+"/way_point",1);
    cancels_=create_publisher<std_msgs::msg::Empty>(PREFIX+"/cancel",1);
    paths_=create_publisher<nav_msgs::msg::Path>(PREFIX+"/path_map",1);
    statuses_=create_publisher<std_msgs::msg::String>(PREFIX+"/status",1);
    if(backend_=="mpc") {
      requests_=create_publisher<Request>(PREFIX+"/mpc_request",1);
      results_=create_subscription<Result>(PREFIX+"/mpc_result",1,[this](Result::ConstSharedPtr m){mpc_result(*m);});
    }else{
      library_paths_=create_subscription<nav_msgs::msg::Path>(PREFIX+"/path",1,
        [this](nav_msgs::msg::Path::ConstSharedPtr m){library_path(*m);});
    }
    for(int i=0;i<2;++i){
      if(mode_!="auto"&&mode_!=SIDE[i])continue;
      inputs_[i]=create_subscription<sensor_msgs::msg::PointCloud2>(CLOUD[i],rclcpp::SensorDataQoS(),
        [this,i](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){cloud(*m,i);});
    }
    odometry_=create_subscription<nav_msgs::msg::Odometry>(ODOM,rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m){nav_.receive(*m,monotonic_(),now().nanoseconds());});
    goal_sub_=create_subscription<geometry_msgs::msg::PoseStamped>("/astribot_viz/goal_pose",1,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m){goal(*m);});
    cancel_sub_=create_subscription<std_msgs::msg::Empty>("/astribot_viz/goal_cancel",1,
      [this](std_msgs::msg::Empty::ConstSharedPtr){cancel("operator cancelled");});
    owner_sub_=create_subscription<std_msgs::msg::String>(BASE,1,
      [this](std_msgs::msg::String::ConstSharedPtr m){
        try{auto s=Json::parse(m->data);owner_=s.is_object()?s:Json::object();}
        catch(const Json::exception&){owner_=Json::object();}owner_received_=monotonic_();});
    reset_mpc();last_command_time_=monotonic_();
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{tick();});
    RCLCPP_INFO(get_logger(),"C++ coordinator backend=%s, collision=%s, output=%s; no automatic enable",
                backend_.c_str(),mode_.c_str(),output_.c_str());
  }
  void cancel(const std::string& reason) {
    reset_mpc();sequence_.cancel();session_.cancel(reason);snapshots_.clear();
    pinned_=false;owner_generation_=Json();selected_=-1;
    commands_->publish(geometry_msgs::msg::Twist());last_translation_.setZero();last_command_time_=monotonic_();
    cancels_->publish(std_msgs::msg::Empty());clear_path();
  }
  bool wait_for_stop_ack() {
    return commands_->wait_for_all_acked(std::chrono::milliseconds(100));
  }
 private:
  friend struct CoordinatorTestAccess;
  std::function<double()> monotonic_;
  struct Snapshot {Pose pose;double received;uint64_t epoch;Point previous;};
  using Identity=std::array<uint8_t,RMW_GID_STORAGE_SIZE>;
  std::vector<Identity> sources() {
    check(selected_>=0,"collision LiDAR has not been selected");
    std::vector<std::string> topics={CLOUD[selected_],ODOM,output_};
    if(backend_=="mpc"){topics.push_back(PREFIX+"/mpc_request");topics.push_back(PREFIX+"/mpc_result");}
    else topics.push_back(PREFIX+"/path");
    if(dispatch_)topics.push_back(BASE);
    std::set<std::string> seen;std::vector<Identity> ids;
    for(const auto& topic:topics)if(seen.insert(topic).second){
      auto endpoints=get_publishers_info_by_topic(topic);
      check(endpoints.size()==1,"expected exactly one publisher: "+topic);
      ids.push_back(endpoints[0].endpoint_gid());
    }
    return ids;
  }
  Pose ready(double t) {
    const Pose pose=nav_.ready(t);check(selected_>=0,"collision LiDAR has not been selected");
    const auto problem=clouds_[selected_]->problem(t,nav_.stamp);check(problem.empty(),problem);
    auto ids=sources();check(!pinned_||ids==source_ids_,"publisher identity changed during goal");
    if(dispatch_) validate_owner(t);
    return pose;
  }
  void validate_owner(double t) const {
    bool valid=t-owner_received_>=0. && t-owner_received_<=.5 && jbool(owner_,"armed",true)
      &&jstring(owner_,"mode","MODEL")&&jbool(owner_,"hardware_writes",true)
      &&jstring(owner_,"feedback_source","hardware")&&jbool(owner_,"stop_pending",false)
      &&jstring(owner_,"selected_source","auto")&&jstring(owner_,"selected_topic","/cmd_vel")
      &&owner_.contains("feedback_age_ms")&&owner_["feedback_age_ms"].is_number()
      &&owner_.contains("generation")&&owner_["generation"].is_number_integer();
    if(valid){const double age=owner_["feedback_age_ms"].get<double>();valid=std::isfinite(age)&&age>=0.&&age<=50.;}
    check(valid,"automatic base owner must be armed with fresh hardware feedback");
    check(owner_generation_.is_null()||owner_generation_==owner_["generation"],"base owner generation changed");
  }
  void goal(const geometry_msgs::msg::PoseStamped& m) {
    cancel("new goal requested");
    try{
      const auto& p=m.pose.position;const auto& q=m.pose.orientation;
      const double age=double(now().nanoseconds()-stamp(m.header.stamp))/1e9;
      check(m.header.frame_id=="map"&&finite({p.x,p.y,p.z,q.x,q.y,q.z,q.w})&&std::abs(p.z)<=.001
        &&std::abs(q.x)<=.001&&std::abs(q.y)<=.001&&std::abs(std::hypot(q.z,q.w)-1.)<=.001
        &&age>=-.05&&age<=2.,"expected a fresh finite planar RViz goal in map");
      const double t=monotonic_();nav_.ready(t);std::string errors;
      for(int i=0;i<2;++i){
        if(mode_!="auto"&&mode_!=SIDE[i])continue;
        auto problem=clouds_[i]->problem(t,nav_.stamp);
        if(!problem.empty()){errors+=(errors.empty()?"":"; ")+problem;continue;}
        for(const auto& topic:{CLOUD[i]})
          check(get_publishers_info_by_topic(topic).size()==1,"expected exactly one publisher: "+topic);
        selected_=i;break;
      }
      check(selected_>=0,"No usable collision LiDAR: "+errors);use_cloud();const Pose pose=ready(t);
      const Point destination(p.x,p.y);check(astribot::relative_goal(destination,pose).norm()<=3.,"local planner goals must be within 3 m");
      source_ids_=sources();pinned_=true;owner_generation_=dispatch_?owner_["generation"]:Json();
      session_.begin(destination,cloud_stamp_);sequence_.begin(destination,2.*std::atan2(q.z,q.w));started_=t;
      RCLCPP_INFO(get_logger(),"Goal accepted: turn, drive, align yaw; collision_lidar=%s",SIDE[selected_].c_str());
    }catch(const std::exception& e){cancel(e.what());RCLCPP_WARN(get_logger(),"%s",e.what());}
  }
  void use_cloud(){points_=clouds_[selected_]->points;cloud_stamp_=clouds_[selected_]->stamp;}
  void cloud(const sensor_msgs::msg::PointCloud2& m,int side) {
    const double t=monotonic_();clouds_[side]->receive(m,t);if(selected_!=side)return;
    try{
      const auto problem=clouds_[side]->problem(t,nav_.stamp);check(problem.empty(),problem);use_cloud();
      if(!session_.active||sequence_.phase!="drive")return;
      const Pose pose=ready(t);const Point local=astribot::relative_goal(session_.goal,pose);
      Snapshot snap{pose,t,mpc_epoch_,last_translation_};
      if(backend_=="mpc"){
        Request request;request.header=m.header;request.header.frame_id=FRAME;request.sent_at=now();request.epoch=mpc_epoch_;
        request.goal.x=local.x();request.goal.y=local.y();request.previous_velocity.x=last_translation_.x();request.previous_velocity.y=last_translation_.y();
        request.obstacles.reserve(points_.size());
        for(const auto& xy:points_){geometry_msgs::msg::Point p;p.x=xy.x();p.y=xy.y();request.obstacles.push_back(p);}
        requests_pending_[cloud_stamp_]=snap;while(requests_pending_.size()>5)requests_pending_.erase(requests_pending_.begin());
        requests_->publish(request);return;
      }
      snapshots_[cloud_stamp_]=snap;while(snapshots_.size()>5)snapshots_.erase(snapshots_.begin());
      geometry_msgs::msg::PointStamped target;target.header=m.header;target.header.frame_id=FRAME;
      target.point.x=local.x();target.point.y=local.y();goals_->publish(target);
      sensor_msgs::msg::LaserScan scan;scan.header=target.header;scan.angle_min=-M_PI;scan.angle_increment=M_PI/720;
      scan.angle_max=scan.angle_min+1439*scan.angle_increment;scan.range_min=.01;scan.range_max=4.;
      scan.ranges.assign(1440,std::numeric_limits<float>::infinity());
      for(const auto& xy:points_){
        int index=int(std::floor((std::atan2(xy.y(),xy.x())+M_PI)/(M_PI/720)))%1440;
        index=(index+1440)%1440;scan.ranges[index]=std::min(scan.ranges[index],static_cast<float>(xy.norm()));
      }
      scans_->publish(scan);
    }catch(const std::exception& e){if(session_.active)cancel(e.what());}
  }
  void clear_path(){nav_msgs::msg::Path p;p.header.frame_id="map";paths_->publish(p);}
  void library_path(const nav_msgs::msg::Path& m) {
    const int64_t s=stamp(m.header.stamp);auto it=snapshots_.find(s);
    if(it==snapshots_.end())return;
    const auto snap=it->second;snapshots_.erase(it);
    if(m.header.frame_id!=FRAME||!session_.active||sequence_.phase!="drive")return;
    if(monotonic_()-snap.received>.25){cancel("planner result expired");return;}
    Path mapped;bool valid=m.poses.size()<=2000;
    for(const auto& item:m.poses){Point local(item.pose.position.x,item.pose.position.y);
      valid=valid&&local.allFinite()&&local.norm()<=4.;mapped.push_back(astribot::world_point(local,snap.pose));}
    if(!valid){cancel("invalid planner path");return;}
    if(session_.accept(s,mapped,snap.received)){
      auto display=m;display.header.frame_id="map";
      for(size_t i=0;i<mapped.size();++i){auto& p=display.poses[i];p.header=display.header;p.pose.position.x=mapped[i].x();p.pose.position.y=mapped[i].y();p.pose.orientation.w=1.;}
      paths_->publish(display);
    }
  }
  void reset_mpc(){++mpc_epoch_;requests_pending_.clear();mpc_last_stamp_=0;mpc_velocity_.setZero();
    mpc_received_=-INFINITY;solve_ms_=Json();mpc_path_.clear();mpc_cloud_stamp_=-1;}
  void mpc_result(const Result& r) {
    const int64_t s=stamp(r.header.stamp);auto it=requests_pending_.find(s);if(it==requests_pending_.end())return;
    const auto snap=it->second;requests_pending_.erase(it);
    if(!session_.active||sequence_.phase!="drive"||r.epoch!=mpc_epoch_||s<=mpc_last_stamp_)return;
    try{
      const double t=monotonic_();ready(t);check(snap.epoch==r.epoch&&t-snap.received>=0.&&t-snap.received<=MPC_RESULT_TIMEOUT_S,"C++ MPC result expired or goal epoch changed");
      check(r.success,"C++ MPC failed: "+r.error);
      const auto& v=r.command.linear;const auto& w=r.command.angular;const Point velocity(v.x,v.y);
      check(r.header.frame_id==FRAME&&r.path.header==r.header&&finite({v.x,v.y,v.z,w.x,w.y,w.z,r.solve_ms})
        &&v.z==0.&&w.x==0.&&w.y==0.&&w.z==0.&&std::abs(v.x)+std::abs(v.y)<=.100001
        &&r.solve_ms>=0.&&r.solve_ms<=40.&&r.path.poses.size()==(r.blocked?1u:21u)
        &&(!r.blocked||velocity.isZero(0.)),"invalid C++ MPC result");
      Path local;for(const auto& item:r.path.poses){const auto& p=item.pose.position;const auto& q=item.pose.orientation;
        Point xy(p.x,p.y);check(xy.allFinite()&&xy.norm()<=.201,"invalid C++ MPC result");
        check(item.header==r.header&&p.z==0.&&q.x==0.&&q.y==0.&&q.z==0.&&q.w==1.,"invalid C++ MPC prediction pose");local.push_back(xy);}
      check(local.front().norm()<=1e-6,"invalid C++ MPC result");
      if(!r.blocked){Point previous=snap.previous;for(size_t i=1;i<local.size();++i){Point u=(local[i]-local[i-1])/.1;
        check((i!=1||(u-velocity).lpNorm<Eigen::Infinity>()<=1e-6)&&u.lpNorm<1>()<=.100001
          &&(u-previous).lpNorm<Eigen::Infinity>()<=.014001,"C++ MPC command and prediction disagree");previous=u;}}
      mpc_velocity_=velocity;mpc_received_=snap.received;solve_ms_=r.solve_ms;mpc_last_stamp_=s;mpc_cloud_stamp_=-1;
      mpc_path_.clear();for(const auto& xy:local)mpc_path_.push_back(astribot::world_point(xy,snap.pose));
      session_.reason=r.blocked?"blocked: MPC clearance":"C++ MPC tracking";
      auto display=r.path;display.header.frame_id="map";for(size_t i=0;i<mpc_path_.size();++i){
        auto& p=display.poses[i];p.header=display.header;p.pose.position.x=mpc_path_[i].x();p.pose.position.y=mpc_path_[i].y();}
      paths_->publish(display);
    }catch(const std::exception& e){cancel(e.what());}
  }
  Point mpc_drive(const Pose& pose,double t) {
    if(mpc_received_==-INFINITY){check(t-started_<=.5,"C++ MPC result missing");return Point::Zero();}
    check(t-mpc_received_>=0.&&t-mpc_received_<=MPC_COMMAND_TIMEOUT_S,"C++ MPC command expired");
    if(mpc_cloud_stamp_!=cloud_stamp_){
      Path local{Point::Zero()};for(const auto& xy:mpc_path_)local.push_back(astribot::relative_goal(xy,pose));
      for(const auto& center:local)for(const auto& p:points_)if((p-center).norm()<.55-1e-5){
        reset_mpc();session_.reason="blocked: new cloud invalidated MPC prediction";return Point::Zero();}
      mpc_cloud_stamp_=cloud_stamp_;
    }
    return mpc_velocity_;
  }
  Point bounded(Point v,double t){if(v.isZero(0.))return v;
    const double step=.14*std::max(0.,std::min(.1,t-last_command_time_));
    for(int i=0;i<2;++i)v[i]=std::max(last_translation_[i]-step,std::min(last_translation_[i]+step,v[i]));
    return v;
  }
  void tick() {
    const double t=monotonic_();Point v=Point::Zero();double w=0.;
    if(session_.active)try{
      const Pose pose=ready(t);const auto previous=sequence_.phase;const auto result=sequence_.update(pose);w=result.first;
      if(previous!=sequence_.phase){reset_mpc();snapshots_.clear();session_.begin(session_.goal,cloud_stamp_);started_=t;
        cancels_->publish(std_msgs::msg::Empty());clear_path();}
      if(result.second)cancel("goal pose reached within 0.03 m and 0.05 rad");
      else if(sequence_.phase=="drive"){
        if(backend_=="mpc"){
          v=bounded(mpc_drive(pose,t),t);
          if(!astribot::collision_free(v.x(),v.y(),0.,points_,.55,.2)){
            v.setZero();reset_mpc();session_.reason="blocked: dispatched MPC clearance";}
        }else if(!session_.valid(t)){check(t-started_<=.5,"local path stale/missing");}
        else{const Point cmd=astribot::follow(session_.goal,pose,session_.path);v.x()=cmd.x();w=cmd.y();}
      }else session_.reason=sequence_.phase;
      if(session_.active&&!astribot::collision_free(v.x(),v.y(),w,points_)){
        v.setZero();w=0.;reset_mpc();session_.reason="blocked: "+SIDE[selected_]+" collision envelope";}
    }catch(const std::exception& e){cancel(e.what());v.setZero();w=0.;}
    // Cloud processing can consume the budget checked at tick entry. Never
    // refresh an old proposal merely because collision validation just finished.
    if(!v.isZero(0.)||w!=0.)try{
      const double send_time=monotonic_();ready(send_time);
      check(!shutdown_requested,"planner shutting down");
      if(sequence_.phase=="drive"&&backend_=="mpc")
        check(send_time-mpc_received_>=0.&&send_time-mpc_received_<=MPC_COMMAND_TIMEOUT_S,"C++ MPC command expired before dispatch");
      else if(sequence_.phase=="drive")check(session_.valid(send_time),"local path expired before dispatch");
    }catch(const std::exception& e){cancel(e.what());v.setZero();w=0.;}
    geometry_msgs::msg::Twist command;command.linear.x=v.x();command.linear.y=v.y();command.angular.z=w;
    commands_->publish(command);last_translation_=v;last_command_time_=monotonic_();
    Json status={{"active",session_.active},{"phase",sequence_.phase},{"reason",session_.reason},{"dispatch",dispatch_},
      {"output_topic",output_},{"planner_backend",backend_},{"mpc_solve_ms",solve_ms_},
      {"collision_lidar",selected_<0?Json():Json(SIDE[selected_])},{"collision_topic",selected_<0?Json():Json(CLOUD[selected_])},
      {"collision_input_errors",{{"front",nullable(clouds_[0]->problem(t,nav_.stamp))},{"rear",nullable(clouds_[1]->problem(t,nav_.stamp))}}},
      {"command",{v.x(),v.y(),w}},{"owner_stop_confirmed",owner_.contains("stop_confirmed")?owner_["stop_confirmed"]:Json()}};
    std_msgs::msg::String message;message.data=status.dump();statuses_->publish(message);
  }
  bool dispatch_=false,pinned_=false;std::string backend_,mode_,output_;int selected_=-1;
  astribot::Navigation nav_;std::array<std::unique_ptr<astribot::CollisionCloud>,2> clouds_;
  astribot::GoalSequence sequence_;astribot::PathSession session_;Path points_,mpc_path_;
  Point last_translation_=Point::Zero(),mpc_velocity_=Point::Zero();
  double last_command_time_=0.,started_=0.,owner_received_=-INFINITY,mpc_received_=-INFINITY;
  int64_t cloud_stamp_=0,mpc_last_stamp_=0,mpc_cloud_stamp_=-1;uint64_t mpc_epoch_=0;
  Json owner_=Json::object(),owner_generation_,solve_ms_;std::vector<Identity> source_ids_;
  std::map<int64_t,Snapshot> snapshots_,requests_pending_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commands_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scans_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goals_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr cancels_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr paths_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statuses_;
  rclcpp::Publisher<Request>::SharedPtr requests_;
  rclcpp::Subscription<Result>::SharedPtr results_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr library_paths_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_;
  std::array<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr,2> inputs_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr owner_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#ifndef ASTRIBOT_COORDINATOR_NO_MAIN
int main(int argc,char** argv){
  try{
    if(argc==2&&std::string(argv[1])=="--default-backend"){
      auto config=YAML::LoadFile(ament_index_cpp::get_package_share_directory("local_planner")+"/config/astribot.yaml");
      const auto backend=config["/**"]["ros__parameters"]["planner_backend"].as<std::string>();
      check(backend=="mpc"||backend=="library","planner_backend must be mpc or library");
      std::cout<<backend;return 0;
    }
    if(argc==2&&std::string(argv[1])=="--check"){
      astribot::load_mount(false);astribot::load_mount(true);
      std::cout<<"PASS: native coordinator geometry and dependencies; no ROS nodes started"<<std::endl;return 0;
    }
    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT,request_shutdown);std::signal(SIGTERM,request_shutdown);
    auto node=std::make_shared<AstribotCoordinator>();rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while(rclcpp::ok()&&!shutdown_requested)executor.spin_once(std::chrono::milliseconds(20));
    if(rclcpp::ok()){
      node->cancel("planner shutting down");
      if(!node->wait_for_stop_ack()) RCLCPP_WARN(node->get_logger(),"Final stop acknowledgement timed out");
    }
    executor.remove_node(node);node.reset();if(rclcpp::ok())rclcpp::shutdown();return 0;
  }catch(const std::exception& e){std::cerr<<"Coordinator error: "<<e.what()<<std::endl;if(rclcpp::ok())rclcpp::shutdown();return 1;}
}
#endif
