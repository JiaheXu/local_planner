// Synthetic observations only: domain 227, localhost, private shadow commands.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

using Json=nlohmann::json;
const std::string prefix="/astribot/local_planner";
double steady() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
void require(bool ok,const std::string& text) { if(!ok) throw std::runtime_error(text); }

class Smoke : public rclcpp::Node {
 public:
  explicit Smoke(std::string backend,std::string scenario):Node("synthetic_local_planner_inputs"),backend_(backend),scenario_(scenario) {
    odom_=create_publisher<nav_msgs::msg::Odometry>("/fast_lio/chassis/odometry",rclcpp::SensorDataQoS());
    cloud_=create_publisher<sensor_msgs::msg::PointCloud2>("/sdk/astribot_lidar/back_scan",rclcpp::SensorDataQoS());
    goal_=create_publisher<geometry_msgs::msg::PoseStamped>("/astribot_viz/goal_pose",1);
    cancel_=create_publisher<std_msgs::msg::Empty>("/astribot_viz/goal_cancel",1);
    commands_=create_subscription<geometry_msgs::msg::Twist>(prefix+"/cmd_vel_shadow",1,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr m){last_=*m; history_.push_back(*m);});
    status_=create_subscription<std_msgs::msg::String>(prefix+"/status",1,
      [this](std_msgs::msg::String::ConstSharedPtr m){state_=Json::parse(m->data);});
  }
  void run() {
    drive(3.);
    require(count_publishers("/cmd_vel")==0,"test must never publish real cmd_vel");
    require(!history_.empty() && stopped(),"startup must be stopped");
    require(count_publishers("/sdk/astribot_lidar/front_scan")==0,"rear test must not require front");
    require(count_subscribers("/sdk/astribot_lidar/front_scan")==0,"rear-only must not subscribe to front clouds");
    require(count_subscribers("/sdk/astribot_lidar/front_imu")==0 && count_subscribers("/sdk/astribot_lidar/back_imu")==0,
            "planner must not subscribe to LiDAR IMU");
    goal(); drive(2.,[this]{return forward();});
    require(forward(),"clear goal: "+reason());
    require(state_.value("collision_lidar",std::string())=="rear","rear source not selected");
    if(scenario_=="shutdown") {
      const char* process=std::getenv("LOCAL_PLANNER_TEST_COORDINATOR_PID");
      require(process!=nullptr,"missing synthetic coordinator PID");
      const int pid=std::stoi(process);require(pid>1,"invalid coordinator PID");
      history_.clear();require(::kill(pid,SIGTERM)==0,"could not stop synthetic coordinator");
      drive(.4);require(!history_.empty()&&stopped(),"shutdown must explicitly deliver zero after active motion");
      pass("SIGTERM sends final zero while publisher context is valid");return;
    }
    cancel(); require(!active() && stopped(),"cancel must clear target and velocity");
    pass("clear goal drives; cancel stops; rear only");

    obstacle_=true; drive(.4); goal(); drive(1.);
    require(!forward() && stopped(),"obstacle must prevent translation");
    cancel(); obstacle_=false; drive(.4);
    pass("obstacle blocks drive");

    goal(); drive(2.,[this]{return forward();}); require(forward(),reason());
    send_cloud_=false; drive(.8);
    require(!active() && stopped(),"cloud loss must cancel");
    send_cloud_=true; drive(.6);
    require(!active() && stopped(),"fresh cloud must not resume cancelled goal");
    pass("cloud expiry cancels and never resumes");

    goal(); drive(2.,[this]{return forward();}); require(forward(),reason());
    send_odom_=false; drive(.6);
    require(!active() && stopped(),"odometry loss must cancel");
    send_odom_=true; drive(.6);
    pass("odometry expiry cancels");

    const double start=x_;
    goal(); move_=true; drive(9.,[this]{return reached();}); move_=false;
    require(reached() && x_-start>=.17 && x_-start<=.205,"0.2m arrival: "+reason());
    drive(.2); require(stopped(),"arrival must zero output");
    pass("0.2m target reaches within 0.03m");

    goal(); drive(2.,[this]{return forward();}); require(forward(),reason());
    invalid_=true; drive(.4);
    require(!active() && stopped(),"zero-range cloud must cancel");
    invalid_=false; yaw_=-.2; drive(.6);
    pass("invalid cloud cancels");

    goal(.2,.2); drive(.8);
    require(phase()=="rotate_to_goal" && last_.angular.z>0. && last_.linear.x==0. && last_.linear.y==0.,"initial turn");
    move_=true; drive(13.,[this]{return reached();}); move_=false;
    require(reached() && std::abs(yaw_-.2)<=.05,"turn drive turn: "+reason());
    drive(.2); require(stopped(),"final rotation arrival must stop");
    pass("turn-drive-turn reaches final yaw with zero translation during turns");

    goal(0.,yaw_+.3); drive(.4);
    require(phase()=="rotate_to_yaw","final rotation requested");
    cancel(); require(!active() && stopped(),"final rotation cancellation");
    pass("cancel during final rotation");

    if(backend_=="mpc") {
      yaw_=0.; drive(.4); goal(); drive(1.,[this]{return phase()=="drive";});
      y_+=.04; history_.clear(); drive(.7);
      bool lateral=false;for(const auto& v:history_) lateral|=v.linear.y<-.001;
      require(lateral,"MPC must correct lateral drift: "+reason());
      move_=true; drive(9.,[this]{return reached();});move_=false;
      require(reached(),"MPC drift arrival: "+reason()); cancel();
      require(count_publishers(prefix+"/path")==0,"MPC must not use lattice output");
      pass("MPC lateral correction");
    }

    yaw_=0.;drive(.4);goal();drive(2.,[this]{return forward();});require(forward(),reason());
    const char* process=std::getenv("LOCAL_PLANNER_TEST_BACKEND_PID");
    require(process!=nullptr,"missing synthetic backend PID");
    const int pid=std::stoi(process);require(pid>1,"invalid backend PID");
    require(::kill(pid,SIGTERM)==0,"could not stop synthetic backend");
    drive(.9);require(!active() && stopped(),"backend loss must cancel");
    require(count_publishers("/cmd_vel")==0,"real cmd_vel appeared");
    pass("backend loss stops; no hardware command publisher");
  }
 private:
  void drive(double seconds,std::function<bool()> until=[] {return false;}) {
    const double end=steady()+seconds;
    while(rclcpp::ok() && steady()<end) {
      const double t=steady(); const int64_t ns=now().nanoseconds()+37000000000LL;
      if(t-last_cloud_>=.1) {
        if(send_odom_) {
          if(move_) {
            x_+=.1*(last_.linear.x*std::cos(yaw_)-last_.linear.y*std::sin(yaw_));
            y_+=.1*(last_.linear.x*std::sin(yaw_)+last_.linear.y*std::cos(yaw_));
            yaw_+=.1*last_.angular.z;
            // Status can lag command by one callback; nonzero yaw must never translate.
            require(last_.angular.z==0. || (last_.linear.x==0. && last_.linear.y==0.) || backend_=="library",
                    "rotation has translation");
          }
          nav_msgs::msg::Odometry m;m.header.frame_id="map";m.child_frame_id="astribot_viz_chassis";
          m.header.stamp=rclcpp::Time(ns-30000000);m.pose.pose.position.x=x_;m.pose.pose.position.y=y_;
          m.pose.pose.orientation.z=std::sin(yaw_/2);m.pose.pose.orientation.w=std::cos(yaw_/2);odom_->publish(m);
        }
        if(send_cloud_) publish_cloud(ns-100000000);
        last_cloud_=t;
      }
      rclcpp::spin_some(shared_from_this());
      if(until())return;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  void publish_cloud(int64_t ns) {
    std::vector<Eigen::Vector3d> pts={{2.,0.,0.},{2.,1.,0.},{2.,-1.,0.}};
    if(obstacle_)for(double y:{-.1,0.,.1})pts.emplace_back(.45,y,.2);
    sensor_msgs::msg::PointCloud2 m;m.header.frame_id="livox_frame";m.header.stamp=rclcpp::Time(ns);
    m.height=1;m.width=pts.size();m.point_step=12;m.row_step=m.width*12;m.data.resize(m.row_step);
    for(size_t i=0;i<3;++i){sensor_msgs::msg::PointField f;f.name=std::string(1,"xyz"[i]);f.offset=i*4;f.datatype=7;f.count=1;m.fields.push_back(f);}
    const Eigen::Quaterniond q=Eigen::Quaterniond(.9238796,0.,0.,-.3826834).normalized();
    for(size_t i=0;i<pts.size();++i){
      Eigen::Vector3d p=q.conjugate()*(pts[i]-Eigen::Vector3d(.17393,.16893,.170397));
      for(size_t j=0;j<3;++j){const float v=invalid_?0.f:static_cast<float>(p[j]);std::memcpy(m.data.data()+i*12+j*4,&v,4);}
    }
    cloud_->publish(m);
  }
  void goal(double distance=.2,double target_yaw=0.) {
    history_.clear();geometry_msgs::msg::PoseStamped m;m.header.frame_id="map";m.header.stamp=now();
    m.pose.position.x=x_+distance;m.pose.position.y=y_;m.pose.orientation.z=std::sin(target_yaw/2);
    m.pose.orientation.w=std::cos(target_yaw/2);goal_->publish(m);
  }
  void cancel(){cancel_->publish(std_msgs::msg::Empty());drive(.3);history_.clear();drive(.2);}
  bool forward()const{for(const auto& v:history_)if(v.linear.x>0.)return true;return false;}
  bool stopped()const{return last_.linear.x==0. && last_.linear.y==0. && last_.angular.z==0.;}
  bool active()const{return state_.value("active",false);}
  std::string phase()const{return state_.value("phase",std::string());}
  std::string reason()const{return state_.value("reason",std::string("no status"));}
  bool reached()const{return reason()=="goal pose reached within 0.03 m and 0.05 rad";}
  void pass(const std::string& s){std::cout<<"PASS: "<<backend_<<": "<<s<<std::endl;}
  std::string backend_,scenario_;Json state_=Json::object();geometry_msgs::msg::Twist last_;
  std::vector<geometry_msgs::msg::Twist> history_;
  double x_=0.,y_=0.,yaw_=0.,last_cloud_=0.;
  bool obstacle_=false,invalid_=false,send_cloud_=true,send_odom_=true,move_=false;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr cancel_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr commands_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_;
};
int main(int argc,char** argv){
  try{
    require(std::getenv("ROS_DOMAIN_ID") && std::string(std::getenv("ROS_DOMAIN_ID"))=="227","requires isolated domain227");
    require(std::getenv("ROS_LOCALHOST_ONLY") && std::string(std::getenv("ROS_LOCALHOST_ONLY"))=="1","requires localhost");
    require((argc==2||argc==3) && (std::string(argv[1])=="mpc" || std::string(argv[1])=="library"),"backend argument required");
    const std::string backend=argv[1],scenario=argc==3?argv[2]:"full";rclcpp::init(argc,argv);
    auto test=std::make_shared<Smoke>(backend,scenario);test->run();test.reset();rclcpp::shutdown();return 0;
  }catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<std::endl;if(rclcpp::ok())rclcpp::shutdown();return 1;}
}
