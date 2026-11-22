#define ASTRIBOT_COORDINATOR_NO_MAIN
#include "../src/astribotCoordinator.cpp"
#include <thread>

// Direct callbacks and a scripted monotonic clock make deadline tests deterministic.
// The coordinator always creates its shadow publisher. Fixtures never enable dispatch.
struct CoordinatorTestAccess {
  double time=10.,step=0.,signal_at=INFINITY;
  AstribotCoordinator node;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rear;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
  rclcpp::Publisher<Result>::SharedPtr result;

  CoordinatorTestAccess():node(rclcpp::NodeOptions(),[this]{
    const double t=time; time+=step;
    if(t>=signal_at) shutdown_requested=1;
    return t;
  }) {
    check(!node.dispatch_ && node.output_==PREFIX+"/cmd_vel_shadow","test requires shadow output");
    rear=node.create_publisher<sensor_msgs::msg::PointCloud2>(CLOUD[1],1);
    odom=node.create_publisher<nav_msgs::msg::Odometry>(ODOM,1);
    result=node.create_publisher<Result>(PREFIX+"/mpc_result",1);
    node.selected_=1;
    for(int i=0;;++i) {
      try { node.sources(); break; }
      catch(const std::invalid_argument &) {
        if(i>=100) throw;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
  }
  void owner_tests() {
    const Json good={{"armed",true},{"mode","MODEL"},{"hardware_writes",true},
      {"feedback_source","hardware"},{"stop_pending",false},{"selected_source","auto"},
      {"selected_topic","/cmd_vel"},{"feedback_age_ms",50.},{"generation",7}};
    node.owner_=good; node.owner_received_=9.5; node.owner_generation_=7;
    node.validate_owner(10.);
    auto rejects=[&](double t) {
      try { node.validate_owner(t); }
      catch(const std::invalid_argument &) { return; }
      throw std::runtime_error("invalid owner accepted: "+node.owner_.dump());
    };
    rejects(10.00001); rejects(9.49);
    for(const auto &item:good.items()) {
      node.owner_=good; node.owner_.erase(item.key()); rejects(10.);
    }
    for(const Json &change:std::vector<Json>{
      {{"armed",1}},{{"armed",false}},{{"mode","HOLD"}},{{"hardware_writes",false}},
      {{"feedback_source","replay"}},{{"stop_pending",true}},{{"selected_source","expert"}},
      {{"selected_topic","/other"}},{{"feedback_age_ms",true}},{{"feedback_age_ms",50.1}},
      {{"feedback_age_ms",-1}},{{"feedback_age_ms",NAN}},{{"generation",8}},{{"generation",7.0}}}) {
      node.owner_=good; node.owner_.update(change); rejects(10.);
    }
  }
  void prepare() {
    time=10.;step=0.;node.cancel("fixture");
    node.selected_=1; node.nav_=astribot::Navigation{};
    for(int i=0;i<=4;++i) {
      nav_msgs::msg::Odometry m;
      m.header.frame_id="map"; m.child_frame_id=FRAME;
      m.header.stamp.sec=1; m.header.stamp.nanosec=i*50000000;
      m.pose.pose.orientation.w=1.;
      node.nav_.receive(m,9.8+i*.05,0);
    }
    auto &cloud=*node.clouds_[1]; cloud.stamp=node.nav_.stamp; cloud.received=10.;
    cloud.points.clear(); cloud.fault=false; cloud.error.clear(); node.use_cloud();
    node.session_.begin(Point(1.,0),cloud.stamp);
    node.sequence_.begin(Point(1.,0),0.);node.sequence_.update(Pose::Zero());
    node.source_ids_=node.sources();node.pinned_=true;node.started_=10.;
    node.ready(time);
  }
  Result reply(int ns=100,double received=10.) {
    Result r;r.header.frame_id=FRAME;r.header.stamp.sec=1;r.header.stamp.nanosec=ns;
    r.epoch=node.mpc_epoch_;r.success=true;r.solve_ms=1.;r.command.linear.x=.01;
    r.path.header=r.header;
    for(int i=0;i<21;++i) {
      geometry_msgs::msg::PoseStamped p;p.header=r.header;p.pose.position.x=i*.001;
      p.pose.orientation.w=1.;r.path.poses.push_back(p);
    }
    node.requests_pending_[stamp(r.header.stamp)]={Pose::Zero(),received,r.epoch,Point::Zero()};
    return r;
  }
  void reply_tests() {
    using Mutator=std::function<void(Result &)>;
    for(const auto &mutate:std::vector<Mutator>{
      [](Result &r){r.command.linear.x=.11;},[](Result &r){r.command.linear.y=NAN;},
      [](Result &r){r.command.angular.z=.01;},[](Result &r){r.solve_ms=41.;},
      [](Result &r){r.path.header.frame_id="map";},[](Result &r){r.path.poses.pop_back();},
      [](Result &r){r.path.poses[2].pose.position.x=NAN;},[](Result &r){r.path.poses[2].pose.position.y=.3;},
      [](Result &r){r.path.poses[0].pose.position.x=.001;},[](Result &r){r.path.poses[1].header.frame_id="map";},
      [](Result &r){r.path.poses[1].pose.orientation.w=-1.;},[](Result &r){r.path.poses[1].pose.position.z=.1;},
      [](Result &r){r.path.poses[1].pose.position.x=.005;},[](Result &r){r.path.poses[5].pose.position.x=.009;},
      [](Result &r){r.blocked=true;r.path.poses.resize(1);},[](Result &r){r.success=false;}}) {
      prepare();auto r=reply(); mutate(r); node.mpc_result(r);
      check(!node.session_.active && node.mpc_velocity_.isZero() &&
            node.session_.reason.find("MPC")!=std::string::npos,"invalid reply failed to cancel");
    }
    prepare();node.mpc_result(reply(10,9.849));
    check(!node.session_.active && node.session_.reason.find("expired")!=std::string::npos,"late reply accepted");
    prepare();auto old=reply(1);auto fresh=reply(2,9.95);
    node.mpc_result(fresh);node.mpc_result(old);
    check(node.mpc_last_stamp_==stamp(fresh.header.stamp) && node.mpc_received_==9.95 &&
          node.mpc_velocity_.x()==.01,"reordered reply or refreshed expiry");
    auto cancelled=reply(3);node.cancel("operator cancelled");node.mpc_result(cancelled);
    check(!node.session_.active && node.mpc_received_==-INFINITY,"cancelled reply activated");
    prepare();auto wrong=reply();++wrong.epoch;node.mpc_result(wrong);
    check(node.session_.active && node.mpc_received_==-INFINITY,"wrong epoch activated");
  }
  void command_tests() {
    // A 148 ms scan gap plus encoding/solver delivery can pass a 150 ms tick.
    prepare();node.mpc_result(reply(1));time=10.18;node.tick();
    check(node.session_.active && node.last_translation_.x()>0.,"scan cadence jitter cancelled drive");
    time=10.19;node.mpc_result(reply(2,10.148));
    check(node.session_.active && node.mpc_received_==10.148,"next cloud reply failed");
    prepare();node.mpc_result(reply(1));
    check(node.mpc_drive(Pose::Zero(),10.25).x()>0.,"command rejected at 250 ms boundary");
    bool expired=false;
    try{node.mpc_drive(Pose::Zero(),10.250001);}
    catch(const std::invalid_argument&){expired=true;}
    check(expired,"command accepted past 250 ms");
    prepare();node.last_command_time_=9.95;
    check(node.bounded(Point(.1,-.1),10.).isApprox(Point(.007,-.007)),"physical acceleration limit");
    node.last_translation_=Point(.05,.05);
    check(node.bounded(Point::Zero(),10.).isZero(),"stop rate limited");
    prepare();node.mpc_result(reply(1,9.99));node.last_command_time_=9.9;node.tick();
    check(node.last_translation_.x()==.01 && node.session_.active,"healthy reply did not drive shadow");
    ++node.cloud_stamp_;node.points_={Point(.54,0)};node.tick();
    check(node.last_translation_.isZero() && node.session_.reason.find("blocked")!=std::string::npos,
          "new obstacle failed to invalidate command");
    prepare();node.last_translation_=Point(.010,0.);node.last_command_time_=9.95;
    auto reversal=reply();reversal.command.linear.x=-.004;
    node.requests_pending_.at(stamp(reversal.header.stamp)).previous=node.last_translation_;
    for(size_t i=0;i<reversal.path.poses.size();++i)
      reversal.path.poses[i].pose.position.x=-.0004*i;
    node.points_={Point(.5501,0.)};node.mpc_result(reversal);
    check(node.mpc_velocity_.x()==-.004 && node.bounded(node.mpc_velocity_,10.).x()>0.,
          "reversal fixture must approach obstacle after limiting");
    node.tick();
    check(node.last_translation_.isZero() && node.session_.reason=="blocked: dispatched MPC clearance",
          "limited reversal bypassed actual command check");
    prepare();node.mpc_result(reply(1,9.86));node.last_command_time_=9.9;
    time=10.10;step=.02;node.tick();step=0.;
    check(!node.session_.active && node.last_translation_.isZero() &&
          node.session_.reason=="C++ MPC command expired before dispatch","expiry during tick failed");
    prepare();node.mpc_result(reply(1,9.99));node.last_command_time_=9.9;
    signal_at=10.01;step=.02;node.tick();step=0.;signal_at=INFINITY;shutdown_requested=0;
    check(!node.session_.active && node.last_translation_.isZero() &&
          node.session_.reason=="planner shutting down","signal during tick failed to stop");
    prepare();node.source_ids_.front().front()^=1;node.tick();
    check(!node.session_.active && node.session_.reason=="publisher identity changed during goal",
          "publisher generation changed without cancellation");
  }
};

int main(int argc,char **argv) {
  setenv("ROS_DOMAIN_ID","228",1);
  setenv("ROS_LOCALHOST_ONLY","1",1);
  try {
    rclcpp::init(argc,argv);
    { CoordinatorTestAccess suite;suite.owner_tests();suite.reply_tests();suite.command_tests(); }
    rclcpp::shutdown();
    std::cout<<"Coordinator owner, reply, command and expiry checks passed\n";
    return 0;
  } catch(const std::exception &e) {
    std::cerr<<e.what()<<'\n'; if(rclcpp::ok()) rclcpp::shutdown();return 1;
  }
}
