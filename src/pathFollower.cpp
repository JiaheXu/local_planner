#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <algorithm>

constexpr double PI = 3.14159265358979323846;

inline double wrapToPi(double a) {
  while (a > PI) a -= 2.0 * PI;
  while (a < -PI) a += 2.0 * PI;
  return a;
}

class PathFollower : public rclcpp::Node {
public:
  PathFollower() : Node("path_follower") {
    // Parameters
    this->declare_parameter("default_speed", 0.6);
    this->declare_parameter("yaw_rate_gain", 3.0);
    this->declare_parameter("lookahead_distance", 1.0);
    this->declare_parameter("goal_tolerance", 0.2);
    this->declare_parameter("request_distance", 1.0);

    // New obstacle-related params
    this->declare_parameter("obstacle_distance_threshold", 0.5);
    this->declare_parameter("obstacle_half_angle_deg", 30.0);
    this->declare_parameter("obstacle_gain", 3.0);   // ✅ ADD THIS
    // Load initial values
    defaultSpeed   = this->get_parameter("default_speed").as_double();
    desiredSpeed   = defaultSpeed;
    yawRateGain    = this->get_parameter("yaw_rate_gain").as_double();
    lookAheadDis   = this->get_parameter("lookahead_distance").as_double();
    goalTolerance  = this->get_parameter("goal_tolerance").as_double();
    requestDistance= this->get_parameter("request_distance").as_double();
    obstacleDistThresh = this->get_parameter("obstacle_distance_threshold").as_double();
    obstacleHalfAngle  = this->get_parameter("obstacle_half_angle_deg").as_double() * PI / 180.0;
    obstacle_gain  = this->get_parameter("obstacle_gain").as_double();
    // Dynamic parameter update callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&PathFollower::on_param_change, this, std::placeholders::_1)
    );

    // Subscribers
    // subOdom = this->create_subscription<nav_msgs::msg::Odometry>(
        // "/state_estimation", 10, std::bind(&PathFollower::odomHandler, this, std::placeholders::_1));
    subPath = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, std::bind(&PathFollower::pathHandler, this, std::placeholders::_1));
    subSpeed = this->create_subscription<std_msgs::msg::Float32>(
        "/desired_speed", 10, std::bind(&PathFollower::speedHandler, this, std::placeholders::_1));
    subMode = this->create_subscription<std_msgs::msg::Int32>(
        "/behavior_mode", 10, std::bind(&PathFollower::modeHandler, this, std::placeholders::_1));
    subScan = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&PathFollower::scanHandler, this, std::placeholders::_1));

    // Publishers
    pubCmd = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

    rclcpp::QoS qos_latched(1);
    qos_latched.transient_local().reliable();
    pubGoalReq = this->create_publisher<std_msgs::msg::Bool>("/goal_request", qos_latched);

    // Control loop
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),   // 10 Hz
        std::bind(&PathFollower::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "PathFollower node started.");
  }

private:
  // State
  double vehicleX{0.0}, vehicleY{0.0}, vehicleYaw{0.0};
  nav_msgs::msg::Path path;
  bool pathInit{false};
  bool nextGoalRequested{false};
  int pathPointID{0};

  // Config
  double defaultSpeed;
  double desiredSpeed;
  double yawRateGain;
  double lookAheadDis;
  double goalTolerance;
  double requestDistance;
  int behavior_mode = 0;

  // Obstacle params
  double obstacleDistThresh;   // [m]
  double obstacleHalfAngle;    // [rad]
  double obstacle_gain;    // [rad]
  bool obstacleClose{false};

  // ROS
  // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subSpeed;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subMode;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subScan;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pubCmd;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubGoalReq;

  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Callbacks
  void modeHandler(const std_msgs::msg::Int32::SharedPtr msg) {
    behavior_mode = msg->data;
  }

  void odomHandler(const nav_msgs::msg::Odometry::SharedPtr odomIn) {
    double roll, pitch, yaw;
    tf2::Quaternion q(
        odomIn->pose.pose.orientation.x,
        odomIn->pose.pose.orientation.y,
        odomIn->pose.pose.orientation.z,
        odomIn->pose.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    vehicleYaw = yaw;
    vehicleX = odomIn->pose.pose.position.x;
    vehicleY = odomIn->pose.pose.position.y;
  }

  void pathHandler(const nav_msgs::msg::Path::SharedPtr pathIn) {
    if (!pathIn || pathIn->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty /path");
      pathInit = false;
      return;
    }
    path = *pathIn;
    pathPointID = 0;
    pathInit = true;
    nextGoalRequested = false;
    RCLCPP_INFO(this->get_logger(), "New path with %zu poses.", path.poses.size());
  }

  void speedHandler(const std_msgs::msg::Float32::SharedPtr msg) {
    desiredSpeed = std::max(0.0f, msg->data);
  }

  void scanHandler(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    obstacleClose = false;
    for (size_t i = 0; i < scan->ranges.size(); i++) {
      double angle = scan->angle_min + i * scan->angle_increment;
      if (angle > -obstacleHalfAngle && angle < obstacleHalfAngle) {
        double r = scan->ranges[i];
        if (std::isfinite(r) && r < obstacleDistThresh) {
          obstacleClose = true;
          break;
        }
      }
    }
  }

  // Dynamic parameter change
  rcl_interfaces::msg::SetParametersResult
  on_param_change(const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &p : params) {
      if (p.get_name() == "default_speed") defaultSpeed = p.as_double();
      if (p.get_name() == "yaw_rate_gain") yawRateGain = p.as_double();
      if (p.get_name() == "lookahead_distance") lookAheadDis = p.as_double();
      if (p.get_name() == "goal_tolerance") goalTolerance = p.as_double();
      if (p.get_name() == "request_distance") requestDistance = p.as_double();
      if (p.get_name() == "obstacle_distance_threshold") obstacleDistThresh = p.as_double();
      if (p.get_name() == "obstacle_half_angle_deg") obstacleHalfAngle = p.as_double() * PI / 180.0;
      if (p.get_name() == "obstacle_gain") obstacle_gain = p.as_double();
    }
    return result;
  }

  // Control Loop
  void controlLoop() {
    if (behavior_mode < 2) return;
    if (!(pathInit && path.poses.size() > 1)) {
      publishStop();
      return;
    }

    auto goal = path.poses.back().pose.position;
    double dxGoal = goal.x - vehicleX;
    double dyGoal = goal.y - vehicleY;
    double distGoal = std::hypot(dxGoal, dyGoal);

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";

    if (!nextGoalRequested && distGoal < requestDistance) {
      std_msgs::msg::Bool m; m.data = true;
      pubGoalReq->publish(m);
      nextGoalRequested = true;
    }

    if (distGoal < goalTolerance) {
      publishStop();
      return;
    }

    while (pathPointID < static_cast<int>(path.poses.size()) - 1) {
      double dx = path.poses[pathPointID].pose.position.x - vehicleX;
      double dy = path.poses[pathPointID].pose.position.y - vehicleY;
      if (std::hypot(dx, dy) < lookAheadDis) {
        pathPointID++;
      } else {
        break;
      }
    }
    pathPointID = std::min(pathPointID, static_cast<int>(path.poses.size()) - 1);

    auto target = path.poses[pathPointID].pose.position;
    double dx = target.x - vehicleX;
    double dy = target.y - vehicleY;

    double pathDir = std::atan2(dy, dx);
    double dirDiff = wrapToPi(pathDir - vehicleYaw);

    double gain = yawRateGain;
    if (obstacleClose) {
      gain *= obstacle_gain;
      RCLCPP_DEBUG(this->get_logger(), "Obstacle detected! Doubling yaw gain.");
    }

    double yawRate = gain * dirDiff;
    double speed = desiredSpeed > 0.0 ? desiredSpeed : defaultSpeed;

    cmd.twist.linear.x = speed;
    cmd.twist.angular.z = yawRate;
    pubCmd->publish(cmd);
  }

  void publishStop() {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;
    pubCmd->publish(cmd);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
