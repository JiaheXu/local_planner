#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

using namespace std;

const double PI = 3.1415926;

rclcpp::Node::SharedPtr nh;
nav_msgs::msg::Path path;
bool pathInit = false;
bool nextGoalRequested = false;

double vehicleX = 0.0, vehicleY = 0.0, vehicleYaw = 0.0;
double defaultSpeed = 0.5;        // m/s
double desiredSpeed = defaultSpeed;
double yawRateGain = 3.0;         // turning gain
double lookAheadDis = 0.5;        // lookahead distance
double goalTolerance = 0.2;       // stop within 0.2 m
double requestDistance = 1.0;     // trigger next-goal request at 1 m
int pathPointID = 0;

void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odomIn) {
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

void pathHandler(const nav_msgs::msg::Path::ConstSharedPtr pathIn) {
  path = *pathIn;
  pathPointID = 0;
  pathInit = true;
  nextGoalRequested = false;  // reset trigger for new path
}

void speedHandler(const std_msgs::msg::Float32::ConstSharedPtr msg) {
  desiredSpeed = msg->data;
  if (desiredSpeed < 0.0) desiredSpeed = 0.0;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("path_follower");

  auto subOdom = nh->create_subscription<nav_msgs::msg::Odometry>(
      "/state_estimation", 10, odomHandler);
  auto subPath = nh->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, pathHandler);
  auto subSpeed = nh->create_subscription<std_msgs::msg::Float32>(
      "/desired_speed", 10, speedHandler);

  auto pubCmd = nh->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", 10);

  // --- Reliable Transient Local QoS for next-goal request ---
  rclcpp::QoS qos_latched(1);
  qos_latched.transient_local().reliable();
  auto pubGoalReq = nh->create_publisher<std_msgs::msg::Bool>(
      "/goal_request", qos_latched);

  rclcpp::Rate rate(30);

  while (rclcpp::ok()) {
    rclcpp::spin_some(nh);

    if (pathInit && path.poses.size() > 1) {
      // --- distance to final goal ---
      auto goal = path.poses.back().pose.position;
      double dxGoal = goal.x - vehicleX;
      double dyGoal = goal.y - vehicleY;
      double distGoal = sqrt(dxGoal * dxGoal + dyGoal * dyGoal);

      geometry_msgs::msg::TwistStamped cmd;
      cmd.header.stamp = nh->get_clock()->now();

      // --- trigger "send next goal" once at 1m ---
      if (!nextGoalRequested && distGoal < requestDistance) {
        std_msgs::msg::Bool msg;
        msg.data = true;
        pubGoalReq->publish(msg);
        RCLCPP_INFO(nh->get_logger(), "Published next-goal request (within 1m).");
        nextGoalRequested = true;
      }

      // --- stop at final goal ---
      if (distGoal < goalTolerance) {
        cmd.twist.linear.x = 0.0;
        cmd.twist.angular.z = 0.0;
        pubCmd->publish(cmd);
        RCLCPP_INFO_ONCE(nh->get_logger(), "Reached final goal. Stopping.");
      } else {
        // follow path
        double dx, dy, dis;
        while (pathPointID < (int)path.poses.size() - 1) {
          dx = path.poses[pathPointID].pose.position.x - vehicleX;
          dy = path.poses[pathPointID].pose.position.y - vehicleY;
          dis = sqrt(dx * dx + dy * dy);
          if (dis < lookAheadDis) {
            pathPointID++;
          } else {
            break;
          }
        }

        auto target = path.poses[min(pathPointID, (int)path.poses.size()-1)].pose.position;
        dx = target.x - vehicleX;
        dy = target.y - vehicleY;

        double pathDir = atan2(dy, dx);
        double dirDiff = pathDir - vehicleYaw;
        if (dirDiff > PI) dirDiff -= 2 * PI;
        if (dirDiff < -PI) dirDiff += 2 * PI;

        // control
        double yawRate = -yawRateGain * dirDiff;
        double speed = desiredSpeed > 0 ? desiredSpeed : defaultSpeed;

        cmd.twist.linear.x = speed;
        cmd.twist.angular.z = yawRate;
        pubCmd->publish(cmd);
      }
    }

    rate.sleep();
  }

  return 0;
}

