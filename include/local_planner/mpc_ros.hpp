#pragma once
#include "local_planner/planar_mpc.hpp"
#include "local_planner/pose_mpc.hpp"
#include "local_planner/msg/mpc_request.hpp"
#include "local_planner/msg/mpc_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace local_planner {
// Stateless requests carry goal epochs and previous *accepted* commands.
// This node proposes commands only; the coordinator gates ROS dispatch and
// XPolicy retains final hardware ownership and command validation.
class MpcRos {
 public:
  explicit MpcRos(const rclcpp::Node::SharedPtr& node) : node_(node) {
    frame_=node_->declare_parameter<std::string>("vehicleFrame", "astribot_viz_chassis");
    result_=node_->create_publisher<msg::MpcResult>("/mpc_result", 1);
    request_=node_->create_subscription<msg::MpcRequest>("/mpc_request", 1,
      [this](msg::MpcRequest::ConstSharedPtr request) { receive(*request); });
    RCLCPP_INFO(node_->get_logger(), "C++ MPC backend: 0.1 m/s, 2 s horizon; proposals only");
  }
 private:
  void receive(const msg::MpcRequest& request) {
    msg::MpcResult response;
    response.header=request.header; response.epoch=request.epoch;
    const int64_t stamp=rclcpp::Time(request.header.stamp).nanoseconds();
    const double age=(node_->now()-rclcpp::Time(request.sent_at)).seconds();
    if (request.header.frame_id!=frame_ || stamp<=last_stamp_ || !request.epoch
        || !std::isfinite(age) || age<-.05 || age>.15 || request.obstacles.size()>500000
        || request.goal.z!=0. || request.previous_velocity.z!=0.) {
      solver_.reset(); response.error="invalid, duplicate or expired MPC request";
      result_->publish(response); return;
    }
    last_stamp_=stamp;
    if (epoch_!=request.epoch) { solver_.reset(); epoch_=request.epoch; }
    if(request.plan_yaw) {
      if(!request.obstacles.empty()) {
        response.error="pose MPC currently requires no obstacles";result_->publish(response);return;
      }
      auto solved=pose_solver_.solve({request.goal.x,request.goal.y},request.goal_yaw,
                                    {request.previous_velocity.x,request.previous_velocity.y},request.previous_yaw_rate);
      response.success=solved.success;response.error=solved.error;response.solve_ms=solved.solve_ms;
      response.command.linear.x=solved.velocity.x();response.command.linear.y=solved.velocity.y();
      response.command.angular.z=solved.yaw_rate;response.path.header=request.header;
      for(const auto& p:solved.poses) {
        geometry_msgs::msg::PoseStamped pose;pose.header=request.header;
        pose.pose.position.x=p.x();pose.pose.position.y=p.y();
        pose.pose.orientation.z=std::sin(p.z()/2.);pose.pose.orientation.w=std::cos(p.z()/2.);
        response.path.poses.push_back(pose);
      }
      result_->publish(response);return;
    }
    Points points;
    points.reserve(request.obstacles.size());
    for (const auto& p : request.obstacles) {
      if (p.z!=0.) { response.error="non-planar obstacle"; solver_.reset(); result_->publish(response); return; }
      points.emplace_back(p.x,p.y);
    }
    auto solved=solver_.solve({request.goal.x,request.goal.y}, points,
                             {request.previous_velocity.x,request.previous_velocity.y});
    response.success=solved.success; response.blocked=solved.blocked;
    response.error=solved.error; response.solve_ms=solved.solve_ms;
    response.command.linear.x=solved.velocity.x(); response.command.linear.y=solved.velocity.y();
    response.path.header=request.header;
    for (const auto& xy : solved.path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header=request.header; pose.pose.orientation.w=1.;
      pose.pose.position.x=xy.x(); pose.pose.position.y=xy.y();
      response.path.poses.push_back(pose);
    }
    result_->publish(response);
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<msg::MpcRequest>::SharedPtr request_;
  rclcpp::Publisher<msg::MpcResult>::SharedPtr result_;
  PlanarMpc solver_;
  PoseMpc pose_solver_;
  std::string frame_;
  uint64_t epoch_=0;
  int64_t last_stamp_=0;
};
}  // namespace local_planner
