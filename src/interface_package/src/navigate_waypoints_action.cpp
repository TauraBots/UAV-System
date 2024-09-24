// src/navigate_waypoints_action.cpp

#include "include/interface_package/navigate_waypoints_action.hpp"

NavigateWaypointsAction::NavigateWaypointsAction(
    rclcpp::Node::SharedPtr node,
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub,
    geometry_msgs::msg::PoseStamped& pose,
    geometry_msgs::msg::Point& current_position)
  : node_(node),
    local_pos_pub_(local_pos_pub),
    pose_(pose),
    current_position_(current_position)
{
  action_server_ = rclcpp_action::create_server<NavigateWaypoints>(
      node_, "navigate_waypoints",
      std::bind(&NavigateWaypointsAction::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&NavigateWaypointsAction::handle_cancel, this, std::placeholders::_1),
      std::bind(&NavigateWaypointsAction::handle_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse NavigateWaypointsAction::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateWaypoints::Goal>)
{
  RCLCPP_INFO(node_->get_logger(), "Received navigate waypoints request.");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavigateWaypointsAction::handle_cancel(
    const std::shared_ptr<GoalHandleNavigateWaypoints> goal_handle)
{
  RCLCPP_INFO(node_->get_logger(), "Navigate waypoints action cancelled");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigateWaypointsAction::handle_accepted(
    const std::shared_ptr<GoalHandleNavigateWaypoints> goal_handle)
{
  std::thread{std::bind(&NavigateWaypointsAction::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void NavigateWaypointsAction::execute(
    const std::shared_ptr<GoalHandleNavigateWaypoints> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<NavigateWaypoints::Feedback>();
  auto result = std::make_shared<NavigateWaypoints::Result>();

  RCLCPP_INFO(node_->get_logger(), "Navigating through waypoints...");

  for (const auto& waypoint : goal->waypoints) {
    pose_.pose = waypoint.pose;

    while (rclcpp::ok()) {
      local_pos_pub_->publish(pose_);

      // Atualizando feedback com a posição atual
      feedback->current_pose = pose_;
      goal_handle->publish_feedback(feedback);

      // Verificar se atingiu o waypoint
      if (is_position_reached(waypoint.pose)) {
        RCLCPP_INFO(node_->get_logger(), "Waypoint reached.");
        break;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Waypoints navigation completed.");
  result->success = true;
  goal_handle->succeed(result);
}

bool NavigateWaypointsAction::is_position_reached(const geometry_msgs::msg::Pose& target_pose) {
  const double tolerance = 0.1;  // Tolerância de posição
  double dx = std::abs(current_position_.x - target_pose.position.x);
  double dy = std::abs(current_position_.y - target_pose.position.y);
  double dz = std::abs(current_position_.z - target_pose.position.z);

  return (dx < tolerance && dy < tolerance && dz < tolerance);
}

