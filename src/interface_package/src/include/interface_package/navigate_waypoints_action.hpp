// include/interface_package/navigate_waypoints_action.hpp

#ifndef NAVIGATE_WAYPOINTS_ACTION_HPP
#define NAVIGATE_WAYPOINTS_ACTION_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interface_package/action/navigate_waypoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class NavigateWaypointsAction {
public:
  using NavigateWaypoints = interface_package::action::NavigateWaypoints;
  using GoalHandleNavigateWaypoints = rclcpp_action::ServerGoalHandle<NavigateWaypoints>;

  NavigateWaypointsAction(
    rclcpp::Node::SharedPtr node,
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub,
    geometry_msgs::msg::PoseStamped& pose,
    geometry_msgs::msg::Point& current_position);

private:
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const NavigateWaypoints::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleNavigateWaypoints> goal_handle);

  void handle_accepted(
      const std::shared_ptr<GoalHandleNavigateWaypoints> goal_handle);

  void execute(
      const std::shared_ptr<GoalHandleNavigateWaypoints> goal_handle);

  bool is_position_reached(const geometry_msgs::msg::Pose& target_pose);

  // Membros
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<NavigateWaypoints>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
  geometry_msgs::msg::PoseStamped& pose_;
  geometry_msgs::msg::Point& current_position_;
};

#endif // NAVIGATE_WAYPOINTS_ACTION_HPP
