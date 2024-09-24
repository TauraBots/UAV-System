// include/interface_package/takeoff_action.hpp

#ifndef TAKEOFF_ACTION_HPP
#define TAKEOFF_ACTION_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interface_package/action/takeoff.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"

class TakeoffAction {
public:
  using Takeoff = interface_package::action::Takeoff;
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<Takeoff>;

  TakeoffAction(
    rclcpp::Node::SharedPtr node,
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client,
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client,
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub,
    geometry_msgs::msg::PoseStamped& pose,
    mavros_msgs::msg::State& current_state,
    geometry_msgs::msg::Point& current_position);

private:
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const Takeoff::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleTakeoff> goal_handle);

  void handle_accepted(
      const std::shared_ptr<GoalHandleTakeoff> goal_handle);

  void execute(
      const std::shared_ptr<GoalHandleTakeoff> goal_handle);

  // Membros
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<Takeoff>::SharedPtr action_server_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
  geometry_msgs::msg::PoseStamped& pose_;
  mavros_msgs::msg::State& current_state_;
  geometry_msgs::msg::Point& current_position_;
};

#endif // TAKEOFF_ACTION_HPP
