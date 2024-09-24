// include/interface_package/land_action.hpp

#ifndef LAND_ACTION_HPP
#define LAND_ACTION_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interface_package/action/land.hpp"
#include "mavros_msgs/srv/command_tol.hpp"

class LandAction {
public:
  using Land = interface_package::action::Land;
  using GoalHandleLand = rclcpp_action::ServerGoalHandle<Land>;

  LandAction(rclcpp::Node::SharedPtr node);

private:
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const Land::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleLand> goal_handle);

  void handle_accepted(
      const std::shared_ptr<GoalHandleLand> goal_handle);

  void execute(
      const std::shared_ptr<GoalHandleLand> goal_handle);

  // Membros
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<Land>::SharedPtr action_server_;
};

#endif // LAND_ACTION_HPP
