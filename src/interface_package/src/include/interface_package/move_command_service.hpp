// include/interface_package/move_command_service.hpp

#ifndef MOVE_COMMAND_SERVICE_HPP
#define MOVE_COMMAND_SERVICE_HPP

#include "rclcpp/rclcpp.hpp"
#include "interface_package/srv/move_command.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class MoveCommandService {
public:
  MoveCommandService(
    rclcpp::Node::SharedPtr node,
    geometry_msgs::msg::PoseStamped& pose);

private:
  void handle_move_command(
      const std::shared_ptr<interface_package::srv::MoveCommand::Request> request,
      std::shared_ptr<interface_package::srv::MoveCommand::Response> response);

  // Membros
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<interface_package::srv::MoveCommand>::SharedPtr service_;
  geometry_msgs::msg::PoseStamped& pose_;
};

#endif // MOVE_COMMAND_SERVICE_HPP
