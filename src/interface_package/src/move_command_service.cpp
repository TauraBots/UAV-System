// src/move_command_service.cpp

#include "include/interface_package/move_command_service.hpp"

MoveCommandService::MoveCommandService(
    rclcpp::Node::SharedPtr node,
    geometry_msgs::msg::PoseStamped& pose)
  : node_(node),
    pose_(pose)
{
  service_ = node_->create_service<interface_package::srv::MoveCommand>(
      "move_command", std::bind(&MoveCommandService::handle_move_command, this, std::placeholders::_1, std::placeholders::_2));
}

void MoveCommandService::handle_move_command(
    const std::shared_ptr<interface_package::srv::MoveCommand::Request> request,
    std::shared_ptr<interface_package::srv::MoveCommand::Response> response)
{
  pose_.pose.position.x += request->x;
  pose_.pose.position.y += request->y;
  pose_.pose.position.z += request->z;

  RCLCPP_INFO(node_->get_logger(), "Received MoveCommand: [x: %.2f, y: %.2f, z: %.2f]",
              request->x, request->y, request->z);
  response->success = true;
}

