// src/land_action.cpp

#include "include/interface_package/land_action.hpp"

LandAction::LandAction(rclcpp::Node::SharedPtr node)
  : node_(node)
{
  action_server_ = rclcpp_action::create_server<Land>(
      node_,
      "land",
      std::bind(&LandAction::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&LandAction::handle_cancel, this, std::placeholders::_1),
      std::bind(&LandAction::handle_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse LandAction::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Land::Goal>)
{
  RCLCPP_INFO(node_->get_logger(), "Received land request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LandAction::handle_cancel(
    const std::shared_ptr<GoalHandleLand> goal_handle)
{
  RCLCPP_INFO(node_->get_logger(), "Land action cancelled");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LandAction::handle_accepted(
    const std::shared_ptr<GoalHandleLand> goal_handle)
{
  std::thread{std::bind(&LandAction::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void LandAction::execute(
    const std::shared_ptr<GoalHandleLand> goal_handle)
{
  auto feedback = std::make_shared<Land::Feedback>();
  auto result = std::make_shared<Land::Result>();

  RCLCPP_INFO(node_->get_logger(), "Initiating landing sequence using MAVROS...");

  // Chamar o serviço de Land do MAVROS
  auto land_client = node_->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/land");
  while (!land_client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(node_->get_logger(), "Waiting for the land service to be available...");
  }

  auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
  request->altitude = 0.0;  // Aterrissar no solo
  request->latitude = 0.0;  // Não precisa ser definido para pouso local
  request->longitude = 0.0; // Não precisa ser definido para pouso local
  request->min_pitch = 0.0;
  request->yaw = 0.0; 

  auto response_future = land_client->async_send_request(request);

  try {
      auto response = response_future.get();
      if (response->success) {
          RCLCPP_INFO(node_->get_logger(), "Land service called successfully.");
          result->success = true;
          goal_handle->succeed(result);
      } else {
          RCLCPP_ERROR(node_->get_logger(), "Failed to land the drone.");
          result->success = false;
          goal_handle->abort(result);
      }
  } catch (const std::exception &e) {
      RCLCPP_ERROR(node_->get_logger(), "Service call failed: %s", e.what());
      result->success = false;
      goal_handle->abort(result);
  }
}

