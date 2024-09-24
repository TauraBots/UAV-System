// src/takeoff_action.cpp

#include "include/interface_package/takeoff_action.hpp"

TakeoffAction::TakeoffAction(
    rclcpp::Node::SharedPtr node,
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client,
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client,
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub,
    geometry_msgs::msg::PoseStamped& pose,
    mavros_msgs::msg::State& current_state,
    geometry_msgs::msg::Point& current_position)
  : node_(node),
    arming_client_(arming_client),
    set_mode_client_(set_mode_client),
    local_pos_pub_(local_pos_pub),
    pose_(pose),
    current_state_(current_state),
    current_position_(current_position)
{
  action_server_ = rclcpp_action::create_server<Takeoff>(
      node_,
      "takeoff",
      std::bind(&TakeoffAction::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TakeoffAction::handle_cancel, this, std::placeholders::_1),
      std::bind(&TakeoffAction::handle_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse TakeoffAction::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Takeoff::Goal> goal)
{
  RCLCPP_INFO(node_->get_logger(), "Received takeoff request with altitude: %.2f", goal->altitude);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TakeoffAction::handle_cancel(
    const std::shared_ptr<GoalHandleTakeoff> goal_handle)
{
  RCLCPP_INFO(node_->get_logger(), "Takeoff action cancelled");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TakeoffAction::handle_accepted(
    const std::shared_ptr<GoalHandleTakeoff> goal_handle)
{
  std::thread{std::bind(&TakeoffAction::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void TakeoffAction::execute(
    const std::shared_ptr<GoalHandleTakeoff> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Takeoff::Feedback>();
  auto result = std::make_shared<Takeoff::Result>();

  RCLCPP_INFO(node_->get_logger(), "Initiating takeoff sequence...");

  // Arm the drone if not armed
  if (!current_state_.armed) {
    RCLCPP_INFO(node_->get_logger(), "Drone is not armed. Arming the drone...");
    auto arming_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arming_request->value = true;
    auto arming_future = arming_client_->async_send_request(arming_request);
    arming_future.wait();
    if (!arming_future.get()->success) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to arm the drone.");
      result->success = false;
      goal_handle->abort(result);
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "Drone armed successfully.");
  }

  // Set the drone to OFFBOARD mode
  if (current_state_.mode != "OFFBOARD") {
    RCLCPP_INFO(node_->get_logger(), "Setting OFFBOARD mode...");
    auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    set_mode_request->custom_mode = "OFFBOARD";
    auto set_mode_future = set_mode_client_->async_send_request(set_mode_request);
    set_mode_future.wait();
    if (!set_mode_future.get()->mode_sent) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to set OFFBOARD mode.");
      result->success = false;
      goal_handle->abort(result);
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "OFFBOARD mode set successfully.");
  }

  // Set the altitude and initiate takeoff
  pose_.pose.position.z = goal->altitude;
  
  // Constantly send the updated pose to maintain OFFBOARD mode
  float lower_bound = goal->altitude * 0.9;  // 90% da altitude alvo
  float upper_bound = goal->altitude * 1.1;  // 110% da altitude alvo

  while (rclcpp::ok()) {
      local_pos_pub_->publish(pose_);

      feedback->current_altitude = current_position_.z;
      goal_handle->publish_feedback(feedback);
      
      // Verificar se a altitude atual está dentro da faixa com margem de erro de 10%
      if (current_position_.z >= lower_bound && current_position_.z <= upper_bound) {
          RCLCPP_INFO(node_->get_logger(), "Reached target altitude: %.2f (within 10%% margin)", goal->altitude);
          result->success = true;
          goal_handle->succeed(result);
          return;
      }

      rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
}

