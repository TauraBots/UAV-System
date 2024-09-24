// src/interface_node.cpp

#include "include/interface_package/interface_node.hpp"

InterfaceNode::InterfaceNode() : Node("interface_node") {
  state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "mavros/state", 10, std::bind(&InterfaceNode::state_cb, this, std::placeholders::_1));

  current_position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SystemDefaultsQoS(), 
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          current_position_ = msg->pose.position;
      });

  local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "mavros/setpoint_position/local", 10);

  arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
  set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&InterfaceNode::control_loop, this));

  last_request_ = this->now();
}

void InterfaceNode::initialize() {
  // Inicializar as ações e serviços
  takeoff_action_ = std::make_shared<TakeoffAction>(
    this->shared_from_this(), arming_client_, set_mode_client_, local_pos_pub_, pose_, current_state_, current_position_);
  land_action_ = std::make_shared<LandAction>(this->shared_from_this());
  navigate_waypoints_action_ = std::make_shared<NavigateWaypointsAction>(
    this->shared_from_this(), local_pos_pub_, pose_, current_position_);
  move_command_service_ = std::make_shared<MoveCommandService>(this->shared_from_this(), pose_);
}

void InterfaceNode::state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
  current_state_ = *msg;
  RCLCPP_INFO(this->get_logger(), "Current mode: %s, Armed: %s", 
              current_state_.mode.c_str(), current_state_.armed ? "Yes" : "No");

  if (!current_state_.connected) {
      RCLCPP_ERROR(this->get_logger(), "Drone not connected to MAVROS.");
  }
}

void InterfaceNode::control_loop() {
  local_pos_pub_->publish(pose_);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InterfaceNode>();
  node->initialize();  // Chame o método initialize aqui
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}