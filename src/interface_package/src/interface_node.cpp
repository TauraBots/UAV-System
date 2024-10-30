// src/interface_node.cpp

#include "include/interface_package/interface_node.hpp"
#include "nav_msgs/msg/odometry.hpp"

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

  zed_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/zedm/zed_node/odom", 10, std::bind(&InterfaceNode::zed_odom_callback, this, std::placeholders::_1));

  mavros_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/mavros/odometry/in", 10);

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

void InterfaceNode::zed_odom_callback(const nav_msgs::msg::Odometry::SharedPtr enu_msg) {
  // Converter ENU para NED
  nav_msgs::msg::Odometry ned_msg;
  ned_msg.header = enu_msg->header;

  double offset_x = 0.1;

  // Converte posição de ENU para NED
  ned_msg.pose.pose.position.x = enu_msg->pose.pose.position.y;
  ned_msg.pose.pose.position.y = enu_msg->pose.pose.position.x - offset_x;
  ned_msg.pose.pose.position.z = -enu_msg->pose.pose.position.z;

  // Converte orientação de ENU para NED
  tf2::Quaternion q_enu(
      enu_msg->pose.pose.orientation.x,
      enu_msg->pose.pose.orientation.y,
      enu_msg->pose.pose.orientation.z,
      enu_msg->pose.pose.orientation.w);

  tf2::Quaternion q_ned;
  q_ned.setRPY(M_PI, 0.0, M_PI / 2);
  q_ned *= q_enu;
  q_ned.normalize();

  ned_msg.pose.pose.orientation.x = q_ned.x();
  ned_msg.pose.pose.orientation.y = q_ned.y();
  ned_msg.pose.pose.orientation.z = q_ned.z();
  ned_msg.pose.pose.orientation.w = q_ned.w();

  // Converte velocidade linear de ENU para NED
  ned_msg.twist.twist.linear.x = enu_msg->twist.twist.linear.y;
  ned_msg.twist.twist.linear.y = enu_msg->twist.twist.linear.x;
  ned_msg.twist.twist.linear.z = -enu_msg->twist.twist.linear.z;

  // Converte velocidade angular de ENU para NED
  ned_msg.twist.twist.angular.x = enu_msg->twist.twist.angular.y;
  ned_msg.twist.twist.angular.y = enu_msg->twist.twist.angular.x;
  ned_msg.twist.twist.angular.z = -enu_msg->twist.twist.angular.z;

  // Publica a odometria convertida
  mavros_odom_pub_->publish(ned_msg);
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