// include/interface_package/interface_node.hpp

#ifndef INTERFACE_NODE_HPP
#define INTERFACE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"

#include "nav_msgs/msg/odometry.hpp"  
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// Inclua as ações e serviços do diretório interface_package
#include "takeoff_action.hpp"
#include "land_action.hpp"
#include "navigate_waypoints_action.hpp"
#include "move_command_service.hpp"

class InterfaceNode : public rclcpp::Node {
public:
  InterfaceNode();
  void initialize();

private:
  // Declaração dos métodos privados
  void state_cb(const mavros_msgs::msg::State::SharedPtr msg);
  void control_loop();
  void zed_odom_callback(const nav_msgs::msg::Odometry::SharedPtr enu_msg);  


  // Variáveis e objetos
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_position_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr zed_odom_sub_;  
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mavros_odom_pub_;
  
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseStamped pose_;
  geometry_msgs::msg::Point current_position_;
  mavros_msgs::msg::State current_state_;
  rclcpp::Time last_request_;

  // Ações e serviços
  std::shared_ptr<TakeoffAction> takeoff_action_;
  std::shared_ptr<LandAction> land_action_;
  std::shared_ptr<NavigateWaypointsAction> navigate_waypoints_action_;
  std::shared_ptr<MoveCommandService> move_command_service_;
};

#endif // INTERFACE_NODE_HPP