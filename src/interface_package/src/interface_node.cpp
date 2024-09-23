#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "interface_package/srv/move_command.hpp"  

class InterfaceNode : public rclcpp::Node {
public:
  InterfaceNode() : Node("interface_node") {
    // Subscrição para o estado do MAVROS
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", 10, std::bind(&InterfaceNode::state_cb, this, std::placeholders::_1));

    // Publicador para setpoints de posição
    local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "mavros/setpoint_position/local", 10);

    // Clientes para armar e setar modo OFFBOARD
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

    // Servidor para receber comandos de movimento
    move_service_ = this->create_service<interface_package::srv::MoveCommand>(
        "move_command", std::bind(&InterfaceNode::move_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Definir a pose inicial
    pose_.pose.position.x = 0.0;
    pose_.pose.position.y = 0.0;
    pose_.pose.position.z = 2.0;

    // Iniciar o loop de controle
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&InterfaceNode::control_loop, this));

    last_request_ = this->now();
  }

private:
  void state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
    current_state_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Current mode: %s, Armed: %s", 
                current_state_.mode.c_str(), current_state_.armed ? "Yes" : "No");
  }

  void move_callback(const std::shared_ptr<interface_package::srv::MoveCommand::Request> request,
                     std::shared_ptr<interface_package::srv::MoveCommand::Response> response) {
    pose_.pose.position.x += request->x;
    pose_.pose.position.y += request->y;
    pose_.pose.position.z += request->z;

    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Received move command: x: %.2f, y: %.2f, z: %.2f", 
                request->x, request->y, request->z);
    RCLCPP_INFO(this->get_logger(), "Updated pose to: (%.2f, %.2f, %.2f)",
                pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z);
  }

  void control_loop() {
    // Verificar se o drone já está no modo OFFBOARD
    if (current_state_.mode != "OFFBOARD" &&
        (this->now() - last_request_ > rclcpp::Duration(5, 0))) {
      if (set_mode_client_->wait_for_service(std::chrono::seconds(1))) {
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "OFFBOARD";
        RCLCPP_INFO(this->get_logger(), "Attempting to set OFFBOARD mode...");
        auto result = set_mode_client_->async_send_request(request);
        last_request_ = this->now();
      } else {
        RCLCPP_WARN(this->get_logger(), "OFFBOARD service not available.");
      }
    } else {
      // Armar o drone
      if (!current_state_.armed &&
          (this->now() - last_request_ > rclcpp::Duration(5, 0))) {
        if (arming_client_->wait_for_service(std::chrono::seconds(1))) {
          auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
          request->value = true;
          RCLCPP_INFO(this->get_logger(), "Attempting to arm the drone...");
          auto result = arming_client_->async_send_request(request);
          last_request_ = this->now();
        } else {
          RCLCPP_WARN(this->get_logger(), "Arming service not available.");
        }
      } else {
        // Publicar a posição alvo
        RCLCPP_INFO_ONCE(this->get_logger(), "Drone is armed and in OFFBOARD mode.");
        RCLCPP_INFO(this->get_logger(), "Publishing target pose: (%.2f, %.2f, %.2f)",
                    pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z);
        local_pos_pub_->publish(pose_);
      }
    }
  }

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Service<interface_package::srv::MoveCommand>::SharedPtr move_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseStamped pose_;
  mavros_msgs::msg::State current_state_;
  rclcpp::Time last_request_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InterfaceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
