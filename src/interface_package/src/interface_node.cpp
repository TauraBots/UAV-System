#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "interface_package/action/takeoff.hpp"
#include "interface_package/action/land.hpp"
#include "interface_package/srv/move_command.hpp"  // Adicionando o MoveCommand
#include "rclcpp_action/rclcpp_action.hpp"

class InterfaceNode : public rclcpp::Node {
public:
  using Takeoff = interface_package::action::Takeoff;
  using Land = interface_package::action::Land;
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<Takeoff>;
  using GoalHandleLand = rclcpp_action::ServerGoalHandle<Land>;

  InterfaceNode() : Node("interface_node") {

    // Subscrição para o estado do MAVROS
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", 10, std::bind(&InterfaceNode::state_cb, this, std::placeholders::_1));

    // Subscrição para a posição atual do drone
    current_position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose", rclcpp::SystemDefaultsQoS(), 
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            current_position_ = msg->pose.position;
        });

    // Publicador para setpoints de posição
    local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "mavros/setpoint_position/local", 10);

    // Cliente para armar e setar modo OFFBOARD
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

    // Action Server para Takeoff
    takeoff_action_server_ = rclcpp_action::create_server<Takeoff>(
        this,
        "takeoff",
        std::bind(&InterfaceNode::handle_goal_takeoff, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&InterfaceNode::handle_cancel_takeoff, this, std::placeholders::_1),
        std::bind(&InterfaceNode::handle_accepted_takeoff, this, std::placeholders::_1));

    // Action Server para Land
    land_action_server_ = rclcpp_action::create_server<Land>(
        this,
        "land",
        std::bind(&InterfaceNode::handle_goal_land, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&InterfaceNode::handle_cancel_land, this, std::placeholders::_1),
        std::bind(&InterfaceNode::handle_accepted_land, this, std::placeholders::_1));

    // Serviço para mover o drone
    move_command_service_ = this->create_service<interface_package::srv::MoveCommand>(
        "move_command", std::bind(&InterfaceNode::handle_move_command, this, std::placeholders::_1, std::placeholders::_2));
    
    // Timer para enviar continuamente setpoints de posição
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&InterfaceNode::control_loop, this));

    last_request_ = this->now();
  }

private:
  // Callback para o estado do drone
  void state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
    current_state_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Current mode: %s, Armed: %s", 
                current_state_.mode.c_str(), current_state_.armed ? "Yes" : "No");

    // Verificar se o drone está conectado
    if (!current_state_.connected) {
        RCLCPP_ERROR(this->get_logger(), "Drone not connected to MAVROS.");
    }
  }

  // Handle goal for takeoff
  rclcpp_action::GoalResponse handle_goal_takeoff(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const Takeoff::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received takeoff request with altitude: %.2f", goal->altitude);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Handle cancellation for takeoff
  rclcpp_action::CancelResponse handle_cancel_takeoff(
      const std::shared_ptr<GoalHandleTakeoff> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Takeoff action cancelled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Handle accepted takeoff goal
  void handle_accepted_takeoff(const std::shared_ptr<GoalHandleTakeoff> goal_handle) {
    std::thread{std::bind(&InterfaceNode::execute_takeoff, this, std::placeholders::_1), goal_handle}.detach();
  }

  // Execute the takeoff action
  void execute_takeoff(const std::shared_ptr<GoalHandleTakeoff> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Takeoff::Feedback>();
    auto result = std::make_shared<Takeoff::Result>();

    RCLCPP_INFO(this->get_logger(), "Initiating takeoff sequence...");

    // Arm the drone if not armed
    if (!current_state_.armed) {
      RCLCPP_INFO(this->get_logger(), "Drone is not armed. Arming the drone...");
      auto arming_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      arming_request->value = true;
      auto arming_future = arming_client_->async_send_request(arming_request);
      arming_future.wait();
      if (!arming_future.get()->success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to arm the drone.");
        result->success = false;
        goal_handle->abort(result);
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Drone armed successfully.");
    }

    // Set the drone to OFFBOARD mode
    if (current_state_.mode != "OFFBOARD") {
      RCLCPP_INFO(this->get_logger(), "Setting OFFBOARD mode...");
      auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      set_mode_request->custom_mode = "OFFBOARD";
      auto set_mode_future = set_mode_client_->async_send_request(set_mode_request);
      set_mode_future.wait();
      if (!set_mode_future.get()->mode_sent) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set OFFBOARD mode.");
        result->success = false;
        goal_handle->abort(result);
        return;
      }
      RCLCPP_INFO(this->get_logger(), "OFFBOARD mode set successfully.");
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
            RCLCPP_INFO(this->get_logger(), "Reached target altitude: %.2f (within 10%% margin)", goal->altitude);
            result->success = true;
            goal_handle->succeed(result);
            return;
        }

        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

  // Handle goal for land
  rclcpp_action::GoalResponse handle_goal_land(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const Land::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received land request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Handle cancellation for land
  rclcpp_action::CancelResponse handle_cancel_land(
      const std::shared_ptr<GoalHandleLand> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Land action cancelled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Handle accepted land goal
  void handle_accepted_land(const std::shared_ptr<GoalHandleLand> goal_handle) {
    std::thread{std::bind(&InterfaceNode::execute_land, this, std::placeholders::_1), goal_handle}.detach();
  }

  // Execute the land action
  void execute_land(const std::shared_ptr<GoalHandleLand> goal_handle) {
    auto feedback = std::make_shared<Land::Feedback>();
    auto result = std::make_shared<Land::Result>();

    RCLCPP_INFO(this->get_logger(), "Initiating landing sequence using MAVROS...");

    // Chamar o serviço de Land do MAVROS
    auto land_client = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/land");
    while (!land_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Waiting for the land service to be available...");
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
            RCLCPP_INFO(this->get_logger(), "Land service called successfully.");
            result->success = true;
            goal_handle->succeed(result);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to land the drone.");
            result->success = false;
            goal_handle->abort(result);
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        result->success = false;
        goal_handle->abort(result);
    }
  }

  // Serviço de comando de movimento
  void handle_move_command(
      const std::shared_ptr<interface_package::srv::MoveCommand::Request> request,
      std::shared_ptr<interface_package::srv::MoveCommand::Response> response) {
    pose_.pose.position.x += request->x;
    pose_.pose.position.y += request->y;
    pose_.pose.position.z += request->z;

    RCLCPP_INFO(this->get_logger(), "Received MoveCommand: [x: %.2f, y: %.2f, z: %.2f]",
                request->x, request->y, request->z);
    response->success = true;
  }

  void control_loop() {
    local_pos_pub_->publish(pose_);
  }

  // Variáveis e objetos
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_position_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp_action::Server<Takeoff>::SharedPtr takeoff_action_server_;
  rclcpp_action::Server<Land>::SharedPtr land_action_server_;
  rclcpp::Service<interface_package::srv::MoveCommand>::SharedPtr move_command_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseStamped pose_;
  geometry_msgs::msg::Point current_position_; 
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
