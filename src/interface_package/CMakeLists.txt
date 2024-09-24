cmake_minimum_required(VERSION 3.8)
project(interface_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontrar as dependências
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)

# Gerar interfaces (serviço e action)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveCommand.srv"
  "action/NavigateWaypoints.action"
  "action/Takeoff.action"
  "action/Land.action"
  DEPENDENCIES geometry_msgs  # Adicionar a dependência necessária
)

# Adicionar executável
add_executable(interface_node src/interface_node.cpp)


# Linkar as dependências ao executável
ament_target_dependencies(interface_node 
  rclcpp 
  std_msgs 
  geometry_msgs 
  mavros_msgs 
  sensor_msgs
  rclcpp_action
)

# Certifique-se de que o executável dependa das interfaces geradas
rosidl_get_typesupport_target(interface_package__rosidl_typesupport_cpp
  interface_package rosidl_typesupport_cpp)

target_link_libraries(interface_node
  interface_package__rosidl_typesupport_cpp)

# Instalação
install(TARGETS
  interface_node
  DESTINATION lib/${PROJECT_NAME})

# Certifique-se de que as interfaces sejam instaladas corretamente
install(
  DIRECTORY srv action
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
