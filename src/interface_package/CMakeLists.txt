cmake_minimum_required(VERSION 3.8)
project(interface_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontrar as dependências
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# Gerar interfaces (serviço e action)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveCommand.srv"
  "action/NavigateWaypoints.action"
  "action/Takeoff.action"
  "action/Land.action"
  DEPENDENCIES geometry_msgs
)

# Tornar as interfaces geradas disponíveis para outras dependências
ament_export_dependencies(rosidl_default_runtime)

# Adicionar executável (inclua todos os arquivos .cpp necessários)
add_executable(interface_node
  src/interface_node.cpp
  src/takeoff_action.cpp
  src/land_action.cpp
  src/navigate_waypoints_action.cpp
  src/move_command_service.cpp
)

# Linkar as dependências ao executável
ament_target_dependencies(interface_node 
  rclcpp 
  rclcpp_action
  std_msgs 
  geometry_msgs 
  mavros_msgs 
  builtin_interfaces
)

# Vincular as interfaces geradas ao executável
rosidl_target_interfaces(interface_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Instalação dos executáveis
install(TARGETS
  interface_node
  DESTINATION lib/${PROJECT_NAME}
)

# Instalação dos arquivos de interface (srv e action)
install(
  DIRECTORY srv action
  DESTINATION share/${PROJECT_NAME}
)

# Instalação dos arquivos de cabeçalho
install(
  DIRECTORY src/include/
  DESTINATION src/include/
)

ament_package()
