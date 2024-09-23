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

# Gerar interfaces (serviço)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveCommand.srv"
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
)

# Certifique-se de que o executável dependa das interfaces geradas
rosidl_target_interfaces(interface_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Instalação
install(TARGETS
  interface_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
