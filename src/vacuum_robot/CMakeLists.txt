cmake_minimum_required(VERSION 3.8)
project(vacuum_robot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages AND actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChargeStatus.msg"
  "action/ChargeBattery.action"
  DEPENDENCIES std_msgs
)

# This is important for using the generated messages/actions in the same package
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Robot node executable (action server)
add_library(robot_action_server SHARED
  src/robot_action_server.cpp)
target_include_directories(robot_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(robot_action_server 
  rclcpp 
  rclcpp_action 
  rclcpp_components
  std_msgs)
target_link_libraries(robot_action_server "${cpp_typesupport_target}")
rclcpp_components_register_node(robot_action_server 
  PLUGIN "vacuum_robot::RobotActionServer" 
  EXECUTABLE robot_node)

# Charging station node executable (action client)
add_library(charging_station_action_client SHARED
  src/charging_station_action_client.cpp)
target_include_directories(charging_station_action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(charging_station_action_client 
  rclcpp 
  rclcpp_action 
  rclcpp_components
  std_msgs)
target_link_libraries(charging_station_action_client "${cpp_typesupport_target}")
rclcpp_components_register_node(charging_station_action_client 
  PLUGIN "vacuum_robot::ChargingStationActionClient" 
  EXECUTABLE charging_station_node)

# Install executables
install(TARGETS
  robot_action_server
  charging_station_action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()