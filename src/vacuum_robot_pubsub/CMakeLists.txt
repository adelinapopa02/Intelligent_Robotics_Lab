cmake_minimum_required(VERSION 3.8)
project(vacuum_robot_pubsub)

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
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChargeStatus.msg"
)

# This is important for using the generated messages in the same package
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Robot node executable
add_executable(robot_node src/robot_node.cpp)
ament_target_dependencies(robot_node rclcpp)
target_link_libraries(robot_node "${cpp_typesupport_target}")

# Charging station node executable
add_executable(charging_station_node src/charging_station_node.cpp)
ament_target_dependencies(charging_station_node rclcpp)
target_link_libraries(charging_station_node "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  robot_node
  charging_station_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()