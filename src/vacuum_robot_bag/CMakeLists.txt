cmake_minimum_required(VERSION 3.8)
project(vacuum_robot_bag)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Add executable
add_executable(vacuum_tracker src/vacuum_tracker.cpp)

# Specify dependencies
ament_target_dependencies(vacuum_tracker
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
)

# Install targets
install(TARGETS
  vacuum_tracker
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()