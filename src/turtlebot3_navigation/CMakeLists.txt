cmake_minimum_required(VERSION 3.8)
project(turtlebot3_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(turtlebot3_interfaces REQUIRED)

include_directories(include)

# Add executables and link dependencies
add_executable(wall_finder_server src/wall_finder_server.cpp)
ament_target_dependencies(wall_finder_server rclcpp sensor_msgs geometry_msgs turtlebot3_interfaces)

add_executable(robot_driver src/robot_driver.cpp)
ament_target_dependencies(robot_driver rclcpp sensor_msgs geometry_msgs turtlebot3_interfaces)

add_executable(lap_time_server src/lap_time_server.cpp)
ament_target_dependencies(lap_time_server rclcpp sensor_msgs geometry_msgs rclcpp_action nav_msgs turtlebot3_interfaces)

add_executable(lap_time_client src/lap_time_client.cpp)
ament_target_dependencies(lap_time_client rclcpp sensor_msgs geometry_msgs rclcpp_action nav_msgs turtlebot3_interfaces)

install(TARGETS
  wall_finder_server
  robot_driver
  lap_time_server
  lap_time_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

