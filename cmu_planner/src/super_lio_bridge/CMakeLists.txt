cmake_minimum_required(VERSION 3.8)
project(super_lio_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(super_lio_bridge src/super_lio_bridge.cpp)
target_include_directories(super_lio_bridge PRIVATE include)
ament_target_dependencies(super_lio_bridge
  rclcpp
  nav_msgs
  sensor_msgs
  tf2_ros
)

install(TARGETS super_lio_bridge
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
