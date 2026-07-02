cmake_minimum_required(VERSION 3.5)
project(rs_converter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

add_executable(rs_converter src/rs_converter.cpp)
target_link_libraries(rs_converter ${PCL_LIBRARIES})
ament_target_dependencies(rs_converter
  rclcpp
  sensor_msgs
  pcl_conversions
)

install(TARGETS rs_converter
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
