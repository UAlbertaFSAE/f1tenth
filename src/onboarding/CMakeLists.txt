cmake_minimum_required(VERSION 3.8)
project(onboarding)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
foreach(PACKAGE ament_cmake rclcpp ackermann_msgs)
  find_package(${PACKAGE} REQUIRED)
endforeach()

# add include directory so cmake knows where our header files are
include_directories(include)

# make the executables for running nodes
add_executable(odom_publisher src/odom_publisher.cpp)
ament_target_dependencies(odom_publisher
  rclcpp ackermann_msgs
)
add_executable(odom_relay src/odom_relay.cpp)
ament_target_dependencies(odom_relay
  rclcpp ackermann_msgs
)

install(TARGETS
  odom_publisher
  odom_relay
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
