cmake_minimum_required(VERSION 3.8)
project(safety_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(drive_watchdog src/drive_watchdog.cpp)
ament_target_dependencies(drive_watchdog rclcpp nav_msgs ackermann_msgs)
target_compile_features(drive_watchdog PRIVATE cxx_std_17)

install(TARGETS drive_watchdog DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
