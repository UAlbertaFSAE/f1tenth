cmake_minimum_required(VERSION 3.8)
project(image_generator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
foreach(PACKAGE
  ament_cmake
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)
  find_package(${PACKAGE} REQUIRED)
endforeach()

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(image_generator src/image_generator.cpp)

ament_target_dependencies(image_generator
  rclcpp std_msgs sensor_msgs cv_bridge OpenCV
)

# Install Cpp executables
install(TARGETS
  image_generator
  DESTINATION lib/${PROJECT_NAME})

ament_package()
