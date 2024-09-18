cmake_minimum_required(VERSION 3.8)
project(detection_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
foreach(PACKAGE
  rclcpp
  ament_cmake
  rc_interfaces
  nav_msgs
  Eigen3
)
  find_package(${PACKAGE} REQUIRED)
endforeach()

# include directories
include_directories(include)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

# executables
add_executable(detection_generator src/detection_generator.cpp)

ament_target_dependencies(detection_generator
    rclcpp
    rc_interfaces
    nav_msgs
)

# install cpp exectuables
install(TARGETS
    detection_generator
    DESTINATION lib/${PROJECT_NAME}
)


target_include_directories(detection_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(detection_generator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS detection_generator
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
