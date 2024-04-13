cmake_minimum_required(VERSION 3.8)
project(waypoint_triangulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
foreach(PACKAGE
  rclcpp
  ament_cmake
  rc_interfaces
)
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

add_executable(triangulator src/triangulator.cpp)
ament_target_dependencies(triangulator
    rc_interfaces
    rclcpp
)

install(TARGETS triangulator
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(triangulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(triangulator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
