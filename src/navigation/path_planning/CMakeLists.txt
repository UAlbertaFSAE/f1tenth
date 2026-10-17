cmake_minimum_required(VERSION 3.8)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Build tool
find_package(ament_cmake REQUIRED)

# C++ dependencies
find_package(rclcpp REQUIRED)
find_package(rc_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(CDT REQUIRED CONFIG)

# C++ Node
add_executable(path_planner
  src/path_planner.cpp
)
ament_target_dependencies(path_planner
  rclcpp
  rc_interfaces
  nav_msgs
  geometry_msgs
  visualization_msgs
)
target_link_libraries(path_planner CDT::CDT)

target_include_directories(path_planner PRIVATE include)
target_compile_features(path_planner PRIVATE cxx_std_17)

install(TARGETS
  path_planner
  DESTINATION lib/${PROJECT_NAME}
)

# Launch files and configs
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../.clang-format")
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(planning_test test/planning_test.cpp)
  ament_target_dependencies(planning_test rclcpp)
  target_include_directories(planning_test PRIVATE include)
  target_compile_features(planning_test PRIVATE cxx_std_17)
  target_link_libraries(planning_test CDT::CDT)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # Use the workspace's clang-format style instead of ROS uncrustify.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
