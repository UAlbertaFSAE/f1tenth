cmake_minimum_required(VERSION 3.8)
project(waypoint_triangulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Build tool
find_package(ament_cmake REQUIRED)

# C++ dependencies
find_package(rclcpp REQUIRED)
find_package(rc_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(CDT REQUIRED CONFIG)

include_directories(include)

# ----------------------------
# C++ Node
# ----------------------------
add_executable(triangulator src/triangulator.cpp)
ament_target_dependencies(triangulator
  rclcpp
  rc_interfaces
  geometry_msgs
  visualization_msgs
)

if(TARGET CDT::CDT)
  target_link_libraries(triangulator CDT::CDT)
endif()

add_executable(waypoint_visualizer src/waypoint_visualizer.cpp)
ament_target_dependencies(waypoint_visualizer
  rclcpp
  geometry_msgs
  visualization_msgs
)

target_include_directories(triangulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(triangulator PUBLIC c_std_99 cxx_std_17)

install(TARGETS
  triangulator
  waypoint_visualizer
  DESTINATION lib/${PROJECT_NAME}
)


# ----------------------------
# Launch files (optional but recommended)
# Create a folder: launch/ and put *.launch.py there
# ----------------------------
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
