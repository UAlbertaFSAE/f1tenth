cmake_minimum_required(VERSION 3.8)
project(waypoint_triangulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Build tool
find_package(ament_cmake REQUIRED)

# C++ dependencies
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rc_interfaces REQUIRED)

include_directories(include)

# ----------------------------
# C++ Node
# ----------------------------
add_executable(triangulator src/triangulator.cpp)
ament_target_dependencies(triangulator
  rclcpp
  nav_msgs
  rc_interfaces
)

target_include_directories(triangulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(triangulator PUBLIC c_std_99 cxx_std_17)

install(TARGETS
  triangulator
  DESTINATION lib/${PROJECT_NAME}
)

# ----------------------------
# Python Node (installed as an executable program)
# ----------------------------
install(PROGRAMS
  src/new_triangulator.py
  DESTINATION lib/${PROJECT_NAME}
)

# ----------------------------
# Launch files (optional but recommended)
# Create a folder: launch/ and put *.launch.py there
# ----------------------------
install(DIRECTORY
  launch
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
