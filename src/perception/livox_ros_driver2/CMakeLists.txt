# Copyright(c) 2020 livoxtech limited.

cmake_minimum_required(VERSION 3.14)
project(livox_ros_driver2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake/modules")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# Print version info
include(cmake/version.cmake)
project(${PROJECT_NAME} VERSION ${LIVOX_ROS_DRIVER2_VERSION} LANGUAGES CXX)
message(STATUS "${PROJECT_NAME} version: ${LIVOX_ROS_DRIVER2_VERSION}")

# ROS2 only build
add_definitions(-DBUILDING_ROS2)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(PCL REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# check apr
find_package(PkgConfig)
pkg_check_modules(APR apr-1)
if(APR_FOUND)
  message(${APR_INCLUDE_DIRS})
  message(${APR_LIBRARIES})
endif()

# generate custom msg headers
set(LIVOX_INTERFACES livox_interfaces2)
rosidl_generate_interfaces(${LIVOX_INTERFACES}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES builtin_interfaces std_msgs
  LIBRARY_NAME ${PROJECT_NAME}
)

# Locate the Livox SDK this driver links against.
#
# The SDK is vendored in this workspace as livox_sdk2 and colcon installs it to
# <ws>/install/livox_sdk2. A copy of an older Livox SDK is also commonly left
# behind in /usr/local by upstream's own install steps, and the unhinted
# find_path this used to be picked that one up. The result was a build that got
# all the way to compiling before dying on:
#
#   error: 'kLivoxLidarTypeMid360s' is not a member of 'LivoxLidarDeviceType'
#
# because the MID360s device type only exists in the newer vendored header.
# Worse, headers and library could be resolved from different SDKs, which links
# cleanly and then misbehaves at runtime.
#
# HINTS are searched ahead of the platform paths, so listing the workspace
# install space (and the vendored tree behind it) here makes the in-workspace
# SDK win, while /usr/local still works as a fallback on a machine that only
# has the system install.
# colcon installs each package to <ws>/install/<package>, so the sibling of our
# own install prefix is where livox_sdk2's lib/ and include/ land. That is the
# one location guaranteed to hold a matching pair, and it is derived from CMake
# state rather than from COLCON_PREFIX_PATH, which is not exported into the
# configure step. The vendored source tree is listed last: it supplies headers
# on a from-scratch configure, but never a library, so it must not outrank the
# install space.
set(LIVOX_SDK2_SEARCH_PATHS "${CMAKE_INSTALL_PREFIX}/../livox_sdk2")
foreach(_livox_prefix $ENV{COLCON_PREFIX_PATH} ${CMAKE_PREFIX_PATH})
  list(APPEND LIVOX_SDK2_SEARCH_PATHS "${_livox_prefix}/livox_sdk2" "${_livox_prefix}")
endforeach()
list(APPEND LIVOX_SDK2_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../livox_sdk2")

find_library(LIVOX_LIDAR_SDK_LIBRARY
  NAMES livox_lidar_sdk_shared
  HINTS ${LIVOX_SDK2_SEARCH_PATHS}
  PATH_SUFFIXES lib
  REQUIRED
)

find_path(LIVOX_LIDAR_SDK_INCLUDE_DIR
  NAMES "livox_lidar_api.h" "livox_lidar_def.h"
  HINTS ${LIVOX_SDK2_SEARCH_PATHS}
  PATH_SUFFIXES include
  REQUIRED
)

message(STATUS "Livox SDK library: ${LIVOX_LIDAR_SDK_LIBRARY}")
message(STATUS "Livox SDK headers: ${LIVOX_LIDAR_SDK_INCLUDE_DIR}")

# PCL library
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# livox ros2 driver target
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/livox_ros_driver2.cpp
  src/lddc.cpp
  src/driver_node.cpp
  src/lds.cpp
  src/lds_lidar.cpp

  src/comm/comm.cpp
  src/comm/ldq.cpp
  src/comm/semaphore.cpp
  src/comm/lidar_imu_data_queue.cpp
  src/comm/cache_index.cpp
  src/comm/pub_handler.cpp

  src/parse_cfg_file/parse_cfg_file.cpp
  src/parse_cfg_file/parse_livox_lidar_cfg.cpp

  src/call_back/lidar_common_callback.cpp
  src/call_back/livox_lidar_callback.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE ${livox_sdk_INCLUDE_DIRS})

# ROS2 Humble-only typesupport target.
rosidl_get_typesupport_target(cpp_typesupport_target
  ${LIVOX_INTERFACES} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME} "${cpp_typesupport_target}")

target_include_directories(${PROJECT_NAME} PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${APR_INCLUDE_DIRS}
  ${LIVOX_LIDAR_SDK_INCLUDE_DIR}
  3rdparty
  src
)

target_link_libraries(${PROJECT_NAME}
  ${LIVOX_LIDAR_SDK_LIBRARY}
  ${PPT_LIBRARY}
  ${Boost_LIBRARY}
  ${PCL_LIBRARIES}
  ${APR_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "livox_ros::DriverNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch_ROS2
)
