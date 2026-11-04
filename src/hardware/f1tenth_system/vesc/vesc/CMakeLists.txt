cmake_minimum_required(VERSION 3.5)
project(vesc)

# This metapackage has no compilation units.
if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS OFF)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_package()
