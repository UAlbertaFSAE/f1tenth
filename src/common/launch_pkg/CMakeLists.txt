cmake_minimum_required(VERSION 3.8)
project(launch_pkg)

find_package(ament_cmake REQUIRED)

# This package ships no compiled code -- it exists to hold the workspace-level
# launch file and the config files that drive it. It is ament_cmake rather than
# ament_python because there is no Python module to install, only data, and
# ament_cmake installs data without the setup.py/resource/ boilerplate.
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
