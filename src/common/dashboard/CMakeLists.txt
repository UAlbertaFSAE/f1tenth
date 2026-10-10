cmake_minimum_required(VERSION 3.8)
project(dashboard)

find_package(ament_cmake REQUIRED)

# Like launch_pkg, this package ships no compiled code: a launch file, its
# config and a static web page. The bridging is done by rosbridge_server and
# topic_tools, so there is nothing of our own to build.
install(
  DIRECTORY launch config web
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
