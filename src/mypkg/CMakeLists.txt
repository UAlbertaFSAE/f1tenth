cmake_minimum_required(VERSION 3.8)
project(mypkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(mynode src/mynode.cpp)
target_include_directories(mynode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mynode PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS mynode
  DESTINATION lib/${PROJECT_NAME})

ament_package()
