cmake_minimum_required(VERSION 3.5)
project(actionizer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stan_common REQUIRED)

include_directories(
  include/actionizer
  ${CMAKE_CURRENT_SOURCE_DIR}/../../install/include
)

add_executable(joy_actionizer src/joy_actionizer.cpp)

ament_target_dependencies(joy_actionizer 
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "stan_common"
)

target_include_directories(joy_actionizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS joy_actionizer
  DESTINATION lib/${PROJECT_NAME})

ament_package()
