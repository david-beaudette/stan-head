cmake_minimum_required(VERSION 3.5)
project(sermonizer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include/sermonizer
  ${CMAKE_CURRENT_SOURCE_DIR}/../../install/include
)

add_executable(sermonizer src/sermonizer.cpp src/serialib.cpp)

ament_target_dependencies(sermonizer 
  "diagnostic_msgs"
  "diagnostic_updater"
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

target_include_directories(sermonizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS sermonizer
  DESTINATION lib/${PROJECT_NAME})

ament_package()
