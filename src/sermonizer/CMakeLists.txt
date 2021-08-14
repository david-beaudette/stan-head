cmake_minimum_required(VERSION 3.5)
project(sermonizer)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  stan_common
)

include_directories(
  include/sermonizer
  ${catkin_INCLUDE_DIRS}
)

add_executable(sermonizer src/sermonizer.cpp src/serialib.cpp)
target_link_libraries(sermonizer ${catkin_LIBRARIES})
