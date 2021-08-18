cmake_minimum_required(VERSION 3.5)
project(actionizer)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(DEPENDENCIES
  roscpp
  std_msgs
  sensor_msgs
  stan_common
)

find_package(catkin REQUIRED COMPONENTS
  ${DEPENDENCIES}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${DEPENDENCIES}
)

add_executable(joy_actionizer src/joy_actionizer.cpp)
target_link_libraries(joy_actionizer ${catkin_LIBRARIES})
add_dependencies(joy_actionizer stan_common_generate_messages_cpp)

install(TARGETS joy_actionizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

