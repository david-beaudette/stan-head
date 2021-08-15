cmake_minimum_required(VERSION 3.5)
project(sermonizer)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(DEPENDENCIES
  diagnostic_updater
  stan_common
)

find_package(catkin REQUIRED COMPONENTS
  ${DEPENDENCIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${DEPENDENCIES}
)

add_executable(sermonizer src/sermonizer.cpp src/serialib.cpp)
target_link_libraries(sermonizer ${catkin_LIBRARIES})
add_dependencies(sermonizer stan_common_generate_messages_cpp)

install(TARGETS sermonizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)