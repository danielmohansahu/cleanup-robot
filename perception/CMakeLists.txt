cmake_minimum_required(VERSION 3.0.2)
project(perception)

set(CMAKE_CXX_STANDARD 14)

set(BUILD_DEPS
  roscpp
)

find_package(
  catkin REQUIRED
  ${BUILD_DEPS}
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${BUILD_DEPS}
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/perception_node.cpp src/perception.cpp src/matrixf.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(
  perception_test
  test/perception_test.launch
  test/perception_test.cpp
  src/perception.cpp
  )
  target_link_libraries(
  perception_test
  ${catkin_LIBRARIES}
  )
  add_dependencies(
  perception_test
  perception_node
  ${catkin_EXPORTED_TARGETS}
  )
endif()

