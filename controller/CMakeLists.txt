cmake_minimum_required(VERSION 3.0.2)
project(controller)

set(CMAKE_CXX_STANDARD 14)

set(BUILD_DEPS
  roscpp
  genmsg
  actionlib_msgs
  actionlib
  navigation
  perception
)

find_package(
  catkin REQUIRED
  ${BUILD_DEPS}
)

#######################
## action generation ##
#######################

add_action_files(DIRECTORY action FILES SetMode.action)
generate_messages(DEPENDENCIES actionlib_msgs)

###################################
## catkin specific configuration ##
###################################

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${BUILD_DEPS}
#  DEPENDS system_lib
)

###########
## Build ##
###########

add_executable(${PROJECT_NAME}_node src/controller_node.cpp src/controller.cpp)
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
  controller_test
  test/controller_test.launch
  test/controller_test.cpp
  src/controller.cpp
  )

  target_link_libraries(
  controller_test
  ${catkin_LIBRARIES}
  )

  add_dependencies(
  controller_test
  controller_node
  ${catkin_EXPORTED_TARGETS}
  )
endif()
