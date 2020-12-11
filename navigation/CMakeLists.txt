cmake_minimum_required(VERSION 3.0.2)
project(navigation)

set(CMAKE_CXX_STANDARD 14)

set(BUILD_DEPS
  roscpp
  geometry_msgs
  turtlebot3_navigation
  gmapping
  move_base
  dwa_local_planner
  message_generation
  message_runtime
)

set(TEST_DEPS
  ${PACKAGE_DEPS}
  turtlebot3_description
  turtlebot3_gazebo
  gazebo
  xacro
  robot_state_publisher
  rviz
)

find_package(
  catkin REQUIRED
  ${BUILD_DEPS}
)

########################
## message generation ##
########################

add_service_files(
  FILES
  SetPoseStamped.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${BUILD_DEPS}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/navigation_node.cpp src/navigation.cpp)
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
  navigation_test
  test/navigation_test.launch
  test/navigation_test.cpp
  src/navigation.cpp
  )
  target_link_libraries(
  navigation_test
  ${catkin_LIBRARIES}
  )
  add_dependencies(
  navigation_test
  navigation_node
  ${catkin_EXPORTED_TARGETS}
  )
endif()
