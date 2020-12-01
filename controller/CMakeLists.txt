cmake_minimum_required(VERSION 3.0.2)
project(controller)

set(CMAKE_CXX_STANDARD 14)

set(BUILD_DEPS
  roscpp
)

find_package(
  catkin REQUIRED
  ${BUILD_DEPS}
)

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

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_navigation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
