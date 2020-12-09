cmake_minimum_required(VERSION 3.0.2)
project(controller)

set(CMAKE_CXX_STANDARD 14)

set(BUILD_DEPS
  roscpp
  rviz
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

######################
## rviz panel setup ##
######################

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
  src/user_panel.cpp
)
add_library(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

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
