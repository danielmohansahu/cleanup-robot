/* @file navigation_node.cpp
 * @brief ROS Node instantiation fo Navigation
 * 
 * @copyright [2020] <Daniel Sahu, Spencer Elyard, Santosh Kesani>
 */

#include <ros/ros.h>

#include <navigation/navigation.h>

int main(int argc, char** argv) {
  // seed random numbers
  srand( time( NULL ) );

  // start ros node
  ros::init(argc, argv, "navigation_node");

  // instantiate Navigation class
  cleanup::Navigation nav;
  
  // spin until shutdown
  ros::spin();

  return 0;
}