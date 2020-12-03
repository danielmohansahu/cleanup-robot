#include <ros/ros.h>

#include <navigation/navigation.h>

int main(int argc, char** argv) {

  // start ros node
  ros::init(argc, argv, "navigation_node");

  // instantiate Navigation class
  cleanup::Navigation nav;
  
  // spin until shutdown
  ros::spin();

  return 0;
}