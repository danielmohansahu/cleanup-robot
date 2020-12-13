/* @file controller_node.cpp
 * @brief Instantiation of the Controller class for overall system management.
 *
 * @copyright [2020] <Daniel Sahu, Spencer Elyard, Santosh Kesani>
 */

#include <ros/ros.h>

#include <controller/controller.h>

int main(int argc, char** argv) {
  // initialize ros node
  ros::init(argc, argv, "controller");

  // construct controller
  cleanup::Controller controller;

  // process callbacks until shutdown
  ros::spin();
}
