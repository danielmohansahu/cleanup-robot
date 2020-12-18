/**
*  @file navigation_test.cpp
 * @brief Implementation of the Navigation test
 *
 * @copyright [2020] <Daniel Sahu, Spencer Elyard, Santosh Kesani>
 */

#pragma once

#include <ros/ros.h>
#include <ros/service_client.h>
#include <navigation/navigation.h>
#include <gtest/gtest.h>

#include <std_srvs/Trigger.h>

#include <navigation/SetPoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <cmath>
#include <thread>

std::unique_ptr<cleanup::Navigation> nav;
std::shared_ptr<ros::NodeHandle> nh;

/**
* @brief Check to ensure able to get current nav mode (default = 0)
*/
TEST(Navigation_Test,getNavModeDefault) {;
  EXPECT_EQ(nav->getCurrNavMode(),0);
}

/**
* @brief Check to ensure the getRobotPose() command works with the launch values
* Note Z height difference from robot config file
*/
TEST(Navigation_Test,GetPose) {
   geometry_msgs::Pose curr_pose = nav->getRobotPose();
   EXPECT_NEAR(curr_pose.position.x,0,1E-5);
   EXPECT_NEAR(curr_pose.position.y,0,1E-5);
   EXPECT_NEAR(curr_pose.position.z,0.01,1E-5);
   EXPECT_NEAR(curr_pose.orientation.x,0,1E-5);
   EXPECT_NEAR(curr_pose.orientation.y,0,1E-5);
   EXPECT_NEAR(curr_pose.orientation.z,0,1E-5);
   EXPECT_NEAR(curr_pose.orientation.w,1,1E-5);
}

/**
* @brief Check to ensure the "Stop" service starts and triggers correct
* nav mode.
*/
TEST(Navigation_Test,stopServiceStarts) {
  ros::ServiceClient client = nh->serviceClient<std_srvs::Trigger>("/navigation/stop");

  // wait for service to become available
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5.0)));

  std_srvs::Trigger srv;
  EXPECT_TRUE(client.call(srv));
  // ros::Duration(1.0).sleep();
  EXPECT_EQ(nav->getCurrNavMode(),0);
}

/**
* @brief Check to ensure the "Explore" service starts and triggers correct
* nav mode.
*/
TEST(Navigation_Test,exploreServiceStarts) {
  ros::ServiceClient client = nh->serviceClient<std_srvs::Trigger>("/navigation/explore");

  // wait for service to become available
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5.0)));

  std_srvs::Trigger srv;
  EXPECT_TRUE(client.call(srv));
  // ros::Duration(1.0).sleep();
  EXPECT_EQ(nav->getCurrNavMode(),1);
}

/**
* @brief Check to ensure the "GoTo" service starts and triggers correct
* nav mode.
*/
TEST(Navigation_Test,gotoServiceStarts) {

  ros::ServiceClient client = nh->serviceClient<navigation::SetPoseStamped>("/navigation/goto");
  // wait for service to become available
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5.0)));

  navigation::SetPoseStamped srv;

  // send goal
  srv.request.pose.header.frame_id = "map";
  srv.request.pose.pose = nav->getRobotPose();

  EXPECT_TRUE(client.call(srv));
  client.call(srv);

  EXPECT_EQ(nav->getCurrNavMode(),2);
}

/**
* @brief Check to ensure the "Explore Loop" starts correctly
*/
TEST(Navigation_Test, exploreLoop) {
  // dummy test - TODO
  // Expect return that exploreLoop is running
  EXPECT_TRUE(true);
}

/**
* @brief Check to ensure the "Stop" function calls correctly
*/
TEST(Navigation_Test, stopCommand) {
  // dummy test - TODO
  // Expect return that vehicle has stopped
  EXPECT_TRUE(true);
}

/**
* @brief Main loop
*/
int main(int argc, char **argv) {
  ros::init(argc,argv, "navigation_test");
  nh.reset(new ros::NodeHandle);
  nav.reset(new cleanup::Navigation);
  testing::InitGoogleTest(&argc, argv);

  // spin of thread to process callbacks
  auto spin_thread = std::thread([](){ros::spin();});

  auto result = RUN_ALL_TESTS();
  ros::shutdown();
  spin_thread.join();

  return result;
}
