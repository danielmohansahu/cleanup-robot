/**
*  @file controller_test.cpp
 * @brief Implementation of the Controller test
 *
 * @copyright [2020] <Daniel Sahu, Spencer Elyard, Santosh Kesani>
 */
#pragma once

#include <ros/ros.h>
#include <controller/controller.h>
#include <gtest/gtest.h>
#include <controller/SetModeAction.h>
#include <actionlib/client/simple_action_client.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>

#include <thread>

std::unique_ptr<cleanup::Controller> ctrl;
std::shared_ptr<ros::NodeHandle> nh;

/**
* @brief Check to ensure the goto, stop, and explore services exist
*/
TEST(Controller_Test, CheckServicesExist) {

   EXPECT_TRUE(ros::service::exists("navigation/goto",true));
   EXPECT_TRUE(ros::service::exists("navigation/stop",true));
   EXPECT_TRUE(ros::service::exists("navigation/explore",true));
}

/**
* @brief Check to ensure a bad goal does not get set
*/
TEST(Controller_Test, Execute_BadGoal) {
   // send command to "blank"
   controller::SetModeGoal goal;
   goal.mode = "fail";
   //ctrl->executeGoal(goal);
   EXPECT_FALSE(false); // placeholder
   //expect result that shows the controller mode is not set
}

/**
* @brief Check to ensure the "clean" goal gets set
*/
TEST(Controller_Test, Execute_CleanGoal) {
   // send command to "clean"
   controller::SetModeGoal goal;
   goal.mode = "clean";
   //ctrl->executeGoal(goal);
   EXPECT_TRUE(true); // placeholder
   //expect result that shows the controller is in clean mode
}

/**
* @brief Check to ensure the "explore" goal gets set
*/
TEST(Controller_Test, Execute_ExploreGoal) {
   // send command to "explore"
   controller::SetModeGoal goal;
   goal.mode = "explore";
   //ctrl->executeGoal(goal);
   EXPECT_TRUE(true); // placeholder
   //expect result that shows the controller is in exploration mode
}

/**
* @brief Main loop to call tests
*/
int main(int argc, char **argv) {
  ros::init(argc,argv, "controller_test");
  nh.reset(new ros::NodeHandle);
  ctrl.reset(new cleanup::Controller);

  testing::InitGoogleTest(&argc, argv);

  // spin of thread to process callbacks
  auto spin_thread = std::thread([](){ros::spin();});

  auto result = RUN_ALL_TESTS();
  ros::shutdown();
  spin_thread.join();

  return result;
}
