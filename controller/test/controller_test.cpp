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

#include <thread>

std::unique_ptr<cleanup::Controller> ctrl;
std::shared_ptr<ros::NodeHandle> nh;
// actionlib::SimpleActionClient<controller::SetModeAction> client_("controller/set_mode",true);

TEST(Controller_TestServicesExist, should_pass) {

   // EXPECT_TRUE(ros::service::exists("navigation/goto",true));
   // EXPECT_TRUE(ros::service::exists("navigation/stop",true));
   // EXPECT_TRUE(ros::service::exists("navigation/explore",true));
}

TEST(Controller_TestActionClient_BadGoal, should_fail) {
   // send command to "blank"
   // TODO: make "goal" message
   // ctrl.executeGoal(controller::SetModeGoal::ConstPtr& goal);

   // construct goal object
   // controller::SetModeGoal goal;
   // goal.mode = "fail";
   // client_.sendGoal(goal);

}

TEST(Controller_TestExecute_CleanGoal, should_pass) {
   // send command to "clean"
   // make "goal" message
   // ctrl.executeGoal(controller::SetModeGoal::ConstPtr& goal);
}
TEST(Controller_TestExecute_ExploreGoal, should_pass) {
   // send command to "explore"
   // make "goal" message
   // ctrl.executeGoal(controller::SetModeGoal::ConstPtr& goal);
}


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
