#include <ros/ros.h>
#include <controller/controller.h>
#include <gtest/gtest.h>
#include <controller/SetModeAction.h>
#include <actionlib/client/simple_action_client.h>

std::shared_ptr<ros::NodeHandle> nh;
std::unique_ptr<cleanup::Controller> ctrl;
actionlib::SimpleActionClient<controller::SetModeAction> client_;

TEST(Controller_TestServicesExist, should_pass) {
   //EXPECT_TRUE(ros::service::exists("navigation/goto",true));
   //EXPECT_TRUE(ros::service::exists("navigation/stop",true));
   //EXPECT_TRUE(ros::service::exists("navigation/explore",true));
}

TEST(Controller_TestActionClient_BadGoal, should_fail) {
   // send command to "blank"
   // TODO: make "goal" message
   //ctrl.executeGoal(controller::SetModeGoal::ConstPtr& goal);

   // construct goal object
   cleanup::controller::SetModeGoal goal;
   goal.mode = "fail";
   client_.sendGoal(goal);
   //actionlib::SimpleActionClient<cleanup::Controller> ac("controller/set_mode",true);
   //controller::SetModeGoal test_goal;
   //test_goal.mode = "errormode";
   //ac.sendGoal(test_goal);
}

TEST(Controller_TestExecute_CleanGoal, should_pass) {
   // send command to "clean"
   // make "goal" message
   //ctrl.executeGoal(controller::SetModeGoal::ConstPtr& goal);
}
TEST(Controller_TestExecute_ExploreGoal, should_pass) {
   // send command to "explore"
   // make "goal" message
   //ctrl.executeGoal(controller::SetModeGoal::ConstPtr& goal);
}


int main(int argc, char **argv){
  ros::init(argc,argv, "controller_test");
  nh.reset(new ros::NodeHandle);
  ctrl.reset(new cleanup::Controller);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
