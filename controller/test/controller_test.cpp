#include <ros/ros.h>
#include <controller/controller.h>
#include <gtest/gtest.h>
#include <controller/SetModeAction.h>

std::unique_ptr<cleanup::Controller> ctrl;
std::shared_ptr<ros::NodeHandle> nh;

TEST(Controller_TestExecute_BadGoal, should_fail) {
   // send command to "blank"
   // TODO: make "goal" message
   //ctrl.executeGoal(controller::SetModeGoal::ConstPtr& goal);
   Controller::SetModeGoal goal;
   goal.mode = "errormode";
   Controller::executeGoal(goal&);
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
