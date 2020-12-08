#include <ros/ros.h>
#include <controller/controller.h>
#include <gtest/gtest.h>

cleanup::Controller ctrl;

TEST(Controller_TestExecute_BadGoal, should_fail) {
   // send command to "blank"
   // TODO: make "goal" message
   //ctrl.executeGoal(controller::SetModeGoal::ConstPtr& goal);
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
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
