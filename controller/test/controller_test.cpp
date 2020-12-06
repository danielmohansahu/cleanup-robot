#include <ros/ros.h>
#include <controller/controller.h>
#include <gtest/gtest.h>

//cleanup::Controller controller;
//cleanup::Controller::SetModeGoal::ConstPtr& goal;

TEST(ControllerTest, TestExplore) {
   //goal->mode = "explore";

}

int main(int argc, char **argv){
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
