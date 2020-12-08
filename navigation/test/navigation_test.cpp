#include <ros/ros.h>
#include <navigation/navigation.h>
#include <gtest/gtest.h>
#include <cmath>

cleanup::Navigation nav;

TEST(NavigationTest_GetPose, should_pass) {
    /*
    // expect to initialize to initial position, initial orientation
    geometry_msgs::Pose curr_pose = getRobotPose();
    EXPECT_FLOAT_EQ(curr_pose.position.x,0);
    EXPECT_FLOAT_EQ(curr_pose.position.y,0);
    EXPECT_FLOAT_EQ(curr_pose.position.z,0);
    EXPECT_FLOAT_EQ(curr_pose.orientation.x,0);
    EXPECT_FLOAT_EQ(curr_pose.orientation.y,0);
    EXPECT_FLOAT_EQ(curr_pose.orientation.z,0);
    EXPECT_FLOAT_EQ(curr_pose.orientation.w,1);
    */
}

TEST(NavigationTest_TestExplore, should_pass) {
   /*
    // expect move in straight line; in empty world will continue indefinitely
    geometry_msgs::Pose init_pose = nav.getRobotPose();
    // command to explore
    nav.exploreLoop();
    // let move
    ros::Duration(2.0).sleep();

    // expect to be in a different position than before
    float new_pos_dist_travel = sqrt(
      pow(init_pose.position.x - nav.getRobotPose().position.x,2) +
      pow(init_pose.position.y - nav.getRobotPose().position.y,2));
    // should have moved
    EXPECT_TRUE(new_pos_dist_travel > 0);
    */
}


TEST(NavigationTest_TestStop, should_pass) {
  /*
  nav.exploreLoop();
  // let move
  ros::Duration(2.0).sleep();
  // record current position
  geometry_msgs::Pose init_pose = nav.getRobotPose();
  // command to stop
  nav.stop();
  // sleep to ensure robot stopped
  ros::Duration(1.0).sleep();
  /// compare new position to old position
  float new_pos_dist_travel = sqrt(
    pow((init_pose.position.x - nav.getRobotPose().position.x),2) +
    pow((init_pose.position.y - nav.getRobotPose().position.y),2));
  // should not have moved
  EXPECT_FLOAT_EQ(new_pos_dist_travel,0);
  */
}

int main(int argc, char **argv){
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
