#include <ros/ros.h>
#include <navigation/navigation.h>
#include <gtest/gtest.h>
#include <cmath>

cleanup::Navigation nav;
std::shared_ptr<ros::NodeHandle> nh;

TEST(NavigationTest_getNavModeDefault, should_pass) {;
  EXPECT_EQ(nav.getCurrNavMode(),0);
}

TEST(NavigationTest_GetPose, should_pass) {
   geometry_msgs::Pose curr_pose = nav.getRobotPose();
   EXPECT_FLOAT_EQ(curr_pose.position.x,0);
   EXPECT_FLOAT_EQ(curr_pose.position.y,0);
   EXPECT_FLOAT_EQ(curr_pose.position.z,0);
   EXPECT_FLOAT_EQ(curr_pose.orientation.x,0);
   EXPECT_FLOAT_EQ(curr_pose.orientation.y,0);
   EXPECT_FLOAT_EQ(curr_pose.orientation.z,0);
   EXPECT_FLOAT_EQ(curr_pose.orientation.w,1);
}

TEST(NavigationTest_stopServiceStarts, should_pass) {
  ros::ServiceClient client = nh->serviceClient<cleanup::Navigation>("/navigation/stop");
  EXPECT_EQ(nav.getCurrNavMode(),0);
}

TEST(NavigationTest_exploreServiceStarts, should_pass) {
  ros::ServiceClient client = nh->serviceClient<cleanup::Navigation>("/navigation/explore");
  EXPECT_EQ(nav.getCurrNavMode(),1);
}

TEST(NavigationTest_gotoServiceStarts, should_pass) {
  ros::ServiceClient client = nh->serviceClient<cleanup::Navigation>("/navigation/goto");
  EXPECT_EQ(nav.getCurrNavMode(),2);
}

int main(int argc, char **argv){
   ros::init(argc,argv, "navigation_test");
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
