#include <thread>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <navigation/navigation.h>
#include <gtest/gtest.h>

#include <std_srvs/Trigger.h>

#include <cmath>

#include <navigation/SetPoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>

std::unique_ptr<cleanup::Navigation> nav;
std::shared_ptr<ros::NodeHandle> nh;

TEST(NavigationTest_getNavModeDefault, should_pass) {;
  EXPECT_EQ(nav->getCurrNavMode(),0);
}

TEST(NavigationTest_GetPose, should_pass) {
   geometry_msgs::Pose curr_pose = nav->getRobotPose();
   EXPECT_NEAR(curr_pose.position.x,0,1E-5);
   EXPECT_NEAR(curr_pose.position.y,0,1E-5);
   EXPECT_NEAR(curr_pose.position.z,0.01,1E-5);
   EXPECT_NEAR(curr_pose.orientation.x,0,1E-5);
   EXPECT_NEAR(curr_pose.orientation.y,0,1E-5);
   EXPECT_NEAR(curr_pose.orientation.z,0,1E-5);
   EXPECT_NEAR(curr_pose.orientation.w,1,1E-5);
}

TEST(NavigationTest_stopServiceStarts, should_pass) {
  ros::ServiceClient client = nh->serviceClient<std_srvs::Trigger>("/navigation/stop");

  // wait for service to become available
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5.0)));

  std_srvs::Trigger srv;
  EXPECT_TRUE(client.call(srv));
  //ros::Duration(1.0).sleep();
  EXPECT_EQ(nav->getCurrNavMode(),0);
}

TEST(NavigationTest_exploreServiceStarts, should_pass) {
  ros::ServiceClient client = nh->serviceClient<std_srvs::Trigger>("/navigation/explore");

  // wait for service to become available
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5.0)));

  std_srvs::Trigger srv;
  EXPECT_TRUE(client.call(srv));
  //ros::Duration(1.0).sleep();
  EXPECT_EQ(nav->getCurrNavMode(),1);
}

TEST(NavigationTest_gotoServiceStarts, should_pass) {

  ros::ServiceClient client = nh->serviceClient<navigation::SetPoseStamped>("/navigation/goto");
  // wait for service to become available
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5.0)));

  navigation::SetPoseStamped srv;

  // send goal
  geometry_msgs::Pose pose = nav->getRobotPose();

  //move_base_msgs::MoveBaseGoal goal;
  //goal.target_pose.pose = pose;
  //goal.target_pose.header.frame_id = "map";
  //goal.target_pose.header.stamp = ros::Time::now();

  //srv.request.pose = goal;

  EXPECT_TRUE(client.call(srv));
  client.call(srv);
  //ros::Duration(1.0).sleep();
  EXPECT_EQ(nav->getCurrNavMode(),2);
}

TEST(NavigationTest_exploreLoop, should_pass) {
  EXPECT_TRUE(false);
}

int main(int argc, char **argv){
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
