#include <perception/perception.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

std::unique_ptr<cleanup::Perception> p;
std::shared_ptr<ros::NodeHandle> nh;
//cv::Mat test_img;

TEST(PerceptionTest_TestGetOutputsNames, should_pass) {

}

// analyze image for objects
TEST(PerceptionTest_TestPredict, should_pass) {
   //std::vector<cv::Rect> p.predict(img);

}

// visual output only - do we need this?
TEST(PerceptionTest_TestPostProcess, should_pass) {

}

TEST(PerceptionTest_TestDetectObjects, should_pass) {

}

TEST(PerceptionTest_TestGetObjectPose, should_pass) {

}

TEST(PerceptionTest_TestImageCallback, should_pass) {

}

TEST(PerceptionTest_TestRunVisionAlgo, should_pass) {

}

int main(int argc, char **argv){

   ros::init(argc,argv, "perception_)test");
   nh.reset(new ros::NodeHandle);
   p.reset(new cleanup::Perception);
   testing::InitGoogleTest(&argc, argv);

   // spin of thread to process callbacks
   auto spin_thread = std::thread([](){ros::spin();});

   auto result = RUN_ALL_TESTS();
   ros::shutdown();
   spin_thread.join();

   return result;
}
