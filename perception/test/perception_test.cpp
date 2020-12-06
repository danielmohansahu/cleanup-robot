#include <perception/perception.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

//Perception::Perception() p;
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
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
