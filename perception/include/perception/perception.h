#pragma once

#include "perception/yolo_ros.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <std_msgs/Int8.h>
#include <math.h>
#include <darknet_ros/bbox_array.h>
#include <darknet_ros/bbox.h>
#include <geometry_msgs/PoseStamped.h>

namespace cleanup {

class Perception {
 public:

  /* @brief Constructor */
  Perception();

  ~Perception();

  // std::vector<int> detectObjects();
  /**
   * @brief      Function to known the object's pose
   *
   * @return     The object pose.
   */
  geometry_msgs::PoseStamped getObjectPose();
  /**
   * @brief      Image callback Function
   */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  /**
   * @brief      Robust vision algorithm to detect objects  
   *             and obstacles
   */

  void drawBBoxes(cv::Mat &input_frame, std::vector<cv::Rect> &classBboxes, int &classObjCount,\
            cv::Scalar &bboxColor, const std::string &classLabels);

  void runVisionAlgo();

 private:
  ros::NodeHandle nh;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub;
  ros::Publisher objectFound;
  ros::Publisher bboxes;

  Yolo od;
  
  const std::string OPENCV_WINDOW = "YOLO object detection";
  int numClasess;
  std::vector<cv::Scalar> bboxColors;
  std::vector<std::string> classLabels;
  ros::Publisher objectFound;
  ros::Publisher bboxes;
  darknet_ros::bbox_array bboxResults;
  double frameWidth;
  double frameHeight;

  cv::Mat image_to_detect;

  std::vector<std::pair<int, geometry_msgs::PoseStamped>> objects;
};

} // namespace cleanup