#include <ros/ros.h>
// #include <darknet_ros/YoloObjectDetector.hpp>
#include <darknet_ros_msgs/BoundingBoxes.h>
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
#include <geometry_msgs/PoseStamped.h>
#include <perception/matrixf.hpp>
#include <perception/ObjectLocations.h>

namespace cleanup {

typedef struct {
  int id;
  std::vector<std::vector<double>> p;
  int d;
}object_loc;

class Perception {
 public:

  /* @brief Constructor */
  Perception();

  void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg);
  /**
   * @brief      Function to store the detected objects
   *
   * @return     vector of ids
   */
  // std::vector<int> detectObjects();
  /**
   * @brief      Function to known the object's pose
   *
   * @return     The object pose.
   */
  // geometry_msgs::PoseStamped getObjectPose();

  std::vector<std::vector<double>> getpose(const double& u, const double& v);
  /**
   * @brief      Image callback Function
   */
  void imageCallback();
  /**
   * @brief      Robust vision algorithm to detect objects  
   *             and obstacles
   */
  void runVisionAlgo();

  void boundingBoxesCallback(const darknet_ros_msgs::BoundingBoxes& bboxes);

  void objectCountCallback(const std_msgs::Int8& msg);

 private:
  ros::NodeHandle nh;
  image_transport::ImageTransport it_;
  image_transport::Subscriber depth_image_sub_;
  ros::Subscriber boudingBoxesSubcriber_;
  ros::Subscriber objectCountSubcriber_;
  ros::Publisher objectLocation;
  int depth;
  int u,v;
  MatrixF mf;
  std_msgs::Int8 objectCount;
  std::vector<object_loc> location_array;
  object_loc objL;

  std::vector<std::vector<double>> pose;
  std::vector<std::pair<int, geometry_msgs::PoseStamped>> objects;
  darknet_ros_msgs::BoundingBox bbox;
};

} // namespace cleanup
