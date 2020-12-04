#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

namespace cleanup {

class Perception {
 public:

  /* @brief Constructor */
  Perception();
  /**
   * @brief      Function to store the detected objects
   *
   * @return     vector of ids
   */

  std::vector<cv::String> Perception::getOutputsNames(const cv::dnn::Net& net);

  std::vector<cv::Rect> Perception::predict(cv::Mat frame);

  void Perception::postProcess(cv::Mat& frame, const std::vector<cv::Mat>& preds, \
          std::vector<int> class_ids, std::vector<float> confidences,\
                           std::vector<int> indices);

  std::vector<int> detectObjects();
  /**
   * @brief      Function to known the object's pose
   *
   * @return     The object pose.
   */
  geometry_msgs::PoseStamped getObjectPose();
  /**
   * @brief      Image callback Function
   */
  void imageCallback();
  /**
   * @brief      Robust vision algorithm to detect objects  
   *             and obstacles
   */
  void runVisionAlgo();

 private:
  std::vector<std::pair<int, geometry_msgs::PoseStamped>> objects;
};

} // namespace cleanup