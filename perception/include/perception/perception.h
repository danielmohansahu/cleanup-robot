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