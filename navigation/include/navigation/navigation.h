#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

namespace cleanup {

class Navigation {
 public:

  /* @brief Constructor */
  Navigation();

  /* @brief Go to the desired position */
  void goTo(const geometry_msgs::PoseStamped& pose);

  /* @brief Explore the given area, avoiding obstacles.
   * 
   * The basic implementation is just to move in straight lines
   * until an obstacle is encountered, then rotate until the
   * way ahead is clear.
   * 
   * This calls stop() implictly.
   */
  void explore();

  /* @brief Stop exploring. */
  void stop();
};

} // namespace cleanup