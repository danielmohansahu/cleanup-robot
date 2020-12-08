/* @file controller.h
 * @brief Header of the Controller class for overall system management.
 * 
 * @copyright [2020] <Daniel Sahu, Spencer Elyard, Santosh Kesani>
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <std_srvs/Trigger.h>
#include <navigation/SetPoseStamped.h>
#include <controller/SetModeAction.h>

namespace cleanup {

class Controller {
 public:
  Controller();

  /* @brief Callback for action server to set a cleanup mode. */
  void executeGoal(const controller::SetModeGoal::ConstPtr& goal);

  private: 
  // navigation service clients
  ros::ServiceClient goto_client_;
  ros::ServiceClient stop_client_;
  ros::ServiceClient explore_client_;

  // perception service clients
  ros::ServiceClient detect_client_;
  ros::ServiceClient get_pose_client_;

  // action server handle
  std::unique_ptr<actionlib::SimpleActionServer<controller::SetModeAction>> as_;
};

} // namespace cleanup
