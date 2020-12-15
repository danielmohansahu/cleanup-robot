/**
 * @file controller.h
 * @brief Header of the Controller class for overall system management.
 *
 * @copyright [2020] <Daniel Sahu, Spencer Elyard, Santosh Kesani>
 */

#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <std_srvs/Trigger.h>
#include <navigation/SetPoseStamped.h>
#include <controller/SetModeAction.h>

#include <memory>

/**
* @brief Namespace for Cleanup Implementation
*/
namespace cleanup {

  /**
  * @brief Implementation for a Controller in ROS to command a turtlebot rover
  */
class Controller {
 public:

     /**
     * @brief Constructor; constructs and initializes parameters of all composition classes.
     */
  Controller();

  /**
  * @brief Callback for action server to set a cleanup mode.
  */
  void executeGoal(const controller::SetModeGoal::ConstPtr& goal);

  private:
    /**
    * @brief "GoTo" navigation service - command to a goal position
    */
  ros::ServiceClient goto_client_;
  /**
  * @brief "StopClient" service - stop the rover at current position
  */
  ros::ServiceClient stop_client_;
  /**
  * @brief "Explore" navigation service - wander the area
  */
  ros::ServiceClient explore_client_;

  /**
  * @brief "Detect" perception service
  */
  ros::ServiceClient detect_client_;
  /**
  * @brief "GetPose" perception service
  */
  ros::ServiceClient get_pose_client_;

  /**
  * @brief Action server handle
  */
  std::unique_ptr<actionlib::SimpleActionServer<controller::SetModeAction>> as_;

  /**
  * @brief Action server feedback
  */
  controller::SetModeFeedback feedback_;
};

} // namespace cleanup
