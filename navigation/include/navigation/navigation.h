/**
 * @file navigation.h
 * @brief Header of the Navigation class for basic navigation.
 *
 * @copyright [2020] <Daniel Sahu, Spencer Elyard, Santosh Kesani>
 */

#pragma once

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>
#include <navigation/SetPoseStamped.h>

#include <atomic>
#include <future>
#include <memory>
#include <string>

/**
* @brief Namespace for Cleanup Implementation
*/
namespace cleanup {

  /**
  * @brief Implementation for navigation routines in ROS to command a turtlebot rover
  */
class Navigation {
 public:

   /**
   * @brief Constructor
   */
  Navigation();

  /**
  * @brief Destructor
  */
  ~Navigation();

  /**
   * @brief Explore the given area, avoiding obstacles.
   *
   * The basic implementation is just to move in straight lines
   * until an obstacle is encountered, then rotate until the
   * way ahead is clear.
   *
   * This calls stop() implictly.
   */
  void exploreLoop();

  /**
  * @brief Stop any running threads and navigation goals.
  */
  void stop();

  /**
  * @brief Get current robot position.
  * @return Current pose of robot (world position, world rotation)
  */
  geometry_msgs::Pose getRobotPose();

  /**
  * @brief Get integer related to current SetModeGoal
  */
  int getCurrNavMode();

  private:

    /**
    * @brief Integer related to current SetModeGoal
    */
  int currNavMode_ {0};

  /**
  * @brief Action client to move_base navigation manager
  */
  std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> goto_client_;

  /**
  * @brief Listen for current robot position
  */
  std::unique_ptr<tf2_ros::TransformListener> tfl_;

  /**
  * @brief Buffer for current robot position
  */
  std::unique_ptr<tf2_ros::Buffer> tfb_;

  /**
  * @brief "Stop" service - stop the rover at current position
  */
  ros::ServiceServer stop_service_;

  /**
  * @brief "Explore" navigation service - wander the area
  */
  ros::ServiceServer explore_service_;

  /**
  * @brief "GoTo" navigation service - command to a goal position
  */
  ros::ServiceServer goto_service_;

  /**
  * @brief Stop handle for thread handling
  */
  std::atomic<bool> stop_;

  /**
  * @brief Thread handling variable
  */
  std::future<void> thread_handle_;

  /**
  * @brief String for base frame (ROS)
  */
  const std::string base_frame_ {"base_link"};

  /**
  * @brief String for map frame (ROS)
  */
  const std::string map_frame_ {"map"};

  /**
  * @brief String for server name (ROS)
  */
  const std::string server_name_ {"move_base"};

  /**
  * @brief Double for explore step size
  */
  const double explore_step_ {2.0};
};

} // namespace cleanup
