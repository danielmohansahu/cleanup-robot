/* @file navigation.h
 * @brief Header of the Navigation class for basic navigation.
 *
 * @copyright [2020] <Daniel Sahu, Spencer Elyard, Santosh Kesani>
 */

#pragma once

#include <atomic>
#include <future>

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

namespace cleanup {

class Navigation {
 public:

  /* @brief Constructor */
  Navigation();

  /* @brief Destructor */
  ~Navigation();

  /* @brief Explore the given area, avoiding obstacles.
   *
   * The basic implementation is just to move in straight lines
   * until an obstacle is encountered, then rotate until the
   * way ahead is clear.
   *
   * This calls stop() implictly.
   */
  void exploreLoop();

  /* @brief Stop any running threads and navigation goals. */
  void stop();

  /* @brief Get current robot position. */
  geometry_msgs::Pose getRobotPose();

  /* @brief get stop_ value */
  int getCurrNavMode();

  private:

  int currNavMode = 0;

  // action client to move_base navigation manager
  std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> goto_client_;

  // tf objects (for accessing current robot position)
  std::unique_ptr<tf2_ros::TransformListener> tfl_;
  std::unique_ptr<tf2_ros::Buffer> tfb_;

  // service handles
  ros::ServiceServer stop_service_;
  ros::ServiceServer explore_service_;
  ros::ServiceServer goto_service_;

  // thread handling
  std::atomic<bool> stop_;
  std::future<void> thread_handle_;

  // other variables
  const std::string base_frame_ {"base_link"};
  const std::string map_frame_ {"map"};
  const std::string server_name_ {"move_base"};
  const double explore_step_ {2.0};
};

} // namespace cleanup
