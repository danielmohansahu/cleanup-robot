#pragma once

#include <atomic>
#include <future>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>

namespace cleanup {

class Navigation {
 public:

  /* @brief Constructor */
  Navigation();

  /* @brief Destructor */
  ~Navigation();

 private:
  /* @brief Send a move base goal to the desired pose.
   * 
   * @TODO consider getting rid of this layer; could just
   * have a client at the Controller level...
   */
  void goTo(const geometry_msgs::PoseStamped& pose);

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

  // action client to move_base navigation manager
  std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> goto_client_;

  // service handles
  ros::ServiceServer stop_service_;
  ros::ServiceServer explore_service_;
  ros::ServiceServer goto_service_;

  // thread handling
  std::atomic<bool> stop_;
  std::future<void> thread_handle_;
};

} // namespace cleanup