/* @file controller.cpp
 * @brief Implementation of the Controller class for overall system management.
 * 
 * @copyright [2020] <Daniel Sahu, Spencer Elyard, Santosh Kesani>
 */

#include <controller/controller.h>

namespace cleanup {

Controller::Controller() {
  ros::NodeHandle pnh;
  
  // construct service clients and wait for the servers to come up
  goto_client_ = pnh.serviceClient<navigation::SetPoseStamped>("navigation/goto");
  while (!goto_client_.waitForExistence(ros::Duration(5.0)))
    ROS_WARN_STREAM("Waiting for navigation goTo service: " << goto_client_.getService());

  stop_client_ = pnh.serviceClient<std_srvs::Trigger>("navigation/stop");
  while (!stop_client_.waitForExistence(ros::Duration(5.0)))
    ROS_WARN_STREAM("Waiting for navigation stop service: " << stop_client_.getService());

  explore_client_ = pnh.serviceClient<std_srvs::Trigger>("navigation/explore");
  while (!explore_client_.waitForExistence(ros::Duration(5.0)))
    ROS_WARN_STREAM("Waiting for navigation explore service: " << explore_client_.getService());

  // detect_client_ = pnh.serviceClient<perception::DetectObjects>("perception/detect");
  // while (!detect_client_.waitForExistence(ros::Duration(5.0)))
  //   ROS_WARN_STREAM("Waiting for perception detectObjects service: " << detect_client_.getService());

  // get_pose_client_ = pnh.serviceClient<perception::GetObjectPose>("perception/get_pose");
  // while (!get_pose_client_.waitForExistence(ros::Duration(5.0)))
    // ROS_WARN_STREAM("Waiting for perception getPose service: " << get_pose_client_.getService());

  // start action server to process mode callbacks
  as_ = std::make_unique<actionlib::SimpleActionServer<controller::SetModeAction>>(
    pnh,
    "set_mode",
    [this](const auto& goal) {this->executeGoal(goal);}
  );
  as_->start();

  ROS_INFO("Controller initialized.");
}

void Controller::executeGoal(const controller::SetModeGoal::ConstPtr& goal) {
  // @TODO
}

} // namespace cleanup