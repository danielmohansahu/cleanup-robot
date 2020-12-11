/* @file navigation.cpp
 * @brief Implementation of the Navigation class for basic navigation.
 *
 * @copyright [2020] <Daniel Sahu, Spencer Elyard, Santosh Kesani>
 */

#include <navigation/navigation.h>

namespace cleanup {

Navigation::Navigation() : stop_ {false} {

  // get core node handle
  ros::NodeHandle pnh("navigation");

  // set up transform listener / buffer
  tfb_ = std::make_unique<tf2_ros::Buffer>();
  tfl_ = std::make_unique<tf2_ros::TransformListener>(*tfb_);

  // Construct actionlib client (to send navigation goals)
  goto_client_ = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(server_name_, true);
  while (!goto_client_->waitForServer(ros::Duration(5.0)))
    ROS_WARN_STREAM("Waiting for action server " << server_name_);

  // a handy type for constructing services
  using TriggerCallback = boost::function<bool(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&)>;
  using SetPoseCallback = boost::function<bool(navigation::SetPoseStamped::Request&, navigation::SetPoseStamped::Response&)>;

  // construct stop service
  stop_service_ = pnh.advertiseService(
    "stop",
    TriggerCallback([this] (const auto& req, auto& res) {
      this->currNavMode_ = 0;
      this->stop();
      res.success = true;
      return true;
    }));

  // construct explore service
  explore_service_ = pnh.advertiseService(
    "explore",
    TriggerCallback([this] (const auto& req, auto& res) {

      std::cout << "EXPLORE" << std::endl;

      // first stop any execution
      this->stop();

      this->currNavMode_ = 1;
      // then spin off exploration thread
      thread_handle_ = std::async(
        std::launch::async,
        [this]()->void {this->exploreLoop();});


      res.success = true;
      return true;
    }));

  // construct goto service
  goto_service_ = pnh.advertiseService(
    "goto",
    SetPoseCallback([this] (const auto& req, auto& res) {

      // sanity check that this is in the right frame
      if (req.pose.header.frame_id != map_frame_) {
        ROS_ERROR("Given a goTo pose in the wrong frame.");
        return false;
      }

      // first stop any execution
      this->stop();

      // then send this as a goal to the action server
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose = req.pose;

      this->currNavMode_ = 2;
      goto_client_->sendGoal(goal);


      return true;
    }));
}

Navigation::~Navigation() {
  // make sure we've rejoined any running threads.
  stop();
}

void Navigation::stop() {
  // stop execution of any running behaviors / goals
  if (thread_handle_.valid()) {
    stop_ = true;
    thread_handle_.get();
    stop_ = false;
  }

  // cancel running goals (if any)
  goto_client_->cancelAllGoals();
}

void Navigation::exploreLoop() {
  // basic explore behavior is to send random goals a fixed distance away in a random direction

  // initialize loop variables

  // execute until stopped or shutdown
  while (!stop_ && ros::ok()) {
    // sleep at the top of the loop; this also ensures we
    //  don't interrupt running goals.
    if (goto_client_->getState().isDone()
        || goto_client_->waitForResult(ros::Duration(0.1))) {
      // we're done, find another spot to explore

      // get current robot state
      auto pose = getRobotPose();

      // get a random pose a little distance away from the current position
      double theta = 2 * M_PI * rand() / static_cast<double>(RAND_MAX);
      pose.position.x += explore_step_ * std::cos(theta);
      pose.position.y += explore_step_ * std::sin(theta);

      // send goal
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.pose = pose;
      goal.target_pose.header.frame_id = map_frame_;
      goal.target_pose.header.stamp = ros::Time::now();
      goto_client_->sendGoal(goal);
    }
  }
}

int Navigation::getCurrNavMode() {
  return this->currNavMode_;
}

geometry_msgs::Pose Navigation::getRobotPose() {
  // initialize results
  geometry_msgs::Pose pose;
  tf2::toMsg(tf2::Transform::getIdentity(), pose);

  // poll TF for the latest transform
  auto transform = tfb_->lookupTransform(map_frame_, base_frame_, ros::Time(0), ros::Duration(0));

  // apply transform
  tf2::doTransform(pose, pose, transform);

  return pose;
}

} // namespace cleanup
