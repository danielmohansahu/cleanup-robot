#include <navigation/navigation.h>

namespace cleanup {

Navigation::Navigation() : stop_ {false} {

  // get core node handle
  ros::NodeHandle pnh("navigation");

  // Construct actionlib client (to send navigation goals)
  const std::string server_name = "move_base";
  goto_client_ = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(server_name, true);
  while (!goto_client_->waitForServer(ros::Duration(5.0)))
    ROS_WARN_STREAM("Waiting for action server " << server_name);

  // a handy type for constructing services
  using TriggerCallback = boost::function<bool(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&)>;
  using SetPoseCallback = boost::function<bool(navigation::SetPoseStamped::Request&, navigation::SetPoseStamped::Response&)>;

  // construct stop service
  stop_service_ = pnh.advertiseService(
    "stop",
    TriggerCallback([this] (const auto& req, const auto& res) {
      this->stop();
      res.success = true;
      return true; 
    }));

  // construct explore service
  explore_service_ = pnh.advertiseService(
    "explore",
    TriggerCallback([this] (const auto& req, const auto& res) {
      // first stop any execution
      this->stop();

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
    SetPoseCallback([this] (const auto& req, const auto& res) {
      // sanity check that this is in the right frame
      if (req.pose.header.frame_id != "map") {
        ROS_ERROR("Given a goTo pose in the wrong frame.");
        return false;
      }

      // first stop any execution
      this->stop();

      // then send this as a goal to the action server
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose = req.pose;
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
  // @TODO...
}

} // namespace cleanup