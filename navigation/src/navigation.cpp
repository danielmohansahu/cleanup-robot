#include <navigation/navigation.h>

namespace cleanup {

Navigation::Navigation() : stop_ {false} {

  // get core node handle
  ros::NodeHandle pnh("navigation");

  // a handy type for constructing services
  using TriggerCallback = boost::function<bool(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&)>;

  // construct stop service
  stop_service_ = pnh.advertiseService(
    "stop",
    TriggerCallback([this] (const auto& req, const auto& res) {
      this->stop();
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

      return true; 
    }));

  // @TODO create goto service

  // Construct actionlib client (to send navigation goals)
  goto_client_ = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("WHATSMYREALNAME", true);
}

Navigation::~Navigation() {
  // make sure we've rejoined any running threads.
  stop();
}

void Navigation::goTo(const geometry_msgs::PoseStamped& pose) {
  // @TODO
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

}

} // namespace cleanup