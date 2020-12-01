#include <ros/ros.h>

namespace cleanup {

class Controller {
 public:
  Controller();

  /* @brief Begin exploration behavior. */
  void explore();

  /* @brief Begin cleaning behavior. */
  void clean();

  /* @brief Stop all motion. */
  void stop();

 private:

  /* @brief Private method controlling overall exploration processing. */
  void exploreLoop();

  /* @brief Private method controlling overall cleaning processing. */
  void cleanLoop();

};

} // namespace cleanup