#include "nodelet/nodelet.h"
#include "sun_robot_ros/ClikNode.h"
#include <thread>

namespace sun {

template <typename ClikType, typename std::enable_if<std::is_base_of<
                                 ClikNode, ClikType>::value>::type * = nullptr>
class ClikNodelet : public nodelet::Nodelet {

public:
  volatile bool running_;
  std::unique_ptr<std::thread> nodeletThread_;
  std::unique_ptr<ClikType> clikNode_;

  ~ClikNodelet() {
    if (running_) {
      running_ = false;
      nodeletThread_->join();
    }
  }

  virtual void onInit() override {

    clikNode_ = std::unique_ptr<ClikType>(
        new ClikType(getNodeHandle(), getPrivateNodeHandle()));

    // spawn device thread
    running_ = true;
    nodeletThread_ = std::unique_ptr<std::thread>(
        new std::thread(std::bind(&ClikNodelet<ClikType>::threadCB, this)));
  }

  /** Nodelet device poll thread main function. */
  void threadCB() {
    clikNode_->run_init();
    while (ros::ok() && running_) {
      clikNode_->run_single_step();
    }
  }
};

} // namespace sun