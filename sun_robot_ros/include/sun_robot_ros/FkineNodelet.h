#include "nodelet/nodelet.h"
#include "sun_robot_ros/FkineNode.h"
#include <thread>

namespace sun {

template <typename FkineNodeType,
          typename std::enable_if<std::is_base_of<
              FkineNode, FkineNodeType>::value>::type * = nullptr>
class FkineNodelet : public nodelet::Nodelet {

public:
  volatile bool running_;
  std::unique_ptr<std::thread> nodeletThread_;
  std::unique_ptr<FkineNodeType> fkine_node_;

  ros::CallbackQueue callbk_queue_;

  ~FkineNodelet() {
    if (running_) {
      running_ = false;
      nodeletThread_->join();
    }
  }

  virtual void onInit() override {

    fkine_node_ = std::unique_ptr<FkineNodeType>(new FkineNodeType(
        getNodeHandle(), getPrivateNodeHandle(), &callbk_queue_));

    // spawn device thread
    running_ = true;
    nodeletThread_ = std::unique_ptr<std::thread>(new std::thread(
        std::bind(&FkineNodelet<FkineNodeType>::threadCB, this)));
  }

  /** Nodelet device poll thread main function. */
  void threadCB() {
    ros::WallDuration timeout(0.1f);
    fkine_node_->start();
    while (ros::ok() && running_) {
      fkine_node_->spinOnce(timeout);
    }
  }
};

} // namespace sun