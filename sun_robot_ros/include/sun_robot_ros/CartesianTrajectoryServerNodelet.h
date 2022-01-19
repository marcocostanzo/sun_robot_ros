#include "nodelet/nodelet.h"
#include "sun_robot_ros/CartesianTrajectoryServer.h"
#include <thread>

namespace sun_robot_ros {

template <typename CartesianTrajectoryServerType =
              sun::CartesianTrajectoryServer,
          typename std::enable_if<std::is_base_of<
              sun::CartesianTrajectoryServer,
              CartesianTrajectoryServerType>::value>::type * = nullptr>
class CartesianTrajectoryServerNodeletBase : public nodelet::Nodelet {

public:
  volatile bool running_;
  std::unique_ptr<std::thread> nodeletThread_;
  std::unique_ptr<CartesianTrajectoryServerType> cartesianServerNode_;

  ~CartesianTrajectoryServerNodeletBase() {
    if (running_) {
      running_ = false;
      nodeletThread_->join();
    }
  }

  virtual void onInit() override {
    cartesianServerNode_ = std::unique_ptr<CartesianTrajectoryServerType>(
        new CartesianTrajectoryServerType(getNodeHandle(),
                                          getPrivateNodeHandle()));

    running_ = true;
    nodeletThread_ = std::unique_ptr<std::thread>(
        new std::thread(std::bind(&CartesianTrajectoryServerNodeletBase<
                                      CartesianTrajectoryServerType>::threadCB,
                                  this)));
  }

  /** Nodelet device poll thread main function. */
  void threadCB() {
    ros::WallDuration timeout(0.1f);
    cartesianServerNode_->init();
    cartesianServerNode_->start();
    while (ros::ok() && running_) {
      cartesianServerNode_->spinOnce(timeout);
    }
  }
};

class CartesianTrajectoryServerNodelet
    : public CartesianTrajectoryServerNodeletBase<
          sun::CartesianTrajectoryServer> {};

} // namespace sun_robot_ros