#include "nodelet/nodelet.h"
#include "sun_robot_ros/JointTrajectoryServer.h"
#include <thread>

namespace sun_robot_ros {

template <
    typename JointTrajectoryServerType = sun::JointTrajectoryServer,
    typename std::enable_if<std::is_base_of<
        sun::JointTrajectoryServer, JointTrajectoryServerType>::value>::type * =
        nullptr>
class JointTrajectoryServerNodeletBase : public nodelet::Nodelet {

public:
  volatile bool running_;
  std::unique_ptr<std::thread> nodeletThread_;
  std::unique_ptr<JointTrajectoryServerType> jointServerNode_;

  ~JointTrajectoryServerNodeletBase() {
    if (running_) {
      running_ = false;
      nodeletThread_->join();
    }
  }

  virtual void onInit() override {
    jointServerNode_ = std::unique_ptr<JointTrajectoryServerType>(
        new JointTrajectoryServerType(getNodeHandle(), getPrivateNodeHandle()));

    running_ = true;
    nodeletThread_ = std::unique_ptr<std::thread>(new std::thread(std::bind(
        &JointTrajectoryServerNodeletBase<JointTrajectoryServerType>::threadCB,
        this)));
  }

  /** Nodelet device poll thread main function. */
  void threadCB() {
    jointServerNode_->init();
    jointServerNode_->start();
    while (ros::ok() && running_) {
      jointServerNode_->spinOnce();
    }
  }
};

class JointTrajectoryServerNodelet
    : public JointTrajectoryServerNodeletBase<sun::JointTrajectoryServer> {};

} // namespace sun_robot_ros