#ifndef SUN_ROBOT_FKINE_NODE_H_
#define SUN_ROBOT_FKINE_NODE_H_

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "ros/ros.h"

#include <sun_robot_lib/Clik.h>
#include <sun_robot_lib/Robot.h>

#include "sun_robot_ros/exceptions.h"

#include "ros/callback_queue.h"

#include "sun_robot_msgs/SetEndEffector.h"

namespace sun {

class FkineNode {
protected:
  std::shared_ptr<sun::Robot> robot_;

  //! ROS
  ros::NodeHandle nh_;
  ros::CallbackQueue *callbk_queue_;
  ros::Publisher pose_pub_, twist_pub_;
  ros::ServiceServer serviceSetEndEffector_;

  std::string out_pose_topic_, out_twist_topic_;

  sun::UnitQuaternion oldQuat; // continuity

  bool setEndEffector_srv_cb(sun_robot_msgs::SetEndEffector::Request &req,
                             sun_robot_msgs::SetEndEffector::Response &res);

  void
  updateParams(const ros::NodeHandle &nh_for_parmas = ros::NodeHandle("~"));

  void publishFkine(const TooN::Vector<> &qR);

  void publishVel(const TooN::Vector<> &qR, const TooN::Vector<> &qdotR);

  /**
   joint_sub_ = ...
   */
  virtual void registerJointSubscriber() = 0;

public:
  FkineNode(const std::shared_ptr<Robot> &robot,
            const ros::NodeHandle &nh_for_topics = ros::NodeHandle(),
            const ros::NodeHandle &nh_for_parmas = ros::NodeHandle("~"),
            ros::CallbackQueue *callbk_queue_ = ros::getGlobalCallbackQueue());

  ~FkineNode() = default;

  void spinOnce(const ros::WallDuration &timeout = ros::WallDuration(0.0));

  void spin();

  void start();
};

} // namespace sun

#endif