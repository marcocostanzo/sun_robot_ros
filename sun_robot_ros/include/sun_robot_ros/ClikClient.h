
#ifndef SUN_ROBOT_ROS_CLIK_CLIENT_H
#define SUN_ROBOT_ROS_CLIK_CLIENT_H

#include "ros/ros.h"
#include "sun_robot_msgs/ClikGetState.h"
#include "sun_robot_msgs/ClikSetEndEffector.h"
#include "sun_robot_msgs/ClikSetMode.h"

namespace sun
{
class ClikClient
{
private:
  ros::NodeHandle nh_;

  ros::ServiceClient sc_set_mode_;
  ros::ServiceClient sc_get_state_;
  ros::ServiceClient sc_set_end_effector_;

public:
  //! nh is the a node handle in the clik namespace
  ClikClient(const ros::NodeHandle& nh = ros::NodeHandle("clik"));

  ~ClikClient() = default;

  void set_mode(int8_t mode);

  void stop();

  void mode_position();

  void mode_velocity();

  void mode_velocity_ee();

  sun_robot_msgs::ClikGetState::Response get_state();

  //! n_pose_ee = end effector pose w.r.t. link n
  void set_end_effector(const geometry_msgs::Pose& n_pose_ee);
};

}  // namespace sun

#endif
