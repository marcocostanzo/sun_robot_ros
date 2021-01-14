
#include "sun_robot_ros/ClikClient.h"

namespace sun
{
//! nh is the a node handle in the clik namespace
ClikClient::ClikClient(const ros::NodeHandle& nh) : nh_(nh)
{
  sc_set_mode_ = nh_.serviceClient<sun_robot_msgs::ClikSetMode>("set_mode");
  sc_get_state_ = nh_.serviceClient<sun_robot_msgs::ClikGetState>("get_state");
  sc_set_end_effector_ = nh_.serviceClient<sun_robot_msgs::ClikSetEndEffector>("set_end_effector");
  sc_set_second_obj_ = nh_.serviceClient<sun_robot_msgs::ClikSetSecondaryObj>("set_second_obj");
}

void ClikClient::waitForServers()
{
  sc_set_mode_.waitForExistence();
  sc_get_state_.waitForExistence();
  sc_set_end_effector_.waitForExistence();
  sc_set_second_obj_.waitForExistence();
}

void ClikClient::set_mode(int8_t mode)
{
  sun_robot_msgs::ClikSetMode msg;
  msg.request.mode = mode;

  bool success = sc_set_mode_.call(msg);
  success = success && msg.response.success;
  if (!success)
  {
    throw std::runtime_error("ClikClient: set_mode no success");
  }
}

void ClikClient::stop()
{
  set_mode(sun_robot_msgs::ClikSetMode::Request::MODE_STOP);
}

void ClikClient::mode_position()
{
  set_mode(sun_robot_msgs::ClikSetMode::Request::MODE_POSITION);
}

void ClikClient::mode_velocity()
{
  set_mode(sun_robot_msgs::ClikSetMode::Request::MODE_VELOCITY);
}

void ClikClient::mode_velocity_ee()
{
  set_mode(sun_robot_msgs::ClikSetMode::Request::MODE_VELOCITY_EE);
}

sun_robot_msgs::ClikGetState::Response ClikClient::get_state()
{
  sun_robot_msgs::ClikGetState msg;

  bool success = sc_get_state_.call(msg);
  if (!success)
  {
    throw std::runtime_error("ClikClient: get_state no success");
  }
  return msg.response;
}

//! n_pose_ee = end effector pose w.r.t. link n
void ClikClient::set_end_effector(const geometry_msgs::Pose& n_pose_ee)
{
  sun_robot_msgs::ClikSetEndEffector msg;
  msg.request.n_pose_ee = n_pose_ee;

  bool success = sc_set_end_effector_.call(msg);
  success = success && msg.response.success;
  if (!success)
  {
    throw std::runtime_error("ClikClient: set_end_effector no success");
  }
}

void ClikClient::set_second_objective_robot_joint_configuration(double second_obj_gain,
                                                                const std::vector<double>& joint_conf,
                                                                const std::vector<double>& joint_weights)
{
  sun_robot_msgs::ClikSetSecondaryObj msg;
  msg.request.second_obj_gain = second_obj_gain;
  msg.request.desired_joint_configuration = joint_conf;
  msg.request.desired_joint_configuration_weights = joint_weights;

  bool success = sc_set_second_obj_.call(msg);
  success = success && msg.response.success;
  if (!success)
  {
    throw std::runtime_error("ClikClient: set_second_objective_robot_joint_configuration no success");
  }
}

void ClikClient::set_second_objective_robot_joint_configuration(double second_obj_gain,
                                                                const std::vector<double>& joint_conf,
                                                                double joint_weights)
{
  std::vector<double> joint_weights_vec;
  if (joint_weights != -1)
  {
    joint_weights_vec.push_back(joint_weights);
  }
  return set_second_objective_robot_joint_configuration(second_obj_gain, joint_conf, joint_weights_vec);
}

}  // namespace sun
