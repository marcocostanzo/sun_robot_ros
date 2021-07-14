
#ifndef SUN_ROBOT_ROS_CLIK_CLIENT_H
#define SUN_ROBOT_ROS_CLIK_CLIENT_H

#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
#include "sun_robot_msgs/ClikGetState.h"
#include "sun_robot_msgs/ClikSetEndEffector.h"
#include "sun_robot_msgs/ClikSetFixedJoints.h"
#include "sun_robot_msgs/ClikSetMode.h"
#include "sun_robot_msgs/ClikSetSecondaryObj.h"
#include "tf2/convert.h"

namespace sun {
class ClikClient {
private:
  ros::NodeHandle nh_;

  ros::ServiceClient sc_set_mode_;
  ros::ServiceClient sc_get_state_;
  ros::ServiceClient sc_set_end_effector_;
  ros::ServiceClient sc_set_second_obj_;
  ros::ServiceClient sc_set_fixed_joints_obj_;

public:
  //! nh is the a node handle in the clik namespace
  ClikClient(const ros::NodeHandle &nh = ros::NodeHandle("clik"));

  ~ClikClient() = default;

  void waitForServers();

  void set_mode(int8_t mode);

  void set_fixed_joints(const std::vector<std::string> &fixed_joints);

  void set_fixed_joints(const std::vector<unsigned int> &fixed_joints_index);

  void set_no_fixed_joints();

  void stop();

  void mode_position();

  void mode_velocity();

  void mode_velocity_ee();

  sun_robot_msgs::ClikGetState::Response get_state();

  //! n_pose_ee = end effector pose w.r.t. link n
  void set_end_effector(const geometry_msgs::Pose &n_pose_ee);

  void set_second_objective_robot_joint_configuration(
      double second_obj_gain = -1, const std::vector<double> &joint_conf = {},
      const std::vector<double> &joint_weights = {});
  void set_second_objective_robot_joint_configuration(
      double second_obj_gain = -1, const std::vector<double> &joint_conf = {},
      double joint_weights = -1);
  void set_second_objective_robot_joint_keep(
      double second_obj_gain = -1,
      const std::vector<double> &joint_weights = {});

  template <typename T>
  void toRobotBaseFrame(T in, T &out, std::string &out_frame_id) {
    sun_robot_msgs::ClikGetState::Response clik_state = get_state();
    geometry_msgs::TransformStamped b_transform_ee;
    b_transform_ee.header = clik_state.ee_pose.header;
    b_transform_ee.child_frame_id = "clik_ee";
    b_transform_ee.transform.translation.x = clik_state.ee_pose.pose.position.x;
    b_transform_ee.transform.translation.y = clik_state.ee_pose.pose.position.y;
    b_transform_ee.transform.translation.z = clik_state.ee_pose.pose.position.z;
    b_transform_ee.transform.rotation = clik_state.ee_pose.pose.orientation;

    out_frame_id = b_transform_ee.child_frame_id;
    out = in;

    tf2::doTransform(out, out, b_transform_ee);
  }
};

} // namespace sun

#endif
