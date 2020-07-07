
/*
    Clik Robot Class
    Copyright 2020 Università della Campania Luigi Vanvitelli
    Author: Marco Costanzo <marco.costanzo@unicampania.it>
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "sun_robot_ros/clikNode.h"

namespace sun
{
TooN::Vector<> getVectorFromParam(const ros::NodeHandle& nh, const std::string& param_str, int expected_size,
                                  const TooN::Vector<>& default_value)
{
  if (nh.hasParam(param_str))
  {
    std::vector<double> vec_std;
    nh.getParam(param_str, vec_std);
    if (vec_std.size() != expected_size)
    {
      ROS_ERROR_STREAM(ros::this_node::getName() << " CLIK Error: " << param_str << " size mismatch");
      throw robot_invalid_parameter(param_str + " size mismatch");
    }
    return TooN::wrapVector(vec_std.data(), vec_std.size());
  }
  else
  {
    return default_value;
  }
}

template <typename T>
std::vector<T> getVectorFromParam(const ros::NodeHandle& nh, const std::string& param_str,
                                  const std::vector<T>& default_value)
{
  if (nh.hasParam(param_str))
  {
    std::vector<T> vec_std;
    nh.getParam(param_str, vec_std);
    return vec_std;
  }
  else
  {
    return default_value;
  }
}

clikNode::clikNode(Robot& robot, const std::function<TooN::Vector<>()>& get_joint_position_fcn,
                   const std::function<void(TooN::Vector<>, TooN::Vector<>)>& joint_publish_fcn,
                   const ros::NodeHandle& nh_for_topics, const ros::NodeHandle& nh_for_parmas)
  : robot_(robot)
  , nh_(nh_for_topics)
  , get_joint_position_fcn_(get_joint_position_fcn)
  , joint_publish_fcn_(joint_publish_fcn)
  , jointDH_target_second_obj_(TooN::Zeros(robot_.getNumJoints()))
  , joint_weights_second_obj_(TooN::Zeros(robot_.getNumJoints()))
  , cartesian_mask_(TooN::Ones)
  , qDH_k_(TooN::Zeros(robot_.getNumJoints()))
  , mode_(sun_robot_msgs::ClikSetMode::Request::MODE_STOP)
{
  // params
  nh_for_parmas.param("rate", Ts_, 1000.0);
  Ts_ = 1.0 / Ts_;
  nh_for_parmas.param("error_gain", error_gain_, 0.5);
  error_gain_ = error_gain_ / Ts_;

  ros_joint_names_ = getVectorFromParam<std::string>(nh_for_parmas, "ros_joint_names", {});
  nh_for_parmas.param("ros_base_frame_id", ros_base_frame_id_, std::string("clik_base_frame"));

  double dls_joint_speed_saturation;
  nh_for_parmas.param("dls_joint_speed_saturation", dls_joint_speed_saturation, 5.0);
  robot_.setDLSJointSpeedSaturation(dls_joint_speed_saturation);
  nh_for_parmas.param("second_obj_gain", second_obj_gain_, 0.0);

  // joint_target_robot_second_obj
  {
    for (int i = 0; i < robot_.getNumJoints(); i++)
    {
      jointDH_target_second_obj_[i] =
          (robot_.getLink(i)->getSoftJointLimits()[1] + robot_.getLink(i)->getSoftJointLimits()[0]) / 2.0;
    }
    jointDH_target_second_obj_ = getVectorFromParam(nh_for_parmas, "joint_target_robot_second_obj",
                                                    robot_.getNumJoints(), jointDH_target_second_obj_);
    jointDH_target_second_obj_ = robot_.joints_Robot2DH(jointDH_target_second_obj_);
  }
  ROS_INFO_STREAM(ros::this_node::getName() << " CLIK Target conf (DH): " << jointDH_target_second_obj_);

  // joint_weights_second_obj
  joint_weights_second_obj_ = getVectorFromParam(nh_for_parmas, "joint_weights_second_obj", robot_.getNumJoints(),
                                                 TooN::Ones(robot_.getNumJoints()));
  ROS_INFO_STREAM(ros::this_node::getName() << " CLIK Target conf weights: " << joint_weights_second_obj_);

  // cartesian_mask
  cartesian_mask_ = getVectorFromParam(nh_for_parmas, "cartesian_mask", robot_.getNumJoints(), TooN::Ones(6));
  ROS_INFO_STREAM(ros::this_node::getName() << " CLIK Cartesian Mask: " << cartesian_mask_);

  // n_T_e
  {
    TooN::Vector<3> n_T_e_position = getVectorFromParam(nh_for_parmas, "n_T_e_position", 3, TooN::Zeros(3));
    TooN::Vector<4> n_T_e_quat_v =
        getVectorFromParam(nh_for_parmas, "n_T_e_quaternion", 4, TooN::makeVector(1.0, 0.0, 0.0, 0.0));
    UnitQuaternion n_T_e_quaternion(n_T_e_quat_v);
    TooN::Matrix<4, 4> n_T_e = transl(n_T_e_position);
    n_T_e.slice<0, 0, 3, 3>() = n_T_e_quaternion.torot();
    robot_.setnTe(n_T_e);
    ROS_INFO_STREAM(ros::this_node::getName() << " CLIK nTe: \n" << robot_.getnTe());
  }
}

/* RUNNERS */

void clikNode::refresh_cartesian_pose()
{
  TooN::Matrix<4, 4> b_T_e = robot_.fkine(qDH_k_);
  pd_k_ = transl(b_T_e);
  quat_d_k_ = UnitQuaternion(b_T_e);
  quat_k_1 = quat_d_k_;
  cartesian_error_k_ = TooN::Zeros;
}

void clikNode::refresh()
{
  // Wait for initial configuration
  ROS_INFO_STREAM(ros::this_node::getName() << " CLIK refresh()");
  ROS_INFO_STREAM(ros::this_node::getName() << " CLIK Wait for joint positions...");
  qDH_k_ = robot_.joints_Robot2DH(get_joint_position_fcn_());
  ROS_INFO_STREAM(ros::this_node::getName() << " CLIK Joint positions: " << qDH_k_);
  // Initialize vars
  refresh_cartesian_pose();
  dpd_k_ = TooN::Zeros;
  omega_d_k_ = TooN::Zeros;
  ROS_INFO_STREAM(ros::this_node::getName() << " CLIK Initialized!");
}

bool clikNode::getState_srv_cb(sun_robot_msgs::ClikGetState::Request& req, sun_robot_msgs::ClikGetState::Response& res)
{
  if (mode_ == sun_robot_msgs::ClikSetMode::Request::MODE_STOP)
  {
    refresh();
  }

  res.mode = mode_;

  res.robot_joints.header.stamp = ros::Time::now();
  res.robot_joints.name = ros_joint_names_;
  TooN::Vector<> qR = robot_.joints_DH2Robot(qDH_k_);
  for (int i = 0; i < qR.size(); i++)
    res.robot_joints.position.push_back(qR[i]);

  res.ee_pose.header.stamp = ros::Time::now();
  res.ee_pose.header.frame_id = ros_base_frame_id_;
  TooN::Matrix<4, 4> b_T_e = robot_.fkine(qDH_k_);
  TooN::Vector<3> b_p_e = transl(b_T_e);
  UnitQuaternion b_Q_e(b_T_e);
  res.ee_pose.pose.position.x = b_p_e[0];
  res.ee_pose.pose.position.y = b_p_e[1];
  res.ee_pose.pose.position.z = b_p_e[2];
  res.ee_pose.pose.orientation.w = b_Q_e.getS();
  res.ee_pose.pose.orientation.x = b_Q_e.getV()[0];
  res.ee_pose.pose.orientation.y = b_Q_e.getV()[1];
  res.ee_pose.pose.orientation.z = b_Q_e.getV()[2];

  return true;
}

bool clikNode::setEndEffector_srv_cb(sun_robot_msgs::ClikSetEndEffector::Request& req,
                                     sun_robot_msgs::ClikSetEndEffector::Response& res)
{
  if (mode_ != sun_robot_msgs::ClikSetMode::Request::MODE_STOP)
  {
    ROS_ERROR_STREAM(ros::this_node::getName() << " setEndEffector service: it is possible only in MODE_STOP");
    res.success = false;
    return true;
  }

  TooN::Vector<3> n_T_e_position =
      TooN::makeVector(req.n_pose_ee.position.x, req.n_pose_ee.position.y, req.n_pose_ee.position.z);
  UnitQuaternion n_T_e_quaternion(
      req.n_pose_ee.orientation.w,
      TooN::makeVector(req.n_pose_ee.orientation.x, req.n_pose_ee.orientation.y, req.n_pose_ee.orientation.z));
  TooN::Matrix<4, 4> n_T_e = transl(n_T_e_position);
  n_T_e.slice<0, 0, 3, 3>() = n_T_e_quaternion.torot();
  robot_.setnTe(n_T_e);

  refresh();

  res.success = true;

  return true;
}

bool clikNode::setMode_srv_cb(sun_robot_msgs::ClikSetMode::Request& req, sun_robot_msgs::ClikSetMode::Response& res)
{
  switch (req.mode)
  {
    case sun_robot_msgs::ClikSetMode::Request::MODE_STOP:
    {
      ROS_WARN_STREAM(ros::this_node::getName() << " CLIK Stopped!");
      break;
    }
    case sun_robot_msgs::ClikSetMode::Request::MODE_POSITION:
    {
      ROS_WARN_STREAM(ros::this_node::getName() << " CLIK MODE_POSITION");
      break;
    }
    case sun_robot_msgs::ClikSetMode::Request::MODE_VELOCITY:
    {
      ROS_WARN_STREAM(ros::this_node::getName() << " CLIK MODE_VELOCITY");
      break;
    }
    case sun_robot_msgs::ClikSetMode::Request::MODE_VELOCITY_EE:
    {
      ROS_WARN_STREAM(ros::this_node::getName() << " CLIK MODE_VELOCITY_EE");
      break;
    }
    default:
    {
      ROS_ERROR_STREAM(ros::this_node::getName() << " CLIK Error in setMode(): Invalid mode in request!");
      res.success = false;
      return true;
    }
  }

  if (mode_ != req.mode)
  {
    refresh();
  }
  mode_ = req.mode;

  res.success = true;
  return true;
}

void clikNode::run()
{
  // Initialize subscribers
  ros::Subscriber desired_pose_sub = nh_.subscribe("desired_pose", 1, &clikNode::desiredPose_cb, this);
  ros::Subscriber desired_twist_sub = nh_.subscribe("desired_twist", 1, &clikNode::desiredTwist_cb, this);

  // Publish error
  ros::Publisher cartesian_error_pub = nh_.advertise<sun_ros_msgs::Float64Stamped>("cartesian_error", 1);

  // Init Services
  ros::ServiceServer serviceSetMode = nh_.advertiseService("set_mode", &clikNode::setMode_srv_cb, this);
  ros::ServiceServer serviceGetState = nh_.advertiseService("get_state", &clikNode::getState_srv_cb, this);
  ros::ServiceServer serviceSetEndEffector =
      nh_.advertiseService("set_end_effector", &clikNode::setEndEffector_srv_cb, this);

  refresh();

  ros::Rate loop_rate(1.0 / Ts_);

  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();

    switch (mode_)
    {
      case sun_robot_msgs::ClikSetMode::Request::MODE_STOP:
      {
        continue;
      }
      case sun_robot_msgs::ClikSetMode::Request::MODE_POSITION:
      {
        clik_core(error_gain_, pd_k_, quat_d_k_, dpd_k_, omega_d_k_);
        break;
      }
      case sun_robot_msgs::ClikSetMode::Request::MODE_VELOCITY:
      case sun_robot_msgs::ClikSetMode::Request::MODE_VELOCITY_EE:
      {
        refresh_cartesian_pose();
        clik_core(0.0, pd_k_, quat_d_k_, dpd_k_, omega_d_k_);
        break;
      }
      default:
      {
        ROS_ERROR_STREAM(ros::this_node::getName() << " CLIK Error in main while: Invalid mode!");
        throw clik_invalid_mode("non valid modality " + mode_);
      }
    }

    // Publish clik error norm
    sun_ros_msgs::Float64Stamped cartesian_error_msg;
    cartesian_error_msg.header.stamp = ros::Time::now();
    cartesian_error_msg.data = norm(cartesian_error_k_);
    cartesian_error_pub.publish(cartesian_error_msg);
  }
}

// Note: for velocity mode it is sufficient clik_gain_=0;
void clikNode::clik_core(double error_gain, const TooN::Vector<3>& pd_k, const UnitQuaternion& quat_d_k,
                         const TooN::Vector<3>& dpd_k, const TooN::Vector<3>& omega_d_k)
{
  TooN::Vector<> dqDH_k = TooN::Zeros(robot_.getNumJoints());

  qDH_k_ =                           //<- qDH at time k+1
      robot_.clik(qDH_k_,            //<- qDH now, time k
                  pd_k,              // <- desired position
                  quat_d_k,          // <- desired quaternion
                  quat_k_1,          // <- quaternion at last time (to ensure continuity)
                  dpd_k,             // <- desired translational velocity
                  omega_d_k,         //<- desired angular velocity
                  cartesian_mask_,   // <- mask, bitmask, if the i-th element is 0 then the i-th operative space
                                     // coordinate will not be used in the error computation
                  error_gain,        // <- CLIK Gain
                  Ts_,               // <- Ts, sampling time
                  second_obj_gain_,  // <- Gain for second objective
                  jointDH_target_second_obj_,  //<- target for joint position (used into the second objective obj)
                  joint_weights_second_obj_,   // <- weights for joints in the second objective
                  // Return Vars
                  dqDH_k,              // <- joints velocity at time k+1
                  cartesian_error_k_,  //<- error vector at time k
                  quat_k_1  // <- Quaternion at time k (usefull for continuity in the next call of these function)
      );

  TooN::Vector<> qR = robot_.joints_DH2Robot(qDH_k_);
  TooN::Vector<> dqR = robot_.jointsvel_DH2Robot(dqDH_k);

  safety_check(qR, dqR);

  joint_publish_fcn_(qR, dqR);
}

void clikNode::safety_check(const TooN::Vector<>& qR, const TooN::Vector<>& dqR)
{
  // check limits
  if (robot_.exceededHardJointLimits(qR))
  {
    ROS_ERROR_STREAM(ros::this_node::getName() << " CLIK ERROR ROBOT JOINT LIMITS!! On joints:"
                                               << robot_.jointsNameFromBitMask(robot_.checkHardJointLimits(qR)));
    throw robot_joint_position_limits("exceededHardJointLimits");
  }

  if (robot_.exceededHardVelocityLimits(dqR))
  {
    ROS_ERROR_STREAM(ros::this_node::getName() << " CLIK ERROR ROBOT Velocity!! On joints:"
                                               << robot_.jointsNameFromBitMask(robot_.checkHardVelocityLimits(dqR)));
    throw robot_joint_position_limits("exceededHardJointLimits");
  }
}

void clikNode::desiredPose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
  if (mode_ == sun_robot_msgs::ClikSetMode::RequestType::MODE_POSITION)
  {
    pd_k_[0] = pose_msg->pose.position.x;
    pd_k_[1] = pose_msg->pose.position.y;
    pd_k_[2] = pose_msg->pose.position.z;

    quat_d_k_ = UnitQuaternion(
        pose_msg->pose.orientation.w,
        TooN::makeVector(pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z));
  }
}

void clikNode::desiredTwist_cb(const geometry_msgs::TwistStamped::ConstPtr& twist_msg)
{
  if (mode_ == sun_robot_msgs::ClikSetMode::RequestType::MODE_POSITION ||
      mode_ == sun_robot_msgs::ClikSetMode::RequestType::MODE_VELOCITY)
  {
    dpd_k_[0] = twist_msg->twist.linear.x;
    dpd_k_[1] = twist_msg->twist.linear.y;
    dpd_k_[2] = twist_msg->twist.linear.z;

    omega_d_k_[0] = twist_msg->twist.angular.x;
    omega_d_k_[1] = twist_msg->twist.angular.y;
    omega_d_k_[2] = twist_msg->twist.angular.z;
  }
  else if (mode_ == sun_robot_msgs::ClikSetMode::Request::MODE_VELOCITY_EE)
  {
    dpd_k_[0] = twist_msg->twist.linear.x;
    dpd_k_[1] = twist_msg->twist.linear.y;
    dpd_k_[2] = twist_msg->twist.linear.z;

    omega_d_k_[0] = twist_msg->twist.angular.x;
    omega_d_k_[1] = twist_msg->twist.angular.y;
    omega_d_k_[2] = twist_msg->twist.angular.z;

    TooN::Matrix<3, 3> b_R_e = robot_.fkine(qDH_k_).slice<0, 0, 3, 3>();
    dpd_k_ = b_R_e * dpd_k_;
    omega_d_k_ = b_R_e * omega_d_k_;
  }
}

}  // Namespace sun