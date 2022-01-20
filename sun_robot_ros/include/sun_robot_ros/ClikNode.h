
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

#ifndef SUN_ROBOT_ROS_CLIK_NODE_H_
#define SUN_ROBOT_ROS_CLIK_NODE_H_

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "ros/ros.h"

#include "sun_ros_msgs/Float64Stamped.h"

#include <sun_robot_lib/Clik6DQuaternionSingleRobot.h>
#include <sun_robot_lib/JointVelocityIntegrator.h>
#include <sun_robot_lib/JointVelocityTargetConfiguration.h>

#include "sun_robot_ros/exceptions.h"

#include "ros/callback_queue.h"
#include "sun_robot_msgs/CartesianStateStamped.h"
#include "sun_robot_msgs/ClikGetState.h"
#include "sun_robot_msgs/ClikSetFixedJoints.h"
#include "sun_robot_msgs/ClikSetMode.h"
#include "sun_robot_msgs/ClikSetSecondaryObj.h"
#include "sun_robot_msgs/SetEndEffector.h"

namespace sun {
class ClikNode {
private:
protected:
  //! state
  unsigned int mode_ = sun_robot_msgs::ClikSetMode::Request::MODE_STOP;

  std::shared_ptr<JointVelocityTargetConfiguration> secondObjTargetConfig_;
  std::shared_ptr<Clik6DQuaternionSingleRobot> clik_;
  JointVelocityIntegrator clik_integrator_;

  //! ROS
  ros::NodeHandle nh_;
  std::vector<std::string> ros_joint_names_;
  std::string ros_base_frame_id_;

  bool b_pub_dbg_ = false;
  bool b_use_realtime_ = false;

  ros::CallbackQueue callbk_queue_;
  ros::Publisher joi_state_pub_dbg_;
  ros::Publisher twist_pub_dbg_;
  ros::Publisher clikError_pub_dbg_;
  ros::Publisher pos_posdes_pub_dbg_;

  ros::Publisher cartesian_error_pub_;
  bool b_pub_cartesian_twist_control_;
  ros::Publisher cartesian_twist_control_pub_;
  bool b_publish_robot_fkine_;
  ros::Publisher robot_fkine_pub_;
  bool b_publish_robot_jacobian_;
  ros::Publisher robot_jacob_pub_;

  ros::Subscriber desired_pose_sub_;
  ros::Subscriber desired_twist_sub_;
  ros::Subscriber desired_pose_twis_sub_;

  ros::ServiceServer serviceSetMode_;
  ros::ServiceServer serviceGetState_;
  ros::ServiceServer serviceSetEndEffector_;
  ros::ServiceServer serviceSetSecondObj_;
  ros::ServiceServer serviceSetFixedJoints_;

  std::unique_ptr<ros::Rate> loop_rate_;

  void spinOnce(const ros::WallDuration &timeout = ros::WallDuration(0.0));

  //! Cbs
  virtual TooN::Vector<> getJointPositionRobot(bool wait_new_sample = true) = 0;

  virtual void publishJointRobot(const TooN::Vector<> &qR,
                                 const TooN::Vector<> &qR_dot) = 0;
  void clik_core();

  void pub_dbg();

public:
  ClikNode(const std::shared_ptr<Robot> &robot,
           const ros::NodeHandle &nh_for_topics = ros::NodeHandle("clik"),
           const ros::NodeHandle &nh_for_parmas = ros::NodeHandle("~"));

  ~ClikNode() = default;

  void
  updateParams(const ros::NodeHandle &nh_for_parmas = ros::NodeHandle("~"));

  /* Getters */
  // TooN::Vector<> get_qR();

  // TooN::Vector<>& get_qDH();

  std::vector<unsigned int>
  jointNamesToJointIndex(const std::vector<std::string> &joint_names) const;

  int getMode();

  std::shared_ptr<Clik6DQuaternionSingleRobot> &getClik();

  std::shared_ptr<Robot> &getRobot();

  /* RUNNERS */

  void reset_desired_cartesian_pose();

  void reset_and_sync_with_robot();

  bool getState_srv_cb(sun_robot_msgs::ClikGetState::Request &req,
                       sun_robot_msgs::ClikGetState::Response &res);

  bool
  setSecondaryObj_srv_cb(sun_robot_msgs::ClikSetSecondaryObj::Request &req,
                         sun_robot_msgs::ClikSetSecondaryObj::Response &res);

  bool setEndEffector_srv_cb(sun_robot_msgs::SetEndEffector::Request &req,
                             sun_robot_msgs::SetEndEffector::Response &res);

  bool setMode_srv_cb(sun_robot_msgs::ClikSetMode::Request &req,
                      sun_robot_msgs::ClikSetMode::Response &res);

  bool setFixedJoints_srv_cb(sun_robot_msgs::ClikSetFixedJoints::Request &req,
                             sun_robot_msgs::ClikSetFixedJoints::Response &res);

  virtual void run_init();
  void run_single_step();
  void run();

  void safety_check(const TooN::Vector<> &qR, const TooN::Vector<> &dqR);

  void desiredPose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);

  void desiredTwist_cb(const geometry_msgs::TwistStamped::ConstPtr &twist_msg);

  void desiredPoseTwist_cb(
      const sun_robot_msgs::CartesianStateStamped::ConstPtr &pose_twist_msg);
};

} // Namespace sun

#endif