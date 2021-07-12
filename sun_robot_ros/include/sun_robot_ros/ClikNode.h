
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

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "ros/ros.h"

#include "sun_ros_msgs/Float64Stamped.h"

#include <sun_robot_lib/Clik6DQuaternionSingleRobot.h>
#include <sun_robot_lib/JointVelocityIntegrator.h>
#include <sun_robot_lib/JointVelocityTargetConfiguration.h>

#include "sun_robot_ros/exceptions.h"

#include "sun_robot_msgs/ClikGetState.h"
#include "sun_robot_msgs/ClikSetEndEffector.h"
#include "sun_robot_msgs/ClikSetFixedJoints.h"
#include "sun_robot_msgs/ClikSetMode.h"
#include "sun_robot_msgs/ClikSetSecondaryObj.h"

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

  //! Cbs
  virtual TooN::Vector<> getJointPositionRobot() = 0;
  virtual void publishJointRobot(const TooN::Vector<> &qR,
                                 const TooN::Vector<> &qR_dot) = 0;

  // Note: for velocity mode it is sufficient clik_gain_=0;
  void clik_core(ros::Publisher &cartesian_error_pub);

public:
  ClikNode(const std::shared_ptr<Robot> &robot,
           const ros::NodeHandle &nh_for_topics = ros::NodeHandle("clik"),
           const ros::NodeHandle &nh_for_parmas = ros::NodeHandle("~"));

  ~ClikNode() = default;

  /* Getters */
  // TooN::Vector<> get_qR();

  // TooN::Vector<>& get_qDH();

  std::vector<unsigned int>
  jointNamesToJointIndex(const std::vector<std::string> &joint_names) const;

  int getMode();

  /* RUNNERS */

  void reset_desired_cartesian_pose();

  void reset_and_sync_with_robot();

  bool getState_srv_cb(sun_robot_msgs::ClikGetState::Request &req,
                       sun_robot_msgs::ClikGetState::Response &res);

  bool
  setSecondaryObj_srv_cb(sun_robot_msgs::ClikSetSecondaryObj::Request &req,
                         sun_robot_msgs::ClikSetSecondaryObj::Response &res);

  bool setEndEffector_srv_cb(sun_robot_msgs::ClikSetEndEffector::Request &req,
                             sun_robot_msgs::ClikSetEndEffector::Response &res);

  bool setMode_srv_cb(sun_robot_msgs::ClikSetMode::Request &req,
                      sun_robot_msgs::ClikSetMode::Response &res);

  bool setFixedJoints_srv_cb(sun_robot_msgs::ClikSetFixedJoints::Request &req,
                             sun_robot_msgs::ClikSetFixedJoints::Response &res);

  void run();

  void safety_check(const TooN::Vector<> &qR, const TooN::Vector<> &dqR);

  void desiredPose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);

  void desiredTwist_cb(const geometry_msgs::TwistStamped::ConstPtr &twist_msg);
};

} // Namespace sun