
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

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include "sun_ros_msgs/Float64Stamped.h"

#include <sun_robot_lib/Robot.h>

#include "sun_robot_ros/exceptions.h"

#include "sun_robot_msgs/ClikSetMode.h"


namespace sun{

class clikNode
{
private:
    
    //! Robot object
    Robot& robot_;

    //! state
    int mode_;
    TooN::Vector<> qDH_k_;
    UnitQuaternion quat_k_1;
    //! zoh
    TooN::Vector<3> pd_k_; 
    UnitQuaternion quat_d_k_;
    TooN::Vector<3> dpd_k_;
    TooN::Vector<3> omega_d_k_;

    //! Outputs
    TooN::Vector<6> cartesian_error_k_;

    //! Params
    TooN::Vector< 6, int > cartesian_mask_;
    double error_gain_; // should be gain/Ts
    double Ts_;
    double second_obj_gain_;  // should be gain/Ts
    TooN::Vector<> jointDH_target_second_obj_;
    TooN::Vector<> joint_weights_second_obj_;

    //! Cbs
    std::function<TooN::Vector<>()> get_joint_position_fcn_;
    std::function<void(TooN::Vector<>, TooN::Vector<>)> joint_publish_fcn_;

    //! ROS
    ros::NodeHandle nh_;
    std::string desired_pose_topic_str_;
    std::string desired_twist_topic_str_;
    std::string cartesian_error_topic_str_;
    std::string service_server_set_mode_str_;

public:

    clikNode(  
            Robot& robot, 
            const ros::NodeHandle& nh_for_topics,
            const ros::NodeHandle& nh_for_parmas,
            const std::function<TooN::Vector<>()>& get_joint_position_fcn, 
            const std::function<void(TooN::Vector<>, TooN::Vector<>)>& joint_publish_fcn
            );

    ~clikNode() = default;

/* RUNNERS */

void refresh_cartesian_pose();

void refresh();

bool setMode_srv_cb(sun_robot_msgs::ClikSetMode::Request  &req, 
   		 		sun_robot_msgs::ClikSetMode::Response &res);

void run();


//Note: for velocity mode it is sufficient clik_gain_=0;
void clik_core(
    double error_gain,
    const TooN::Vector<3>& pd_k, 
    const UnitQuaternion& quat_d_k, 
    const TooN::Vector<3>& dpd_k,
    const TooN::Vector<3>& omega_d_k
    );

void safety_check(const TooN::Vector<>& qR, const TooN::Vector<>& dqR);

void desiredPose_cb( const geometry_msgs::PoseStamped::ConstPtr& pose_msg );

void desiredTwist_cb( const geometry_msgs::TwistStamped::ConstPtr& twist_msg );

private:

TooN::Vector<> getVectorFromParam(
    const ros::NodeHandle& nh, 
    const std::string& param_str, 
    int expected_size, 
    const TooN::Vector<>& default_value
    );

};

} // Namespace sun