#include "ros/ros.h"

#include "sensor_msgs/JointState.h"

#include <actionlib/server/simple_action_server.h>

#include "sun_robot_msgs/MicroJointTrajAction.h"

#include "sun_traj_lib/Vector_Independent_Traj.h"
#include "sun_traj_lib/Quintic_Poly_Traj.h"


std::unique_ptr<actionlib::SimpleActionServer<sun_robot_msgs::MicroJointTrajAction>> as_micro_joint_traj;

ros::Publisher joint_state_pub;


void micro_joint_traj_execute_cb(const sun_robot_msgs::MicroJointTrajGoalConstPtr& goal)
{

    int num_joints = goal->initial_position.size();

    sun::Vector_Independent_Traj traj;

    for(int i=0; i<num_joints; i++)
    {            
        traj.push_back_traj(
            sun::Quintic_Poly_Traj(
                goal->traj_duration.toSec(),
                goal->initial_position[i],
                goal->final_position[i],
                goal->initial_time.toSec(),
                goal->initial_velocity[i],
                goal->final_velocity[i],
                goal->initial_acceleration[i],
                goal->final_acceleration[i]
            )
        );
    }

    sensor_msgs::JointState out_msg;
    out_msg.name.resize(num_joints);
    out_msg.position.resize(num_joints);
    out_msg.velocity.resize(num_joints);
    out_msg.effort.resize(num_joints);
    for(int i=0; i<num_joints; i++)
    {
        out_msg.name[i] = goal->joint_names[i];
        out_msg.effort[i] = 0.0;
    }

    sun_robot_msgs::MicroJointTrajFeedback feedbk;

    ros::Time time_now = ros::Time::now();
    double time_now_sec = time_now.toSec();

    if(goal->initial_time.toSec() == 0.0)
    {
        traj.changeInitialTime(time_now_sec);
    }

    while (ros::ok() && !traj.isCompleate(time_now_sec) && !as_micro_joint_traj->isPreemptRequested())
    {
        time_now = ros::Time::now();
        time_now_sec = time_now.toSec();

        TooN::Vector<> q = traj.getPosition(time_now_sec);
        TooN::Vector<> dq = traj.getVelocity(time_now_sec);
        //TooN::Vector<> ddq = traj.getAcceleration(time_now_sec);

        feedbk.time_left = traj.getTimeLeft(time_now_sec);

        for(int i=0; i<num_joints; i++)
        {
            out_msg.position[i] = q[i];
            out_msg.velocity[i] = dq[i];
        }

        out_msg.header.stamp = time_now;

        joint_state_pub.publish(out_msg);
        as_micro_joint_traj->publishFeedback(feedbk);
            
    }

    if(as_micro_joint_traj->isPreemptRequested())
    {
        as_micro_joint_traj->setPreempted();
    }
    else
    {
        as_micro_joint_traj->setSucceeded();
    }
        
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv, "micro_joint_traj");

    ros::NodeHandle nh_private = ros::NodeHandle("~");
    ros::NodeHandle nh_public = ros::NodeHandle();

    std::string traj_out_topic_str;
    nh_private.param("traj_out_topic", traj_out_topic_str, std::string("traj_out"));
    std::string action_name_str;
    nh_private.param("action_name", action_name_str, std::string("micro_traj_action"));

    joint_state_pub = nh_public.advertise<sensor_msgs::JointState>(traj_out_topic_str, 1);

    as_micro_joint_traj = 
        std::unique_ptr<actionlib::SimpleActionServer<sun_robot_msgs::MicroJointTrajAction>>(
            new actionlib::SimpleActionServer<sun_robot_msgs::MicroJointTrajAction>(
                nh_public, 
                action_name_str, 
                micro_joint_traj_execute_cb,
                false
            )
        );

    as_micro_joint_traj->start();

    ros::spin();

    return 0;
}



