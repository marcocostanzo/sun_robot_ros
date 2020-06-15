#include "ros/ros.h"

#include "sensor_msgs/JointState.h"

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>


#include "sun_robot_msgs/JointTrajectoryAction.h"
#include "sun_robot_msgs/MicroJointTrajAction.h"


std::unique_ptr<actionlib::SimpleActionServer<sun_robot_msgs::JointTrajectoryAction>> as_joint_traj;
std::unique_ptr<actionlib::SimpleActionClient<sun_robot_msgs::MicroJointTrajAction>> ac_micro_joint_traj;

void joint_traj_execute_cb(const sun_robot_msgs::JointTrajectoryGoalConstPtr& goal)
{

    ros::Time t0 = goal->trajectory.header.stamp;
    if(t0.toSec() == 0)
    {
        t0 = ros::Time::now();
    }
    ros::Time tf = t0 + goal->trajectory.points.back().time_from_start;

    for(int i=1; i<goal->trajectory.points.size(); i++ )
    {

        if(as_joint_traj->isPreemptRequested())
        {
            break;
        }
        

        sun_robot_msgs::MicroJointTrajGoal micro_goal_msg;

        micro_goal_msg.initial_time = t0 + goal->trajectory.points[i-1].time_from_start;
        micro_goal_msg.traj_duration = goal->trajectory.points[i].time_from_start - goal->trajectory.points[i-1].time_from_start;
        micro_goal_msg.joint_names = goal->trajectory.joint_names;

        micro_goal_msg.initial_position = goal->trajectory.points[i-1].positions;
        micro_goal_msg.final_position = goal->trajectory.points[i].positions;

        micro_goal_msg.initial_velocity = goal->trajectory.points[i-1].velocities;
        micro_goal_msg.final_velocity = goal->trajectory.points[i].velocities;

        micro_goal_msg.initial_acceleration = goal->trajectory.points[i-1].accelerations;
        micro_goal_msg.final_acceleration = goal->trajectory.points[i].accelerations;

        // TODO CALL uACTION!
        ac_micro_joint_traj->sendGoal(
            micro_goal_msg
            //, TODO feedbk
            //actionlib::SimpleActionClient<sun_robot_msgs::MicroJointTrajAction>::SimpleDoneCallback(),
            //actionlib::SimpleActionClient<sun_robot_msgs::MicroJointTrajAction>::SimpleActiveCallback(),
            //actionlib::SimpleActionClient<sun_robot_msgs::MicroJointTrajAction>::SimpleFeedbackCallback()
        );
        
        //wait uACTION
        while (!ac_micro_joint_traj->waitForResult(ros::Duration(0.0005)))
        {
            if(as_joint_traj->isPreemptRequested())
            {
                ac_micro_joint_traj->cancelAllGoals();
                break;
            }
        }
        
        //check action result
        sun_robot_msgs::MicroJointTrajResultConstPtr result = ac_micro_joint_traj->getResult();
        if(ac_micro_joint_traj->getState().state_ != actionlib::TerminalState::SUCCEEDED)
        {
            ROS_ERROR_STREAM( ros::this_node::getName() << " Trajectory node, invalid terminal state in uTraj" << ac_micro_joint_traj->getState().text_ );
        }

    }

    if(as_joint_traj->isPreemptRequested())
    {
        as_joint_traj->setPreempted();
    }
    else
    {
        as_joint_traj->setSucceeded();
    }
        
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv, "micro_joint_traj");

    ros::NodeHandle nh_private = ros::NodeHandle("~");
    ros::NodeHandle nh_public = ros::NodeHandle();

    std::string micro_traj_action;
    nh_private.param("micro_traj_action", micro_traj_action, std::string("micro_traj_action"));
    std::string action_name_str;
    nh_private.param("action_name", action_name_str, std::string("trajectory_action"));

    ac_micro_joint_traj = 
        std::unique_ptr<actionlib::SimpleActionClient<sun_robot_msgs::MicroJointTrajAction>>(
            new actionlib::SimpleActionClient<sun_robot_msgs::MicroJointTrajAction>(
                nh_public,
                micro_traj_action,
                true
            )
        );

    as_joint_traj = 
        std::unique_ptr<actionlib::SimpleActionServer<sun_robot_msgs::JointTrajectoryAction>>(
            new actionlib::SimpleActionServer<sun_robot_msgs::JointTrajectoryAction>(
                nh_public, 
                action_name_str, 
                joint_traj_execute_cb,
                false
            )
        );

    ac_micro_joint_traj->waitForServer();

    as_joint_traj->start();

    ros::spin();

    return 0;
}



