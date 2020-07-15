#ifndef SUN_ROBOT_ROS_ACTIONCLIENTUTILS
#define SUN_ROBOT_ROS_ACTIONCLIENTUTILS

#include "actionlib/client/simple_action_client.h"
#include "sun_robot_msgs/JointTrajectoryAction.h"
#include "sun_robot_msgs/LineSegmentTrajectoryAction.h"
#include "sun_robot_ros/ClikClient.h"
#include "tf2_ros/buffer.h"

namespace sun
{

void executeJointTraj(actionlib::SimpleActionClient<sun_robot_msgs::JointTrajectoryAction>& ac,
                      const trajectory_msgs::JointTrajectory& traj, double sampling_freq,
                      const ros::Time& t0 = ros::Time::now(), bool use_exponential_junction = false,
                      const std::vector<double>& initial_joints = {}, double junction_time_constant = 1.0);

void executeLineSegmentTraj(actionlib::SimpleActionClient<sun_robot_msgs::LineSegmentTrajectoryAction>& ac,
                            const geometry_msgs::Pose& initial_pose, const geometry_msgs::Pose& final_pose,
                            const ros::Duration& duration, double sampling_freq, const std::string frame_id = "",
                            const ros::Time& t0 = ros::Time::now());

void executeLineSegmentTraj(actionlib::SimpleActionClient<sun_robot_msgs::LineSegmentTrajectoryAction>& ac,
                            ClikClient& clik, std::shared_ptr<tf2_ros::Buffer>& tf2_buffer,
                            geometry_msgs::PoseStamped desired_pose, const ros::Duration& duration,
                            double sampling_freq, const ros::Time& t0 = ros::Time::now());

//* IMPL *//

void executeJointTraj(actionlib::SimpleActionClient<sun_robot_msgs::JointTrajectoryAction>& ac,
                      const trajectory_msgs::JointTrajectory& traj, double sampling_freq, const ros::Time& t0,
                      bool use_exponential_junction, const std::vector<double>& initial_joints,
                      double junction_time_constant)
{
  sun_robot_msgs::JointTrajectoryGoal goal;
  goal.trajectory = traj;
  goal.trajectory.header.stamp = t0;
  goal.sampling_freq = sampling_freq;
  goal.use_exponential_junction = use_exponential_junction;
  goal.initial_joints = initial_joints;
  goal.junction_time_constant = junction_time_constant;

  ac.sendGoalAndWait(goal);

  ROS_INFO("client: executeJointTraj returned");

  if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    throw std::runtime_error("Fail to execute JointTraj");
  }
}

void executeLineSegmentTraj(actionlib::SimpleActionClient<sun_robot_msgs::LineSegmentTrajectoryAction>& ac,
                            const geometry_msgs::Pose& initial_pose, const geometry_msgs::Pose& final_pose,
                            const ros::Duration& duration, double sampling_freq, const std::string frame_id,
                            const ros::Time& t0)
{
  sun_robot_msgs::LineSegmentTrajectoryGoal goal;
  goal.initial_time = t0;
  goal.traj_duration = duration;
  goal.frame_id = frame_id;
  goal.initial_pose = initial_pose;
  goal.final_pose = final_pose;
  goal.sampling_freq = sampling_freq;

  ac.sendGoalAndWait(goal);

  if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    throw std::runtime_error("Fail to execute LineSegmentTraj");
  }
}

void executeLineSegmentTraj(actionlib::SimpleActionClient<sun_robot_msgs::LineSegmentTrajectoryAction>& ac,
                            ClikClient& clik, std::shared_ptr<tf2_ros::Buffer>& tf2_buffer,
                            geometry_msgs::PoseStamped desired_pose, const ros::Duration& duration,
                            double sampling_freq, const ros::Time& t0)
{
  clik.stop();
  sun_robot_msgs::ClikGetState::Response clik_state = clik.get_state();
  geometry_msgs::PoseStamped initial_pose = clik_state.ee_pose;
  desired_pose = tf2_buffer->transform(desired_pose, initial_pose.header.frame_id, ros::Duration(1.0));
  clik.mode_position();
  executeLineSegmentTraj(ac, initial_pose.pose, desired_pose.pose, duration, sampling_freq,
                         desired_pose.header.frame_id, t0);
  clik.stop();
}

}  // namespace sun

#endif