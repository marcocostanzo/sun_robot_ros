#ifndef SUN_ROBOT_ROS_ACTIONCLIENTUTILS
#define SUN_ROBOT_ROS_ACTIONCLIENTUTILS

#include "actionlib/client/simple_action_client.h"
#include "sun_robot_msgs/JointTrajectoryAction.h"
#include "sun_robot_msgs/LineSegmentTrajectoryAction.h"

namespace sun
{
void executeJointTraj(actionlib::SimpleActionClient<sun_robot_msgs::JointTrajectoryAction>& ac,
                      const trajectory_msgs::JointTrajectory& traj, double sampling_freq,
                      const ros::Time& t0 = ros::Time::now(), bool reverse = false);

trajectory_msgs::JointTrajectory reverseTrajectory(const trajectory_msgs::JointTrajectory& traj);

void executeJointTraj(actionlib::SimpleActionClient<sun_robot_msgs::JointTrajectoryAction>& ac,
                      const std::vector<trajectory_msgs::JointTrajectory>& trajs, double sampling_freq,
                      const ros::Time& t0 = ros::Time::now(), bool reverse = false);

trajectory_msgs::JointTrajectory mergeTrajs(const std::vector<trajectory_msgs::JointTrajectory>& trajs,
                                            bool reverse = false);

void executeLineSegmentTraj(actionlib::SimpleActionClient<sun_robot_msgs::LineSegmentTrajectoryAction>& ac,
                            const geometry_msgs::Pose& initial_pose, const geometry_msgs::Pose& final_pose,
                            const ros::Duration& duration, double sampling_freq, const std::string frame_id = "",
                            const ros::Time& t0 = ros::Time::now());

//* IMPL *//

void executeJointTraj(actionlib::SimpleActionClient<sun_robot_msgs::JointTrajectoryAction>& ac,
                      const trajectory_msgs::JointTrajectory& traj, double sampling_freq, const ros::Time& t0,
                      bool reverse)
{
  if (reverse)
  {
    executeJointTraj(ac, reverseTrajectory(traj), sampling_freq, t0, false);
  }

  sun_robot_msgs::JointTrajectoryGoal goal;
  goal.trajectory = traj;
  goal.trajectory.header.stamp = t0;
  goal.sampling_freq = sampling_freq;

  ac.sendGoalAndWait(goal);

  if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    throw std::runtime_error("Fail to execute JointTraj");
  }
}

trajectory_msgs::JointTrajectory reverseTrajectory(const trajectory_msgs::JointTrajectory& traj)
{
  ROS_ERROR_STREAM("reverseTrajectory NOT IMPLEMENTED");
  throw std::runtime_error("reverseTrajectory NOT IMPLEMENTED");

  trajectory_msgs::JointTrajectory traj_out;
  traj_out.header = traj.header;
  traj_out.joint_names = traj.joint_names;

  ros::Duration total_duration = traj.points.back().time_from_start;

  for (int p = (traj.points.size() - 1); p >= 0; p--)
  {
    trajectory_msgs::JointTrajectoryPoint traj_point = traj.points[p];
    traj_point.time_from_start = total_duration - traj.points[p].time_from_start;
    traj_out.points.push_back(traj_point);
  }
}

void executeJointTraj(actionlib::SimpleActionClient<sun_robot_msgs::JointTrajectoryAction>& ac,
                      const std::vector<trajectory_msgs::JointTrajectory>& trajs, double sampling_freq,
                      const ros::Time& t0, bool reverse)
{
  executeJointTraj(ac, mergeTrajs(trajs, reverse), sampling_freq, t0, false);
}

trajectory_msgs::JointTrajectory mergeTrajs(const std::vector<trajectory_msgs::JointTrajectory>& trajs, bool reverse)
{
  int i0 = (reverse ? (trajs.size() - 1) : (0));

  trajectory_msgs::JointTrajectory traj;
  traj.header = trajs[i0].header;
  traj.joint_names = trajs[i0].joint_names;

  for (int i = i0; (reverse ? (i >= 0) : (i < trajs.size())); (reverse ? (i--) : (i++)))
  {
    trajectory_msgs::JointTrajectory in_traj_i = (reverse ? (reverseTrajectory(trajs[i])) : (trajs[i]));

    // Assert the same joint names!
    if (traj.joint_names != in_traj_i.joint_names)
    {
      throw std::runtime_error("mergeTrajs: inconsistent joint names in traj vector");
    }

    ros::Duration last_time_from_start = (i == i0) ? ros::Duration(0.0) : traj.points.back().time_from_start;
    for (int p = 0; p < in_traj_i.points.size(); p++)
    {
      if (in_traj_i.points[p].time_from_start == ros::Duration(0.0) && i != i0)
      {
        if (p != 0)
        {
          throw std::runtime_error("mergeTrajs: zero Duration in a non initial traj");
        }
        if (in_traj_i.points[p] != traj.points.back())
        {
          throw std::runtime_error("mergeTrajs: point in successive trajs not equals");
        }
        continue;
      }
      trajectory_msgs::JointTrajectoryPoint traj_point = in_traj_i.points[p];
      traj_point.time_from_start = last_time_from_start + traj_point.time_from_start;
      traj.points.push_back(traj_point);
    }
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
                            std::shared_ptr<tf2_ros::Buffer>& tf2_buffer, geometry_msgs::PoseStamped desired_pose,
                            const ros::Duration& duration, double sampling_freq, const ros::Time& t0 = ros::Time::now())
{
  stopClik();
  geometry_msgs::PoseStamped initial_pose = getCurrentClikEEPose();
  desired_pose = tf2_buffer->transform(desired_pose, initial_pose.header.frame_id, ros::Duration(1.0));
  startClikPositionMode();
  executeLineSegmentTraj(ac, initial_pose.pose, desired_pose.pose, duration, sampling_freq,
                         desired_pose.header.frame_id, t0);
  stopClik();
}

}  // namespace sun

#endif