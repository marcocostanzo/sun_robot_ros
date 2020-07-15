#ifndef SUN_ROBOT_ROS_UTILS
#define SUN_ROBOT_ROS_ZUTILS

#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"

namespace sun
{
trajectory_msgs::JointTrajectory reverseTrajectory(const trajectory_msgs::JointTrajectory& traj);

trajectory_msgs::JointTrajectory mergeTrajs(const std::vector<trajectory_msgs::JointTrajectory>& trajs);

trajectory_msgs::JointTrajectory filterJointNames(const trajectory_msgs::JointTrajectory& traj,
                                                  const std::vector<std::string>& jont_names);

sensor_msgs::JointState filterJointNames(const sensor_msgs::JointState& j_state,
                                         const std::vector<std::string>& jont_names);

//* IMPL *//

trajectory_msgs::JointTrajectory reverseTrajectory(const trajectory_msgs::JointTrajectory& traj)
{
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
  return traj_out;
}

trajectory_msgs::JointTrajectory mergeTrajs(const std::vector<trajectory_msgs::JointTrajectory>& trajs)
{
  trajectory_msgs::JointTrajectory traj;
  traj.header = trajs[0].header;
  traj.joint_names = trajs[0].joint_names;

  for (int i = 0; i < trajs.size(); i++)
  {
    trajectory_msgs::JointTrajectory in_traj_i = trajs[i];

    // Assert the same joint names!
    if (traj.joint_names != in_traj_i.joint_names)
    {
      throw std::runtime_error("mergeTrajs: inconsistent joint names in traj vector");
    }

    ros::Duration last_time_from_start = (i == 0) ? ros::Duration(0.0) : traj.points.back().time_from_start;
    for (int p = 0; p < in_traj_i.points.size(); p++)
    {
      // First point must have zero time_from_start & position = prev position, it will be discarded
      if (p == 0 && i != 0)
      {
        if (in_traj_i.points[p].time_from_start != ros::Duration(0.0))
        {
          throw std::runtime_error("mergeTrajs: the first time_from_start of traj " + std::to_string(i) +
                                   " is not zero");
        }
        if (in_traj_i.points[p].positions != traj.points.back().positions)
        {
          throw std::runtime_error("mergeTrajs: initial point in traj " + std::to_string(i) + " and final in traj " +
                                   std::to_string(i - 1) + " not equals");
        }
        continue;  // discard the point
      }

      if (i != 0 && in_traj_i.points[p].time_from_start == ros::Duration(0.0))
      {
        throw std::runtime_error("mergeTrajs: zero duration in traj " + std::to_string(i) + "[" + std::to_string(p) +
                                 "]");
      }

      trajectory_msgs::JointTrajectoryPoint traj_point = in_traj_i.points[p];
      traj_point.time_from_start = last_time_from_start + traj_point.time_from_start;
      traj.points.push_back(traj_point);
    }
  }
  return traj;
}

trajectory_msgs::JointTrajectory filterJointNames(const trajectory_msgs::JointTrajectory& traj,
                                                  const std::vector<std::string>& jont_names)
{
  trajectory_msgs::JointTrajectory out_traj;
  out_traj.header = traj.header;
  out_traj.joint_names = jont_names;

  for (auto& point : traj.points)
  {
    trajectory_msgs::JointTrajectoryPoint out_point;
    out_point.time_from_start = point.time_from_start;
    for (const auto& joint_name : jont_names)
    {
      int i = -1;
      for (int j = 0; j < traj.joint_names.size(); j++)
      {
        if (joint_name == traj.joint_names[j])
        {
          i = j;
          break;
        }
      }
      if (i == -1)
      {
        throw std::runtime_error("filterJointNames: joint " + joint_name + " not found in traj");
      }
      if (point.positions.size())
        out_point.positions.push_back(point.positions[i]);
      if (point.velocities.size())
        out_point.velocities.push_back(point.velocities[i]);
      if (point.accelerations.size())
        out_point.accelerations.push_back(point.accelerations[i]);
      if (point.effort.size())
        out_point.effort.push_back(point.effort[i]);
    }
    out_traj.points.push_back(out_point);
  }
  return out_traj;
}

sensor_msgs::JointState filterJointNames(const sensor_msgs::JointState& j_state,
                                         const std::vector<std::string>& jont_names)
{
  sensor_msgs::JointState out_j_state;
  out_j_state.header = j_state.header;
  out_j_state.name = jont_names;

  for (const auto& joint_name : jont_names)
  {
    int i = -1;
    for (int j = 0; j < j_state.name.size(); j++)
    {
      if (joint_name == j_state.name[j])
      {
        i = j;
        break;
      }
    }
    if (i == -1)
    {
      throw std::runtime_error("filterJointNames: joint " + joint_name + " not found in joint state");
    }
    if (j_state.position.size())
      out_j_state.position.push_back(j_state.position[i]);
    if (j_state.velocity.size())
      out_j_state.velocity.push_back(j_state.velocity[i]);
    if (j_state.effort.size())
      out_j_state.effort.push_back(j_state.effort[i]);
  }
  return out_j_state;
}

}  // namespace sun

#endif