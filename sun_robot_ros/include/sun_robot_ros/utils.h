#ifndef SUN_ROBOT_ROS_UTILS
#define SUN_ROBOT_ROS_ZUTILS

#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"

namespace sun
{
trajectory_msgs::JointTrajectory scaleTrajectoryInTime(trajectory_msgs::JointTrajectory traj, double scale_factor);

trajectory_msgs::JointTrajectory reverseTrajectory(const trajectory_msgs::JointTrajectory& traj);

trajectory_msgs::JointTrajectory mergeTrajs(const std::vector<trajectory_msgs::JointTrajectory>& trajs,
                                            bool skip_empty_traj = false);

trajectory_msgs::JointTrajectory filterJointNames(const trajectory_msgs::JointTrajectory& traj,
                                                  const std::vector<std::string>& jont_names);

sensor_msgs::JointState filterJointNames(const sensor_msgs::JointState& j_state,
                                         const std::vector<std::string>& jont_names);

//* IMPL *//

namespace internal
{
template <typename T>
std::vector<T> vector_diff(const std::vector<T>& v1, const std::vector<T>& v2)
{
  if (v1.size() != v2.size())
  {
    throw std::runtime_error("sun::internal::vector_diff vector have different size");
  }
  std::vector<T> out;
  for (int i = 0; i < v1.size(); i++)
  {
    out.push_back(v1[i] - v2[i]);
  }
  return out;
}

template <typename T>
T abs(T in)
{
  return fabs(in);
}

template <typename T>
T vector_norm_2(const std::vector<T>& v)
{
  T out = 0;
  for (int i = 0; i < v.size(); i++)
  {
    out += pow(v[i], 2);
  }
  return out;
}

template <typename T>
T vector_norm(const std::vector<T>& v)
{
  return sqrt(vector_norm_2(v));
}

}  // namespace internal

trajectory_msgs::JointTrajectory scaleTrajectoryInTime(trajectory_msgs::JointTrajectory traj, double scale_factor)
{
  for (auto& point : traj.points)
  {
    point.time_from_start *= scale_factor;
    for (auto& velocity : point.velocities)
    {
      velocity /= scale_factor;
    }
    double scale_factor_2 = pow(scale_factor, 2);
    for (auto& acceleration : point.accelerations)
    {
      acceleration /= scale_factor_2;
    }
    for (auto& effort : point.effort)
    {
      effort /= scale_factor_2;
    }
  }
  return traj;
}

trajectory_msgs::JointTrajectory reverseTrajectory(const trajectory_msgs::JointTrajectory& traj)
{
  trajectory_msgs::JointTrajectory traj_out;
  traj_out.header = traj.header;
  traj_out.joint_names = traj.joint_names;

  ros::Duration total_duration = traj.points.back().time_from_start;

  for (int p = (traj.points.size() - 1); p >= 0; p--)
  {
    trajectory_msgs::JointTrajectoryPoint traj_point = traj.points[p];
    for (auto& velocity : traj_point.velocities)
    {
      velocity = -velocity;
    }
    traj_point.time_from_start = total_duration - traj.points[p].time_from_start;
    traj_out.points.push_back(traj_point);
  }
  return traj_out;
}

trajectory_msgs::JointTrajectory mergeTrajs(const std::vector<trajectory_msgs::JointTrajectory>& trajs,
                                            bool skip_empty_traj)
{
  trajectory_msgs::JointTrajectory traj;

  int i0 = 0;

  for (int i = 0; i < trajs.size(); i++)
  {
    trajectory_msgs::JointTrajectory in_traj_i = trajs[i];

    if (skip_empty_traj && in_traj_i.points.size() == 0)
    {
      if (i == i0)
      {
        i0++;  // i0 is important for the first time_from_start
      }
      continue;
    }

    if (i == i0)
    {
      traj.header = trajs[i].header;
      traj.joint_names = trajs[i].joint_names;
    }

    // Assert the same joint names!
    if (traj.joint_names != in_traj_i.joint_names)
    {
      throw std::runtime_error("mergeTrajs: inconsistent joint names in traj vector");
    }

    ros::Duration last_time_from_start = (i == i0) ? ros::Duration(0.0) : traj.points.back().time_from_start;
    for (int p = 0; p < in_traj_i.points.size(); p++)
    {
      // First point must have zero time_from_start & position = prev position, it will be discarded
      if (p == 0 && i != i0)
      {
        if (in_traj_i.points[p].time_from_start != ros::Duration(0.0))
        {
          throw std::runtime_error("mergeTrajs: the first time_from_start of traj " + std::to_string(i) +
                                   " is not zero");
        }
        if (in_traj_i.points[p].positions != traj.points.back().positions)
        {
          double distance =
              internal::vector_norm(internal::vector_diff(in_traj_i.points[p].positions, traj.points.back().positions));
          if (distance > 0.01)
            throw std::runtime_error("mergeTrajs: initial point in traj " + std::to_string(i) + " and final in traj " +
                                     std::to_string(i - 1) + " not equals. distance=" + std::to_string(distance) );
        }
        continue;  // discard the point
      }

      if (i != i0 && in_traj_i.points[p].time_from_start == ros::Duration(0.0))
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