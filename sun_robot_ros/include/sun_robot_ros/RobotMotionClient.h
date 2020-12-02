
#ifndef SUN_ROBOT_ROS_ROBOTMOTIONCLIENT_H
#define SUN_ROBOT_ROS_ROBOTMOTIONCLIENT_H

#include "actionlib/client/simple_action_client.h"
#include "sun_robot_msgs/JointTrajectoryAction.h"
#include "sun_robot_msgs/LineSegmentTrajectoryAction.h"
#include "sun_robot_ros/ClikClient.h"
#include "sun_robot_ros/utils.h"

namespace sun
{
/*!
    This class wraps all the sun robot ros interface for a specific robot
    it uses ClikClient, LineSegmentTraj, JointTraj
*/
class RobotMotionClient
{
private:
  /* data */
public:
  ros::NodeHandle nh_;
  ClikClient clik_;
  actionlib::SimpleActionClient<sun_robot_msgs::JointTrajectoryAction> ac_joint_trajectory_;
  actionlib::SimpleActionClient<sun_robot_msgs::LineSegmentTrajectoryAction> ac_line_segment_trajectory_;

  double traj_generators_sampling_freq = 1000.0;
  double junction_time_constant = 1.0;

  RobotMotionClient(const ros::NodeHandle& nh)
    : nh_(nh)
    , clik_(ros::NodeHandle(nh_, "clik"))
    , ac_joint_trajectory_(nh_, "joint_traj_action", true)
    , ac_line_segment_trajectory_(nh_, "line_segment_action", true)
  {
  }

  ~RobotMotionClient() = default;

  void waitForServers()
  {
    //   clik_.waitForServers();
    ac_joint_trajectory_.waitForServer();
    ac_line_segment_trajectory_.waitForServer();
  }

  void goTo(const std::vector<double>& qf, const ros::Duration& duration, const ros::Time& t0 = ros::Time::now())
  {
    clik_.stop();
    sensor_msgs::JointState q0 = clik_.get_state().robot_joints;
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names = q0.name;
    traj.points.resize(2);
    traj.points[0].time_from_start = ros::Duration(0.0);
    traj.points[0].positions = q0.position;
    traj.points[0].velocities = std::vector<double>(q0.name.size(), 0.0);
    traj.points[0].accelerations = traj.points[0].velocities;
    traj.points[0].effort = traj.points[0].velocities;

    traj.points[1].time_from_start = duration;
    traj.points[1].positions = qf;
    traj.points[1].velocities = traj.points[0].velocities;
    traj.points[1].accelerations = traj.points[0].velocities;
    traj.points[1].effort = traj.points[0].velocities;

    sun_robot_msgs::JointTrajectoryGoal goal;
    goal.trajectory = traj;
    goal.trajectory.header.stamp = t0;
    goal.sampling_freq = traj_generators_sampling_freq;
    goal.use_exponential_junction = false;

    ac_joint_trajectory_.sendGoalAndWait(goal);

    if (ac_joint_trajectory_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      throw std::runtime_error("Fail to execute JointTraj");
    }
  }

  void executeJointTraj(trajectory_msgs::JointTrajectory traj, bool use_exponential_junction = false,
                        const ros::Time& t0 = ros::Time::now())
  {
    clik_.stop();
    sensor_msgs::JointState q0 = clik_.get_state().robot_joints;
    traj = sun::filterJointNames(traj, q0.name);

    sun_robot_msgs::JointTrajectoryGoal goal;
    goal.trajectory = traj;
    goal.trajectory.header.stamp = t0;
    goal.sampling_freq = traj_generators_sampling_freq;
    goal.use_exponential_junction = use_exponential_junction;
    goal.initial_joints = q0.position;
    goal.junction_time_constant = junction_time_constant;

    ac_joint_trajectory_.sendGoalAndWait(goal);

    if (ac_joint_trajectory_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      throw std::runtime_error("Fail to execute JointTraj");
    }
  }
};

}  // namespace sun

#endif