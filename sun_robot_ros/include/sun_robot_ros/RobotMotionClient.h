
#ifndef SUN_ROBOT_ROS_ROBOTMOTIONCLIENT_H
#define SUN_ROBOT_ROS_ROBOTMOTIONCLIENT_H

#include "actionlib/client/simple_action_client.h"
#include "sun_robot_msgs/JointTrajectoryAction.h"
#include "sun_robot_msgs/LineSegmentTrajectoryAction.h"
#include "sun_robot_ros/ClikClient.h"
#include "sun_robot_ros/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

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

    RobotMotionClient(const ros::NodeHandle &nh)
        : nh_(nh), clik_(ros::NodeHandle(nh_, "clik")), ac_joint_trajectory_(nh_, "joint_traj_action", true), ac_line_segment_trajectory_(nh_, "line_segment_action", true)
    {
    }

    ~RobotMotionClient() = default;

    void waitForServers()
    {
      clik_.waitForServers();
      ac_joint_trajectory_.waitForServer();
      ac_line_segment_trajectory_.waitForServer();
    }

    bool jointTrajWaitForResult(){
      return ac_joint_trajectory_.waitForResult();
    }

    void goTo(const std::vector<double> &qf, const ros::Duration &duration, const ros::Time &t0 = ros::Time::now(), bool wait = true)
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

      if(wait)
      {
        ac_joint_trajectory_.sendGoalAndWait(goal);
        if (ac_joint_trajectory_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          throw std::runtime_error("Fail to execute JointTraj");
        }
      }
      else
      {
        ac_joint_trajectory_.sendGoal(goal);
      }
      
    }

    void executeJointTraj(trajectory_msgs::JointTrajectory traj, bool use_exponential_junction = false,
                          const ros::Time &t0 = ros::Time::now())
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

    void executeLineSegment(const geometry_msgs::Pose &desired_pose, const ros::Duration &duration,
                            const ros::Time &t0 = ros::Time::now(), bool wait = true)
    {
      clik_.stop();

      sun_robot_msgs::ClikGetState::Response clik_state = clik_.get_state();

      sun_robot_msgs::LineSegmentTrajectoryGoal goal;
      goal.initial_time = t0;
      goal.traj_duration = duration;
      goal.frame_id = clik_state.ee_pose.header.frame_id;
      goal.initial_pose = clik_state.ee_pose.pose;
      goal.final_pose = desired_pose;
      goal.sampling_freq = traj_generators_sampling_freq;

      clik_.mode_position();

      if(wait) {

        ac_line_segment_trajectory_.sendGoalAndWait(goal);

        if (ac_line_segment_trajectory_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          throw std::runtime_error("Fail to execute LineSegmentTraj");
        }

        clik_.stop();

      }

      ac_line_segment_trajectory_.sendGoal(goal);
    
    }

    void executeLineSegmentDeltaEE(const geometry_msgs::Pose &desired_pose, const ros::Duration &duration,
                                 const ros::Time &t0 = ros::Time::now())
    {
      clik_.stop();

      sun_robot_msgs::ClikGetState::Response clik_state = clik_.get_state();

      geometry_msgs::TransformStamped w_Tr_g;
      poseStampedToTransformStamped(clik_state.ee_pose,
                                    w_Tr_g);

      geometry_msgs::Pose desired_pose_base;
      tf2::doTransform(desired_pose, desired_pose_base, w_Tr_g);

      return executeLineSegment(desired_pose_base, duration,
                                t0);
    }

    static void poseStampedToTransformStamped(const geometry_msgs::PoseStamped &pose,
                                              geometry_msgs::TransformStamped &transform,
                                              const std::string &child_frame_id = "noframe")
    {
      transform.header = pose.header;
      transform.child_frame_id = child_frame_id;
      transform.transform.rotation = pose.pose.orientation;
      transform.transform.translation.x = pose.pose.position.x;
      transform.transform.translation.y = pose.pose.position.y;
      transform.transform.translation.z = pose.pose.position.z;
    }
  };

} // namespace sun

#endif