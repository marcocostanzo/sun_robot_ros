#include "ros/ros.h"

#include "sensor_msgs/JointState.h"

#include <actionlib/server/simple_action_server.h>

#include "sun_traj_lib/Quintic_Poly_Traj.h"
#include "sun_traj_lib/Vector_Independent_Traj.h"

#include "sun_robot_msgs/JointTrajectoryAction.h"

std::unique_ptr<actionlib::SimpleActionServer<sun_robot_msgs::JointTrajectoryAction>> as_joint_traj;
ros::Publisher joint_state_pub;

void joint_traj_execute_cb(const sun_robot_msgs::JointTrajectoryGoalConstPtr& goal)
{
  ros::Time t0 = goal->trajectory.header.stamp;
  if (t0.toSec() == 0)
  {
    t0 = ros::Time::now();
  }
  ros::Time tf = t0 + goal->trajectory.points.back().time_from_start;

  sensor_msgs::JointState out_msg;
  out_msg.name.resize(goal->trajectory.joint_names.size());
  out_msg.position.resize(goal->trajectory.joint_names.size());
  out_msg.velocity.resize(goal->trajectory.joint_names.size());
  out_msg.effort.resize(goal->trajectory.joint_names.size());
  for (int i = 0; i < goal->trajectory.joint_names.size(); i++)
  {
    out_msg.name[i] = goal->trajectory.joint_names[i];
    out_msg.effort[i] = 0.0;
  }

  for (int i = 1; i < goal->trajectory.points.size(); i++)
  {
    if (as_joint_traj->isPreemptRequested())
    {
      break;
    }

    const trajectory_msgs::JointTrajectoryPoint& traj_point_prev = goal->trajectory.points[i - 1];
    const trajectory_msgs::JointTrajectoryPoint& traj_point_next = goal->trajectory.points[i];

    sun::Vector_Independent_Traj traj;
    for (int i = 0; i < goal->trajectory.joint_names.size(); i++)
    {
      traj.push_back_traj(sun::Quintic_Poly_Traj(
          (traj_point_next.time_from_start - traj_point_prev.time_from_start).toSec(), traj_point_prev.positions[i],
          traj_point_next.positions[i], (t0 + traj_point_prev.time_from_start).toSec(), traj_point_prev.velocities[i],
          traj_point_next.velocities[i], traj_point_prev.accelerations[i], traj_point_next.accelerations[i]));
    }

    sun_robot_msgs::JointTrajectoryFeedback feedbk;
    ros::Rate loop_rate(goal->sampling_freq);
    ros::Time time_now = ros::Time::now();
    double time_now_sec = time_now.toSec();
    while (ros::ok() && !traj.isCompleate(time_now_sec) && !as_joint_traj->isPreemptRequested())
    {
      time_now = ros::Time::now();
      time_now_sec = time_now.toSec();

      TooN::Vector<> q = traj.getPosition(time_now_sec);
      TooN::Vector<> dq = traj.getVelocity(time_now_sec);
      // TooN::Vector<> ddq = traj.getAcceleration(time_now_sec);

      if (goal->use_exponential_junction)
      {
        TooN::Vector<> q_i = TooN::wrapVector(goal->initial_joints.data(), goal->initial_joints.size());
        q = (q_i - q) * exp(-(time_now_sec - t0.toSec()) / goal->junction_time_constant) + q;
        dq = -(dq + (q_i - q) / goal->junction_time_constant) *
                 exp(-(time_now_sec - t0.toSec()) / goal->junction_time_constant) +
             dq;
      }

      feedbk.time_left = tf - time_now;

      for (int i = 0; i < goal->trajectory.joint_names.size(); i++)
      {
        out_msg.position[i] = q[i];
        out_msg.velocity[i] = dq[i];
      }

      out_msg.header.stamp = time_now;

      joint_state_pub.publish(out_msg);
      as_joint_traj->publishFeedback(feedbk);
      loop_rate.sleep();
    }
  }

  if (as_joint_traj->isPreemptRequested())
  {
    as_joint_traj->setPreempted();
  }
  else
  {
    as_joint_traj->setSucceeded();
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "micro_joint_traj");

  ros::NodeHandle nh_private = ros::NodeHandle("~");
  ros::NodeHandle nh_public = ros::NodeHandle();

  std::string traj_out_topic_str;
  nh_private.param("traj_out_topic", traj_out_topic_str, std::string("traj_out"));
  std::string action_name_str;
  nh_private.param("action_name", action_name_str, std::string("trajectory_action"));

  joint_state_pub = nh_public.advertise<sensor_msgs::JointState>(traj_out_topic_str, 1);

  as_joint_traj = std::unique_ptr<actionlib::SimpleActionServer<sun_robot_msgs::JointTrajectoryAction>>(
      new actionlib::SimpleActionServer<sun_robot_msgs::JointTrajectoryAction>(nh_public, action_name_str,
                                                                               joint_traj_execute_cb, false));

  as_joint_traj->start();

  ros::spin();

  return 0;
}
