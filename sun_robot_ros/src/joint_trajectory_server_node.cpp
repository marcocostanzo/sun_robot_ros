#include "ros/ros.h"

#include "sensor_msgs/JointState.h"

#include <actionlib/server/simple_action_server.h>

#include "sun_traj_lib/Quintic_Poly_Traj.h"
#include "sun_traj_lib/Vector_Independent_Traj.h"

#include "sun_robot_msgs/JointTrajectoryAction.h"

// #define DBG
#undef DBG

#define NUM_OF_TIME_CONSTANT_TO_WAIT 5.0

std::unique_ptr<actionlib::SimpleActionServer<sun_robot_msgs::JointTrajectoryAction>> as_joint_traj;
ros::Publisher joint_state_pub;
#ifdef DBG
ros::Publisher joint_state_no_exp_pub;
#endif

void computeExponentialJunction(double t, double time_constant, const TooN::Vector<>& q_i, const TooN::Vector<>& q,
                                const TooN::Vector<>& dq, TooN::Vector<>& q_out, TooN::Vector<>& dq_out)
{
  q_out = (q_i - q) * exp(-t / time_constant) + q;
  dq_out = -(dq + (q_i - q) / time_constant) * exp(-t / time_constant) + dq;
}

void joint_traj_execute_cb(const sun_robot_msgs::JointTrajectoryGoalConstPtr& goal)
{
  ros::Time t0 = goal->trajectory.header.stamp;
  if (t0.toSec() == 0)
  {
    t0 = ros::Time::now();
  }

  if (goal->trajectory.points.size() == 0)
  {
    ROS_WARN("joint_traj_server: requested an empty traj -> the action is consideres immediatly succeeded");
    as_joint_traj->setSucceeded();
    return;
  }

  if (goal->trajectory.points.size() == 1)
  {
    ROS_WARN("joint_traj_server: requested a traj with one point! it is not possible!");
    as_joint_traj->setAborted();
    return;
  }
  std::cout << "joint traj start!" << std::endl;
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

  sun_robot_msgs::JointTrajectoryFeedback feedbk;
  ros::Rate loop_rate(goal->sampling_freq);
  ros::Time time_now = ros::Time::now();
  sensor_msgs::JointState last_state_no_junction = out_msg;

  for (int i = 1; i < goal->trajectory.points.size(); i++)
  {
    if (as_joint_traj->isPreemptRequested())
    {
      break;
    }

    const trajectory_msgs::JointTrajectoryPoint& traj_point_prev = goal->trajectory.points[i - 1];
    const trajectory_msgs::JointTrajectoryPoint& traj_point_next = goal->trajectory.points[i];

    sun::Vector_Independent_Traj traj;
    for (int j = 0; j < goal->trajectory.joint_names.size(); j++)
    {
      traj.push_back_traj(sun::Quintic_Poly_Traj(
          (traj_point_next.time_from_start - traj_point_prev.time_from_start).toSec(), traj_point_prev.positions[j],
          traj_point_next.positions[j], (t0 + traj_point_prev.time_from_start).toSec(), traj_point_prev.velocities[j],
          traj_point_next.velocities[j], traj_point_prev.accelerations[j], traj_point_next.accelerations[j]));
    }

    // while (
    //     ros::ok() && !as_joint_traj->isPreemptRequested() &&
    //     // traj compleate and exponential junction terminated (if active)
    //     !(traj.isCompleate(time_now_sec) &&
    //       ((goal->use_exponential_junction && ((time_now_sec - t0.toSec()) > (7.0 * goal->junction_time_constant)))
    //       ||
    //        !goal->use_exponential_junction)))
    while (ros::ok() && !as_joint_traj->isPreemptRequested() && !traj.isCompleate(time_now.toSec()))
    {
      time_now = ros::Time::now();

      TooN::Vector<> q = traj.getPosition(time_now.toSec());
      TooN::Vector<> dq = traj.getVelocity(time_now.toSec());
      // TooN::Vector<> ddq = traj.getAcceleration(time_now.toSec());

      for (int i = 0; i < goal->trajectory.joint_names.size(); i++)
      {
        last_state_no_junction.position[i] = q[i];
        last_state_no_junction.velocity[i] = dq[i];
      }

      if (goal->use_exponential_junction)
      {
        computeExponentialJunction((time_now - t0).toSec(), goal->junction_time_constant,
                                   TooN::wrapVector(goal->initial_joints.data(), goal->initial_joints.size()), q, dq, q,
                                   dq);
      }

      feedbk.time_left = tf - time_now;

      for (int i = 0; i < goal->trajectory.joint_names.size(); i++)
      {
        out_msg.position[i] = q[i];
        out_msg.velocity[i] = dq[i];
      }

      out_msg.header.stamp = ros::Time::now();

      joint_state_pub.publish(out_msg);
      as_joint_traj->publishFeedback(feedbk);
#ifdef DBG
      joint_state_no_exp_pub.publish(last_state_no_junction);
#endif
      loop_rate.sleep();
    }
  }

  if (as_joint_traj->isPreemptRequested())
  {
    as_joint_traj->setPreempted();
    return;
  }

  // Wait exponential junction to compleate
  if (goal->use_exponential_junction)
  {
    // Build last q dq from msg
    TooN::Vector<> last_qn =
        TooN::wrapVector(last_state_no_junction.position.data(), last_state_no_junction.position.size());
    TooN::Vector<> last_dqn =
        TooN::wrapVector(last_state_no_junction.velocity.data(), last_state_no_junction.velocity.size());
    ros::Time time_now = ros::Time::now();
    TooN::Vector<> q = TooN::Zeros(last_state_no_junction.name.size());
    TooN::Vector<> dq = TooN::Zeros(last_state_no_junction.name.size());
    while (ros::ok() && !as_joint_traj->isPreemptRequested() &&
           ((time_now - t0).toSec()) < (NUM_OF_TIME_CONSTANT_TO_WAIT * goal->junction_time_constant))
    {
      ROS_WARN_ONCE("jont_traj_server: traj compleate but exp not... waiting exp...");
      time_now = ros::Time::now();
      computeExponentialJunction((time_now - t0).toSec(), goal->junction_time_constant,
                                 TooN::wrapVector(goal->initial_joints.data(), goal->initial_joints.size()), last_qn,
                                 last_dqn, q, dq);

      feedbk.time_left = tf - time_now;

      for (int i = 0; i < goal->trajectory.joint_names.size(); i++)
      {
        out_msg.position[i] = q[i];
        out_msg.velocity[i] = dq[i];
      }

      out_msg.header.stamp = ros::Time::now();

      joint_state_pub.publish(out_msg);
      as_joint_traj->publishFeedback(feedbk);
#ifdef DBG
      joint_state_no_exp_pub.publish(last_state_no_junction);
#endif
      loop_rate.sleep();
    }

    if (ros::ok() && !as_joint_traj->isPreemptRequested())
    {
      // At the end publish the last traj point
      last_state_no_junction.header.stamp = ros::Time::now();
      joint_state_pub.publish(last_state_no_junction);
    }
  }

  if (as_joint_traj->isPreemptRequested())
  {
    as_joint_traj->setPreempted();
    return;
  }

  as_joint_traj->setSucceeded();
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

#ifdef DBG
  joint_state_no_exp_pub = ros::NodeHandle().advertise<sensor_msgs::JointState>("/debug/no_junction_traj", 1);
#endif

  as_joint_traj = std::unique_ptr<actionlib::SimpleActionServer<sun_robot_msgs::JointTrajectoryAction>>(
      new actionlib::SimpleActionServer<sun_robot_msgs::JointTrajectoryAction>(nh_public, action_name_str,
                                                                               joint_traj_execute_cb, false));

  as_joint_traj->start();

  ros::spin();

  return 0;
}
