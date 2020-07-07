#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include <actionlib/server/simple_action_server.h>

#include "sun_traj_lib/Cartesian_Independent_Traj.h"
#include "sun_traj_lib/Line_Segment_Traj.h"
#include "sun_traj_lib/Quintic_Poly_Traj.h"
#include "sun_traj_lib/Rotation_Const_Axis_Traj.h"

#include "sun_robot_msgs/LineSegmentTrajectoryAction.h"

std::unique_ptr<actionlib::SimpleActionServer<sun_robot_msgs::LineSegmentTrajectoryAction>> as_traj;
ros::Publisher pose_pub;
ros::Publisher twist_pub;

void traj_execute_cb(const sun_robot_msgs::LineSegmentTrajectoryGoalConstPtr& goal)
{
  ros::Time t0 = goal->initial_time;
  if (t0.toSec() == 0)
  {
    t0 = ros::Time::now();
  }
  ros::Time tf = t0 + goal->traj_duration;

  geometry_msgs::PoseStamped out_pose;
  geometry_msgs::TwistStamped out_twist;
  out_pose.header.frame_id = goal->frame_id;
  out_twist.header.frame_id = goal->frame_id;

  sun::UnitQuaternion initial_quat(goal->initial_pose.orientation.w,
                                   TooN::makeVector(goal->initial_pose.orientation.x, goal->initial_pose.orientation.y,
                                                    goal->initial_pose.orientation.z));
  sun::UnitQuaternion final_quat(
      goal->final_pose.orientation.w,
      TooN::makeVector(goal->final_pose.orientation.x, goal->final_pose.orientation.y, goal->final_pose.orientation.z));

  sun::UnitQuaternion Delta_Q = final_quat * inv(initial_quat);
  sun::AngVec Delta_angvec = Delta_Q.toangvec();

  sun::Cartesian_Independent_Traj traj(
      sun::Line_Segment_Traj(
          TooN::makeVector(goal->initial_pose.position.x, goal->initial_pose.position.y, goal->initial_pose.position.z),
          TooN::makeVector(goal->final_pose.position.x, goal->final_pose.position.y, goal->final_pose.position.z),
          sun::Quintic_Poly_Traj(goal->traj_duration.toSec(), 0.0, 1.0, t0.toSec())),
      sun::Rotation_Const_Axis_Traj(
          initial_quat, Delta_angvec.getVec(),
          sun::Quintic_Poly_Traj(goal->traj_duration.toSec(), 0.0, Delta_angvec.getAng(), t0.toSec())));

  sun_robot_msgs::LineSegmentTrajectoryFeedback feedbk;
  ros::Rate loop_rate(goal->sampling_freq);
  ros::Time time_now = ros::Time::now();
  double time_now_sec = time_now.toSec();
  while (ros::ok() && !traj.isCompleate(time_now_sec) && !as_traj->isPreemptRequested())
  {
    time_now = ros::Time::now();
    time_now_sec = time_now.toSec();

    TooN::Vector<3> p = traj.getPosition(time_now_sec);
    sun::UnitQuaternion Q = traj.getQuaternion(time_now_sec);
    TooN::Vector<3> dp = traj.getLinearVelocity(time_now_sec);
    TooN::Vector<3> w = traj.getAngularVelocity(time_now_sec);

    feedbk.time_left = tf - time_now;

    out_pose.pose.position.x = p[0];
    out_pose.pose.position.y = p[1];
    out_pose.pose.position.z = p[2];
    out_pose.pose.orientation.w = Q.getS();
    out_pose.pose.orientation.x = Q.getV()[0];
    out_pose.pose.orientation.y = Q.getV()[1];
    out_pose.pose.orientation.z = Q.getV()[2];

    out_twist.twist.linear.x = dp[0];
    out_twist.twist.linear.y = dp[1];
    out_twist.twist.linear.z = dp[2];
    out_twist.twist.angular.x = w[0];
    out_twist.twist.angular.y = w[1];
    out_twist.twist.angular.z = w[2];

    out_pose.header.stamp = time_now;
    out_twist.header.stamp = time_now;

    pose_pub.publish(out_pose);
    twist_pub.publish(out_twist);
    as_traj->publishFeedback(feedbk);
    loop_rate.sleep();
  }

  if (as_traj->isPreemptRequested())
  {
    as_traj->setPreempted();
  }
  else
  {
    as_traj->setSucceeded();
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "line_segment_traj");

  ros::NodeHandle nh_private = ros::NodeHandle("~");
  ros::NodeHandle nh_public = ros::NodeHandle();

  std::string pose_out_topic_str;
  nh_private.param("pose_out_topic", pose_out_topic_str, std::string("pose_out"));
  std::string twist_out_topic_str;
  nh_private.param("twist_out_topic", twist_out_topic_str, std::string("twist_out"));
  std::string action_name_str;
  nh_private.param("action_name", action_name_str, std::string("line_segment_traj_action"));

  pose_pub = nh_public.advertise<geometry_msgs::PoseStamped>(pose_out_topic_str, 1);
  twist_pub = nh_public.advertise<geometry_msgs::TwistStamped>(twist_out_topic_str, 1);

  as_traj = std::unique_ptr<actionlib::SimpleActionServer<sun_robot_msgs::LineSegmentTrajectoryAction>>(
      new actionlib::SimpleActionServer<sun_robot_msgs::LineSegmentTrajectoryAction>(nh_public, action_name_str,
                                                                                     traj_execute_cb, false));

  as_traj->start();

  ros::spin();

  return 0;
}
