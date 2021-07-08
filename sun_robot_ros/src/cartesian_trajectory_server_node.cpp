#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include <actionlib/server/simple_action_server.h>

#include "sun_traj_lib/Cartesian_Independent_Traj.h"
#include "sun_traj_lib/Line_Segment_Traj.h"
#include "sun_traj_lib/Quintic_Poly_Traj.h"
#include "sun_traj_lib/Rotation_Const_Axis_Traj.h"

#include "sun_robot_msgs/CartesianTrajectoryAction.h"

std::unique_ptr<actionlib::SimpleActionServer<sun_robot_msgs::CartesianTrajectoryAction>> as_traj;
ros::Publisher pose_pub;
ros::Publisher twist_pub;

TooN::Vector<3> toTooN(const geometry_msgs::Point &p)
{
  return TooN::makeVector(p.x, p.y, p.z);
}

TooN::Vector<3> toTooN(const geometry_msgs::Vector3 &p)
{
  return TooN::makeVector(p.x, p.y, p.z);
}

double projectOnAxis(const TooN::Vector<3> &vec, const TooN::Vector<3> &axis)
{
  double n = norm(axis);
  if (n < 10.0 * std::numeric_limits<double>::epsilon())
  {
    return 0.0;
  }
  return vec * axis / n;
}

void traj_execute_cb(const sun_robot_msgs::CartesianTrajectoryGoalConstPtr &goal)
{

  ros::Time t0 = goal->trajectory.header.stamp;
  if (t0.toSec() == 0)
  {
    t0 = ros::Time::now();
  }

  if (goal->trajectory.points.size() == 0)
  {
    ROS_WARN("cartesian_traj_server: requested an empty traj -> the action is consideres immediatly succeeded");
    as_traj->setSucceeded();
    return;
  }

  if (goal->trajectory.points.size() == 1)
  {
    ROS_WARN("cartesian_traj_server: requested a traj with one point! it is not possible!");
    as_traj->setAborted();
    return;
  }

  std::cout << "cartesian traj start!" << std::endl;
  ros::Time tf = t0 + goal->trajectory.points.back().time_from_start;

  geometry_msgs::PoseStamped out_pose;
  geometry_msgs::TwistStamped out_twist;
  out_pose.header.frame_id = goal->trajectory.header.frame_id;
  out_twist.header.frame_id = goal->trajectory.header.frame_id;

  sun_robot_msgs::CartesianTrajectoryFeedback feedbk;
  ros::Rate loop_rate(goal->sampling_freq);
  ros::Time time_now = ros::Time::now();

  for (int i = 1; i < goal->trajectory.points.size(); i++)
  {
    if (as_traj->isPreemptRequested())
    {
      break;
    }

    const sun_robot_msgs::CartesianTrajectoryPoint &traj_point_prev = goal->trajectory.points[i - 1];
    const sun_robot_msgs::CartesianTrajectoryPoint &traj_point_next = goal->trajectory.points[i];

    sun::UnitQuaternion quat_prev(traj_point_prev.pose.orientation.w,
                                  TooN::makeVector(traj_point_prev.pose.orientation.x, traj_point_prev.pose.orientation.y,
                                                   traj_point_prev.pose.orientation.z));
    sun::UnitQuaternion quat_next(
        traj_point_next.pose.orientation.w,
        TooN::makeVector(traj_point_next.pose.orientation.x, traj_point_next.pose.orientation.y, traj_point_next.pose.orientation.z));

    sun::UnitQuaternion Delta_Q = quat_next * inv(quat_prev);
    sun::AngVec Delta_angvec = Delta_Q.toangvec();

    ros::Duration prev_next_duration = traj_point_next.time_from_start - traj_point_prev.time_from_start;
    
    // velocities and acceleration are simply projected on the direction axes between i-1 and i
    TooN::Vector<3> linearDirectionAxis = toTooN(traj_point_next.pose.position) - toTooN(traj_point_prev.pose.position);
    double prev_velocity_linear = projectOnAxis(toTooN(traj_point_prev.velocity.linear), linearDirectionAxis);
    double next_velocity_linear = projectOnAxis(toTooN(traj_point_next.velocity.linear), linearDirectionAxis);
    double prev_acceleration_linear = projectOnAxis(toTooN(traj_point_prev.acceleration.linear), linearDirectionAxis);
    double next_acceleration_linear = projectOnAxis(toTooN(traj_point_next.acceleration.linear), linearDirectionAxis);
    double prev_velocity_angular = projectOnAxis(toTooN(traj_point_prev.velocity.angular), Delta_angvec.getVec());
    double next_velocity_angular = projectOnAxis(toTooN(traj_point_next.velocity.angular), Delta_angvec.getVec());
    double prec_acceleration_angular = projectOnAxis(toTooN(traj_point_prev.acceleration.angular), Delta_angvec.getVec());
    double next_acceleration_angular = projectOnAxis(toTooN(traj_point_next.acceleration.angular), Delta_angvec.getVec());

    sun::Cartesian_Independent_Traj traj(
        sun::Line_Segment_Traj(
            TooN::makeVector(traj_point_prev.pose.position.x, traj_point_prev.pose.position.y, traj_point_prev.pose.position.z),
            TooN::makeVector(traj_point_next.pose.position.x, traj_point_next.pose.position.y, traj_point_next.pose.position.z),
            sun::Quintic_Poly_Traj(prev_next_duration.toSec(), 0.0, 1.0, (t0 + traj_point_prev.time_from_start).toSec(), prev_velocity_linear, next_velocity_linear, prev_acceleration_linear, next_acceleration_linear)),
        sun::Rotation_Const_Axis_Traj(
            quat_prev, Delta_angvec.getVec(),
            sun::Quintic_Poly_Traj(prev_next_duration.toSec(), 0.0, Delta_angvec.getAng(), (t0 + traj_point_prev.time_from_start).toSec(), prev_velocity_angular, next_velocity_angular, prec_acceleration_angular, next_acceleration_angular)));

    while (ros::ok() && !traj.isCompleate(time_now.toSec()) && !as_traj->isPreemptRequested())
    {
      time_now = ros::Time::now();

      TooN::Vector<3> p = traj.getPosition(time_now.toSec());
      sun::UnitQuaternion Q = traj.getQuaternion(time_now.toSec());
      TooN::Vector<3> dp = traj.getLinearVelocity(time_now.toSec());
      TooN::Vector<3> w = traj.getAngularVelocity(time_now.toSec());

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

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "cartesian_segment_traj");

  ros::NodeHandle nh_private = ros::NodeHandle("~");
  ros::NodeHandle nh_public = ros::NodeHandle();

  std::string pose_out_topic_str;
  nh_private.param("pose_out_topic", pose_out_topic_str, std::string("pose_out"));
  std::string twist_out_topic_str;
  nh_private.param("twist_out_topic", twist_out_topic_str, std::string("twist_out"));
  std::string action_name_str;
  nh_private.param("action_name", action_name_str, std::string("cartesian_traj_action"));

  pose_pub = nh_public.advertise<geometry_msgs::PoseStamped>(pose_out_topic_str, 1);
  twist_pub = nh_public.advertise<geometry_msgs::TwistStamped>(twist_out_topic_str, 1);

  as_traj = std::unique_ptr<actionlib::SimpleActionServer<sun_robot_msgs::CartesianTrajectoryAction>>(
      new actionlib::SimpleActionServer<sun_robot_msgs::CartesianTrajectoryAction>(nh_public, action_name_str,
                                                                                   traj_execute_cb, false));

  as_traj->start();

  ros::spin();

  return 0;
}
