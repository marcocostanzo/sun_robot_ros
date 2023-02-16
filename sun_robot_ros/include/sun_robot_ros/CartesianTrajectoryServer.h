#ifndef SUN_ROBOT_ROS_CARTESIAN_TRAJECTORY_SERVER_H_
#define SUN_ROBOT_ROS_CARTESIAN_TRAJECTORY_SERVER_H_

#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include <actionlib/server/simple_action_server.h>

#include "sun_traj_lib/Cartesian_Independent_Traj.h"
#include "sun_traj_lib/Line_Segment_Traj.h"
#include "sun_traj_lib/Quintic_Poly_Traj.h"
#include "sun_traj_lib/Rotation_Const_Axis_Traj.h"
#include "sun_traj_lib/Trapez_Traj.h"

#include "ros/callback_queue.h"
#include "sun_robot_msgs/CartesianStateStamped.h"
#include "sun_robot_msgs/CartesianTrajectoryAction.h"

#include "check_realtime.h"

namespace sun {

class CartesianTrajectoryServer {
private:
  /* data */
public:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_for_params_;

  ros::CallbackQueue callbackQueue_;

  std::unique_ptr<
      actionlib::SimpleActionServer<sun_robot_msgs::CartesianTrajectoryAction>>
      as_traj_;

  bool b_use_realtime_;
  bool b_publish_on_pose_twist_;
  ros::Publisher pose_twist_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher twist_pub_;

  ~CartesianTrajectoryServer() = default;

  CartesianTrajectoryServer(
      const ros::NodeHandle &nh_for_topics = ros::NodeHandle("clik"),
      const ros::NodeHandle &nh_for_params = ros::NodeHandle("~"))
      : nh_(nh_for_topics), nh_for_params_(nh_for_params) {}

  void init() {

    nh_.setCallbackQueue(&callbackQueue_);

    nh_for_params_.param("use_realtime", b_use_realtime_, false);

    nh_for_params_.param("publish_on_pose_twist", b_publish_on_pose_twist_,
                         false);
    std::string pose_out_topic_str;
    nh_for_params_.param("pose_out_topic", pose_out_topic_str,
                         std::string("pose_out"));
    std::string twist_out_topic_str;
    nh_for_params_.param("twist_out_topic", twist_out_topic_str,
                         std::string("twist_out"));
    std::string pose_twist_out_topic_str;
    nh_for_params_.param("pose_twist_out_topic", pose_twist_out_topic_str,
                         std::string("cartesian_traj"));
    std::string action_name_str;
    nh_for_params_.param("action_name", action_name_str,
                         std::string("cartesian_traj_action"));

    if (b_publish_on_pose_twist_) {
      pose_twist_pub_ = nh_.advertise<sun_robot_msgs::CartesianStateStamped>(
          pose_twist_out_topic_str, 1);
    } else {
      pose_pub_ =
          nh_.advertise<geometry_msgs::PoseStamped>(pose_out_topic_str, 1);
      twist_pub_ =
          nh_.advertise<geometry_msgs::TwistStamped>(twist_out_topic_str, 1);
    }

    initRealTime();

    as_traj_ = std::unique_ptr<actionlib::SimpleActionServer<
        sun_robot_msgs::CartesianTrajectoryAction>>(
        new actionlib::SimpleActionServer<
            sun_robot_msgs::CartesianTrajectoryAction>(
            nh_, action_name_str,
            boost::bind(&CartesianTrajectoryServer::traj_execute_cb, this, _1),
            false));
  }

  void initRealTime() {
    if (b_use_realtime_) {

      if (!check_realtime()) {
        throw std::runtime_error("REALTIME NOT AVAILABLE");
      }

      if (!set_realtime_SCHED_FIFO()) {
        throw std::runtime_error("ERROR IN set_realtime_SCHED_FIFO");
      }

      std::cout << "[CARTESIAN TRAJ SERVER] REALTIME MODE SCHED_FIFO!\n";
    }
  }

  void start() { as_traj_->start(); }

  void
  spinOnce(const ros::WallDuration &wallDuration = ros::WallDuration(0.0)) {
    callbackQueue_.callAvailable(wallDuration);
  }

  void spin() {
    ros::WallDuration timeout(0.1f);
    while (ros::ok()) {
      spinOnce(ros::WallDuration(timeout));
    }
  }

  static TooN::Vector<3> toTooN(const geometry_msgs::Point &p) {
    return TooN::makeVector(p.x, p.y, p.z);
  }

  static TooN::Vector<3> toTooN(const geometry_msgs::Vector3 &p) {
    return TooN::makeVector(p.x, p.y, p.z);
  }

  static double projectOnAxis(const TooN::Vector<3> &vec,
                              const TooN::Vector<3> &axis) {
    double n = norm(axis);
    if (n < 10.0 * std::numeric_limits<double>::epsilon()) {
      return 0.0;
    }
    return vec * axis / n;
  }

  void
  traj_execute_cb(const sun_robot_msgs::CartesianTrajectoryGoalConstPtr &goal) {

    initRealTime(); // this is a different thread

    ros::Time t0 = goal->trajectory.header.stamp;
    if (t0.toSec() == 0) {
      t0 = ros::Time::now();
    }

    if (goal->trajectory.points.size() == 0) {
      ROS_WARN("cartesian_traj_server: requested an empty traj -> the action "
               "is consideres immediatly succeeded");
      as_traj_->setSucceeded();
      return;
    }

    if (goal->trajectory.points.size() == 1) {
      ROS_WARN("cartesian_traj_server: requested a traj with one point! it is "
               "not possible!");
      as_traj_->setAborted();
      return;
    }

    ROS_INFO_STREAM("cartesian traj start!" << std::endl);
    ros::Time tf = t0 + goal->trajectory.points.back().time_from_start;

    ros::Rate loop_rate(goal->sampling_freq);
    ros::Time time_now = ros::Time::now();

    for (int i = 1; i < goal->trajectory.points.size(); i++) {
      if (as_traj_->isPreemptRequested()) {
        break;
      }

      const sun_robot_msgs::CartesianTrajectoryPoint &traj_point_prev =
          goal->trajectory.points[i - 1];
      const sun_robot_msgs::CartesianTrajectoryPoint &traj_point_next =
          goal->trajectory.points[i];

      sun::UnitQuaternion quat_prev(
          traj_point_prev.pose.orientation.w,
          TooN::makeVector(traj_point_prev.pose.orientation.x,
                           traj_point_prev.pose.orientation.y,
                           traj_point_prev.pose.orientation.z));
      sun::UnitQuaternion quat_next(
          traj_point_next.pose.orientation.w,
          TooN::makeVector(traj_point_next.pose.orientation.x,
                           traj_point_next.pose.orientation.y,
                           traj_point_next.pose.orientation.z));
      quat_next = sun::UnitQuaternion(quat_next, quat_prev); // continuity

      sun::UnitQuaternion Delta_Q = quat_next * inv(quat_prev);
      sun::AngVec Delta_angvec = Delta_Q.toangvec();

      ros::Duration prev_next_duration =
          traj_point_next.time_from_start - traj_point_prev.time_from_start;

      // velocities and acceleration are simply projected on the direction axes
      // between i-1 and i
      TooN::Vector<3> linearDirectionAxis =
          toTooN(traj_point_next.pose.position) -
          toTooN(traj_point_prev.pose.position);
      double prev_velocity_linear = projectOnAxis(
          toTooN(traj_point_prev.velocity.linear), linearDirectionAxis);
      double next_velocity_linear = projectOnAxis(
          toTooN(traj_point_next.velocity.linear), linearDirectionAxis);
      double prev_acceleration_linear = projectOnAxis(
          toTooN(traj_point_prev.acceleration.linear), linearDirectionAxis);
      double next_acceleration_linear = projectOnAxis(
          toTooN(traj_point_next.acceleration.linear), linearDirectionAxis);
      double prev_velocity_angular = projectOnAxis(
          toTooN(traj_point_prev.velocity.angular), Delta_angvec.getVec());
      double next_velocity_angular = projectOnAxis(
          toTooN(traj_point_next.velocity.angular), Delta_angvec.getVec());
      double prec_acceleration_angular = projectOnAxis(
          toTooN(traj_point_prev.acceleration.angular), Delta_angvec.getVec());
      double next_acceleration_angular = projectOnAxis(
          toTooN(traj_point_next.acceleration.angular), Delta_angvec.getVec());

      std::unique_ptr<sun::Scalar_Traj_Interface> scalar_linear_traj;
      std::unique_ptr<sun::Scalar_Traj_Interface> scalar_rot_traj;
      if (!goal->use_trapez) {
        scalar_linear_traj =
            std::unique_ptr<sun::Quintic_Poly_Traj>(new sun::Quintic_Poly_Traj(
                prev_next_duration.toSec(), 0.0, 1.0,
                (t0 + traj_point_prev.time_from_start).toSec(),
                prev_velocity_linear, next_velocity_linear,
                prev_acceleration_linear, next_acceleration_linear));
        scalar_rot_traj =
            std::unique_ptr<sun::Quintic_Poly_Traj>(new sun::Quintic_Poly_Traj(
                prev_next_duration.toSec(), 0.0, Delta_angvec.getAng(),
                (t0 + traj_point_prev.time_from_start).toSec(),
                prev_velocity_angular, next_velocity_angular,
                prec_acceleration_angular, next_acceleration_angular));
      } else {
        scalar_linear_traj =
            std::unique_ptr<sun::Trapez_Traj>(new sun::Trapez_Traj(
                prev_next_duration.toSec(), 0.0, 1.0, prev_velocity_linear,
                (t0 + traj_point_prev.time_from_start).toSec()));

        scalar_rot_traj =
            std::unique_ptr<sun::Trapez_Traj>(new sun::Trapez_Traj(
                prev_next_duration.toSec(), 0.0, Delta_angvec.getAng(),
                prev_velocity_angular,
                (t0 + traj_point_prev.time_from_start).toSec()));
      }

      sun::Cartesian_Independent_Traj traj(
          sun::Line_Segment_Traj(
              TooN::makeVector(traj_point_prev.pose.position.x,
                               traj_point_prev.pose.position.y,
                               traj_point_prev.pose.position.z),
              TooN::makeVector(traj_point_next.pose.position.x,
                               traj_point_next.pose.position.y,
                               traj_point_next.pose.position.z),
              *scalar_linear_traj),
          sun::Rotation_Const_Axis_Traj(quat_prev, Delta_angvec.getVec(),
                                        *scalar_rot_traj));

      while (ros::ok() && !traj.isCompleate(time_now.toSec()) &&
             !as_traj_->isPreemptRequested()) {
        time_now = ros::Time::now();

        TooN::Vector<3> p = traj.getPosition(time_now.toSec());
        sun::UnitQuaternion Q = traj.getQuaternion(time_now.toSec());
        TooN::Vector<3> dp = traj.getLinearVelocity(time_now.toSec());
        TooN::Vector<3> w = traj.getAngularVelocity(time_now.toSec());

        sun_robot_msgs::CartesianTrajectoryFeedbackPtr feedbk(
            new sun_robot_msgs::CartesianTrajectoryFeedback);
        feedbk->time_left = tf - time_now;

        if (b_publish_on_pose_twist_) {

          sun_robot_msgs::CartesianStateStampedPtr out_pose_twist_msg(
              new sun_robot_msgs::CartesianStateStamped);
          out_pose_twist_msg->header.frame_id =
              goal->trajectory.header.frame_id;

          out_pose_twist_msg->pose.position.x = p[0];
          out_pose_twist_msg->pose.position.y = p[1];
          out_pose_twist_msg->pose.position.z = p[2];
          out_pose_twist_msg->pose.orientation.w = Q.getS();
          out_pose_twist_msg->pose.orientation.x = Q.getV()[0];
          out_pose_twist_msg->pose.orientation.y = Q.getV()[1];
          out_pose_twist_msg->pose.orientation.z = Q.getV()[2];

          out_pose_twist_msg->velocity.linear.x = dp[0];
          out_pose_twist_msg->velocity.linear.y = dp[1];
          out_pose_twist_msg->velocity.linear.z = dp[2];
          out_pose_twist_msg->velocity.angular.x = w[0];
          out_pose_twist_msg->velocity.angular.y = w[1];
          out_pose_twist_msg->velocity.angular.z = w[2];

          out_pose_twist_msg->header.stamp = time_now;

          pose_twist_pub_.publish(out_pose_twist_msg);

        } else {

          geometry_msgs::PoseStampedPtr out_pose(
              new geometry_msgs::PoseStamped);
          geometry_msgs::TwistStampedPtr out_twist(
              new geometry_msgs::TwistStamped);
          out_pose->header.frame_id = goal->trajectory.header.frame_id;
          out_twist->header.frame_id = goal->trajectory.header.frame_id;

          out_pose->pose.position.x = p[0];
          out_pose->pose.position.y = p[1];
          out_pose->pose.position.z = p[2];
          out_pose->pose.orientation.w = Q.getS();
          out_pose->pose.orientation.x = Q.getV()[0];
          out_pose->pose.orientation.y = Q.getV()[1];
          out_pose->pose.orientation.z = Q.getV()[2];

          out_twist->twist.linear.x = dp[0];
          out_twist->twist.linear.y = dp[1];
          out_twist->twist.linear.z = dp[2];
          out_twist->twist.angular.x = w[0];
          out_twist->twist.angular.y = w[1];
          out_twist->twist.angular.z = w[2];

          out_pose->header.stamp = time_now;
          out_twist->header.stamp = time_now;

          pose_pub_.publish(out_pose);
          twist_pub_.publish(out_twist);
        }

        as_traj_->publishFeedback(feedbk);
        loop_rate.sleep();
      }
    }

    if (as_traj_->isPreemptRequested()) {
      as_traj_->setPreempted();
    } else {
      as_traj_->setSucceeded();
    }
  }
};

} // namespace sun

#endif