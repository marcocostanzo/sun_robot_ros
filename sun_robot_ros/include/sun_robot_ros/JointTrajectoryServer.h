#ifndef SUN_ROBOT_ROS_JOINT_TRAJECTORY_SERVER_H_
#define SUN_ROBOT_ROS_JOINT_TRAJECTORY_SERVER_H_

#include "ros/ros.h"

#include "sensor_msgs/JointState.h"

#include <actionlib/server/simple_action_server.h>

#include "sun_traj_lib/Quintic_Poly_Traj.h"
#include "sun_traj_lib/Vector_Independent_Traj.h"

#include "sun_robot_msgs/JointTrajectoryAction.h"

#include "ros/callback_queue.h"

#include "check_realtime.h"

// #define DBG
#undef DBG
#define NUM_OF_TIME_CONSTANT_TO_WAIT 5.0

namespace sun {

class JointTrajectoryServer {
private:
  /* data */
public:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_for_params_;

  ros::CallbackQueue callbackQueue_;

  std::unique_ptr<
      actionlib::SimpleActionServer<sun_robot_msgs::JointTrajectoryAction>>
      as_traj_;

  bool b_use_realtime_;
  ros::Publisher joint_state_pub_;
#ifdef DBG
  ros::Publisher joint_state_no_exp_pub_;
#endif

  ~JointTrajectoryServer() = default;

  JointTrajectoryServer(
      const ros::NodeHandle &nh_for_topics = ros::NodeHandle(),
      const ros::NodeHandle &nh_for_params = ros::NodeHandle("~"))
      : nh_(nh_for_topics), nh_for_params_(nh_for_params) {}

  void init() {

    nh_.setCallbackQueue(&callbackQueue_);

    nh_for_params_.param("use_realtime", b_use_realtime_, false);

    std::string traj_out_topic_str;
    nh_for_params_.param("traj_out_topic", traj_out_topic_str,
                         std::string("traj_out"));
    std::string action_name_str;
    nh_for_params_.param("action_name", action_name_str,
                         std::string("trajectory_action"));

    joint_state_pub_ =
        nh_.advertise<sensor_msgs::JointState>(traj_out_topic_str, 1);

#ifdef DBG
    joint_state_no_exp_pub_ =
        ros::NodeHandle().advertise<sensor_msgs::JointState>(
            "/debug/no_junction_traj", 1);
#endif

    initRealTime();

    as_traj_ = std::unique_ptr<
        actionlib::SimpleActionServer<sun_robot_msgs::JointTrajectoryAction>>(
        new actionlib::SimpleActionServer<
            sun_robot_msgs::JointTrajectoryAction>(
            nh_, action_name_str,
            boost::bind(&JointTrajectoryServer::joint_traj_execute_cb, this,
                        _1),
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

      std::cout << "[JOINT TRAJ SERVER] REALTIME MODE SCHED_FIFO!\n";
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
      spinOnce(timeout);
    }
  }

  void computeExponentialJunction(double t, double time_constant,
                                  const TooN::Vector<> &q_i,
                                  const TooN::Vector<> &q,
                                  const TooN::Vector<> &dq,
                                  TooN::Vector<> &q_out,
                                  TooN::Vector<> &dq_out) {
    q_out = (q_i - q) * exp(-t / time_constant) + q;
    dq_out = -(dq + (q_i - q) / time_constant) * exp(-t / time_constant) + dq;
  }

  void joint_traj_execute_cb(
      const sun_robot_msgs::JointTrajectoryGoalConstPtr &goal) {

    initRealTime(); // this is a different thread

    ros::Time t0 = goal->trajectory.header.stamp;
    if (t0.toSec() == 0) {
      t0 = ros::Time::now();
    }

    if (goal->trajectory.points.size() == 0) {
      ROS_WARN("joint_traj_server: requested an empty traj -> the action is "
               "consideres immediatly succeeded");
      as_traj_->setSucceeded();
      return;
    }

    if (goal->trajectory.points.size() == 1) {
      ROS_WARN("joint_traj_server: requested a traj with one point! it is not "
               "possible!");
      as_traj_->setAborted();
      return;
    }
    ROS_INFO_STREAM("joint traj start!");
    ros::Time tf = t0 + goal->trajectory.points.back().time_from_start;

    sensor_msgs::JointState out_msg;
    out_msg.name.resize(goal->trajectory.joint_names.size());
    out_msg.position.resize(goal->trajectory.joint_names.size());
    out_msg.velocity.resize(goal->trajectory.joint_names.size());
    out_msg.effort.resize(goal->trajectory.joint_names.size());
    for (int i = 0; i < goal->trajectory.joint_names.size(); i++) {
      out_msg.name[i] = goal->trajectory.joint_names[i];
      out_msg.effort[i] = 0.0;
    }

    sun_robot_msgs::JointTrajectoryFeedback feedbk;
    ros::Rate loop_rate(goal->sampling_freq);
    ros::Time time_now = ros::Time::now();
    sensor_msgs::JointState last_state_no_junction = out_msg;

    for (int i = 1; i < goal->trajectory.points.size(); i++) {
      if (as_traj_->isPreemptRequested()) {
        break;
      }

      const trajectory_msgs::JointTrajectoryPoint &traj_point_prev =
          goal->trajectory.points[i - 1];
      const trajectory_msgs::JointTrajectoryPoint &traj_point_next =
          goal->trajectory.points[i];

      sun::Vector_Independent_Traj traj;
      for (int j = 0; j < goal->trajectory.joint_names.size(); j++) {
        traj.push_back_traj(sun::Quintic_Poly_Traj(
            (traj_point_next.time_from_start - traj_point_prev.time_from_start)
                .toSec(),
            traj_point_prev.positions[j], traj_point_next.positions[j],
            (t0 + traj_point_prev.time_from_start).toSec(),
            traj_point_prev.velocities[j], traj_point_next.velocities[j],
            traj_point_prev.accelerations[j],
            traj_point_next.accelerations[j]));
      }

      // while (
      //     ros::ok() && !as_traj_->isPreemptRequested() &&
      //     // traj compleate and exponential junction terminated (if active)
      //     !(traj.isCompleate(time_now_sec) &&
      //       ((goal->use_exponential_junction && ((time_now_sec - t0.toSec())
      //       > (7.0 * goal->junction_time_constant)))
      //       ||
      //        !goal->use_exponential_junction)))
      while (ros::ok() && !as_traj_->isPreemptRequested() &&
             !traj.isCompleate(time_now.toSec())) {
        time_now = ros::Time::now();

        TooN::Vector<> q = traj.getPosition(time_now.toSec());
        TooN::Vector<> dq = traj.getVelocity(time_now.toSec());
        // TooN::Vector<> ddq = traj.getAcceleration(time_now.toSec());

        for (int i = 0; i < goal->trajectory.joint_names.size(); i++) {
          last_state_no_junction.position[i] = q[i];
          last_state_no_junction.velocity[i] = dq[i];
        }

        if (goal->use_exponential_junction) {
          computeExponentialJunction(
              (time_now - t0).toSec(), goal->junction_time_constant,
              TooN::wrapVector(goal->initial_joints.data(),
                               goal->initial_joints.size()),
              q, dq, q, dq);
        }

        feedbk.time_left = tf - time_now;

        for (int i = 0; i < goal->trajectory.joint_names.size(); i++) {
          out_msg.position[i] = q[i];
          out_msg.velocity[i] = dq[i];
        }

        out_msg.header.stamp = ros::Time::now();

        {
          sensor_msgs::JointStatePtr out_msg_ptr =
              boost::make_shared<sensor_msgs::JointState>(out_msg);
          sun_robot_msgs::JointTrajectoryFeedbackPtr feedbk_ptr =
              boost::make_shared<sun_robot_msgs::JointTrajectoryFeedback>(
                  feedbk);
          joint_state_pub_.publish(out_msg_ptr);
          as_traj_->publishFeedback(feedbk_ptr);

#ifdef DBG
          sensor_msgs::JointStatePtr last_state_no_junction_ptr =
              boost::make_shared<sensor_msgs::JointState>(
                  last_state_no_junction);
          joint_state_no_exp_pub_.publish(last_state_no_junction_ptr);
#endif
        }
        loop_rate.sleep();
      }
    }

    if (as_traj_->isPreemptRequested()) {
      as_traj_->setPreempted();
      return;
    }

    // Wait exponential junction to compleate
    if (goal->use_exponential_junction) {
      // Build last q dq from msg
      TooN::Vector<> last_qn =
          TooN::wrapVector(last_state_no_junction.position.data(),
                           last_state_no_junction.position.size());
      TooN::Vector<> last_dqn =
          TooN::wrapVector(last_state_no_junction.velocity.data(),
                           last_state_no_junction.velocity.size());
      ros::Time time_now = ros::Time::now();
      TooN::Vector<> q = TooN::Zeros(last_state_no_junction.name.size());
      TooN::Vector<> dq = TooN::Zeros(last_state_no_junction.name.size());
      while (ros::ok() && !as_traj_->isPreemptRequested() &&
             ((time_now - t0).toSec()) < (NUM_OF_TIME_CONSTANT_TO_WAIT *
                                          goal->junction_time_constant)) {
        ROS_WARN_ONCE(
            "jont_traj_server: traj compleate but exp not... waiting exp...");
        time_now = ros::Time::now();
        computeExponentialJunction(
            (time_now - t0).toSec(), goal->junction_time_constant,
            TooN::wrapVector(goal->initial_joints.data(),
                             goal->initial_joints.size()),
            last_qn, last_dqn, q, dq);

        feedbk.time_left = tf - time_now;

        for (int i = 0; i < goal->trajectory.joint_names.size(); i++) {
          out_msg.position[i] = q[i];
          out_msg.velocity[i] = dq[i];
        }

        out_msg.header.stamp = ros::Time::now();

        {
          sensor_msgs::JointStatePtr out_msg_ptr =
              boost::make_shared<sensor_msgs::JointState>(out_msg);
          sun_robot_msgs::JointTrajectoryFeedbackPtr feedbk_ptr =
              boost::make_shared<sun_robot_msgs::JointTrajectoryFeedback>(
                  feedbk);
          joint_state_pub_.publish(out_msg_ptr);
          as_traj_->publishFeedback(feedbk_ptr);

#ifdef DBG
          sensor_msgs::JointStatePtr last_state_no_junction_ptr =
              boost::make_shared<sensor_msgs::JointState>(
                  last_state_no_junction);
          joint_state_no_exp_pub_.publish(last_state_no_junction_ptr);
#endif
        }

        loop_rate.sleep();
      }

      if (ros::ok() && !as_traj_->isPreemptRequested()) {
        // At the end publish the last traj point
        last_state_no_junction.header.stamp = ros::Time::now();
        {
          sensor_msgs::JointStatePtr last_state_no_junction_ptr =
              boost::make_shared<sensor_msgs::JointState>(
                  last_state_no_junction);
          joint_state_pub_.publish(last_state_no_junction_ptr);
        }
      }
    }

    if (as_traj_->isPreemptRequested()) {
      as_traj_->setPreempted();
      return;
    }

    as_traj_->setSucceeded();
  }
};

} // namespace sun

#endif