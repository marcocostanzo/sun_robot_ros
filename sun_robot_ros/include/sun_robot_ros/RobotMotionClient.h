
#ifndef SUN_ROBOT_ROS_ROBOTMOTIONCLIENT_H
#define SUN_ROBOT_ROS_ROBOTMOTIONCLIENT_H

#include "actionlib/client/simple_action_client.h"
#include "sun_robot_msgs/CartesianTrajectoryAction.h"
#include "sun_robot_msgs/JointTrajectoryAction.h"
#include "sun_robot_ros/ClikClient.h"
#include "sun_robot_ros/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace sun {
/*!
  This class wraps all the sun robot ros interface for a specific robot
  it uses ClikClient, CartesianTraj, JointTraj
*/
class RobotMotionClient {
private:
  /* data */
public:
  ros::NodeHandle nh_;
  ClikClient clik_;
  ros::ServiceClient sc_fkine_set_end_effector_;
  actionlib::SimpleActionClient<sun_robot_msgs::JointTrajectoryAction>
      ac_joint_trajectory_;
  actionlib::SimpleActionClient<sun_robot_msgs::CartesianTrajectoryAction>
      ac_cartesian_trajectory_;

  std::vector<std::string> joint_trajectory_joints_to_exclude_;

  double traj_generators_sampling_freq = 1000.0;
  double junction_time_constant = 1.0;

  RobotMotionClient(const ros::NodeHandle &nh)
      : nh_(nh), clik_(ros::NodeHandle(nh_, "clik")),
        ac_joint_trajectory_(nh_, "joint_traj_action", true),
        ac_cartesian_trajectory_(nh_, "cartesian_traj_action", true) {
    sc_fkine_set_end_effector_ =
        nh_.serviceClient<sun_robot_msgs::SetEndEffector>("set_end_effector");
  }

  ~RobotMotionClient() = default;

  void waitForServers() {
    clik_.waitForServers();
    ac_joint_trajectory_.waitForServer();
    ac_cartesian_trajectory_.waitForServer();
    if (!sc_fkine_set_end_effector_.waitForExistence(ros::Duration(0.1))) {
      ROS_WARN_STREAM("SERVICE CLIENT FKINE SET END EFFECTOR NOT FOUND");
    }
  }

  void goTo(const std::vector<double> &qf, double max_joint_mean_vel,
            const ros::Time &t0 = ros::Time::now(), bool wait = true,
            const ros::Duration &min_duration = ros::Duration(1.0)) {
    if (max_joint_mean_vel <= 0) {
      throw std::runtime_error("goTo - max_joint_mean_vel has to be positive");
    }
    clik_.stop();
    // find max joint distance
    sensor_msgs::JointState q0 = clik_.get_state().robot_joints;
    double max_distance = 0.0;
    for (int i = 0; i < q0.position.size(); i++) {
      double distance = fabs(q0.position[i] - qf[i]);
      if (distance > max_distance) {
        max_distance = distance;
      }
    }
    ros::Duration duration(max_distance / max_joint_mean_vel);
    std::cout << "goTo DURATION " << duration << "\n";
    std::cout << "goTo max_distance " << max_distance << "\n";
    std::cout << "goTo max_joint_mean_vel " << max_joint_mean_vel << "\n";
    if (duration <= min_duration) {
      duration = min_duration;
    }
    return goTo(qf, duration, t0, wait);
  }

  void goTo(const std::vector<double> &qf, const ros::Duration &duration,
            const ros::Time &t0 = ros::Time::now(), bool wait = true) {
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

    traj = sun::trajExcludeJoints(traj, joint_trajectory_joints_to_exclude_);

    sun_robot_msgs::JointTrajectoryGoal goal;
    goal.trajectory = traj;
    goal.trajectory.header.stamp = t0;
    goal.sampling_freq = traj_generators_sampling_freq;
    goal.use_exponential_junction = false;

    if (wait) {
      ac_joint_trajectory_.sendGoalAndWait(goal);
      if (ac_joint_trajectory_.getState() !=
          actionlib::SimpleClientGoalState::SUCCEEDED) {
        throw std::runtime_error("Fail to execute JointTraj");
      }
    } else {
      ac_joint_trajectory_.sendGoal(goal);
    }
  }

  void executeJointTraj(trajectory_msgs::JointTrajectory traj,
                        bool use_exponential_junction = false,
                        const ros::Time &t0 = ros::Time::now(),
                        bool wait = true) {
    clik_.stop();
    sensor_msgs::JointState q0 = clik_.get_state().robot_joints;
    traj = sun::filterJointNames(traj, q0.name);

    traj = sun::trajExcludeJoints(traj, joint_trajectory_joints_to_exclude_);

    sun_robot_msgs::JointTrajectoryGoal goal;
    goal.trajectory = traj;
    goal.trajectory.header.stamp = t0;
    goal.sampling_freq = traj_generators_sampling_freq;
    goal.use_exponential_junction = use_exponential_junction;
    goal.initial_joints = q0.position;
    goal.junction_time_constant = junction_time_constant;

    if (wait) {
      ac_joint_trajectory_.sendGoalAndWait(goal);

      if (ac_joint_trajectory_.getState() !=
          actionlib::SimpleClientGoalState::SUCCEEDED) {
        throw std::runtime_error("Fail to execute JointTraj");
      }
    } else {
      ac_joint_trajectory_.sendGoal(goal);
    }
  }

  void executeCartesianTraj(sun_robot_msgs::CartesianTrajectory traj,
                            const ros::Time &t0 = ros::Time::now(),
                            bool wait = true, bool use_trapez = false) {
    clik_.stop();
    sun_robot_msgs::ClikGetState::Response clik_state = clik_.get_state();

    sun_robot_msgs::CartesianTrajectoryGoal goal;
    goal.sampling_freq = traj_generators_sampling_freq;
    goal.trajectory = traj; // NOTE: SET T0 _AFTER_ THIS
    goal.trajectory.header.stamp = t0;
    goal.trajectory.header.frame_id = clik_state.ee_pose.header.frame_id;
    goal.use_trapez = use_trapez;

    clik_.mode_position();

    if (wait) {
      ac_cartesian_trajectory_.sendGoalAndWait(goal);
      clik_.stop();

      if (ac_cartesian_trajectory_.getState() !=
          actionlib::SimpleClientGoalState::SUCCEEDED) {
        throw std::runtime_error("Fail to execute CartesianTraj");
      }
    } else {
      ac_cartesian_trajectory_.sendGoal(goal);
    }
  }

  void goTo(const geometry_msgs::Pose &desired_pose,
            const ros::Duration &duration,
            const ros::Time &t0 = ros::Time::now(), bool wait = true,
            bool use_trapez = false, double trapez_lin_vel = 0.0,
            double trapez_rot_vel = 0.0) {
    clik_.stop();

    sun_robot_msgs::ClikGetState::Response clik_state = clik_.get_state();

    sun_robot_msgs::CartesianTrajectoryGoal goal;
    goal.trajectory.header.stamp = t0;
    goal.sampling_freq = traj_generators_sampling_freq;
    goal.trajectory.header.frame_id = clik_state.ee_pose.header.frame_id;
    goal.use_trapez = use_trapez;

    goal.trajectory.points.resize(2);

    // set vel
    geometry_msgs::Twist twist_zero;
    twist_zero.linear.x = 0.0;
    twist_zero.linear.y = 0.0;
    twist_zero.linear.z = 0.0;
    twist_zero.angular.x = 0.0;
    twist_zero.angular.y = 0.0;
    twist_zero.angular.z = 0.0;
    if (use_trapez) {

      geometry_msgs::Vector3 lin_direction;
      {
        tf2::Vector3 lin_direction_tf(
            desired_pose.position.x - clik_state.ee_pose.pose.position.x,
            desired_pose.position.y - clik_state.ee_pose.pose.position.y,
            desired_pose.position.z - clik_state.ee_pose.pose.position.z);
        lin_direction_tf.normalize();
        lin_direction_tf = lin_direction_tf * trapez_lin_vel;
        lin_direction.x = lin_direction_tf.x();
        lin_direction.y = lin_direction_tf.y();
        lin_direction.z = lin_direction_tf.z();
      }

      geometry_msgs::Vector3 rot_direction;
      {
        tf2::Quaternion quat_prev(clik_state.ee_pose.pose.orientation.x,
                                  clik_state.ee_pose.pose.orientation.y,
                                  clik_state.ee_pose.pose.orientation.z,
                                  clik_state.ee_pose.pose.orientation.w);

        tf2::Quaternion quat_next(
            desired_pose.orientation.x, desired_pose.orientation.y,
            desired_pose.orientation.z, desired_pose.orientation.w);

        { // continuity
          tf2::Vector3 quat_next_V(quat_next.x(), quat_next.y(), quat_next.z());
          tf2::Vector3 quat_prev_V(quat_prev.x(), quat_prev.y(), quat_prev.z());

          if ((quat_next_V.dot(quat_prev_V)) < -0.01) {
            quat_next = -quat_next;
          }
        }

        tf2::Quaternion Delta_Q = quat_next * (quat_prev.inverse());

        tf2::Vector3 rot_direction_tf = Delta_Q.getAxis();
        rot_direction_tf.normalize();
        rot_direction_tf = rot_direction_tf * trapez_rot_vel;

        rot_direction.x = rot_direction_tf.x();
        rot_direction.y = rot_direction_tf.y();
        rot_direction.z = rot_direction_tf.z();
      }

      goal.trajectory.points[0].velocity.linear = lin_direction;
      goal.trajectory.points[0].velocity.angular = rot_direction;
      goal.trajectory.points[0].acceleration = twist_zero;
      goal.trajectory.points[1].velocity = twist_zero;
      goal.trajectory.points[1].acceleration = twist_zero;

    } else {
      goal.trajectory.points[0].velocity = twist_zero;
      goal.trajectory.points[0].acceleration = twist_zero;
      goal.trajectory.points[1].velocity = twist_zero;
      goal.trajectory.points[1].acceleration = twist_zero;
    }

    goal.trajectory.points[0].time_from_start = ros::Duration(0.0);
    goal.trajectory.points[0].pose = clik_state.ee_pose.pose;

    goal.trajectory.points[1].time_from_start = duration;
    goal.trajectory.points[1].pose = desired_pose;

    clik_.mode_position();

    if (wait) {

      ac_cartesian_trajectory_.sendGoalAndWait(goal);

      if (ac_cartesian_trajectory_.getState() !=
          actionlib::SimpleClientGoalState::SUCCEEDED) {
        throw std::runtime_error("Fail to execute LineSegmentTraj");
      }

      clik_.stop();
    }

    ac_cartesian_trajectory_.sendGoal(goal);
  }

  void goToDeltaEE(const geometry_msgs::Pose &desired_pose,
                   const ros::Duration &duration,
                   const ros::Time &t0 = ros::Time::now(), bool wait = true,
                   bool use_trapez = false, double trapez_lin_vel = 0.0,
                   double trapez_rot_vel = 0.0) {
    clik_.stop();

    sun_robot_msgs::ClikGetState::Response clik_state = clik_.get_state();

    geometry_msgs::TransformStamped w_Tr_g;
    poseStampedToTransformStamped(clik_state.ee_pose, w_Tr_g);

    geometry_msgs::Pose desired_pose_base;
    tf2::doTransform(desired_pose, desired_pose_base, w_Tr_g);

    return goTo(desired_pose_base, duration, t0, wait, use_trapez,
                trapez_lin_vel, trapez_rot_vel);
  }

  void goToDeltaEEInBaseFrame(const geometry_msgs::Point &translation,
                              const geometry_msgs::Quaternion &deltaRotation,
                              const ros::Duration &duration,
                              const ros::Time &t0 = ros::Time::now(),
                              bool wait = true, bool use_trapez = false,
                              double trapez_lin_vel = 0.0,
                              double trapez_rot_vel = 0.0) {
    clik_.stop();

    sun_robot_msgs::ClikGetState::Response clik_state = clik_.get_state();

    tf2::Quaternion b_quat_ee;
    tf2::fromMsg(clik_state.ee_pose.pose.orientation, b_quat_ee);
    clik_state.ee_pose.pose.position;
    tf2::Quaternion b_delta_quat;
    tf2::fromMsg(deltaRotation, b_delta_quat);

    tf2::Quaternion b_quat_desired = b_delta_quat * b_quat_ee;

    geometry_msgs::Pose b_pose_derired;
    b_pose_derired.position.x =
        clik_state.ee_pose.pose.position.x + translation.x;
    b_pose_derired.position.y =
        clik_state.ee_pose.pose.position.y + translation.y;
    b_pose_derired.position.z =
        clik_state.ee_pose.pose.position.z + translation.z;
    b_pose_derired.orientation = tf2::toMsg(b_quat_desired);

    return goTo(b_pose_derired, duration, t0, wait, use_trapez, trapez_lin_vel,
                trapez_rot_vel);
  }

  //! n_pose_ee = end effector pose w.r.t. link n
  void fkine_set_end_effector(const geometry_msgs::Pose &n_pose_ee) {
    ROS_INFO_STREAM("fkine_set_end_effector\n" << n_pose_ee);
    sun_robot_msgs::SetEndEffector msg;
    msg.request.n_pose_ee = n_pose_ee;

    bool success = sc_fkine_set_end_effector_.call(msg);
    success = success && msg.response.success;
    if (!success) {
      throw std::runtime_error(
          "RobotMotionClient: set_end_effector no success");
    }
  }

  //! n_pose_ee = end effector pose w.r.t. link n
  void set_end_effector(const geometry_msgs::Pose &n_pose_ee) {
    sc_fkine_set_end_effector_.waitForExistence(ros::Duration(0.5));
    // if (sc_fkine_set_end_effector_.exists()) {
    ROS_WARN_STREAM("sc_fkine_set_end_effector_ CALL");
    fkine_set_end_effector(n_pose_ee);
    // }
    // ROS_WARN_STREAM("sc_fkine_set_end_effector_ OK exists");
    clik_.set_end_effector(n_pose_ee);
  }

  static void
  poseStampedToTransformStamped(const geometry_msgs::PoseStamped &pose,
                                geometry_msgs::TransformStamped &transform,
                                const std::string &child_frame_id = "noframe") {
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