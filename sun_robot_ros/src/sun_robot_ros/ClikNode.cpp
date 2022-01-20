
/*
    Clik Robot Class
    Copyright 2020 Università della Campania Luigi Vanvitelli
    Author: Marco Costanzo <marco.costanzo@unicampania.it>
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "sun_robot_ros/ClikNode.h"
#include "std_msgs/Float64MultiArray.h"
#include "sun_robot_ros/check_realtime.h"

namespace sun {

TooN::Vector<> getTooNVectorFromParam(const ros::NodeHandle &nh,
                                      const std::string &param_str,
                                      int expected_size,
                                      const TooN::Vector<> &default_value) {
  if (nh.hasParam(param_str)) {
    std::vector<double> vec_std;
    nh.getParam(param_str, vec_std);
    if (vec_std.size() != expected_size) {
      ROS_ERROR_STREAM(ros::this_node::getName()
                       << " CLIK Error: " << param_str << " size mismatch");
      throw robot_invalid_parameter(param_str + " size mismatch");
    }
    return Clik::getTooNFromSTD(vec_std);
  } else {
    return default_value;
  }
}

std::string to_string(const std::vector<std::string> &in,
                      const std::string &separator = " ") {
  std::string out;
  if (in.size() > 0) {
    out += in[0];
  }
  for (int i = 1; i < in.size(); i++) {
    out += separator + in[i];
  }
  return out;
}

std::string to_string(const std::vector<unsigned int> &in,
                      const std::string &separator = " ") {
  std::string out;
  if (in.size() > 0) {
    out += std::to_string(in[0]);
  }
  for (int i = 1; i < in.size(); i++) {
    out += separator + std::to_string(in[i]);
  }
  return out;
}

template <typename T> void remove_duplicates(std::vector<T> &v) {
  auto end = v.end();
  for (auto i = v.begin(); i != end; ++i) {
    end = std::remove(i + 1, end, *i);
  }
  v.erase(end, v.end());
}

template <typename T>
std::vector<T> getSTDVectorFromParam(const ros::NodeHandle &nh,
                                     const std::string &param_str,
                                     const std::vector<T> &default_value) {
  if (nh.hasParam(param_str)) {
    std::vector<T> vec_std;
    nh.getParam(param_str, vec_std);
    return vec_std;
  } else {
    return default_value;
  }
}

std::vector<unsigned int> ClikNode::jointNamesToJointIndex(
    const std::vector<std::string> &joint_names) const {
  std::vector<unsigned int> out;
  for (const auto &joint_name : joint_names) {
    auto it =
        std::find(ros_joint_names_.begin(), ros_joint_names_.end(), joint_name);
    if (it != ros_joint_names_.end()) {
      out.push_back(it - ros_joint_names_.begin());
    }
  }
  return out;
}

void ClikNode::updateParams(const ros::NodeHandle &nh_for_params) {
  // params
  nh_for_params.param("use_realtime", b_use_realtime_, false);

  nh_for_params.param("pub_dbg", b_pub_dbg_, false);

  nh_for_params.param("pub_cartesian_twist_control",
                      b_pub_cartesian_twist_control_, false);

  nh_for_params.param("pub_robot_fkine", b_publish_robot_fkine_, true);

  nh_for_params.param("pub_robot_jacobian", b_publish_robot_jacobian_, false);

  {
    nh_for_params.param("rate", clik_integrator_.Ts_, 1000.0);
    clik_integrator_.Ts_ = 1.0 / clik_integrator_.Ts_;
  }
  {
    double error_gain;
    nh_for_params.param("error_gain", error_gain, 0.5);
    error_gain = error_gain / clik_integrator_.Ts_;
    clik_->setGainError(error_gain);
  }

  ros_joint_names_ =
      getSTDVectorFromParam<std::string>(nh_for_params, "ros_joint_names", {});
  nh_for_params.param("ros_base_frame_id", ros_base_frame_id_,
                      std::string("clik_base_frame"));

  {
    double dls_joint_speed_saturation;
    nh_for_params.param("dls_joint_speed_saturation",
                        dls_joint_speed_saturation, 5.0);
    clik_->setDLSJointSpeedSaturation(dls_joint_speed_saturation);
  }
  {
    double second_obj_gain;
    nh_for_params.param("second_obj_gain", second_obj_gain, 0.0);
    clik_->setGainNullSpace(second_obj_gain);
  }

  // joint_target_robot_second_obj
  {
    TooN::Vector<> secondObj_desiredConfig =
        getTooNVectorFromParam(nh_for_params, "joint_target_robot_second_obj",
                               clik_->robot_->getNumJoints(),
                               clik_->robot_->getCenterOfSoftJointLimits());
    secondObjTargetConfig_->setDesiredConfiguration(
        clik_->robot_->joints_Robot2DH(secondObj_desiredConfig));
    ROS_INFO_STREAM(ros::this_node::getName()
                    << " CLIK Target conf (DH): "
                    << secondObjTargetConfig_->getDesiredConfiguration());
  }

  // joint_weights_second_obj
  secondObjTargetConfig_->setDesiredConfigurationJointWeights(
      getTooNVectorFromParam(nh_for_params, "joint_weights_second_obj",
                             clik_->robot_->getNumJoints(),
                             TooN::Ones(clik_->robot_->getNumJoints())));
  ROS_INFO_STREAM(
      ros::this_node::getName()
      << " CLIK Target conf weights: "
      << secondObjTargetConfig_->getDesiredConfigurationJointWeights());

  {
    // fixed_joints
    std::vector<std::string> fixed_joints_names =
        getSTDVectorFromParam<std::string>(nh_for_params, "fixed_joints_names",
                                           {});
    remove_duplicates(fixed_joints_names);
    std::vector<unsigned int> fixed_joints_index =
        jointNamesToJointIndex(fixed_joints_names);
    std::sort(fixed_joints_index.begin(), fixed_joints_index.end());
    clik_->setFixedJoints(fixed_joints_index);
    ROS_INFO_STREAM(ros::this_node::getName()
                    << "fixed_joints_names:\n"
                    << to_string(fixed_joints_names, ", ")
                    << "fixed_joint_index:\n"
                    << to_string(fixed_joints_index, ", "));
  }
  // n_T_e
  {
    if (nh_for_params.hasParam("n_T_e_position") ||
        nh_for_params.hasParam("n_T_e_quaternion")) {
      if (!nh_for_params.hasParam("n_T_e_position") ||
          !nh_for_params.hasParam("n_T_e_quaternion")) {
        throw std::runtime_error("The n_T_e must be fully defined");
      }
    }
    if (nh_for_params.hasParam("n_T_e_position") &&
        nh_for_params.hasParam("n_T_e_quaternion")) {
      TooN::Vector<3> n_T_e_position = getTooNVectorFromParam(
          nh_for_params, "n_T_e_position", 3, TooN::Zeros(3));
      TooN::Vector<4> n_T_e_quat_v =
          getTooNVectorFromParam(nh_for_params, "n_T_e_quaternion", 4,
                                 TooN::makeVector(1.0, 0.0, 0.0, 0.0));
      UnitQuaternion n_T_e_quaternion(n_T_e_quat_v);
      TooN::Matrix<4, 4> n_T_e = transl(n_T_e_position);
      n_T_e.slice<0, 0, 3, 3>() = n_T_e_quaternion.torot();
      clik_->robot_->setnTe(n_T_e);
      ROS_INFO_STREAM(ros::this_node::getName() << " CLIK nTe: \n"
                                                << clik_->robot_->getnTe());
    }
  }

  // b_T_0
  {
    if (nh_for_params.hasParam("b_T_0_position") ||
        nh_for_params.hasParam("b_T_0_quaternion")) {
      if (!nh_for_params.hasParam("b_T_0_position") ||
          !nh_for_params.hasParam("b_T_0_quaternion")) {
        throw std::runtime_error("The base transform must be fully defined");
      }
    }
    if (nh_for_params.hasParam("b_T_0_position") &&
        nh_for_params.hasParam("b_T_0_quaternion")) {
      TooN::Vector<3> b_T_0_position = getTooNVectorFromParam(
          nh_for_params, "b_T_0_position", 3, TooN::Zeros(3));
      TooN::Vector<4> b_T_0_quat_v =
          getTooNVectorFromParam(nh_for_params, "b_T_0_quaternion", 4,
                                 TooN::makeVector(1.0, 0.0, 0.0, 0.0));
      UnitQuaternion b_T_0_quaternion(b_T_0_quat_v);
      TooN::Matrix<4, 4> b_T_0 = transl(b_T_0_position);
      b_T_0.slice<0, 0, 3, 3>() = b_T_0_quaternion.torot();
      clik_->robot_->setbT0(b_T_0);
    }
    ROS_INFO_STREAM(ros::this_node::getName() << " CLIK bT0: \n"
                                              << clik_->robot_->getbT0());
  }
}

ClikNode::ClikNode(const std::shared_ptr<Robot> &robot,
                   const ros::NodeHandle &nh_for_topics,
                   const ros::NodeHandle &nh_for_params)
    : clik_(std::make_shared<Clik6DQuaternionSingleRobot>(robot)),
      secondObjTargetConfig_(
          std::make_shared<JointVelocityTargetConfiguration>(robot)),
      clik_integrator_(TooN::Zeros(robot->getNumJoints())),
      nh_(nh_for_topics, "") {

  // arrange shared pointers
  clik_->secondObjQdotDHgenerator_ = secondObjTargetConfig_;
  clik_integrator_.jointVelocityGenerator_ = clik_;

  updateParams(nh_for_params);

  nh_.setCallbackQueue(&callbk_queue_);
}

void ClikNode::spinOnce(const ros::WallDuration &timeout) {
  callbk_queue_.callAvailable(timeout);
}

/* Getters */

int ClikNode::getMode() { return mode_; }

std::shared_ptr<Clik6DQuaternionSingleRobot> &ClikNode::getClik() {
  return clik_;
}

std::shared_ptr<Robot> &ClikNode::getRobot() { return clik_->robot_; }

/* RUNNERS */

void ClikNode::reset_desired_cartesian_pose() {

  TooN::Matrix<4, 4> b_T_e =
      clik_->robot_->fkine(clik_integrator_.getJointsDH());
  clik_->desiredPosition_ = transl(b_T_e);
  clik_->desiredQuaternion_ = UnitQuaternion(b_T_e);
  clik_->resetCurrentQaternion();
}

void ClikNode::reset_and_sync_with_robot() {
  // Wait for initial configuration
  ROS_INFO_STREAM(ros::this_node::getName() << " CLIK refresh()");
  ROS_INFO_STREAM(ros::this_node::getName()
                  << " CLIK Wait for joint positions...");
  clik_integrator_.setJointsDH(
      clik_->robot_->joints_Robot2DH(getJointPositionRobot()));
  clik_integrator_.resetJointsVelDH();
  ROS_INFO_STREAM(ros::this_node::getName() << " CLIK Joint positions: "
                                            << clik_integrator_.getJointsDH());
  // Initialize vars
  reset_desired_cartesian_pose();
  clik_->resetDesiredCartesianTwist();
  ROS_INFO_STREAM(ros::this_node::getName() << " CLIK Initialized!");
}

bool ClikNode::getState_srv_cb(sun_robot_msgs::ClikGetState::Request &req,
                               sun_robot_msgs::ClikGetState::Response &res) {
  if (mode_ == sun_robot_msgs::ClikSetMode::Request::MODE_STOP) {
    reset_and_sync_with_robot();
  }

  ros::Time time_now = ros::Time::now();

  res.mode = mode_;

  res.robot_joints.header.stamp = time_now;
  res.dh_joints.header.stamp = time_now;
  res.robot_joints.name = ros_joint_names_;
  res.dh_joints.name = ros_joint_names_;
  TooN::Vector<> qR =
      clik_->robot_->joints_DH2Robot(clik_integrator_.getJointsDH());
  for (int i = 0; i < qR.size(); i++) {
    res.robot_joints.position.push_back(qR[i]);
    res.dh_joints.position.push_back(clik_integrator_.getJointsDH()[i]);
  }

  res.ee_pose.header.stamp = time_now;
  res.ee_pose.header.frame_id = ros_base_frame_id_;
  TooN::Matrix<4, 4> b_T_e =
      clik_->robot_->fkine(clik_integrator_.getJointsDH());
  TooN::Vector<3> b_p_e = transl(b_T_e);
  UnitQuaternion b_Q_e(b_T_e);
  res.ee_pose.pose.position.x = b_p_e[0];
  res.ee_pose.pose.position.y = b_p_e[1];
  res.ee_pose.pose.position.z = b_p_e[2];
  res.ee_pose.pose.orientation.w = b_Q_e.getS();
  res.ee_pose.pose.orientation.x = b_Q_e.getV()[0];
  res.ee_pose.pose.orientation.y = b_Q_e.getV()[1];
  res.ee_pose.pose.orientation.z = b_Q_e.getV()[2];

  return true;
}

bool ClikNode::setFixedJoints_srv_cb(
    sun_robot_msgs::ClikSetFixedJoints::Request &req,
    sun_robot_msgs::ClikSetFixedJoints::Response &res) {
  if (mode_ != sun_robot_msgs::ClikSetMode::Request::MODE_STOP) {
    res.success = false;
    res.message = "Clik is not in STOP mode";
    return true;
  }

  ROS_INFO_STREAM("Set fixed joints: req: " << req);

  // fixed_joints
  std::vector<std::string> fixed_joints_names = req.fixed_joints_names;
  remove_duplicates(fixed_joints_names);
  std::vector<unsigned int> fixed_joints_index =
      jointNamesToJointIndex(fixed_joints_names);
  std::sort(fixed_joints_index.begin(), fixed_joints_index.end());
  clik_->setFixedJoints(fixed_joints_index);
  ROS_INFO_STREAM(ros::this_node::getName()
                  << "New fixed_joints_names:\n"
                  << to_string(fixed_joints_names, ", ") << "\n"
                  << "New fixed_joint_index:\n"
                  << to_string(fixed_joints_index, ", "));

  res.success = true;
  return true;
}

bool ClikNode::setSecondaryObj_srv_cb(
    sun_robot_msgs::ClikSetSecondaryObj::Request &req,
    sun_robot_msgs::ClikSetSecondaryObj::Response &res) {
  /*
  Check inputs
*/
  if (req.second_obj_joint_velocity_topic != "" &&
      (req.desired_joint_configuration.size() != 0 ||
       req.desired_joint_configuration_weights.size() != 0)) {
    ROS_ERROR_STREAM(
        ros::this_node::getName()
        << " Second obj - Cannot set both topic and configuration objective");
    res.success = false;
    return true;
  }

  if (req.desired_joint_configuration.size() != 0 &&
      req.desired_joint_configuration.size() != clik_->robot_->getNumJoints()) {
    ROS_ERROR_STREAM(ros::this_node::getName()
                     << " Second obj - Invalid desired_joint_configuration");
    res.success = false;
    return true;
  }

  if (req.desired_joint_configuration_weights.size() != 0 &&
      req.desired_joint_configuration_weights.size() !=
          clik_->robot_->getNumJoints() &&
      req.desired_joint_configuration_weights.size() != 1) {
    ROS_ERROR_STREAM(
        ros::this_node::getName()
        << " Second obj - Invalid desired_joint_configuration_weights");
    res.success = false;
    return true;
  }

  for (const auto &w : req.desired_joint_configuration_weights) {
    if (w < 0) {
      ROS_ERROR_STREAM(
          ros::this_node::getName()
          << " Second obj - Invalid desired_joint_configuration_weights, "
             "someone is negative!");
      res.success = false;
      return true;
    }
  }

  /*
  Check not implemented
*/
  if (req.second_obj_joint_velocity_topic != "") {
    ROS_ERROR_STREAM(
        ros::this_node::getName()
        << " Second obj - Objective from topic not implemented yet");
    res.success = false;
    return true;
  }

  /*
  Print
*/
  if (req.second_obj_gain == 0) {
    ROS_WARN_STREAM(ros::this_node::getName() << " Second Obj Disabled");
  } else if (req.second_obj_gain > 0) {
    ROS_WARN_STREAM(ros::this_node::getName() << " Second Obj Enabled");
  }

  if (req.second_obj_gain >= 0) {
    clik_->setGainNullSpace(req.second_obj_gain);
  }

  if (req.second_obj_joint_velocity_topic != "") {
    /*
    Impl
  */
    ROS_ERROR_STREAM(ros::this_node::getName()
                     << " FATAL ERROR! I should not stay here");
    res.success = false;
    return true;
  }

  if (req.desired_joint_configuration.size() != 0) {
    auto target_second_obj =
        Clik::getTooNFromSTD(req.desired_joint_configuration);
    ROS_INFO_STREAM(ros::this_node::getName()
                    << " New target_second_obj (Robot):\n"
                    << target_second_obj << "\n");
    secondObjTargetConfig_->setDesiredConfiguration(
        clik_->robot_->joints_Robot2DH(target_second_obj));
  }

  if (req.desired_joint_configuration_weights.size() != 0) {
    if (req.desired_joint_configuration_weights.size() == 1) {
      secondObjTargetConfig_->setDesiredConfigurationJointWeights(
          req.desired_joint_configuration_weights[0] *
          TooN::Ones(clik_->robot_->getNumJoints()));
    } else {
      secondObjTargetConfig_->setDesiredConfigurationJointWeights(
          Clik::getTooNFromSTD(req.desired_joint_configuration_weights));
    }
    ROS_INFO_STREAM(
        ros::this_node::getName()
        << " New weights_second_obj:\n"
        << secondObjTargetConfig_->getDesiredConfigurationJointWeights()
        << "\n");
  }

  res.success = true;
  return true;
}

bool ClikNode::setEndEffector_srv_cb(
    sun_robot_msgs::SetEndEffector::Request &req,
    sun_robot_msgs::SetEndEffector::Response &res) {
  if (mode_ != sun_robot_msgs::ClikSetMode::Request::MODE_STOP) {
    ROS_ERROR_STREAM(
        ros::this_node::getName()
        << " setEndEffector service: it is possible only in MODE_STOP");
    res.success = false;
    return true;
  }

  TooN::Vector<3> n_T_e_position =
      TooN::makeVector(req.n_pose_ee.position.x, req.n_pose_ee.position.y,
                       req.n_pose_ee.position.z);
  UnitQuaternion n_T_e_quaternion(
      req.n_pose_ee.orientation.w,
      TooN::makeVector(req.n_pose_ee.orientation.x, req.n_pose_ee.orientation.y,
                       req.n_pose_ee.orientation.z));
  TooN::Matrix<4, 4> n_T_e = transl(n_T_e_position);
  n_T_e.slice<0, 0, 3, 3>() = n_T_e_quaternion.torot();
  clik_->robot_->setnTe(n_T_e);

  reset_and_sync_with_robot();

  res.success = true;

  return true;
}

bool ClikNode::setMode_srv_cb(sun_robot_msgs::ClikSetMode::Request &req,
                              sun_robot_msgs::ClikSetMode::Response &res) {
  switch (req.mode) {
  case sun_robot_msgs::ClikSetMode::Request::MODE_STOP: {
    ROS_WARN_STREAM(ros::this_node::getName() << " CLIK Stopped!");
    break;
  }
  case sun_robot_msgs::ClikSetMode::Request::MODE_POSITION: {
    clik_->b_desired_twist_only_ = false;
    ROS_WARN_STREAM(ros::this_node::getName() << " CLIK MODE_POSITION");
    break;
  }
  case sun_robot_msgs::ClikSetMode::Request::MODE_VELOCITY: {
    clik_->b_desired_twist_only_ = true;
    ROS_WARN_STREAM(ros::this_node::getName() << " CLIK MODE_VELOCITY");
    break;
  }
  case sun_robot_msgs::ClikSetMode::Request::MODE_VELOCITY_EE: {
    clik_->b_desired_twist_only_ = true;
    ROS_WARN_STREAM(ros::this_node::getName() << " CLIK MODE_VELOCITY_EE");
    break;
  }
  default: {
    ROS_ERROR_STREAM(ros::this_node::getName()
                     << " CLIK Error in setMode(): Invalid mode in request!");
    res.success = false;
    return true;
  }
  }

  if (req.mode == sun_robot_msgs::ClikSetMode::Request::MODE_STOP ||
      mode_ != req.mode) {
    reset_and_sync_with_robot();
  }
  mode_ = req.mode;

  res.success = true;
  return true;
}

void ClikNode::run() {
  run_init();
  while (ros::ok()) {
    run_single_step();
  }
}

void ClikNode::run_init() {

  try {

    if (b_use_realtime_) {

      if (!check_realtime()) {
        throw std::runtime_error("REALTIME NOT AVAILABLE");
      }

      if (!set_realtime_SCHED_FIFO()) {
        throw std::runtime_error("ERROR IN set_realtime_SCHED_FIFO");
      }

      std::cout << "[CLIK NODE] REALTIME MODE SCHED_FIFO!\n";
    }

    reset_and_sync_with_robot();

    // Initialize subscribers
    desired_pose_sub_ =
        nh_.subscribe("desired_pose", 1, &ClikNode::desiredPose_cb, this);
    desired_twist_sub_ =
        nh_.subscribe("desired_twist", 1, &ClikNode::desiredTwist_cb, this);
    desired_pose_twis_sub_ = nh_.subscribe(
        "desired_pose_twist", 1, &ClikNode::desiredPoseTwist_cb, this);

    // Publish
    cartesian_error_pub_ =
        nh_.advertise<sun_ros_msgs::Float64Stamped>("cartesian_error", 1);
    if (b_pub_cartesian_twist_control_) {
      cartesian_twist_control_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
          "cartesian_twist_control", 1);
    }
    if (b_publish_robot_fkine_) {
      robot_fkine_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("fkine", 1);
    }
    if (b_publish_robot_jacobian_) {
      robot_jacob_pub_ =
          nh_.advertise<std_msgs::Float64MultiArray>("jacobian", 1);
    }
    if (b_pub_dbg_) {
      joi_state_pub_dbg_ =
          nh_.advertise<sensor_msgs::JointState>("dbg/command_joint_state", 1);
      twist_pub_dbg_ =
          nh_.advertise<geometry_msgs::TwistStamped>("dbg/twist", 1);
      clikError_pub_dbg_ =
          nh_.advertise<geometry_msgs::TwistStamped>("dbg/error", 1);
      pos_posdes_pub_dbg_ =
          nh_.advertise<geometry_msgs::TwistStamped>("dbg/pos_posdes", 1);
    }

    // Init Services
    serviceSetMode_ =
        nh_.advertiseService("set_mode", &ClikNode::setMode_srv_cb, this);
    serviceGetState_ =
        nh_.advertiseService("get_state", &ClikNode::getState_srv_cb, this);
    serviceSetEndEffector_ = nh_.advertiseService(
        "set_end_effector", &ClikNode::setEndEffector_srv_cb, this);
    serviceSetSecondObj_ = nh_.advertiseService(
        "set_second_obj", &ClikNode::setSecondaryObj_srv_cb, this);
    serviceSetFixedJoints_ = nh_.advertiseService(
        "set_fixed_joints", &ClikNode::setFixedJoints_srv_cb, this);

    loop_rate_ =
        std::unique_ptr<ros::Rate>(new ros::Rate(1.0 / clik_integrator_.Ts_));

  } catch (const std::exception &e) {
    ROS_ERROR_STREAM(ros::this_node::getName() << e.what());
    std::cerr << e.what() << '\n'; // or whatever
    throw;
  } catch (...) {
    // well ok, still unknown what to do now,
    // but a std::exception_ptr doesn't help the situation either.
    ROS_ERROR_STREAM(ros::this_node::getName() << "unknown exception\n");
    std::cerr << "unknown exception\n";
    std::rethrow_exception(std::current_exception());
  }
}

void ClikNode::run_single_step() {

  try {

    loop_rate_->sleep();
    spinOnce();

    geometry_msgs::TwistStampedPtr cartesian_twist_control_msg(
        new geometry_msgs::TwistStamped);
    if (b_pub_cartesian_twist_control_) {
      TooN::Vector<> cartesian_twist_control =
          clik_->getCartesianTwistControl(clik_integrator_.getJointsDH());
      cartesian_twist_control_msg->twist.linear.x = cartesian_twist_control[0];
      cartesian_twist_control_msg->twist.linear.y = cartesian_twist_control[1];
      cartesian_twist_control_msg->twist.linear.z = cartesian_twist_control[2];
      cartesian_twist_control_msg->twist.angular.x = cartesian_twist_control[3];
      cartesian_twist_control_msg->twist.angular.y = cartesian_twist_control[4];
      cartesian_twist_control_msg->twist.angular.z = cartesian_twist_control[5];
    }

    bool b_publish_joints = false;

    switch (mode_) {
    case sun_robot_msgs::ClikSetMode::Request::MODE_STOP: {
      break;
    }
    case sun_robot_msgs::ClikSetMode::Request::MODE_POSITION: {
      clik_core();
      b_publish_joints = true;
      break;
    }
    case sun_robot_msgs::ClikSetMode::Request::MODE_VELOCITY:
    case sun_robot_msgs::ClikSetMode::Request::MODE_VELOCITY_EE: {
      reset_desired_cartesian_pose();
      clik_core();
      b_publish_joints = true;
      break;
    }
    default: {
      ROS_ERROR_STREAM(ros::this_node::getName()
                       << " CLIK Error in main while: Invalid mode!");
      throw clik_invalid_mode("non valid modality " + mode_);
    }
    }

    if (b_pub_dbg_) {
      pub_dbg();
    }

    if (b_publish_joints) {
      publishJointRobot(
          clik_->robot_->joints_DH2Robot(clik_integrator_.getJointsDH()),
          clik_->robot_->jointsvel_DH2Robot(clik_integrator_.getJointsVelDH()));
    }

    if (b_pub_cartesian_twist_control_) {
      cartesian_twist_control_msg->header.stamp = ros::Time::now();
      cartesian_twist_control_pub_.publish(cartesian_twist_control_msg);
    }

    if (b_publish_robot_fkine_) {
      TooN::Matrix<4, 4> b_T_e = clik_->robot_->fkine(
          clik_->robot_->joints_Robot2DH(getJointPositionRobot(false)));
      sun::UnitQuaternion b_Q_e(b_T_e, clik_->getCurrentQuaternion());
      auto b_p_e = sun::transl(b_T_e);
      geometry_msgs::PoseStampedPtr fkine_msg(new geometry_msgs::PoseStamped);
      fkine_msg->header.stamp = ros::Time::now();
      fkine_msg->pose.position.x = b_p_e[0];
      fkine_msg->pose.position.y = b_p_e[1];
      fkine_msg->pose.position.z = b_p_e[2];
      fkine_msg->pose.orientation.w = b_Q_e.getS();
      fkine_msg->pose.orientation.x = b_Q_e.getV()[0];
      fkine_msg->pose.orientation.y = b_Q_e.getV()[1];
      fkine_msg->pose.orientation.z = b_Q_e.getV()[2];

      robot_fkine_pub_.publish(fkine_msg);
    }

    if (b_publish_robot_jacobian_) {
      auto jacob = clik_->robot_->jacob_geometric(
          clik_->robot_->joints_Robot2DH(getJointPositionRobot(false)));

      std_msgs::Float64MultiArrayPtr jacob_msg =
          boost::make_shared<std_msgs::Float64MultiArray>();

      auto jacob_size = jacob.num_cols() * jacob.num_rows();
      jacob_msg->data.reserve(jacob_size);
      memcpy(jacob_msg->data.data(), jacob.get_data_ptr(), jacob_size);

      jacob_msg->layout.data_offset = 0.0;
      jacob_msg->layout.dim.resize(2);
      jacob_msg->layout.dim[0].label = "rows";
      jacob_msg->layout.dim[0].size = jacob.num_rows();
      jacob_msg->layout.dim[0].stride =
          jacob.num_cols() *
          jacob.num_rows(); // dim[0] stride is just the total size
      jacob_msg->layout.dim[1].label = "cols";
      jacob_msg->layout.dim[2].size = jacob.num_cols();
      jacob_msg->layout.dim[3].stride = jacob.num_cols();

      robot_jacob_pub_.publish(jacob_msg);
    }

    // Publish clik error norm
    sun_ros_msgs::Float64StampedPtr cartesian_error_msg(
        new sun_ros_msgs::Float64Stamped);
    cartesian_error_msg->header.stamp = ros::Time::now();
    cartesian_error_msg->data =
        norm(clik_->getClikError(clik_integrator_.getJointsDH()));
    cartesian_error_pub_.publish(cartesian_error_msg);

  } catch (const std::exception &e) {
    ROS_ERROR_STREAM(ros::this_node::getName() << e.what());
    std::cerr << e.what() << '\n'; // or whatever
    throw;
  } catch (...) {
    // well ok, still unknown what to do now,
    // but a std::exception_ptr doesn't help the situation either.
    ROS_ERROR_STREAM(ros::this_node::getName() << "unknown exception\n");
    std::cerr << "unknown exception\n";
    std::rethrow_exception(std::current_exception());
  }
}

void ClikNode::pub_dbg() {
  ros::Time time_now = ros::Time::now();
  TooN::Vector<> qDH = clik_integrator_.getJointsDH();
  TooN::Vector<> qDH_dot = clik_integrator_.getJointsVelDH();
  TooN::Vector<> x_dot =
      clik_->robot_->jacob_geometric(clik_integrator_.getJointsDH()) *
      clik_integrator_.getJointsVelDH();
  sensor_msgs::JointStatePtr joi_state(new sensor_msgs::JointState);
  joi_state->header.stamp = time_now;
  joi_state->name = ros_joint_names_;
  joi_state->position.resize(qDH.size());
  joi_state->velocity.resize(qDH_dot.size());
  for (int i = 0; i < qDH.size(); i++) {
    joi_state->position[i] = qDH[i];
    joi_state->velocity[i] = qDH_dot[i];
  }
  geometry_msgs::TwistStampedPtr twist_msg(new geometry_msgs::TwistStamped);
  twist_msg->header.stamp = time_now;
  twist_msg->twist.linear.x = x_dot[0];
  twist_msg->twist.linear.y = x_dot[1];
  twist_msg->twist.linear.z = x_dot[2];
  twist_msg->twist.angular.x = x_dot[3];
  twist_msg->twist.angular.y = x_dot[4];
  twist_msg->twist.angular.z = x_dot[5];

  TooN::Vector<> clikError =
      clik_->getClikError(clik_integrator_.getJointsDH());
  geometry_msgs::TwistStampedPtr clikError_msg(new geometry_msgs::TwistStamped);
  clikError_msg->header.stamp = time_now;
  clikError_msg->twist.linear.x = clikError[0];
  clikError_msg->twist.linear.y = clikError[1];
  clikError_msg->twist.linear.z = clikError[2];
  clikError_msg->twist.angular.x = clikError[3];
  clikError_msg->twist.angular.y = clikError[4];
  clikError_msg->twist.angular.z = clikError[5];

  geometry_msgs::TwistStampedPtr pos_posdes_msg(
      new geometry_msgs::TwistStamped);
  TooN::Vector<3> position =
      clik_->robot_->fkine(clik_integrator_.getJointsDH()).T()[3].slice<0, 3>();
  TooN::Vector<3> desiredPosition = clik_->desiredPosition_;
  pos_posdes_msg->header.stamp = time_now;
  pos_posdes_msg->twist.linear.x = position[0];
  pos_posdes_msg->twist.linear.y = position[1];
  pos_posdes_msg->twist.linear.z = position[2];
  pos_posdes_msg->twist.angular.x = desiredPosition[0];
  pos_posdes_msg->twist.angular.y = desiredPosition[1];
  pos_posdes_msg->twist.angular.z = desiredPosition[2];

  joi_state_pub_dbg_.publish(joi_state);
  twist_pub_dbg_.publish(twist_msg);
  clikError_pub_dbg_.publish(clikError_msg);
  pos_posdes_pub_dbg_.publish(pos_posdes_msg);
}

// Note: for velocity mode it is sufficient clik_gain_=0
void ClikNode::clik_core() {

  clik_integrator_.exec_single_step();

  safety_check(clik_integrator_.getJointsDH(),
               clik_integrator_.getJointsVelDH());
}

void ClikNode::safety_check(const TooN::Vector<> &qDH,
                            const TooN::Vector<> &qDH_dot) {
  try {
    clik_->safetyCheck(qDH, qDH_dot);
  } catch (const sun::robot::ExceededJointLimits &e) {
    ROS_ERROR_STREAM(ros::this_node::getName() << e.what());
    std::rethrow_exception(std::make_exception_ptr(e));
  }
}

void ClikNode::desiredPoseTwist_cb(
    const sun_robot_msgs::CartesianStateStamped::ConstPtr &pose_twist_msg) {
  if (mode_ == sun_robot_msgs::ClikSetMode::RequestType::MODE_POSITION) {
    clik_->desiredPosition_[0] = pose_twist_msg->pose.position.x;
    clik_->desiredPosition_[1] = pose_twist_msg->pose.position.y;
    clik_->desiredPosition_[2] = pose_twist_msg->pose.position.z;

    clik_->desiredQuaternion_ =
        UnitQuaternion(pose_twist_msg->pose.orientation.w,
                       TooN::makeVector(pose_twist_msg->pose.orientation.x,
                                        pose_twist_msg->pose.orientation.y,
                                        pose_twist_msg->pose.orientation.z));
  }
  if (mode_ == sun_robot_msgs::ClikSetMode::RequestType::MODE_POSITION ||
      mode_ == sun_robot_msgs::ClikSetMode::RequestType::MODE_VELOCITY) {
    clik_->desiredLinearVelocity_[0] = pose_twist_msg->velocity.linear.x;
    clik_->desiredLinearVelocity_[1] = pose_twist_msg->velocity.linear.y;
    clik_->desiredLinearVelocity_[2] = pose_twist_msg->velocity.linear.z;

    clik_->desiredAngularVelocity_[0] = pose_twist_msg->velocity.angular.x;
    clik_->desiredAngularVelocity_[1] = pose_twist_msg->velocity.angular.y;
    clik_->desiredAngularVelocity_[2] = pose_twist_msg->velocity.angular.z;
  } else if (mode_ == sun_robot_msgs::ClikSetMode::Request::MODE_VELOCITY_EE) {
    clik_->desiredLinearVelocity_[0] = pose_twist_msg->velocity.linear.x;
    clik_->desiredLinearVelocity_[1] = pose_twist_msg->velocity.linear.y;
    clik_->desiredLinearVelocity_[2] = pose_twist_msg->velocity.linear.z;

    clik_->desiredAngularVelocity_[0] = pose_twist_msg->velocity.angular.x;
    clik_->desiredAngularVelocity_[1] = pose_twist_msg->velocity.angular.y;
    clik_->desiredAngularVelocity_[2] = pose_twist_msg->velocity.angular.z;

    TooN::Matrix<3, 3> b_R_e =
        clik_->robot_->fkine(clik_integrator_.getJointsDH())
            .slice<0, 0, 3, 3>();
    clik_->desiredLinearVelocity_ = b_R_e * clik_->desiredLinearVelocity_;
    clik_->desiredAngularVelocity_ = b_R_e * clik_->desiredAngularVelocity_;
  }
}

void ClikNode::desiredPose_cb(
    const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
  if (mode_ == sun_robot_msgs::ClikSetMode::RequestType::MODE_POSITION) {
    clik_->desiredPosition_[0] = pose_msg->pose.position.x;
    clik_->desiredPosition_[1] = pose_msg->pose.position.y;
    clik_->desiredPosition_[2] = pose_msg->pose.position.z;

    clik_->desiredQuaternion_ =
        UnitQuaternion(pose_msg->pose.orientation.w,
                       TooN::makeVector(pose_msg->pose.orientation.x,
                                        pose_msg->pose.orientation.y,
                                        pose_msg->pose.orientation.z));
  }
}

void ClikNode::desiredTwist_cb(
    const geometry_msgs::TwistStamped::ConstPtr &twist_msg) {
  if (mode_ == sun_robot_msgs::ClikSetMode::RequestType::MODE_POSITION ||
      mode_ == sun_robot_msgs::ClikSetMode::RequestType::MODE_VELOCITY) {
    clik_->desiredLinearVelocity_[0] = twist_msg->twist.linear.x;
    clik_->desiredLinearVelocity_[1] = twist_msg->twist.linear.y;
    clik_->desiredLinearVelocity_[2] = twist_msg->twist.linear.z;

    clik_->desiredAngularVelocity_[0] = twist_msg->twist.angular.x;
    clik_->desiredAngularVelocity_[1] = twist_msg->twist.angular.y;
    clik_->desiredAngularVelocity_[2] = twist_msg->twist.angular.z;
  } else if (mode_ == sun_robot_msgs::ClikSetMode::Request::MODE_VELOCITY_EE) {
    clik_->desiredLinearVelocity_[0] = twist_msg->twist.linear.x;
    clik_->desiredLinearVelocity_[1] = twist_msg->twist.linear.y;
    clik_->desiredLinearVelocity_[2] = twist_msg->twist.linear.z;

    clik_->desiredAngularVelocity_[0] = twist_msg->twist.angular.x;
    clik_->desiredAngularVelocity_[1] = twist_msg->twist.angular.y;
    clik_->desiredAngularVelocity_[2] = twist_msg->twist.angular.z;

    TooN::Matrix<3, 3> b_R_e =
        clik_->robot_->fkine(clik_integrator_.getJointsDH())
            .slice<0, 0, 3, 3>();
    clik_->desiredLinearVelocity_ = b_R_e * clik_->desiredLinearVelocity_;
    clik_->desiredAngularVelocity_ = b_R_e * clik_->desiredAngularVelocity_;
  }
}

} // Namespace sun