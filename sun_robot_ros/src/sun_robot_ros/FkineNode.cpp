

#include "sun_robot_ros/FkineNode.h"
#include "std_msgs/Float64MultiArray.h"

namespace sun {

// Defined il Clik.cpp
TooN::Vector<> getTooNVectorFromParam(const ros::NodeHandle &nh,
                                      const std::string &param_str,
                                      int expected_size,
                                      const TooN::Vector<> &default_value);

FkineNode::FkineNode(const std::shared_ptr<Robot> &robot,
                     const ros::NodeHandle &nh_for_topics,
                     const ros::NodeHandle &nh_for_parmas,
                     ros::CallbackQueue *callbk_queue)
    : callbk_queue_(callbk_queue), robot_(robot), nh_(nh_for_topics),
      joints_updated_(false), joints_vel_updated_(false),
      qDH_(TooN::Zeros(robot_->getNumJoints())),
      qDH_dot_(TooN::Zeros(robot_->getNumJoints())),
      jacob_geometric_(TooN::Zeros(6, robot_->getNumJoints())) {

  updateParams(nh_for_parmas);
  nh_.setCallbackQueue(callbk_queue_);
}

bool FkineNode::setEndEffector_srv_cb(
    sun_robot_msgs::SetEndEffector::Request &req,
    sun_robot_msgs::SetEndEffector::Response &res) {

  TooN::Vector<3> n_T_e_position =
      TooN::makeVector(req.n_pose_ee.position.x, req.n_pose_ee.position.y,
                       req.n_pose_ee.position.z);
  UnitQuaternion n_T_e_quaternion(
      req.n_pose_ee.orientation.w,
      TooN::makeVector(req.n_pose_ee.orientation.x, req.n_pose_ee.orientation.y,
                       req.n_pose_ee.orientation.z));
  TooN::Matrix<4, 4> n_T_e = transl(n_T_e_position);
  n_T_e.slice<0, 0, 3, 3>() = n_T_e_quaternion.torot();
  robot_->setnTe(n_T_e);

  res.success = true;

  return true;
}

void FkineNode::updateParams(const ros::NodeHandle &nh_for_parmas) {

  nh_for_parmas.param("out_pose", out_pose_topic_, std::string("ee_pose"));
  nh_for_parmas.param("out_twist", out_twist_topic_, std::string("ee_twist"));
  nh_for_parmas.param("out_jacobian", out_jacob_topic_,
                      std::string("jacobian"));

  double rate;
  nh_for_parmas.param("rate", rate, -1.0);
  if (rate > 0.0) {
    loop_rate_ = std::unique_ptr<ros::Rate>(new ros::Rate(rate));
  }

  // n_T_e
  {
    if (nh_for_parmas.hasParam("n_T_e_position") ||
        nh_for_parmas.hasParam("n_T_e_quaternion")) {
      if (!nh_for_parmas.hasParam("n_T_e_position") ||
          !nh_for_parmas.hasParam("n_T_e_quaternion")) {
        throw std::runtime_error("The n_T_e must be fully defined");
      }
    }
    if (nh_for_parmas.hasParam("n_T_e_position") &&
        nh_for_parmas.hasParam("n_T_e_quaternion")) {
      TooN::Vector<3> n_T_e_position = getTooNVectorFromParam(
          nh_for_parmas, "n_T_e_position", 3, TooN::Zeros(3));
      TooN::Vector<4> n_T_e_quat_v =
          getTooNVectorFromParam(nh_for_parmas, "n_T_e_quaternion", 4,
                                 TooN::makeVector(1.0, 0.0, 0.0, 0.0));
      UnitQuaternion n_T_e_quaternion(n_T_e_quat_v);
      TooN::Matrix<4, 4> n_T_e = transl(n_T_e_position);
      n_T_e.slice<0, 0, 3, 3>() = n_T_e_quaternion.torot();
      robot_->setnTe(n_T_e);
      ROS_INFO_STREAM(ros::this_node::getName() << " Fkine Node nTe: \n"
                                                << robot_->getnTe());
    }
  }
  // b_T_0
  {
    if (nh_for_parmas.hasParam("b_T_0_position") ||
        nh_for_parmas.hasParam("b_T_0_quaternion")) {
      if (!nh_for_parmas.hasParam("b_T_0_position") ||
          !nh_for_parmas.hasParam("b_T_0_quaternion")) {
        throw std::runtime_error("The base transform must be fully defined");
      }
    }
    if (nh_for_parmas.hasParam("b_T_0_position") &&
        nh_for_parmas.hasParam("b_T_0_quaternion")) {
      TooN::Vector<3> b_T_0_position = getTooNVectorFromParam(
          nh_for_parmas, "b_T_0_position", 3, TooN::Zeros(3));
      TooN::Vector<4> b_T_0_quat_v =
          getTooNVectorFromParam(nh_for_parmas, "b_T_0_quaternion", 4,
                                 TooN::makeVector(1.0, 0.0, 0.0, 0.0));
      UnitQuaternion b_T_0_quaternion(b_T_0_quat_v);
      TooN::Matrix<4, 4> b_T_0 = transl(b_T_0_position);
      b_T_0.slice<0, 0, 3, 3>() = b_T_0_quaternion.torot();
      robot_->setbT0(b_T_0);
    }
    ROS_INFO_STREAM(ros::this_node::getName() << " CLIK bT0: \n"
                                              << robot_->getbT0());
  }
}

void FkineNode::spinOnce(const ros::WallDuration &timeout) {
  callbk_queue_->callAvailable(timeout);
}

void FkineNode::spin() {
  if (loop_rate_) {
    loop_rate_->reset();
    while (ros::ok()) {
      spinOnce();
      loop_rate_->sleep();
    }
  } else {
    ros::WallDuration timeout(0.1f);
    while (ros::ok()) {
      spinOnce(timeout);
    }
  }
}

void FkineNode::start() {

  serviceSetEndEffector_ = nh_.advertiseService(
      "set_end_effector", &FkineNode::setEndEffector_srv_cb, this);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(out_pose_topic_, 1);
  twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(out_twist_topic_, 1);
  robot_jacob_pub_ =
      nh_.advertise<std_msgs::Float64MultiArray>(out_jacob_topic_, 1);
  registerJointSubscriber();
}

void FkineNode::updateJoint(const TooN::Vector<> &qR) {
  qDH_ = robot_->joints_Robot2DH(qR);
  jacob_geometric_ = robot_->jacob_geometric(qDH_);
  joints_updated_ = true;
}

void FkineNode::updateJointVel(const TooN::Vector<> &qR_dot) {
  qDH_dot_ = robot_->jointsvel_Robot2DH(qR_dot);
  joints_vel_updated_ = true;
}

void FkineNode::publishAll() {
  if (joints_updated_) {
    publishFkine();
    publishJacobian();
  }
  if (joints_updated_ || joints_vel_updated_) {
    publishVel();
  }
  joints_updated_ = false;
  joints_vel_updated_ = false;
}

void FkineNode::publishFkine() {

  TooN::Matrix<4, 4> b_T_ee = robot_->fkine(qDH_);

  TooN::Vector<3> pos = transl(b_T_ee);

  sun::UnitQuaternion quat(b_T_ee, oldQuat);

  oldQuat = quat;

  geometry_msgs::PoseStampedPtr out(new geometry_msgs::PoseStamped);

  out->header.stamp = ros::Time::now();

  out->pose.position.x = pos[0];
  out->pose.position.y = pos[1];
  out->pose.position.z = pos[2];
  out->pose.orientation.w = quat.getS();
  out->pose.orientation.x = quat.getV()[0];
  out->pose.orientation.y = quat.getV()[1];
  out->pose.orientation.z = quat.getV()[2];

  pose_pub_.publish(out);
}

void FkineNode::publishVel() {

  TooN::Vector<6> vel = jacob_geometric_ * qDH_dot_;

  geometry_msgs::TwistStampedPtr out(new geometry_msgs::TwistStamped);

  out->header.stamp = ros::Time::now();

  out->twist.linear.x = vel[0];
  out->twist.linear.y = vel[1];
  out->twist.linear.z = vel[2];
  out->twist.angular.x = vel[3];
  out->twist.angular.y = vel[4];
  out->twist.angular.z = vel[5];

  twist_pub_.publish(out);
}

void FkineNode::publishJacobian() {
  // return;

  std_msgs::Float64MultiArrayPtr jacob_msg =
      boost::make_shared<std_msgs::Float64MultiArray>();

  auto jacob_size = jacob_geometric_.num_cols() * jacob_geometric_.num_rows();
  jacob_msg->data.resize(jacob_size);
  memcpy(jacob_msg->data.data(), jacob_geometric_.get_data_ptr(),
         jacob_size * sizeof(double));

  jacob_msg->layout.data_offset = 0.0;
  jacob_msg->layout.dim.resize(2);
  jacob_msg->layout.dim[0].label = "rows";
  jacob_msg->layout.dim[0].size = jacob_geometric_.num_rows();
  jacob_msg->layout.dim[0].stride =
      jacob_geometric_.num_cols() *
      jacob_geometric_.num_rows(); // dim[0] stride is just the total size
  jacob_msg->layout.dim[1].label = "cols";
  jacob_msg->layout.dim[1].size = jacob_geometric_.num_cols();
  jacob_msg->layout.dim[1].stride = jacob_geometric_.num_cols();

  robot_jacob_pub_.publish(jacob_msg);
}

} // namespace sun
