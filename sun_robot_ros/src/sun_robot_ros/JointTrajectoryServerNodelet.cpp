// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>

// Include your header
#include "sun_robot_ros/JointTrajectoryServerNodelet.h"

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(sun_robot_ros::JointTrajectoryServerNodelet,
                       nodelet::Nodelet)
