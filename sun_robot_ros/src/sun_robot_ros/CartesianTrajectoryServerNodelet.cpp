// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>

// Include your header
#include "sun_robot_ros/CartesianTrajectoryServerNodelet.h"

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(sun_robot_ros::CartesianTrajectoryServerNodelet, nodelet::Nodelet)
