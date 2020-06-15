#include <stdexcept>

namespace sun
{

class robot_ros_base_exception : public std::exception
{
public:
  robot_ros_base_exception() : std::exception()
  {
  }
};

class robot_joint_limits : public std::runtime_error, robot_ros_base_exception
{
public:
  robot_joint_limits(std::string const& msg = "") : std::runtime_error(msg)
  {
  }
};

class robot_joint_position_limits : public robot_joint_limits
{
public:
  robot_joint_position_limits(std::string const& msg = "") : robot_joint_limits(msg)
  {
  }
};

class robot_joint_velocity_limits : public robot_joint_limits
{
public:
  robot_joint_velocity_limits(std::string const& msg = "") : robot_joint_limits(msg)
  {
  }
};

class robot_invalid_parameter : public std::runtime_error, robot_ros_base_exception
{
public:
  robot_invalid_parameter(std::string const& msg = "") : std::runtime_error(msg)
  {
  }
};

class clik_invalid_mode : public robot_invalid_parameter
{
public:
  clik_invalid_mode(std::string const& msg = "") : robot_invalid_parameter(msg)
  {
  }
};

}
