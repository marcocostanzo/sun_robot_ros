#include "ros/ros.h"

class ClikClient
{
private:
  
    ros::ServiceClient sc_set_mode_;
    ros::ServiceClient sc_get_state_;
    ros::ServiceClient sc_set_end_effector_;

public:

  //! nh is the a node handle in the clik namespace
  ClikClient(const ros::NodeHandle& nh = ros::NodeHandle("~/clik"))
  {
  }

  ~ClikClient() = default;
};
