#include <fstream>
#include "ros/ros.h"
#include "sun_robot_ros/ClikClient.h"

#define TAB "  "

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "aquire_robot_config");

  ros::NodeHandle nh;

  std::string clik_ns = argv[1];
  std::string file_name = argv[2];

  sun::ClikClient clik(ros::NodeHandle(nh, clik_ns));

  char ans;

  std::ofstream file;
  file.open(file_name);

  while (true)
  {
    ans = 0;
    while (ans != 'e' && ans != 'n')
    {
      std::cout << "Exit? [e/n]: ";
      std::cin >> ans;
    }
    if (ans == 'e')
    {
      file.close();
      return 0;
    }

    std::string measure_label;
    std::cout << "Insert Measure Label: ";
    std::cin >> measure_label;
    std::cout << "Measure Label: " << measure_label;

    ans = 0;
    while (ans != 'y')
    {
      std::cout << "Bring the robot to the desired position, press y when done (e=exit): ";
      std::cin >> ans;
      if (ans == 'e')
      {
        return 0;
      }
    }

    std::vector<double> q = clik.get_state().robot_joints.position;

    file << measure_label << ": [";
    for(int i=0; i<(q.size()-1); i++)
    {
        file << q[i] << ", ";
    }
    file << q.back() << "]\n";

  }

  file.close();

  return 0;
}
