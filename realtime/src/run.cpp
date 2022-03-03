#include "../include/robot_control.h"
//#include "vision_control.cpp"



int main(int argc, char** argv)
{
  ros::init(argc, argv, "RobotControl");
  RobotControl robot_control;

  ros::Rate loop_rate(0.4);
  ros::Duration(1).sleep();
  int count = 1;

  while (ros::ok())
  {
    if (count % 2 == 0)
    {
      robot_control.set_cartesian_pose(0.2, 0.1, 1, 0, 0, 0, 1);
    }
    else
    {
      robot_control.set_cartesian_pose(0.2, 0.0, 0.9, 0, 0, 0, 1);
    }

    count++;
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}