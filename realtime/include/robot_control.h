#ifndef REALTIME_POS_CONTROL_H
#define REALTIME_POS_CONTROL_H

#include <stdlib.h>
#include <iostream>
#include <string>

#include "ros/ros.h"
#include <ros/console.h>
#include <ros/master.h>

#include "std_msgs/String.h"
#include "iiwa_msgs/SetPTPCartesianSpeedLimits.h"
#include <iiwa_ros/command/cartesian_pose.hpp>
#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_msgs/CartesianPose.h>
#include <geometry_msgs/TwistStamped.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/SetPTPCartesianSpeedLimits.h>
#include <iiwa_msgs/JointPosition.h>

class RobotControl
{
  ros::NodeHandle node_handle;
  ros::Subscriber cartesian_pos_sub;
  ros::Publisher cartesian_pose_pub;
  ros::Publisher joint_pub;
  ros::ServiceClient cartesian_speed_client;
  iiwa_msgs::CartesianPose cartesian_pose;
  int sequence;

public:
  RobotControl();
  ~RobotControl();
  int set_cartesian_pose(float x, float y, float z, float ox, float oy, float oz, float ow);
  int set_cartesian_pose_rel_to_current_pose(float x, float y, float z, float ox, float oy, float oz, float ow);
  void set_cartesian_vel(float max_cart_vel, float max_cart_acc);
  int set_joint_positions(float j1, float j2, float j3, float j4, float j5, float j6, float j7, float sleep);

private:
  int check_workspace(geometry_msgs::PoseStamped& pose);
  void CartesianPoseCB(const iiwa_msgs::CartesianPose& pose);
  int get_sequence();
};

#endif  // REALTIME_POS_CONTROL_H