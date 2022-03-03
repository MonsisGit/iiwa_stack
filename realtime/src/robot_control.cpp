#include "../include/robot_control.h"

RobotControl::RobotControl()
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
    ROS_INFO_STREAM("SET logger to debug");
  }
  if (ros::master::check())
  {
    cartesian_speed_client = node_handle.serviceClient<iiwa_msgs::SetPTPCartesianSpeedLimits>("/iiwa/configuration/"
                                                                                              "setPTPCartesianLimits",
                                                                                              this);
    ROS_DEBUG("Setup Service to: /iiwa/configuration/setPTPCartesianLimits");
    cartesian_pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1, this);
    ROS_DEBUG("Setup Publisher to: /iiwa/command/CartesianPose");
    joint_pub = node_handle.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1, this);
    ROS_DEBUG("Setup Publisher to: /iiwa/command/JointPosition");
    cartesian_pos_sub = node_handle.subscribe("/iiwa/state/CartesianPose", 1, &RobotControl::CartesianPoseCB, this);
    ROS_DEBUG("Setup Subscriber to: /iiwa/state/CartesianPose");
    this->sequence = 0;
    this->set_joint_positions(0.4, 0.2, -0.4, -0.4, 0.0, 0.0, 0.0, 5.0);
  }
  else
  {
    ROS_ERROR_STREAM("ROS master not online, please start roscore and check IP-Adress (Should be 172.31.1.150, Subnet "
                     "255.0.0.0");
  }
}

RobotControl::~RobotControl()
{
  //
}

int RobotControl::check_workspace(geometry_msgs::PoseStamped& cart_pose)
{
  if (std::abs(cart_pose.pose.position.x) > 0.51)
  {
    ROS_ERROR("Cartesian X outside Workspace");
    return 1;
  }
  else if (std::abs(cart_pose.pose.position.y) > 0.41)
  {
    ROS_ERROR("Cartesian Y outside Workspace");
    return 1;
  }
  else if (std::abs(cart_pose.pose.position.z) < 0.2)
  {
    ROS_ERROR("Cartesian Z outside Workspace");
    return 1;
  };
  return 0;
}

int RobotControl::get_sequence()
{
  this->sequence++;
  return this->sequence - 1;
}

void RobotControl::CartesianPoseCB(const iiwa_msgs::CartesianPose& pose)
{
  this->cartesian_pose = pose;
  std::string pose_s = std::string("This message only prints every second. Cartesian Pose Callback: \n"
                                   "X: ") +
                       std::to_string(pose.poseStamped.pose.position.x) + std::string("Y: ") +
                       std::to_string(pose.poseStamped.pose.position.y) + std::string("Z: ") +
                       std::to_string(pose.poseStamped.pose.position.z);

  ROS_DEBUG_STREAM_DELAYED_THROTTLE(1, pose_s);
}

void RobotControl::set_cartesian_vel(float max_cart_vel = -1, float max_cart_acc = -1)
{
  iiwa_msgs::SetPTPCartesianSpeedLimits cart_limits;
  cart_limits.request.maxCartesianAcceleration = max_cart_acc;
  cart_limits.request.maxCartesianVelocity = max_cart_vel;

  if (this->cartesian_speed_client.call(cart_limits))
  {
    ROS_DEBUG_STREAM("Set cartesian velocity to: " << max_cart_vel << "Succes: " << cart_limits.response.success);
  }
  else
  {
    ROS_ERROR_STREAM("Error: " << cart_limits.response.error);
  }
}

int RobotControl::set_cartesian_pose_rel_to_current_pose(float x, float y, float z, float ox = 0, float oy = 0,
                                                         float oz = 0, float ow = 1)
{
  geometry_msgs::PoseStamped commanded_pose;
  commanded_pose.pose.position.x = this->cartesian_pose.poseStamped.pose.position.x + x;
  commanded_pose.pose.position.y = this->cartesian_pose.poseStamped.pose.position.y + y;
  commanded_pose.pose.position.z = this->cartesian_pose.poseStamped.pose.position.z + z;

  commanded_pose.pose.orientation.x = this->cartesian_pose.poseStamped.pose.orientation.x + x;
  commanded_pose.pose.orientation.y = this->cartesian_pose.poseStamped.pose.orientation.y + oy;
  commanded_pose.pose.orientation.z = this->cartesian_pose.poseStamped.pose.orientation.z + ox;
  commanded_pose.pose.orientation.w = this->cartesian_pose.poseStamped.pose.orientation.w + ow;

  commanded_pose.header.stamp = ros::Time::now();
  commanded_pose.header.frame_id = "iiwa_link_0";
  commanded_pose.header.seq = this->get_sequence();

  if (!this->check_workspace(commanded_pose))
  {
    this->cartesian_pose_pub.publish(commanded_pose);
    ROS_DEBUG_STREAM("set cartesian pose to: \n"
                     << "X: " << commanded_pose.pose.position.x << "Y: " << commanded_pose.pose.position.y
                     << "Z: " << commanded_pose.pose.position.z << "\n"
                     << "OX: " << commanded_pose.pose.orientation.x << "OY: " << commanded_pose.pose.orientation.y
                     << "OZ: " << commanded_pose.pose.orientation.z << "OW: " << commanded_pose.pose.orientation.w);
  }

  return 0;
};

int RobotControl::set_cartesian_pose(float x, float y, float z, float ox = 0, float oy = 0, float oz = 0, float ow = 1)
{
  geometry_msgs::PoseStamped commanded_pose;
  commanded_pose.pose.position.x = x;
  commanded_pose.pose.position.y = y;
  commanded_pose.pose.position.z = z;

  commanded_pose.pose.orientation.x = ox;
  commanded_pose.pose.orientation.y = oy;
  commanded_pose.pose.orientation.z = ox;
  commanded_pose.pose.orientation.w = ow;

  commanded_pose.header.stamp = ros::Time::now();
  commanded_pose.header.frame_id = "iiwa_link_0";
  commanded_pose.header.seq = this->get_sequence();

  if (!this->check_workspace(commanded_pose))
  {
    this->cartesian_pose_pub.publish(commanded_pose);
    ROS_DEBUG_STREAM("set cartesian pose to: \n"
                     << "X: " << commanded_pose.pose.position.x << "Y: " << commanded_pose.pose.position.y
                     << "Z: " << commanded_pose.pose.position.z << "\n"
                     << "OX: " << commanded_pose.pose.orientation.x << "OY: " << commanded_pose.pose.orientation.y
                     << "OZ: " << commanded_pose.pose.orientation.z << "OW: " << commanded_pose.pose.orientation.w);
  }

  return 0;
}

int RobotControl::set_joint_positions(float j1, float j2, float j3, float j4, float j5, float j6, float j7,
                                      float sleep = 0)
{
  iiwa_msgs::JointPosition joint_pos;
  joint_pos.header.seq = this->get_sequence();
  joint_pos.header.stamp = ros::Time::now();

  joint_pos.position.a1 = j1;
  joint_pos.position.a2 = j2;
  joint_pos.position.a3 = j3;
  joint_pos.position.a4 = j4;
  joint_pos.position.a5 = j5;
  joint_pos.position.a6 = j6;
  joint_pos.position.a7 = j7;

  this->joint_pub.publish(joint_pos);
  ROS_DEBUG_STREAM("Set joint angles to: \n"
                   << "A1: " << joint_pos.position.a1 << "A2: " << joint_pos.position.a2 << "A3: "
                   << joint_pos.position.a3 << "A4: " << joint_pos.position.a4 << "A5: " << joint_pos.position.a5
                   << "A6: " << joint_pos.position.a6 << "A7: " << joint_pos.position.a7);
  if (sleep > 0)
  {
    ros::Duration(sleep).sleep();
  }

  return 0;
}
