// An ideal hardware that instantly reaches the commanded pos, vel, tau
// Useful for debugging

#ifndef HARDWARE__ROS_HARDWARE
#define HARDWARE__ROS_HARDWARE

#pragma once

#include <ros/ros.h>
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"

#include "robot/gazebo_comm.h"
#include "hardware_interface.h"

class RosHardware : public interfaces::HardwareInterface
{
public:
    int start() override; 
    int read() override;
    int write() override; 
    int stop() override;

private:
    ros::NodeHandle* nh;
    unitree_legged_msgs::LowCmd cmd_;
    unitree_legged_msgs::LowState data_;

    GazeboCommunicationChannel* gazebo_comm;

};

#endif // HARDWARE__ROS_HARDWARE