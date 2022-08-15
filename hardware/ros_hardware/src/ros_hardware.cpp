#include "ros_hardware.h"

RosHardware(const ros::NodeHandle& nh)
    : nh(nh)
{
    
};

int RosHardware::start()
{
    return 0;
}

int RosHardware::stop()
{
    return 0;
}

int RosHardware::read() 
{
    return 0;
}

int RosHardware::write() 
{
    return 0;
}  