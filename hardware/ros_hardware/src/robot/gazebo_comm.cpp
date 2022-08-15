#include "robot/gazebo_comm.h"

GazeboCommunicationChannel::GazeboCommunicationChannel (
        std::string _robotName,
        ros::NodeHandle *nh, 
        unitree_legged_msgs::LowCmd* lowCmdPtr, 
        unitree_legged_msgs::LowState* lowStatePtr
)
    : _robotName(_robotName),
      _lowCmd(lowCmdPtr),
      _lowState(lowStatePtr)
{ 

    /* Subscriber Initialization */
    imu_sub = nh->subscribe("/trunk_imu", 1, &GazeboCommunicationChannel::imuCallback, this);
    footForce_sub[legged_robot::FL_] = nh->subscribe("/visual/FL_foot_contact/the_force", 1, &GazeboCommunicationChannel::FLfootCallback, this);
    footForce_sub[legged_robot::FR_] = nh->subscribe("/visual/FR_foot_contact/the_force", 1, &GazeboCommunicationChannel::FRfootCallback, this);
    footForce_sub[legged_robot::RL_] = nh->subscribe("/visual/RL_foot_contact/the_force", 1, &GazeboCommunicationChannel::RLfootCallback, this);
    footForce_sub[legged_robot::RR_] = nh->subscribe("/visual/RR_foot_contact/the_force", 1, &GazeboCommunicationChannel::RRfootCallback, this);
    servo_sub[legged_robot::FL_0] = nh->subscribe("/" + _robotName + "_gazebo/FL_hip_controller/state", 1, &GazeboCommunicationChannel::FLhipCallback, this);
    servo_sub[legged_robot::FL_1] = nh->subscribe("/" + _robotName + "_gazebo/FL_thigh_controller/state", 1, &GazeboCommunicationChannel::FLthighCallback, this);
    servo_sub[legged_robot::FL_2] = nh->subscribe("/" + _robotName + "_gazebo/FL_calf_controller/state", 1, &GazeboCommunicationChannel::FLcalfCallback, this);
    servo_sub[legged_robot::FR_0] = nh->subscribe("/" + _robotName + "_gazebo/FR_hip_controller/state", 1, &GazeboCommunicationChannel::FRhipCallback, this);
    servo_sub[legged_robot::FR_1] = nh->subscribe("/" + _robotName + "_gazebo/FR_thigh_controller/state", 1, &GazeboCommunicationChannel::FRthighCallback, this);
    servo_sub[legged_robot::FR_2] = nh->subscribe("/" + _robotName + "_gazebo/FR_calf_controller/state", 1, &GazeboCommunicationChannel::FRcalfCallback, this);
    servo_sub[legged_robot::RL_0] = nh->subscribe("/" + _robotName + "_gazebo/RL_hip_controller/state", 1, &GazeboCommunicationChannel::RLhipCallback, this);
    servo_sub[legged_robot::RL_1] = nh->subscribe("/" + _robotName + "_gazebo/RL_thigh_controller/state", 1, &GazeboCommunicationChannel::RLthighCallback, this);
    servo_sub[legged_robot::RL_2] = nh->subscribe("/" + _robotName + "_gazebo/RL_calf_controller/state", 1, &GazeboCommunicationChannel::RLcalfCallback, this);
    servo_sub[legged_robot::RR_0] = nh->subscribe("/" + _robotName + "_gazebo/RR_hip_controller/state", 1, &GazeboCommunicationChannel::RRhipCallback, this);
    servo_sub[legged_robot::RR_1] = nh->subscribe("/" + _robotName + "_gazebo/RR_thigh_controller/state", 1, &GazeboCommunicationChannel::RRthighCallback, this);
    servo_sub[legged_robot::RR_2] = nh->subscribe("/" + _robotName + "_gazebo/RR_calf_controller/state", 1, &GazeboCommunicationChannel::RRcalfCallback, this);


    /* Publisher Initialization */    
    servo_pub[legged_robot::FL_0] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + _robotName + "_gazebo/FL_hip_controller/command", 1);
    servo_pub[legged_robot::FL_1] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + _robotName + "_gazebo/FL_thigh_controller/command", 1);
    servo_pub[legged_robot::FL_2] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + _robotName + "_gazebo/FL_calf_controller/command", 1);
    servo_pub[legged_robot::FR_0] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + _robotName + "_gazebo/FR_hip_controller/command", 1);
    servo_pub[legged_robot::FR_1] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + _robotName + "_gazebo/FR_thigh_controller/command", 1);
    servo_pub[legged_robot::FR_2] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + _robotName + "_gazebo/FR_calf_controller/command", 1);
    servo_pub[legged_robot::RL_0] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + _robotName + "_gazebo/RL_hip_controller/command", 1);
    servo_pub[legged_robot::RL_1] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + _robotName + "_gazebo/RL_thigh_controller/command", 1);
    servo_pub[legged_robot::RL_2] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + _robotName + "_gazebo/RL_calf_controller/command", 1);
    servo_pub[legged_robot::RR_0] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + _robotName + "_gazebo/RR_hip_controller/command", 1);
    servo_pub[legged_robot::RR_1] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + _robotName + "_gazebo/RR_thigh_controller/command", 1);
    servo_pub[legged_robot::RR_2] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + _robotName + "_gazebo/RR_calf_controller/command", 1);
    lowState_pub = nh->advertise<unitree_legged_msgs::LowState>("/" + _robotName + "_gazebo/lowState/state", 1);
}

void GazeboCommunicationChannel::sendServoCmd()
{
    for(size_t m=0; m < 12; m++)
        servo_pub[m].publish(_lowCmd->motorCmd[m]);

    pubLowState();
    
    // //Upadate Frequency - 1Khz
    // usleep(1000); 
}

void GazeboCommunicationChannel::pubLowState()
{
    lowState_pub.publish(*_lowState);
}

void GazeboCommunicationChannel::imuCallback(const sensor_msgs::Imu & msg)
{ 
    _lowState->imu.quaternion[0] = msg.orientation.w;
    _lowState->imu.quaternion[1] = msg.orientation.x;
    _lowState->imu.quaternion[2] = msg.orientation.y;
    _lowState->imu.quaternion[3] = msg.orientation.z;

    _lowState->imu.gyroscope[0] = msg.angular_velocity.x;
    _lowState->imu.gyroscope[1] = msg.angular_velocity.y;
    _lowState->imu.gyroscope[2] = msg.angular_velocity.z;

    _lowState->imu.accelerometer[0] = msg.linear_acceleration.x;
    _lowState->imu.accelerometer[1] = msg.linear_acceleration.y;
    _lowState->imu.accelerometer[2] = msg.linear_acceleration.z;
}



void GazeboCommunicationChannel::FRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[legged_robot::FR_0].mode = msg.mode;
    _lowState->motorState[legged_robot::FR_0].q = msg.q;
    _lowState->motorState[legged_robot::FR_0].dq = msg.dq;
    _lowState->motorState[legged_robot::FR_0].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::FRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[legged_robot::FR_1].mode = msg.mode;
    _lowState->motorState[legged_robot::FR_1].q = msg.q;
    _lowState->motorState[legged_robot::FR_1].dq = msg.dq;
    _lowState->motorState[legged_robot::FR_1].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::FRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[legged_robot::FR_2].mode = msg.mode;
    _lowState->motorState[legged_robot::FR_2].q = msg.q;
    _lowState->motorState[legged_robot::FR_2].dq = msg.dq;
    _lowState->motorState[legged_robot::FR_2].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::FLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[legged_robot::FL_0].mode = msg.mode;
    _lowState->motorState[legged_robot::FL_0].q = msg.q;
    _lowState->motorState[legged_robot::FL_0].dq = msg.dq;
    _lowState->motorState[legged_robot::FL_0].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::FLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[legged_robot::FL_1].mode = msg.mode;
    _lowState->motorState[legged_robot::FL_1].q = msg.q;
    _lowState->motorState[legged_robot::FL_1].dq = msg.dq;
    _lowState->motorState[legged_robot::FL_1].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::FLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[legged_robot::FL_2].mode = msg.mode;
    _lowState->motorState[legged_robot::FL_2].q = msg.q;
    _lowState->motorState[legged_robot::FL_2].dq = msg.dq;
    _lowState->motorState[legged_robot::FL_2].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::RRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[legged_robot::RR_0].mode = msg.mode;
    _lowState->motorState[legged_robot::RR_0].q = msg.q;
    _lowState->motorState[legged_robot::RR_0].dq = msg.dq;
    _lowState->motorState[legged_robot::RR_0].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::RRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[legged_robot::RR_1].mode = msg.mode;
    _lowState->motorState[legged_robot::RR_1].q = msg.q;
    _lowState->motorState[legged_robot::RR_1].dq = msg.dq;
    _lowState->motorState[legged_robot::RR_1].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::RRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[legged_robot::RR_2].mode = msg.mode;
    _lowState->motorState[legged_robot::RR_2].q = msg.q;
    _lowState->motorState[legged_robot::RR_2].dq = msg.dq;
    _lowState->motorState[legged_robot::RR_2].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::RLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[legged_robot::RL_0].mode = msg.mode;
    _lowState->motorState[legged_robot::RL_0].q = msg.q;
    _lowState->motorState[legged_robot::RL_0].dq = msg.dq;
    _lowState->motorState[legged_robot::RL_0].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::RLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[legged_robot::RL_1].mode = msg.mode;
    _lowState->motorState[legged_robot::RL_1].q = msg.q;
    _lowState->motorState[legged_robot::RL_1].dq = msg.dq;
    _lowState->motorState[legged_robot::RL_1].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::RLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[legged_robot::RL_2].mode = msg.mode;
    _lowState->motorState[legged_robot::RL_2].q = msg.q;
    _lowState->motorState[legged_robot::RL_2].dq = msg.dq;
    _lowState->motorState[legged_robot::RL_2].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::FLfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    _lowState->eeForce[legged_robot::FL_].x = msg.wrench.force.x;
    _lowState->eeForce[legged_robot::FL_].y = msg.wrench.force.y;
    _lowState->eeForce[legged_robot::FL_].z = msg.wrench.force.z;
    _lowState->footForce[legged_robot::FL_] = msg.wrench.force.z;
}

void GazeboCommunicationChannel::FRfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    _lowState->eeForce[legged_robot::FR_].x = msg.wrench.force.x;
    _lowState->eeForce[legged_robot::FR_].y = msg.wrench.force.y;
    _lowState->eeForce[legged_robot::FR_].z = msg.wrench.force.z;
    _lowState->footForce[legged_robot::FR_] = msg.wrench.force.z;
}

void GazeboCommunicationChannel::RLfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    _lowState->eeForce[legged_robot::RL_].x = msg.wrench.force.x;
    _lowState->eeForce[legged_robot::RL_].y = msg.wrench.force.y;
    _lowState->eeForce[legged_robot::RL_].z = msg.wrench.force.z;
    _lowState->footForce[legged_robot::RL_] = msg.wrench.force.z;
}

void GazeboCommunicationChannel::RRfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    _lowState->eeForce[legged_robot::RR_].x = msg.wrench.force.x;
    _lowState->eeForce[legged_robot::RR_].y = msg.wrench.force.y;
    _lowState->eeForce[legged_robot::RR_].z = msg.wrench.force.z;
    _lowState->footForce[legged_robot::RR_] = msg.wrench.force.z;
}

GazeboCommunicationChannel::~GazeboCommunicationChannel()
{
    std::cout << "Deallocating Gazebo Communication Channel\n";
}
