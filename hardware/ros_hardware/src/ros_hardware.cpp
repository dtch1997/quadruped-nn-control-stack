#include "ros_hardware.h"

int RosHardware::start()
{
    gazebo_comm = new GazeboCommunicationChannel(
        "a1", nh, &cmd_, &data_
    );
    return 0;
}

int RosHardware::stop()
{
    delete gazebo_comm;
    return 0;
}

int RosHardware::read() 
{
    // Update _robot_state.legs
    for (uint leg = 0; leg<4; leg++)
    {
        for(uint jindx = 0; jindx<3; jindx++)
        {
            _robot_state.legs[leg].joints[jindx].q = data_.motorState[leg*3+jindx].q;
            _robot_state.legs[leg].joints[jindx].qd = data_.motorState[leg*3+jindx].dq;
            _robot_state.legs[leg].joints[jindx].tau = data_.motorState[leg*3+jindx].tauEst;
        }
    }

    // Update _robot_state.imu
    for (uint i = 0; i < 4; i++)
    {
        _robot_state.imu.quad[i] = data_.imu.quaternion[i];
    }
    for (uint i = 0; i < 3; i++)
    {
        _robot_state.imu.gyro[i] = data_.imu.gyroscope[i];
    }
    for (uint i = 0; i < 3; i++)
    {
        _robot_state.imu.acc[i] = data_.imu.accelerometer[i];
    }
    return 0;
    
    gazebo_comm->pubLowState();
}

int RosHardware::write() 
{
    // Update cmd_ from _robot_command
    for (uint leg = 0; leg<4; leg++)
    {
        for(uint jindx = 0; jindx<3; jindx++)
        {
            cmd_.motorCmd[leg*3+jindx].q = _robot_command.legs[leg].joints[jindx].q;
            cmd_.motorCmd[leg*3+jindx].dq = _robot_command.legs[leg].joints[jindx].qd;
            cmd_.motorCmd[leg*3+jindx].tau = _robot_command.legs[leg].joints[jindx].tau;
            cmd_.motorCmd[leg*3+jindx].Kp = _robot_command.legs[leg].joints[jindx].kp;
            cmd_.motorCmd[leg*3+jindx].Kd = _robot_command.legs[leg].joints[jindx].kd;
        }
    }

    gazebo_comm->sendServoCmd();

    return 0;
}  