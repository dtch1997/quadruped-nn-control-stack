#ifndef CONTROLLER__SINE_CONTROLLER
#define CONTROLLER__SINE_CONTROLLER

#pragma once

#include "controller_interface.h"

/*  Controller that implements sinusoidal movement for single joint. 
    Adapted from official Unitree example:
    https://github.com/unitreerobotics/unitree_legged_sdk/blob/master/example/example_position.cpp
    */
class SineController : public interfaces::ControllerInterface
{
public:
    int start() override; 
    int step_control() override;
    int stop() override;

private:
    float qInit[3]={0};
    float qDes[3]={0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};  
    float Kd[3] = {0};
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
}; // SineController

#endif // CONTROLLER__SINE_CONTROLLER